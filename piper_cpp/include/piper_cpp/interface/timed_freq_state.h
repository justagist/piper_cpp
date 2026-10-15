#pragma once

#include <algorithm>
#include <chrono>
#include <mutex>
#include <numeric>
#include <sstream>
#include <vector>

namespace piper_cpp
{

template <typename T> struct StateSnapshot;

/// Pass to `TimedFreqState::setHealthTimeout()` for one-shot responses, which stay healthy
/// once received.
inline constexpr std::chrono::steady_clock::duration kNoHealthTimeout = std::chrono::steady_clock::duration::max();

/// A value updated from the CAN read thread, with per-part arrival tracking so callers can tell
/// a live stream from a stale one. Multipart messages register one caller per part.
template <typename T> class TimedFreqState
{
    using clock = std::chrono::steady_clock;

public:
    TimedFreqState()
    {
        setNumCallers(1); // Default to 1 caller
    }

    void setNumCallers(size_t num_callers)
    {
        std::lock_guard<std::mutex> lk(mutex_);
        last_received_.resize(num_callers);
        initialized_.resize(num_callers, false);
        hz_.resize(num_callers, 0.0);
    }

    size_t getNumCallers() const
    {
        std::lock_guard<std::mutex> lk(mutex_);
        return last_received_.size();
    }

    /// How long after the last `set()` of any part the state stops being healthy. Defaults to
    /// one second, which suits the periodic feedback streams.
    void setHealthTimeout(clock::duration timeout)
    {
        std::lock_guard<std::mutex> lk(mutex_);
        health_timeout_ = timeout;
    }

    /// Atomically replace the contained value, and update timestamp/hz.
    void set(const T& newValue, double timestamp, int caller_id = 0)
    {
        std::lock_guard<std::mutex> lk(mutex_);
        const auto now = clock::now();

        // compute hz only if we've been initialized once before
        if (initialized_.at(caller_id))
        {
            const double dt = std::chrono::duration<double>(now - last_received_.at(caller_id)).count();
            hz_.at(caller_id) = (dt > 0.0) ? (1.0 / dt) : 0.0;
        }
        value_ = newValue;
        timestamp_ = timestamp; // Use provided timestamp, not now
        last_received_.at(caller_id) = now;
        initialized_.at(caller_id) = true;
    }

    /// Forget the value and every part's arrival, as at construction.
    void reset()
    {
        std::lock_guard<std::mutex> lk(mutex_);
        value_ = T{};
        timestamp_ = 0.0;
        std::fill(initialized_.begin(), initialized_.end(), false);
        std::fill(hz_.begin(), hz_.end(), 0.0);
    }

    bool isValid() const
    {
        std::lock_guard<std::mutex> lk(mutex_);
        return isValidLocked();
    }

    bool isHealthy() const
    {
        std::lock_guard<std::mutex> lk(mutex_);
        return isHealthyLocked(clock::now());
    }

    /// Atomically read out the payload
    T getValue() const
    {
        std::lock_guard<std::mutex> lk(mutex_);
        return value_;
    }
    /// Timestamp as set in the last set() call by the caller
    double getTimestamp() const
    {
        std::lock_guard<std::mutex> lk(mutex_);
        return timestamp_;
    }
    /// Instantaneous rate based on last set() calls (averaged if multiple callers)
    double getHz() const
    {
        std::lock_guard<std::mutex> lk(mutex_);
        return getHzLocked();
    }

    /// Value and metadata taken under one lock, so they describe the same update.
    StateSnapshot<T> snapshot() const;

private:
    bool isValidLocked() const
    {
        return initialized_.size() > 0 &&
               std::all_of(initialized_.begin(), initialized_.end(), [](bool init) { return init; });
    }

    bool isHealthyLocked(clock::time_point now) const
    {
        if (!isValidLocked())
            return false;
        for (const auto& received : last_received_)
        {
            if (now - received > health_timeout_)
                return false;
        }
        return true;
    }

    double getHzLocked() const { return std::accumulate(hz_.begin(), hz_.end(), 0.0) / hz_.size(); }

    mutable std::mutex mutex_;
    T value_{};
    double timestamp_{0.0};
    clock::duration health_timeout_{std::chrono::seconds(1)};
    std::vector<double> hz_; // Instantaneous rate
    std::vector<clock::time_point> last_received_;
    std::vector<bool> initialized_;
};

template <typename T> struct StateSnapshot
{
    T value;
    double timestamp;
    double hz;
    bool is_valid;
    bool is_healthy;

    StateSnapshot(const TimedFreqState<T>& state) : StateSnapshot(state.snapshot()) {}

    StateSnapshot(T value_, double timestamp_, double hz_, bool is_valid_, bool is_healthy_)
        : value(std::move(value_)), timestamp(timestamp_), hz(hz_), is_valid(is_valid_), is_healthy(is_healthy_)
    {
    }

    std::string toString() const
    {
        std::ostringstream oss;
        oss << "StateSnapshot(valid=" << (is_valid ? "true" : "false") << "; timestamp=" << timestamp << "; hz=" << hz
            << "; is_healthy=" << (is_healthy ? "true" : "false") << "; value=" << value.toString() << ")";
        return oss.str();
    }
};

template <typename T> StateSnapshot<T> TimedFreqState<T>::snapshot() const
{
    std::lock_guard<std::mutex> lk(mutex_);
    const auto now = clock::now();
    return StateSnapshot<T>(value_, timestamp_, getHzLocked(), isValidLocked(), isHealthyLocked(now));
}

} // namespace piper_cpp
