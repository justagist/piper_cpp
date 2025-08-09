#pragma once

#include <algorithm>
#include <chrono>
#include <mutex>
#include <numeric>
#include <sstream>

namespace piper_cpp
{

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
        last_timestamp_.resize(num_callers, 0.0);
        initialized_.resize(num_callers, false);
        hz_.resize(num_callers, 0.0);
    }

    size_t getNumCallers() const
    {
        std::lock_guard<std::mutex> lk(mutex_);
        return static_cast<size_t>(last_timestamp_.size());
    }

    /// Atomically replace the contained value, and update timestamp/hz.
    void set(const T& newValue, double timestamp, int caller_id = 0)
    {
        std::lock_guard<std::mutex> lk(mutex_);
        double now_sec =
            std::chrono::duration_cast<std::chrono::duration<double>>(clock::now().time_since_epoch()).count();

        // compute hz only if we've been initialized once before
        if (initialized_.at(caller_id))
        {
            double dt = now_sec - last_timestamp_.at(caller_id);
            hz_.at(caller_id) = (dt > 0.0) ? (1.0 / dt) : 0.0;
        }
        value_ = newValue;
        timestamp_ = timestamp; // Use provided timestamp, not now_sec
        last_timestamp_.at(caller_id) = now_sec;
        initialized_.at(caller_id) = true;
    }

    bool isValid() const
    {
        std::lock_guard<std::mutex> lk(mutex_);
        return initialized_.size() > 0 &&
               std::any_of(initialized_.begin(), initialized_.end(), [](bool init) { return init; });
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
    /// Instantaneous rate based on last two set() calls (averaged if multiple callers)
    double getHz() const
    {
        std::lock_guard<std::mutex> lk(mutex_);
        return std::accumulate(hz_.begin(), hz_.end(), 0.0) / hz_.size();
    }

private:
    mutable std::mutex mutex_;
    T value_{};
    double timestamp_{0.0};
    std::vector<double> hz_; // Instantaneous rate
    std::vector<double> last_timestamp_{};
    std::vector<bool> initialized_;
};

template <typename T> struct StateSnapshot
{
    T value;
    double timestamp;
    double hz;
    bool is_valid;

    StateSnapshot(const TimedFreqState<T>& state)
        : value(state.getValue()), timestamp(state.getTimestamp()), hz(state.getHz()), is_valid(state.isValid())
    {
    }

    std::string toString() const
    {
        std::ostringstream oss;
        oss << "StateSnapshot(valid=" << (is_valid ? "true" : "false") << "; timestamp=" << timestamp << "; hz=" << hz
            << " value=" << value.toString() << ")";
        return oss.str();
    }
};

} // namespace piper_cpp
