#pragma once
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

    void setNumCallers(int num_callers)
    {
        std::lock_guard<std::mutex> lk(mutex_);
        last_timestamp_.resize(num_callers, 0.0);
        initialized_.resize(num_callers, false);
        hz_.resize(num_callers, 0.0);
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

template <typename T> struct StateSnapShot
{
    T value;
    double timestamp;
    double hz;

    StateSnapShot(const TimedFreqState<T>& state)
        : value(state.getValue()), timestamp(state.getTimestamp()), hz(state.getHz())
    {
    }

    std::string toString() const
    {
        std::ostringstream oss;
        oss << "StateSnapShot(value=" << value.toString() << ", timestamp=" << timestamp << ", hz=" << hz << ")";
        return oss.str();
    }
};

} // namespace piper_cpp
