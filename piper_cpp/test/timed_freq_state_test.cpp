#include "piper_cpp/interface/timed_freq_state.h"

#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <string>
#include <thread>

using namespace std::chrono_literals;
using piper_cpp::StateSnapshot;
using piper_cpp::TimedFreqState;

namespace
{

struct Reading
{
    int n{0};
    std::string toString() const { return std::to_string(n); }
};

} // namespace

// Timeouts are generous relative to the sleeps so a descheduled test thread cannot expire a
// state before the "still healthy" assertions run.
TEST(TimedFreqState, HealthExpiresAfterTheTimeout)
{
    TimedFreqState<Reading> state;
    state.setHealthTimeout(200ms);
    state.set(Reading{1}, 0.0);
    EXPECT_TRUE(state.isHealthy());

    std::this_thread::sleep_for(400ms);
    EXPECT_FALSE(state.isHealthy()) << "no update for longer than the timeout";
    EXPECT_TRUE(state.isValid()) << "validity is about having received, not freshness";
}

TEST(TimedFreqState, NoTimeoutStaysHealthyOnceReceived)
{
    TimedFreqState<Reading> state;
    state.setHealthTimeout(1ms);
    state.set(Reading{1}, 0.0);
    std::this_thread::sleep_for(5ms);
    ASSERT_FALSE(state.isHealthy());

    state.setHealthTimeout(piper_cpp::kNoHealthTimeout);
    EXPECT_TRUE(state.isHealthy()) << "a one-shot response does not go stale";
}

TEST(TimedFreqState, MultipartHealthNeedsEveryPartFresh)
{
    TimedFreqState<Reading> state;
    state.setNumCallers(3);
    state.setHealthTimeout(200ms);
    for (int part = 0; part < 3; ++part)
        state.set(Reading{part}, 0.0, part);
    ASSERT_TRUE(state.isHealthy());

    std::this_thread::sleep_for(400ms);
    state.set(Reading{0}, 0.0, 0);
    EXPECT_FALSE(state.isHealthy()) << "one fresh part cannot hide two stale ones";
    state.set(Reading{1}, 0.0, 1);
    state.set(Reading{2}, 0.0, 2);
    EXPECT_TRUE(state.isHealthy());
}

TEST(TimedFreqState, SnapshotPairsValueWithItsOwnMetadata)
{
    // The writer keeps value and timestamp equal, so any snapshot where they differ mixed
    // two updates. Separately locked getters did exactly that.
    TimedFreqState<Reading> state;
    std::atomic<bool> stop{false};
    std::thread writer(
        [&]
        {
            for (int n = 1; !stop.load(); ++n)
                state.set(Reading{n}, static_cast<double>(n));
        }
    );

    int valid = 0;
    int mismatches = 0;
    for (int i = 0; i < 20000; ++i)
    {
        const StateSnapshot<Reading> snap(state);
        if (!snap.is_valid)
            continue;
        ++valid;
        if (static_cast<double>(snap.value.n) != snap.timestamp)
            ++mismatches;
    }
    stop.store(true);
    writer.join();
    EXPECT_GT(valid, 0) << "the check needs snapshots that saw the writer";
    EXPECT_EQ(mismatches, 0);
}

TEST(TimedFreqState, ResetForgetsValueAndArrivals)
{
    TimedFreqState<Reading> state;
    state.set(Reading{7}, 3.0);
    ASSERT_TRUE(state.isValid());

    state.reset();
    EXPECT_FALSE(state.isValid());
    EXPECT_FALSE(state.isHealthy());
    EXPECT_EQ(state.getValue().n, 0);
    EXPECT_EQ(state.getTimestamp(), 0.0);

    state.set(Reading{8}, 4.0);
    EXPECT_TRUE(state.isValid());
}
