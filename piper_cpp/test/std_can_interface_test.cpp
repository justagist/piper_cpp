// Exercises StdCanInterface over a socketpair instead of a CAN adapter: each datagram written
// to the peer end arrives at the adopted end as one can_frame, which is exactly what a raw
// CAN socket delivers.

#include "piper_cpp/std_can_interface.h"

#include <gtest/gtest.h>

#include <chrono>
#include <future>
#include <sys/socket.h>
#include <thread>
#include <vector>

using namespace std::chrono_literals;
using piper_cpp::StdCanInterface;

namespace
{

// Returns {adopted, peer}: the first is handed to a StdCanInterface, the second stays raw.
std::pair<int, int> makeSocketPair()
{
    int sv[2];
    if (socketpair(AF_UNIX, SOCK_DGRAM, 0, sv) != 0)
        throw std::runtime_error("socketpair failed");
    return {sv[0], sv[1]};
}

struct RawFd
{
    int fd;
    ~RawFd() { ::close(fd); }
};

} // namespace

TEST(StdCanInterface, InterruptReadUnblocksSilentSocket)
{
    auto [adopted, peer] = makeSocketPair();
    RawFd peer_guard{peer};
    StdCanInterface can(adopted, [](const can_frame&, double) {});

    auto result = std::async(std::launch::async, [&] { return can.readCanMessage(); });
    ASSERT_EQ(result.wait_for(100ms), std::future_status::timeout) << "read should block while the bus is silent";

    can.interruptRead();
    ASSERT_EQ(result.wait_for(1s), std::future_status::ready) << "interruptRead() must wake the blocked read";
    EXPECT_FALSE(result.get());
}

TEST(StdCanInterface, PendingInterruptCancelsOnlyTheNextRead)
{
    auto [adopted, peer] = makeSocketPair();
    RawFd peer_guard{peer};
    int frames = 0;
    StdCanInterface can(adopted, [&](const can_frame&, double) { ++frames; });

    can.interruptRead();
    can_frame f{};
    f.can_dlc = 1;
    ASSERT_EQ(::send(peer, &f, sizeof(f), 0), static_cast<ssize_t>(sizeof(f)));

    // The interrupt wins over queued traffic, so a disconnect on a busy bus still stops the
    // reader; it is then consumed and must not swallow the frame behind it.
    EXPECT_FALSE(can.readCanMessage());
    EXPECT_TRUE(can.readCanMessage());
    EXPECT_EQ(frames, 1);
}

TEST(StdCanInterface, DeliversEveryFrameInOrder)
{
    constexpr uint32_t kFrames = 2000;
    const auto deadline = std::chrono::steady_clock::now() + 5s;

    auto [adopted, peer] = makeSocketPair();
    RawFd peer_guard{peer};
    std::vector<uint32_t> received;
    StdCanInterface can(adopted, [&](const can_frame& f, double) { received.push_back(f.can_id); });

    // The socket buffer holds far fewer than kFrames datagrams, so the producer only finishes
    // if the reader keeps draining; MSG_DONTWAIT plus the deadline keep it from hanging if not.
    std::thread producer(
        [&, peer = peer]
        {
            can_frame f{};
            f.can_dlc = 1;
            for (uint32_t id = 0; id < kFrames; ++id)
            {
                f.can_id = id;
                while (::send(peer, &f, sizeof(f), MSG_DONTWAIT) < 0 && errno == EAGAIN &&
                       std::chrono::steady_clock::now() < deadline)
                    std::this_thread::yield();
            }
        }
    );

    std::promise<void> done;
    std::thread watchdog(
        [&, finished = done.get_future()]
        {
            if (finished.wait_until(deadline) == std::future_status::timeout)
                can.interruptRead();
        }
    );

    while (received.size() < kFrames && can.readCanMessage())
    {
    }
    done.set_value();
    watchdog.join();
    producer.join();

    ASSERT_EQ(received.size(), kFrames);
    for (uint32_t id = 0; id < kFrames; ++id)
        ASSERT_EQ(received[id], id) << "frame " << id << " arrived out of order";
}
