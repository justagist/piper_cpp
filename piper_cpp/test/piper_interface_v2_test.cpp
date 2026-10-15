// Drives PiperInterfaceV2 through a socketpair in place of the CAN bus: frames written to the
// peer end reach the interface's read thread exactly as they would from a raw CAN socket.

#include "piper_cpp/interface/piper_interface_v2.h"

#include <gtest/gtest.h>

#include <chrono>
#include <future>
#include <memory>
#include <sys/socket.h>
#include <thread>

using namespace std::chrono_literals;
using piper_cpp::PiperInterfaceV2;
using piper_cpp::StdCanInterface;
using std::chrono::steady_clock;

namespace
{

struct Bus
{
    int peer;
    std::unique_ptr<PiperInterfaceV2> piper; // destroyed first, so the read thread is gone before peer closes
    ~Bus() { ::close(peer); }
};

Bus makeBus()
{
    int sv[2];
    if (socketpair(AF_UNIX, SOCK_SEQPACKET, 0, sv) != 0)
        throw std::runtime_error("socketpair failed");
    return {sv[1], std::make_unique<PiperInterfaceV2>(std::make_unique<StdCanInterface>(sv[0]))};
}

// Joint feedback frames carry two big-endian int32 angles in 0.001 deg.
can_frame jointFeedbackFrame(uint32_t id, int32_t a, int32_t b)
{
    can_frame f{};
    f.can_id = id;
    f.can_dlc = 8;
    const auto ua = static_cast<uint32_t>(a);
    const auto ub = static_cast<uint32_t>(b);
    for (int i = 0; i < 4; ++i)
    {
        f.data[i] = static_cast<uint8_t>(ua >> (24 - 8 * i));
        f.data[4 + i] = static_cast<uint8_t>(ub >> (24 - 8 * i));
    }
    return f;
}

void sendFrame(int fd, const can_frame& f, steady_clock::time_point deadline)
{
    while (::send(fd, &f, sizeof(f), MSG_DONTWAIT) < 0 && errno == EAGAIN && steady_clock::now() < deadline)
        std::this_thread::yield();
}

} // namespace

TEST(PiperInterfaceV2, DisconnectReturnsPromptlyOnSilentBus)
{
    auto bus = makeBus();
    ASSERT_TRUE(bus.piper->connectPort(/*can_init=*/false, /*piper_init=*/false, /*start_threads=*/true));
    std::this_thread::sleep_for(50ms); // let the read thread block on traffic that never comes

    auto done = std::async(std::launch::async, [&] { bus.piper->disconnectPort(); });
    ASSERT_EQ(done.wait_for(2s), std::future_status::ready) << "disconnectPort() hung on a silent bus";
    EXPECT_FALSE(bus.piper->isConnected());
}

TEST(PiperInterfaceV2, ReadLoopKeepsUpWithBurst)
{
    // 3000 frames is well beyond what the socket buffer holds, so the sender only finishes
    // quickly if the read thread drains as fast as frames arrive. With a 1 ms sleep per frame
    // this took several seconds; the bound below excludes that by an order of magnitude.
    constexpr int kSets = 1000;
    auto bus = makeBus();
    ASSERT_TRUE(bus.piper->connectPort(/*can_init=*/false, /*piper_init=*/false, /*start_threads=*/true));

    const auto deadline = steady_clock::now() + 10s;
    const auto t0 = steady_clock::now();
    for (int n = 1; n <= kSets; ++n)
    {
        sendFrame(bus.peer, jointFeedbackFrame(0x2A5, n, -n), deadline);
        sendFrame(bus.peer, jointFeedbackFrame(0x2A6, 2 * n, -2 * n), deadline);
        sendFrame(bus.peer, jointFeedbackFrame(0x2A7, 3 * n, -3 * n), deadline);
    }
    const auto send_ms = std::chrono::duration_cast<std::chrono::milliseconds>(steady_clock::now() - t0).count();
    EXPECT_LT(send_ms, 1000);

    // Every set must reach the state cache, the last one winning.
    auto js = bus.piper->getArmJointStates();
    for (auto t = steady_clock::now(); js.value.joint_6 != -3 * kSets && steady_clock::now() - t < 2s;)
    {
        std::this_thread::sleep_for(1ms);
        js = bus.piper->getArmJointStates();
    }
    EXPECT_TRUE(js.is_valid);
    EXPECT_EQ(js.value.joint_1, kSets);
    EXPECT_EQ(js.value.joint_2, -kSets);
    EXPECT_EQ(js.value.joint_3, 2 * kSets);
    EXPECT_EQ(js.value.joint_4, -2 * kSets);
    EXPECT_EQ(js.value.joint_5, 3 * kSets);
    EXPECT_EQ(js.value.joint_6, -3 * kSets);
}
