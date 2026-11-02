// Shared helpers for the parser and interface tests. The interface tests drive an interface
// through a socketpair in place of the CAN bus: frames written to the peer end reach the
// interface's read thread exactly as they would from a raw CAN socket, and frames the
// interface sends can be read back from the peer end.
#pragma once

#include "piper_cpp/interface/piper_interface_v2.h"

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <initializer_list>
#include <linux/can.h>
#include <memory>
#include <optional>
#include <poll.h>
#include <stdexcept>
#include <sys/socket.h>
#include <thread>
#include <unistd.h>

namespace test_support
{

using namespace std::chrono_literals;
using std::chrono::steady_clock;

template <typename Interface> struct Bus
{
    int peer;
    std::unique_ptr<Interface> piper;
    // The interface and its read thread must be gone before the peer end closes, and members
    // are only destroyed after this body runs.
    ~Bus()
    {
        piper.reset();
        ::close(peer);
    }
};

/// An interface of the given protocol class wired to the returned peer socket. `args` are the
/// constructor parameters after the transport (dh_offset, sdk_joint_limit, sdk_gripper_limit).
template <typename Interface = piper_cpp::PiperInterfaceV2, typename... Args> Bus<Interface> makeBus(Args... args)
{
    int sv[2];
    if (socketpair(AF_UNIX, SOCK_SEQPACKET, 0, sv) != 0)
        throw std::runtime_error("socketpair failed");
    return {sv[1], std::make_unique<Interface>(std::make_unique<piper_cpp::StdCanInterface>(sv[0]), args...)};
}

inline can_frame frame(uint32_t id, std::initializer_list<uint8_t> bytes)
{
    can_frame f{};
    f.can_id = id;
    f.can_dlc = static_cast<uint8_t>(bytes.size());
    std::copy(bytes.begin(), bytes.end(), f.data);
    return f;
}

/// Joint-pair frames (feedback, command echo, IK) carry two big-endian int32 angles in 0.001 deg.
inline can_frame jointPairFrame(uint32_t id, int32_t a, int32_t b)
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

template <typename Pred> bool waitFor(Pred pred, std::chrono::milliseconds timeout = 2s)
{
    const auto deadline = steady_clock::now() + timeout;
    while (!pred())
    {
        if (steady_clock::now() > deadline)
            return false;
        std::this_thread::sleep_for(1ms);
    }
    return true;
}

inline void sendFrame(int fd, const can_frame& f, steady_clock::time_point deadline)
{
    while (::send(fd, &f, sizeof(f), MSG_DONTWAIT) < 0 && errno == EAGAIN && steady_clock::now() < deadline)
        std::this_thread::yield();
}

/// The next frame the interface sent, or nothing if none arrives within `timeout`.
inline std::optional<can_frame> receiveFrame(int fd, std::chrono::milliseconds timeout = 1s)
{
    pollfd pfd{fd, POLLIN, 0};
    can_frame f{};
    if (poll(&pfd, 1, static_cast<int>(timeout.count())) <= 0 ||
        ::recv(fd, &f, sizeof(f), 0) != static_cast<ssize_t>(sizeof(f)))
        return std::nullopt;
    return f;
}

} // namespace test_support
