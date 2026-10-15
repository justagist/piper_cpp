#ifndef PIPER_CPP_STD_CAN_INTERFACE_H
#define PIPER_CPP_STD_CAN_INTERFACE_H

#include "piper_cpp/can_utils.h"
#include <cerrno>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <functional>
#include <iostream>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <optional>
#include <poll.h>
#include <string>
#include <sys/eventfd.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>
#include <vector>

namespace piper_cpp
{

class StdCanInterface
{
public:
    using CanCallback = std::function<void(const struct can_frame&, double)>;

    StdCanInterface(
        std::string channel_name = "can0", int expected_bitrate = 1000000, bool judge_flag = true,
        bool auto_init = true, CanCallback callback = nullptr
    )
        : channel_name_(std::move(channel_name)), expected_bitrate_(expected_bitrate), sock_(-1), wake_fd_(-1),
          callback_(std::move(callback))
    {
        // Validate before creating the eventfd: a throwing constructor runs no destructor.
        if (judge_flag)
            judgeCanInfo();
        wake_fd_ = makeWakeFd();
        if (auto_init)
            init();
    }

    /// Adopt an already-open socket that delivers one `can_frame` per datagram (a raw CAN socket,
    /// or a socketpair in tests). Takes ownership of `socket_fd`; `init()` is a no-op.
    explicit StdCanInterface(int socket_fd, CanCallback callback = nullptr)
        : channel_name_(), expected_bitrate_(0), sock_(socket_fd), wake_fd_(makeWakeFd()),
          callback_(std::move(callback))
    {
    }

    // Owns file descriptors, so copying would double-close them.
    StdCanInterface(const StdCanInterface&) = delete;
    StdCanInterface& operator=(const StdCanInterface&) = delete;

    ~StdCanInterface()
    {
        close();
        ::close(wake_fd_);
    }

    /// Open and bind the raw CAN socket. On failure nothing is kept, so a later call retries
    /// from scratch instead of reporting an unbound socket as open.
    bool init()
    {
        if (sock_ != -1)
            return true;
        int fd = socket(PF_CAN, SOCK_RAW, CAN_RAW);
        if (fd < 0)
        {
            perror("socket");
            return false;
        }

        int enable = 1;
        if (setsockopt(fd, SOL_SOCKET, SO_TIMESTAMP, &enable, sizeof(enable)) < 0)
            perror("setsockopt SO_TIMESTAMP");

        struct ifreq ifr;
        strncpy(ifr.ifr_name, channel_name_.c_str(), IFNAMSIZ);
        if (ioctl(fd, SIOCGIFINDEX, &ifr) < 0)
        {
            perror("ioctl");
            ::close(fd);
            return false;
        }

        addr_.can_family = AF_CAN;
        addr_.can_ifindex = ifr.ifr_ifindex;
        if (bind(fd, (struct sockaddr*)&addr_, sizeof(addr_)) < 0)
        {
            perror("bind");
            ::close(fd);
            return false;
        }
        drainWake(); // an interrupt left over from a previous session must not cancel the first read
        sock_ = fd;
        return true;
    }

    void close()
    {
        if (sock_ >= 0)
        {
            ::close(sock_);
            sock_ = -1;
        }
    }

    void judgeCanInfo()
    {
        if (!isInterfaceAvailable())
            throw std::runtime_error("CAN interface " + channel_name_ + " not found");
        if (!piper_cpp::interfaceIsUp(channel_name_))
            throw std::runtime_error("CAN interface " + channel_name_ + " is not up");
        auto br = piper_cpp::getInterfaceBitrate(channel_name_);
        if (!br || *br != expected_bitrate_)
            throw std::runtime_error(
                "CAN bitrate mismatch: expected " + std::to_string(expected_bitrate_) + ", got " +
                std::to_string(br.value_or(-1))
            );
    }

    /// Block until one frame arrives and hand it to the callback, or until `interruptRead()` is
    /// called. Returns true only when a frame was delivered to the callback.
    bool readCanMessage()
    {
        if (sock_ < 0)
            return false;

        struct pollfd fds[2] = {
            {sock_,    POLLIN, 0},
            {wake_fd_, POLLIN, 0}
        };
        int ready;
        do
        {
            ready = poll(fds, 2, -1);
        } while (ready < 0 && errno == EINTR);
        if (ready < 0)
            return false;
        if (fds[1].revents & POLLIN)
        {
            drainWake();
            return false;
        }

        struct can_frame frame;
        struct msghdr msg = {};
        struct iovec iov;
        char ctrlmsg[CMSG_SPACE(sizeof(struct timeval))];

        iov.iov_base = &frame;
        iov.iov_len = sizeof(frame);

        msg.msg_iov = &iov;
        msg.msg_iovlen = 1;
        msg.msg_control = ctrlmsg;
        msg.msg_controllen = sizeof(ctrlmsg);

        ssize_t nbytes = recvmsg(sock_, &msg, 0);
        if (nbytes < 0)
            return false;

        double timestamp = 0.0;
        struct timeval tv;
        memset(&tv, 0, sizeof(tv));
        for (struct cmsghdr* cmsg = CMSG_FIRSTHDR(&msg); cmsg != nullptr; cmsg = CMSG_NXTHDR(&msg, cmsg))
        {
            if (cmsg->cmsg_level == SOL_SOCKET && cmsg->cmsg_type == SO_TIMESTAMP)
            {
                memcpy(&tv, CMSG_DATA(cmsg), sizeof(tv));
                timestamp = tv.tv_sec + tv.tv_usec / 1e6;
                break;
            }
        }

        if (callback_)
        {
            callback_(frame, timestamp);
            return true;
        }
        return false;
    }

    /// Wake a thread blocked in `readCanMessage()`, which then returns false. Safe to call from
    /// any thread. If no read is in progress, the next `readCanMessage()` returns false once.
    void interruptRead() { eventfd_write(wake_fd_, 1); }

    /// Replace the receive callback. Not synchronised with `readCanMessage()`; set it before
    /// reading starts.
    void setCallback(CanCallback callback) { callback_ = std::move(callback); }

    bool sendCanMessage(uint32_t id, const std::vector<uint8_t>& data)
    {
        if (sock_ < 0)
            return false;
        struct can_frame frame
        {
        };
        frame.can_id = id;
        frame.can_dlc = std::min((size_t)8, data.size());
        std::copy(data.begin(), data.begin() + frame.can_dlc, frame.data);
        return write(sock_, &frame, sizeof(frame)) > 0;
    }

    static std::vector<std::string> getCanPorts()
    {
        std::vector<std::string> ports;
        for (const auto& p : piper_cpp::findPorts())
            ports.push_back(p.first);
        return ports;
    }

    static std::string getCanPortInfo(const std::string& iface)
    {
        std::ifstream op("/sys/class/net/" + iface + "/operstate");
        std::ifstream tp("/sys/class/net/" + iface + "/type");
        auto bitrate = piper_cpp::getInterfaceBitrate(iface);
        std::string state = op ? std::string((std::istreambuf_iterator<char>(op)), {}) : "?";
        std::string type = tp ? std::string((std::istreambuf_iterator<char>(tp)), {}) : "?";
        return "CAN port " + iface + ": State=" + state + ", Type=" + type +
               ", Bitrate=" + (bitrate ? std::to_string(*bitrate) : "Unknown");
    }

private:
    bool isInterfaceAvailable()
    {
        std::ifstream f("/sys/class/net/" + channel_name_ + "/operstate");
        return f.good();
    }

    static int makeWakeFd()
    {
        int fd = eventfd(0, EFD_NONBLOCK | EFD_CLOEXEC);
        if (fd < 0)
            throw std::runtime_error(std::string("eventfd: ") + strerror(errno));
        return fd;
    }

    void drainWake()
    {
        eventfd_t value;
        (void)eventfd_read(wake_fd_, &value);
    }

    std::string channel_name_;
    int expected_bitrate_;
    int sock_;
    int wake_fd_;
    struct sockaddr_can addr_
    {
    };
    CanCallback callback_;
};

} // namespace piper_cpp

#endif // PIPER_CPP_STD_CAN_INTERFACE_H
