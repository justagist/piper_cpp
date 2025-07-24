#ifndef PIPER_CPP_STD_CAN_INTERFACE_H
#define PIPER_CPP_STD_CAN_INTERFACE_H

#include "piper_cpp/can_utils.h"
#include <cstdint>
#include <cstring>
#include <fstream>
#include <functional>
#include <iostream>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <optional>
#include <string>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>
#include <vector>

namespace piper_cpp
{

class StdCanInterface
{
public:
    using CanCallback = std::function<void(const struct can_frame&)>;

    StdCanInterface(
        std::string channel_name = "can0", int expected_bitrate = 1000000, bool judge_flag = true,
        bool auto_init = true, CanCallback callback = nullptr
    )
        : channel_name_(std::move(channel_name)), expected_bitrate_(expected_bitrate), callback_(std::move(callback)),
          sock_(-1)
    {
        if (judge_flag)
            judgeCanInfo();
        if (auto_init)
            init();
    }

    ~StdCanInterface() { close(); }

    bool init()
    {
        if (sock_ != -1)
            return true;
        sock_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
        if (sock_ < 0)
        {
            perror("socket");
            return false;
        }
        struct ifreq ifr
        {
        };
        std::strncpy(ifr.ifr_name, channel_name_.c_str(), IFNAMSIZ);
        if (ioctl(sock_, SIOCGIFINDEX, &ifr) < 0)
        {
            perror("ioctl");
            return false;
        }
        addr_.can_family = AF_CAN;
        addr_.can_ifindex = ifr.ifr_ifindex;
        if (bind(sock_, (struct sockaddr*)&addr_, sizeof(addr_)) < 0)
        {
            perror("bind");
            return false;
        }
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

    bool readCanMessage()
    {
        if (sock_ < 0)
            return false;
        struct can_frame frame;
        int nbytes = read(sock_, &frame, sizeof(frame));
        if (nbytes > 0 && callback_)
        {
            callback_(frame);
            return true;
        }
        return false;
    }

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

    std::string channel_name_;
    int expected_bitrate_;
    int sock_;
    struct sockaddr_can addr_
    {
    };
    CanCallback callback_;
};

} // namespace piper_cpp

#endif // PIPER_CPP_STD_CAN_INTERFACE_H
