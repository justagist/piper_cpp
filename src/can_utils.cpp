#include "piper_cpp/can_utils.h"
#include <algorithm>
#include <array>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <iostream>
#include <libsocketcan.h>
#include <memory>
#include <net/if.h>
#include <sstream>
#include <stdexcept>
#include <sys/ioctl.h>
#include <thread>
#include <unistd.h>

namespace piper_cpp
{

static std::vector<PortPair> getCanInterfacesInternal()
{
    std::vector<PortPair> result;
    std::array<char, 256> buf;
    FILE* pipe = popen("ip -br link show type can", "r");
    if (!pipe)
        return result;
    while (fgets(buf.data(), buf.size(), pipe))
    {
        std::string line(buf.data());
        if (line.empty())
            continue;
        auto iface = line.substr(0, line.find(' '));
        FILE* epipe = popen((std::string("ethtool -i ") + iface).c_str(), "r");
        if (!epipe)
            continue;
        while (fgets(buf.data(), buf.size(), epipe))
        {
            std::string el(buf.data());
            if (el.find("bus-info") != std::string::npos)
            {
                auto usb = el.substr(el.find_last_of(' ') + 1);
                usb.erase(std::remove(usb.begin(), usb.end(), '\n'), usb.end());
                result.emplace_back(iface, usb);
                break;
            }
        }
        pclose(epipe);
    }
    pclose(pipe);
    return result;
}

bool interfaceIsUp(const std::string& name)
{
    int sock = socket(AF_INET, SOCK_DGRAM, 0);
    if (sock < 0)
        return false;
    struct ifreq ifr;
    std::memset(&ifr, 0, sizeof(ifr));
    std::strncpy(ifr.ifr_name, name.c_str(), IFNAMSIZ - 1);
    bool is_up = false;
    if (ioctl(sock, SIOCGIFFLAGS, &ifr) == 0)
    {
        is_up = (ifr.ifr_flags & IFF_UP) != 0;
    }
    close(sock);
    return is_up;
}

static bool renameInterface(const std::string& old_name, const std::string& new_name)
{
    int sock = socket(AF_INET, SOCK_DGRAM, 0);
    if (sock < 0)
        return false;
    struct ifreq ifr;
    std::memset(&ifr, 0, sizeof(ifr));
    std::strncpy(ifr.ifr_name, old_name.c_str(), IFNAMSIZ - 1);
    std::strncpy(ifr.ifr_newname, new_name.c_str(), IFNAMSIZ - 1);
    bool ok = (ioctl(sock, SIOCSIFNAME, &ifr) == 0);
    close(sock);
    return ok;
}

std::vector<PortPair> findPorts() { return getCanInterfacesInternal(); }

std::vector<std::string> activePorts()
{
    std::vector<std::string> up;
    for (auto& p : findPorts())
    {
        if (interfaceIsUp(p.first))
            up.push_back(p.first);
    }
    return up;
}

void activate(const std::vector<PortPair>& ports, const std::string& prefix, int bitrate, std::optional<int> timeout)
{
    auto list = ports.empty() ? findPorts() : ports;
    if (list.empty() && timeout)
    {
        auto start = std::chrono::steady_clock::now();
        while (std::chrono::duration_cast<std::chrono::seconds>(std::chrono::steady_clock::now() - start).count() <
               *timeout)
        {
            list = findPorts();
            if (!list.empty())
                break;
            std::this_thread::sleep_for(std::chrono::seconds(5));
        }
        if (list.empty())
            throw std::runtime_error("Timed out waiting for CAN devices");
    }
    std::sort(list.begin(), list.end(), [](auto& a, auto& b) { return a.second < b.second; });
    for (size_t i = 0; i < list.size(); ++i)
    {
        const std::string iface = list[i].first;
        const std::string target = prefix + std::to_string(i);
        // stop interface
        if (can_do_stop(iface.c_str()) < 0)
            throw std::runtime_error("Failed to stop interface " + iface);
        // set bitrate
        if (can_set_bitrate(iface.c_str(), static_cast<unsigned int>(bitrate)) < 0)
            throw std::runtime_error("Failed to set bitrate on " + iface);
        // start interface
        if (can_do_start(iface.c_str()) < 0)
            throw std::runtime_error("Failed to start interface " + iface);
        // rename if necessary
        if (target != iface)
        {
            if (!renameInterface(iface, target))
                std::cerr << "[WARN] rename " << iface << "->" << target << " failed";
        }
    }
}

std::optional<std::string> getCanAdapterSerial(const std::string& port)
{
    std::array<char, 128> buf;
    FILE* pipe = popen((std::string("ethtool -i ") + port).c_str(), "r");
    if (!pipe)
        return std::nullopt;
    std::string usb;
    while (fgets(buf.data(), buf.size(), pipe))
    {
        std::string line(buf.data());
        if (line.find("bus-info") != std::string::npos)
        {
            usb = line.substr(line.find_last_of(' ') + 1);
            usb = usb.substr(0, usb.find(':'));
            break;
        }
    }
    pclose(pipe);
    if (usb.empty())
        return std::nullopt;
    std::ifstream f("/sys/bus/usb/devices/" + usb + "/serial");
    std::string serial;
    if (f && std::getline(f, serial))
        return serial;
    return std::nullopt;
}

std::optional<int> getInterfaceBitrate(const std::string& interface)
{
    try
    {
        std::array<char, 512> buffer;
        std::string command = "ip -details link show " + interface;
        FILE* pipe = popen(command.c_str(), "r");
        if (!pipe)
            return std::nullopt;

        std::string output;
        while (fgets(buffer.data(), buffer.size(), pipe))
            output += buffer.data();
        pclose(pipe);

        std::istringstream stream(output);
        std::string line;
        while (std::getline(stream, line))
        {
            auto pos = line.find("bitrate");
            if (pos != std::string::npos)
            {
                std::istringstream ls(line.substr(pos + 7));
                int br;
                ls >> br;
                return br;
            }
        }
    }
    catch (...)
    {
        return std::nullopt;
    }
    return std::nullopt;
}

} // namespace piper_cpp
