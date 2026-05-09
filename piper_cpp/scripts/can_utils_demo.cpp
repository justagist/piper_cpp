#include "piper_cpp/can_utils.h"
#include <iostream>
int main(int argc, char* argv[])
{
    if (argc < 2)
    {
        std::cerr << "Usage: " << argv[0] << " {--find|--active|--activate|--get-serial} [timeout/interface]\n";
        return 1;
    }
    std::string cmd = argv[1];
    if (cmd == "--find")
    {
        for (auto& p : piper_cpp::findPorts())
            std::cout << p.first << " " << p.second << "\n";
    }
    else if (cmd == "--active")
    {
        for (auto& iface : piper_cpp::activePorts())
            std::cout << iface << "\n";
    }
    else if (cmd == "--activate")
    {
        int timeout = (argc > 2) ? std::stoi(argv[2]) : 0;
        piper_cpp::activate({}, "can", 1000000, timeout > 0 ? std::optional<int>(timeout) : std::nullopt);
    }
    else if (cmd == "--get-serial")
    {
        if (argc < 3)
        {
            std::cerr << "Need interface name\n";
            return 1;
        }
        auto s = piper_cpp::getCanAdapterSerial(argv[2]);
        if (s)
            std::cout << *s << "\n";
    }
    else
    {
        std::cerr << "Unknown command\n";
        return 1;
    }
    return 0;
}
