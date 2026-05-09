#include "piper_cpp/can_utils.h"
#include <iostream>

// CAN-bus utilities demo / multi-tool. Wraps the four helpers in `piper_cpp::can_utils`:
//
//   --find                List all CAN-capable USB adapters detected on the host (vendor-id +
//                         serial-number pairs). Useful for confirming the arm's adapter is
//                         present before trying to bring it up.
//
//   --active              List all CAN interfaces that are currently UP and ready to talk.
//
//   --activate [timeout]  Bring up every detected CAN port at 1 Mbit/s. Optional integer
//                         `timeout` argument is passed to the underlying helper (seconds) --
//                         leave it off for the default. Equivalent to running `can_activate`.
//
//   --get-serial <iface>  Print the USB serial number of the adapter behind a given CAN
//                         interface (e.g. `can0`). Useful for matching specific interfaces to
//                         specific physical adapters in multi-arm setups.
//
// Usage examples:
//   can_utils_demo --find
//   can_utils_demo --active
//   can_utils_demo --activate
//   can_utils_demo --activate 5
//   can_utils_demo --get-serial can0
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
