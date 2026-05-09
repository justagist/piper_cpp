#include "piper_cpp/interface/piper_interface_v2.h"
#include <cctype>
#include <chrono>
#include <cstdlib>
#include <iostream>
#include <string>
#include <thread>

// Mirror of `piper_read_firmware.py` / `piper_read_version.py`: connect to the arm, wait
// briefly for the firmware-info bytes to stream in, then print the parsed version tag.

int main(int argc, char* argv[])
{
    int wait_ms = 1000;
    bool show_raw = false;

    for (int i = 1; i < argc; ++i)
    {
        std::string a = argv[i];
        if (a == "--wait-ms" && i + 1 < argc)
            wait_ms = std::atoi(argv[++i]);
        else if (a == "--raw")
            show_raw = true;
        else
        {
            std::cerr << "usage: piper_read_firmware [--wait-ms N] [--raw]\n";
            return 1;
        }
    }

    using namespace piper_cpp;
    PiperInterfaceV2 piper("can0");
    if (!piper.connectPort(/*can_init=*/true, /*piper_init=*/true, /*start_threads=*/true))
    {
        std::cerr << "Failed to open CAN port\n";
        return 1;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(wait_ms));

    auto version = piper.getFirmwareVersion();
    if (version)
    {
        std::cout << "Arm firmware version: " << *version << "\n";
    }
    else
    {
        std::cout << "Firmware version not received yet -- try increasing --wait-ms.\n";
    }

    if (show_raw)
    {
        const auto data = piper.getFirmwareData();
        std::cout << "Raw firmware-info buffer (" << data.size() << " bytes): ";
        for (uint8_t b : data)
            std::cout << (std::isprint(b) ? char(b) : '.');
        std::cout << "\n";
    }

    piper.disconnectPort(std::chrono::milliseconds{200});
    return version.has_value() ? 0 : 2;
}
