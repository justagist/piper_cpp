#include "piper_cpp/std_can_interface.h"
#include <chrono>
#include <csignal>
#include <iomanip>
#include <iostream>
#include <thread>

// Lowest-level read smoke test. Doesn't use any Piper protocol decoding -- just opens
// `can0` via `StdCanInterface` and prints every raw CAN frame as hex. Equivalent to
// `candump can0`, but goes through the same SocketCAN wrapper the rest of the SDK uses, so
// it's useful for confirming that the wrapper itself works on a given system.
//
// Use this if you suspect the SocketCAN wrapper is broken (e.g. frames visible to candump
// but not to your program). For protocol-level diagnostics use `piper_parser_v2_read_test`;
// for the full SDK use `piper_interface_v2_read_test`.
//
// Usage:
//   piper_cpp_std_can_interface_read
//
// Ctrl+C exits cleanly. No flags. The arm (or any CAN traffic source) must be on `can0`.

using namespace piper_cpp;

bool stop = false;

void signalHandler(int) { stop = true; }

void exampleReceiveCallback(const struct can_frame& frame, double timestamp)
{
    std::cout << "[Callback] Received CAN frame:" << " ID=0x" << std::hex << frame.can_id << " DLC=" << std::dec
              << (int)frame.can_dlc << " Data=";
    for (int i = 0; i < frame.can_dlc; ++i)
    {
        std::printf("%02X ", frame.data[i]);
    }
    std::cout << std::fixed << std::setprecision(6) << " Timestamp=" << timestamp << "s";
    std::cout << std::endl;
}

int main(int argc, char* argv[])
{
    std::signal(SIGINT, signalHandler); // Handle Ctrl+C

    std::cout << "Initializing CAN interface..." << std::endl;

    try
    {
        std::cout << "Available CAN ports:" << std::endl;
        for (const auto& port : StdCanInterface::getCanPorts())
        {
            std::cout << " - " << port << std::endl;
            std::cout << StdCanInterface::getCanPortInfo(port) << std::endl;
        }

        StdCanInterface can("can0", 1000000, true, true, exampleReceiveCallback);
        std::cout << "CAN initialized on: can0" << std::endl;

        std::cout << "Listening for CAN messages (Ctrl+C to exit)..." << std::endl;
        while (!stop)
        {
            can.readCanMessage();
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }

        std::cout << "\nShutting down CAN..." << std::endl;
        can.close();
    }
    catch (const std::exception& e)
    {
        std::cerr << "[ERROR] " << e.what() << std::endl;
        return 1;
    }

    std::cout << "Exited cleanly.\n";
    return 0;
}
