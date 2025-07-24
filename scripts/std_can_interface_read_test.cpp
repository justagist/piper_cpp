#include "piper_cpp/std_can_interface.h"
#include <chrono>
#include <csignal>
#include <iostream>
#include <thread>

using namespace piper_cpp;

bool stop = false;

void signalHandler(int) { stop = true; }

void exampleReceiveCallback(const struct can_frame& frame)
{
    std::cout << "[Callback] Received CAN frame:" << " ID=0x" << std::hex << frame.can_id << " DLC=" << std::dec
              << (int)frame.can_dlc << " Data=";
    for (int i = 0; i < frame.can_dlc; ++i)
    {
        std::printf("%02X ", frame.data[i]);
    }
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
