#include "piper_cpp/protocol/piper_parser_v2.h"
#include "piper_cpp/std_can_interface.h"
#include <chrono>
#include <csignal>
#include <iomanip>
#include <iostream>
#include <thread>

using namespace piper_cpp;

bool stop = false;

void signalHandler(int) { stop = true; }

int main(int argc, char* argv[])
{
    std::signal(SIGINT, signalHandler); // Handle Ctrl+C

    std::cout << "Initializing CAN interface..." << std::endl;

    auto parser = std::make_unique<PiperParserV2>();
    if (!parser)
    {
        std::cerr << "[ERROR] Failed to create PiperParserV2 instance." << std::endl;
        return 1;
    }

    try
    {
        std::cout << "Available CAN ports:" << std::endl;
        for (const auto& port : StdCanInterface::getCanPorts())
        {
            std::cout << " - " << port << std::endl;
            std::cout << StdCanInterface::getCanPortInfo(port) << std::endl;
        }

        PiperMessage msg;
        StdCanInterface can(
            "can0", 1000000, true, true,
            [&parser, &msg](const struct can_frame& frame, double timestamp)
            {
                if (parser->decodeMessage(frame, timestamp, msg))
                {
                    std::cout << "[Callback] Received CAN frame: ID=0x" << std::hex << msg.can_id
                              << " Type=" << piper_cpp::toString(msg.type) << " Data=";
                    for (int i = 0; i < msg.raw_data_len; ++i)
                    {
                        std::printf("%02X ", msg.raw_data[i]);
                    }
                    std::cout << std::dec << " DLC=" << msg.raw_data_len;
                    std::cout << std::fixed << std::setprecision(6) << " Timestamp=" << msg.timestamp << "s";
                    std::cout << std::endl;
                    std::cout << msg.toString() << std::endl;
                }
                else
                {
                    std::cerr << "[Callback] Failed to decode CAN frame with ID=0x" << std::hex << frame.can_id
                              << std::dec << " at timestamp " << timestamp << "s" << std::endl;
                }
                /// Uncomment to see default behavior to compare
                // std::cout << "--- default ---" << std::endl;
                // std::cout << "[Callback] Received CAN frame:" << " ID=0x" << std::hex << frame.can_id
                //           << " DLC=" << std::dec << (int)frame.can_dlc << " Data=";
                // for (int i = 0; i < frame.can_dlc; ++i)
                // {
                //     std::printf("%02X ", frame.data[i]);
                // }
                // std::cout << std::fixed << std::setprecision(6) << " Timestamp=" << timestamp << "s";
                // std::cout << "------" <<  std::endl;
            }
        );
        std::cout << "CAN initialized on: can0" << std::endl;

        std::cout << "Listening for CAN messages (Ctrl+C to exit)..." << std::endl;
        while (!stop)
        {
            can.readCanMessage();
            // std::this_thread::sleep_for(std::chrono::milliseconds(100));
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
