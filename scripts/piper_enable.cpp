#include "piper_cpp/interface/piper_interface_v2.h"
#include <chrono>
#include <iostream>
#include <string>

int main(int argc, char* argv[])
{
    bool disable = false;
    if (argc > 1 && std::string(argv[1]) == "--disable")
    {
        disable = true;
    }

    // 1) Create interface on can0:
    piper_cpp::PiperInterfaceV2 piper("can0" /*, judge_flag, auto_init, dh_offset, ... */);

    // 2) Try to open port, start threads, send init queries:
    if (!piper.connectPort(
            /*can_init=*/true,
            /*piper_init=*/true,
            /*start_threads=*/true
        ))
    {
        std::cerr << "Failed to open CAN port\n";
        return 1;
    }

    if (disable)
    {
        std::cout << "Connected. Disabling robot...\n";
        piper.disableRobot();
        while (piper.isEnabled())
        {
            std::cout << "Waiting for robot to be disabled...\n";
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
        }
        std::cout << "Robot disabled.\n";
    }
    else
    {
        std::cout << "Connected. Enabling robot...\n";
        piper.enableRobot();
        while (!piper.isEnabled())
        {
            std::cout << "Waiting for robot to be enabled...\n";
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
        }
        std::cout << "Robot enabled.\n";
    }

    // 4) Clean up
    piper.disconnectPort(std::chrono::milliseconds{200});
    std::cout << "Interface closed cleanly" << std::endl;
    return 0;
}
