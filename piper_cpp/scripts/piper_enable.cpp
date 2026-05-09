#include "piper_cpp/interface/piper_interface_v2.h"
#include <chrono>
#include <iostream>
#include <string>

// Enable or disable all six joint motors plus the gripper motor (broadcast 0x471). Polls
// `isEnabled()` until the requested state is reflected in the per-motor low-speed feedback,
// which usually takes a few hundred milliseconds.
//
// IMPORTANT: `--disable` cuts torque to all motors, so the arm will fall under gravity unless
// physically supported. Catch the wrist or rest the arm before disabling.
//
// Usage:
//   piper_enable               # enable all joints + gripper motor
//   piper_enable --disable     # disable all joints + gripper motor
//
// Note: this controls the joint *motors* only; the gripper has a separate position-control
// enable inside the 0x159 command frame -- see `piper_gripper_sweep` for the full story.
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
