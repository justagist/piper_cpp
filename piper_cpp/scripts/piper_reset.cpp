#include "piper_cpp/interface/piper_interface_v2.h"
#include <chrono>
#include <cstdlib>
#include <iostream>
#include <string>
#include <thread>

// One-shot reset for the Piper arm. Sends `0x150` with `EmergencyStop::Resume` (0x02), which
// the firmware treats as a "reset arm" command: it drops power to all joints, clears every
// latched error code, and resets internal state flags. Mirror of `piper_ctrl_reset.py`.
//
// IMPORTANT: this drops power immediately. The arm WILL fall under gravity unless physically
// supported. Catch the wrist or rest the arm before running.
//
// Common use cases:
//   - The arm has latched a fault that won't clear via `clearJointError()`.
//   - The arm is in a stuck state from a previous control session (MIT mode left active,
//     drag-teach mode left active, etc.) and you want a clean slate.
//   - You want to use this in scripts as a software-equivalent of a power cycle for the
//     arm-controller's runtime state (firmware-stored config like master/slave role and
//     installation pos is preserved -- only runtime state is cleared).
//
// After reset you'll need to call `enableRobot()` and re-establish motion mode before
// commanding motion again.
//
// Usage:
//   piper_reset            # confirms first; pass --force to skip the prompt
//   piper_reset --force    # send the reset immediately, no confirmation

int main(int argc, char* argv[])
{
    bool force = false;
    for (int i = 1; i < argc; ++i)
    {
        std::string a = argv[i];
        if (a == "--force")
            force = true;
        else
        {
            std::cerr << "usage: piper_reset [--force]\n";
            return 1;
        }
    }

    if (!force)
    {
        std::cout << "WARNING: piper_reset will drop power to ALL joints. The arm will fall.\n"
                     "Make sure the arm is supported, then type 'reset' to continue, anything else to cancel.\n"
                     "> " << std::flush;
        std::string input;
        std::getline(std::cin, input);
        if (input != "reset")
        {
            std::cout << "Cancelled.\n";
            return 0;
        }
    }

    using namespace piper_cpp;
    PiperInterfaceV2 piper("can0");
    if (!piper.connectPort(/*can_init=*/true, /*piper_init=*/false, /*start_threads=*/false))
    {
        std::cerr << "Failed to open CAN port\n";
        return 1;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    if (!piper.resetPiper())
    {
        std::cerr << "Failed to send reset command.\n";
        piper.disconnectPort(std::chrono::milliseconds{200});
        return 1;
    }

    std::cout << "Reset sent. Joints are now unpowered and all error/state flags cleared.\n"
                 "Run `piper_enable` and your motion-mode setup before commanding motion again.\n";

    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    piper.disconnectPort(std::chrono::milliseconds{200});
    return 0;
}
