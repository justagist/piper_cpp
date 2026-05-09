#include "piper_cpp/interface/piper_interface_v2.h"
#include <chrono>
#include <cstdlib>
#include <iostream>
#include <string>
#include <thread>

// One-shot zero-reference setter for either the gripper or a single joint.
// Mirrors `piper_set_gripper_zero.py` and the simple-mode path of `piper_set_joint_zero.py`.
//
// IMPORTANT: position the target manually before running -- this latches whatever position the
// motor is currently at as the new zero reference. For joints, this also disables the chosen
// motor (so you can hand-position it); the script re-enables before exiting.
//
// Flags (one is required):
//   --gripper          Latch the current jaw position as the gripper's zero.
//   --joint <1..6>     Latch the current angle of joint N as that joint's zero.

int main(int argc, char* argv[])
{
    bool do_gripper = false;
    int joint_num = 0; // 1..6 if set

    for (int i = 1; i < argc; ++i)
    {
        std::string a = argv[i];
        if (a == "--gripper")
            do_gripper = true;
        else if (a == "--joint" && i + 1 < argc)
            joint_num = std::atoi(argv[++i]);
        else
        {
            std::cerr << "usage: piper_set_zero (--gripper | --joint <1..6>)\n";
            return 1;
        }
    }
    if (do_gripper == (joint_num != 0))
    {
        std::cerr << "Specify exactly one of --gripper or --joint <N>.\n";
        return 1;
    }
    if (joint_num != 0 && (joint_num < 1 || joint_num > 6))
    {
        std::cerr << "--joint must be in [1, 6]\n";
        return 1;
    }

    using namespace piper_cpp;
    PiperInterfaceV2 piper("can0");
    if (!piper.connectPort(/*can_init=*/true, /*piper_init=*/true, /*start_threads=*/true))
    {
        std::cerr << "Failed to open CAN port\n";
        return 1;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(200));

    if (do_gripper)
    {
        std::cout << "Setting gripper zero. Position the jaws now (typically fully closed).\n";
        // Mirrors piper_set_gripper_zero.py timing: Disable, sleep 1.5s, then Disable+SetZero.
        piper.controlGripper(0, 1000, GripperStatusCode::Disable);
        std::this_thread::sleep_for(std::chrono::milliseconds(1500));
        piper.setGripperZero();
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        std::cout << "Gripper zero latched. Verify via getGripperStates().value.foc_status.homing_status.\n";
    }
    else
    {
        const auto motor = static_cast<MotorIndex>(joint_num);
        std::cout << "Disabling joint " << joint_num << " so you can position it by hand.\n";
        piper.disableMotor(motor);
        std::cout << "Move joint " << joint_num << " to its desired zero position, then press Enter.\n";
        std::cin.get();
        std::cout << "Latching current position as joint " << joint_num << " zero...\n";
        piper.setJointAsCurrentZero(motor);
        std::this_thread::sleep_for(std::chrono::milliseconds(300));
        piper.enableMotor(motor);
        std::cout << "Joint " << joint_num << " zero set and motor re-enabled.\n";
    }

    piper.disconnectPort(std::chrono::milliseconds{200});
    return 0;
}
