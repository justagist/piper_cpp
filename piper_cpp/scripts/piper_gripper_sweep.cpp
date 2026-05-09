#include "piper_cpp/interface/piper_interface_v2.h"
#include <chrono>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <string>
#include <thread>

// Mirror of the official Python `piper_ctrl_gripper.py` demo.
//
// Workflow:
//   1. Connect, start read thread.
//   2. Enable joint motors (does NOT enable the gripper position loop).
//   3. Optionally one-shot set-zero -- required at least once per power-up before the
//      gripper firmware will act on position commands. Skip with --no-zero if already done.
//      Close the jaws by hand before running unless you have a different reference in mind.
//   4. Sweep between fully-closed (0 mm) and `--max-mm` (default 50 mm) on a fixed period,
//      keeping `GripperStatusCode::Enable` on every cycle so the gripper doesn't auto-disable.
//
// Flags:
//   --no-zero        skip the set-zero step (if the gripper is already homed)
//   --max-mm <N>     half-amplitude of the sweep in mm (default 50)
//   --period-ms <N>  half-period of the sweep in ms (default 1500)
//   --duration-s <N> total run duration in seconds (default: run forever)
int main(int argc, char *argv[])
{
    bool do_zero = true;
    int32_t max_mm = 50;
    int period_ms = 1500;
    int duration_s = 0; // 0 = forever

    for (int i = 1; i < argc; ++i)
    {
        std::string a = argv[i];
        if (a == "--no-zero")
            do_zero = false;
        else if (a == "--max-mm" && i + 1 < argc)
            max_mm = std::atoi(argv[++i]);
        else if (a == "--period-ms" && i + 1 < argc)
            period_ms = std::atoi(argv[++i]);
        else if (a == "--duration-s" && i + 1 < argc)
            duration_s = std::atoi(argv[++i]);
        else
        {
            std::cerr << "usage: piper_gripper_sweep [--no-zero] [--max-mm N] [--period-ms N] [--duration-s N]\n";
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
    std::cout << "Connected.\n";

    std::cout << "Enabling joint motors...\n";
    piper.enableRobot();
    while (!piper.isEnabled())
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }
    std::cout << "Joint motors enabled.\n";

    if (do_zero)
    {
        std::cout << "Set-zero one-shot. Make sure jaws are closed (or at the desired zero reference)...\n";
        // Wash any latched error state first, then latch the current jaw position as zero.
        piper.controlGripper(0, 0, GripperStatusCode::DisableAndClearError);
        std::this_thread::sleep_for(std::chrono::milliseconds(300));
        piper.setGripperZero();
        std::this_thread::sleep_for(std::chrono::milliseconds(300));
        std::cout << "Zero set.\n";
    }

    const int32_t max_micrometers = max_mm * 1000; // 0.001 mm units
    const auto t_start = std::chrono::steady_clock::now();
    const auto period = std::chrono::milliseconds(period_ms);
    const auto loop_dt = std::chrono::milliseconds(20); // 50 Hz control loop

    std::cout << "Sweeping 0 -> " << max_mm << " mm with period " << period_ms << " ms (each direction).\n";
    while (true)
    {
        auto now = std::chrono::steady_clock::now();
        if (duration_s > 0 && now - t_start >= std::chrono::seconds(duration_s))
            break;

        // square-wave between 0 and max for now; replace with a triangle/sine if you prefer
        auto phase = ((now - t_start) / period) % 2;
        int32_t target = (phase == 0) ? 0 : max_micrometers;

        piper.controlGripper(target, 1000, GripperStatusCode::Enable);

        auto fb = piper.getGripperStates();
        if (fb.is_valid)
        {
            std::cout << "target=" << target << " us" << " | angle=" << fb.value.grippers_angle << " us ("
                      << fb.value.grippers_angle * 0.001 << " mm)" << " | effort=" << fb.value.grippers_effort
                      << " | enabled=" << fb.value.foc_status.driver_enable_status
                      << " | homed=" << fb.value.foc_status.homing_status << "\n";
        }

        std::this_thread::sleep_for(loop_dt);
    }

    // Disable gripper before disconnect so it doesn't continue holding position.
    piper.controlGripper(0, 0, GripperStatusCode::Disable);
    piper.disconnectPort(std::chrono::milliseconds{200});
    std::cout << "Done.\n";
    return 0;
}
