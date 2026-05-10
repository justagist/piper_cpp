#include "piper_cpp/interface/piper_interface_v2.h"
#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <string>
#include <thread>

// SIGINT/SIGTERM-safe stop flag. The gripper firmware may lock itself into a fault state if its
// command stream is cut while it's actively driving, requiring a power-cycle to recover, so we want to shut down
// cleanly on Ctrl+C by sending an explicit disable command before exiting.
static std::atomic<bool> g_stop_requested{false};
static void on_signal(int) { g_stop_requested.store(true); }

// Mirror of the official Python `piper_ctrl_gripper.py` demo.
//
// Workflow:
//   1. Connect, start read thread.
//   2. Enable joint motors (does NOT enable the gripper position loop).
//   3. Optionally one-shot set-zero -- required at least once per power-up before the
//      gripper firmware will act on position commands. Skip with --no-zero if already done.
//      Close the jaws by hand before running unless you have a different reference in mind.
//   4. Sweep between fully-closed (0 mm) and `--max-mm` (default 50 mm) on a fixed period,
//      sending `controlGripper(target, effort, Enable)` on every loop iteration.
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

    std::signal(SIGINT, on_signal);
    std::signal(SIGTERM, on_signal);

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
        std::cout << "Set-zero. Make sure jaws are closed (or at the desired zero reference)...\n";
        // Mirror the official `piper_set_gripper_zero.py` timing exactly: send a plain Disable,
        // wait 1.5 s for the gripper to settle, then send the SetZero one-shot.
        piper.controlGripper(0, 1000, GripperStatusCode::Disable);
        std::this_thread::sleep_for(std::chrono::milliseconds(1500));
        piper.setGripperZero();
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        std::cout << "Zero set.\n";
    }

    // Put the arm into CAN-command mode with joint-space movement. Without this, the firmware
    // silently ignores joint and gripper position commands even when motors are enabled.
    // The setting persists until power-cycle.
    piper.sendMotionCtrl2(CtrlMode::CanCmd, MoveMode::MoveJ, /*speed%=*/30, MitMode::PositionVelocity);
    std::cout << "Motion mode configured: CanCmd / MoveJ / 30%.\n";

    const int32_t max_micrometers = max_mm * 1000; // 0.001 mm units
    const auto t_start = std::chrono::steady_clock::now();
    const auto period = std::chrono::milliseconds(period_ms);
    const auto loop_dt = std::chrono::milliseconds(20); // 50 Hz control loop

    std::cout << "Sweeping 0 -> " << max_mm << " mm with period " << period_ms << " ms (each direction).\n";
    while (true)
    {
        if (g_stop_requested.load())
        {
            std::cout << "\nStop requested, shutting down gripper cleanly...\n";
            break;
        }
        auto now = std::chrono::steady_clock::now();
        if (duration_s > 0 && now - t_start >= std::chrono::seconds(duration_s))
            break;

        auto phase = ((now - t_start) / period) % 2;
        int32_t target = (phase == 0) ? 0 : max_micrometers;

        piper.controlGripper(target, 1000, GripperStatusCode::Enable);

        auto fb = piper.getGripperStates();
        if (fb.is_valid)
        {
            std::cout << "target=" << target << " um" << " | angle=" << fb.value.grippers_angle << " um ("
                      << fb.value.grippers_angle * 0.001 << " mm)" << " | effort=" << fb.value.grippers_effort
                      << " | enabled=" << fb.value.foc_status.driver_enable_status
                      << " | homed=" << fb.value.foc_status.homing_status << "\n";
        }

        std::this_thread::sleep_for(loop_dt);
    }

    for (int i = 0; i < 5; ++i)
    {
        piper.controlGripper(0, 0, GripperStatusCode::Disable);
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    // piper.disableRobot();
    piper.disconnectPort(std::chrono::milliseconds{200});
    std::cout << "Done.\n";
    return 0;
}
