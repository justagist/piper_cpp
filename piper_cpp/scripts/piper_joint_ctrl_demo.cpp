#include "piper_cpp/interface/piper_interface_v2.h"
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <string>
#include <thread>

// Mirror of the official Python `piper_ctrl_joint.py` demo.
//
// Workflow:
//   1. Connect, start the read thread.
//   2. Enable joint motors.
//   3. Put the arm into CAN-command mode + MoveJ at the requested speed cap.
//   4. Sweep all six joints between two poses (home and a small offset) on a fixed period,
//      sending JointCtrl frames at 50 Hz.
//
// The arm holds the last commanded position when the loop ends -- joints are NOT disabled on
// exit (that would drop the arm). Catch SIGINT cleanly so we don't leave the firmware in the
// middle of a partially-sent command frame.
//
// Flags:
//   --speed <0..100>  speed cap percentage (default 30)
//   --offset-rad <r>  per-joint offset for the alternate pose (default 0.1 rad ~= 5.7 deg)
//   --period-ms <N>   half-period of the sweep, ms (default 3000)
//   --duration-s <N>  total run duration in seconds (default: forever)

static std::atomic<bool> g_stop_requested{false};
static void on_signal(int) { g_stop_requested.store(true); }

static int32_t radToMilliDeg(double rad)
{
    constexpr double k = 1000.0 * 180.0 / M_PI;
    return static_cast<int32_t>(std::lround(rad * k));
}

int main(int argc, char* argv[])
{
    int speed_pct = 30;
    double offset_rad = 0.1;
    int period_ms = 3000;
    int duration_s = 0;

    for (int i = 1; i < argc; ++i)
    {
        std::string a = argv[i];
        if (a == "--speed" && i + 1 < argc)
            speed_pct = std::atoi(argv[++i]);
        else if (a == "--offset-rad" && i + 1 < argc)
            offset_rad = std::atof(argv[++i]);
        else if (a == "--period-ms" && i + 1 < argc)
            period_ms = std::atoi(argv[++i]);
        else if (a == "--duration-s" && i + 1 < argc)
            duration_s = std::atoi(argv[++i]);
        else
        {
            std::cerr << "usage: piper_joint_ctrl_demo [--speed 0..100] [--offset-rad R] "
                         "[--period-ms N] [--duration-s N]\n";
            return 1;
        }
    }

    if (speed_pct < 0)
        speed_pct = 0;
    if (speed_pct > 100)
        speed_pct = 100;

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
        if (g_stop_requested.load())
        {
            piper.disconnectPort(std::chrono::milliseconds{200});
            return 0;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }
    std::cout << "Joint motors enabled.\n";

    piper.sendMotionCtrl2(
        CtrlMode::CanCmd, MoveMode::MoveJ, static_cast<uint8_t>(speed_pct), MitMode::PositionVelocity
    );
    std::cout << "Motion mode: CanCmd / MoveJ / " << speed_pct << "%.\n";

    // Two-pose sweep: home (all zeros) and a small offset on every joint.
    const int32_t home_md = 0;
    const int32_t off_md = radToMilliDeg(offset_rad);
    std::cout << "Sweeping home <-> [" << offset_rad << " rad on each joint] (= " << off_md
              << " millideg) every " << period_ms << " ms.\n";

    const auto t_start = std::chrono::steady_clock::now();
    const auto period = std::chrono::milliseconds(period_ms);
    const auto loop_dt = std::chrono::milliseconds(20); // 50 Hz

    while (true)
    {
        if (g_stop_requested.load())
        {
            std::cout << "\nStop requested, exiting cleanly...\n";
            break;
        }
        auto now = std::chrono::steady_clock::now();
        if (duration_s > 0 && now - t_start >= std::chrono::seconds(duration_s))
            break;

        auto phase = ((now - t_start) / period) % 2;
        int32_t target = (phase == 0) ? home_md : off_md;

        // Sign-flip joint 3 so the elbow goes the "natural" way; matches the Python demo's
        // [0.2, 0.2, -0.2, 0.3, -0.2, 0.5] sign pattern in spirit.
        piper.controlJoints(target, target, -target, target, -target, target);

        auto js = piper.getArmJointStates();
        if (js.is_valid)
        {
            std::cout << std::fixed << std::setprecision(0) << "target=" << target << " md"
                      << " | actual=[" << js.value.joint_1 << ", " << js.value.joint_2 << ", " << js.value.joint_3
                      << ", " << js.value.joint_4 << ", " << js.value.joint_5 << ", " << js.value.joint_6 << "] md\n";
        }

        std::this_thread::sleep_for(loop_dt);
    }

    // Don't disable the joints on exit -- that would drop the arm. The arm firmware will hold
    // the last commanded pose against gravity. Just stop the command stream and disconnect.
    piper.disconnectPort(std::chrono::milliseconds{200});
    std::cout << "Done. Joints left enabled and holding pose.\n";
    return 0;
}
