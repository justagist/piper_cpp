#include "piper_cpp/interface/piper_interface_v2.h"
#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <string>
#include <thread>

// Mirror of `piper_ctrl_end_pose.py`: cycles the end-effector between two cartesian poses
// (default: a low pose and a higher pose, swept along Z) using MoveP. Pair this with the
// `setMoveMode(MoveP)` call -- the arm uses its cartesian interpolator.
//
// SIGINT-safe shutdown: leaves the arm at the last commanded pose without disabling joints
// (disabling would drop the arm). Run `piper_enable --disable` afterwards if you need to
// fully release the joints.
//
// Flags:
//   --speed <0..100>   speed cap percentage (default 50)
//   --period-ms <N>    half-period of the cartesian sweep in ms (default 2000)
//   --duration-s <N>   total run duration in seconds (default: forever)

static std::atomic<bool> g_stop_requested{false};
static void on_signal(int) { g_stop_requested.store(true); }

int main(int argc, char* argv[])
{
    int speed_pct = 50;
    int period_ms = 2000;
    int duration_s = 0;

    for (int i = 1; i < argc; ++i)
    {
        std::string a = argv[i];
        if (a == "--speed" && i + 1 < argc)
            speed_pct = std::atoi(argv[++i]);
        else if (a == "--period-ms" && i + 1 < argc)
            period_ms = std::atoi(argv[++i]);
        else if (a == "--duration-s" && i + 1 < argc)
            duration_s = std::atoi(argv[++i]);
        else
        {
            std::cerr << "usage: piper_end_pose_demo [--speed 0..100] [--period-ms N] [--duration-s N]\n";
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
    std::cout << "Connected. Enabling joints...\n";
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
    std::cout << "Joints enabled. Switching to MoveP at " << speed_pct << "%.\n";
    piper.sendMotionCtrl2(CtrlMode::CanCmd, MoveMode::MoveP, static_cast<uint8_t>(speed_pct), MitMode::PositionVelocity);

    // Two cartesian targets. Units: 0.001 mm for X/Y/Z, 0.001 deg for RX/RY/RZ.
    // Defaults match piper_ctrl_end_pose.py: same XY/RY, swept Z between 215 mm and 260 mm.
    const int32_t pose_low[6]  = {57000,  0,    215000, 0, 85000, 0};
    const int32_t pose_high[6] = {57000,  0,    260000, 0, 85000, 0};

    const auto t_start = std::chrono::steady_clock::now();
    const auto period = std::chrono::milliseconds(period_ms);
    const auto loop_dt = std::chrono::milliseconds(20); // 50 Hz

    std::cout << "Sweeping end-pose Z between 215 mm and 260 mm with " << period_ms << " ms half-period.\n";
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
        const auto& target = (phase == 0) ? pose_low : pose_high;
        piper.controlEndPose(target[0], target[1], target[2], target[3], target[4], target[5]);

        auto pose = piper.getArmEndPose();
        if (pose.is_valid)
        {
            std::cout << std::fixed << std::setprecision(0) << "target=[" << target[0] << ", " << target[1] << ", "
                      << target[2] << ", " << target[3] << ", " << target[4] << ", " << target[5] << "] um/mdeg"
                      << " | actual_xyz=[" << pose.value.X_axis << ", " << pose.value.Y_axis << ", " << pose.value.Z_axis
                      << "]\n";
        }
        std::this_thread::sleep_for(loop_dt);
    }

    piper.disconnectPort(std::chrono::milliseconds{200});
    std::cout << "Done. Joints left enabled and holding pose.\n";
    return 0;
}
