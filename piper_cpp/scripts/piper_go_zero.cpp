#include "piper_cpp/interface/piper_interface_v2.h"
#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdlib>
#include <iostream>
#include <string>
#include <thread>

// Mirror of `piper_ctrl_go_zero.py`: drives all six joints to zero (and the gripper to closed)
// using MoveJ. Sends targets in a loop until stopped (Ctrl-C) or for a fixed duration.
//
// SIGINT-safe shutdown: stops the command stream and disconnects without disabling joints.
//
// Flags:
//   --speed <0..100>   speed cap percentage (default 30)
//   --duration-s <N>   total run duration in seconds (default 10; pass 0 to run forever)

static std::atomic<bool> g_stop_requested{false};
static void on_signal(int) { g_stop_requested.store(true); }

int main(int argc, char* argv[])
{
    int speed_pct = 30;
    int duration_s = 10;

    for (int i = 1; i < argc; ++i)
    {
        std::string a = argv[i];
        if (a == "--speed" && i + 1 < argc)
            speed_pct = std::atoi(argv[++i]);
        else if (a == "--duration-s" && i + 1 < argc)
            duration_s = std::atoi(argv[++i]);
        else
        {
            std::cerr << "usage: piper_go_zero [--speed 0..100] [--duration-s N]\n";
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
    piper.sendMotionCtrl2(
        CtrlMode::CanCmd, MoveMode::MoveJ, static_cast<uint8_t>(speed_pct), MitMode::PositionVelocity
    );
    std::cout << "Driving all joints + gripper to zero at " << speed_pct << "%.\n";

    const auto t_start = std::chrono::steady_clock::now();
    const auto loop_dt = std::chrono::milliseconds(20); // 50 Hz

    while (true)
    {
        if (g_stop_requested.load())
        {
            std::cout << "\nStop requested, exiting...\n";
            break;
        }
        if (duration_s > 0 && std::chrono::steady_clock::now() - t_start >= std::chrono::seconds(duration_s))
            break;

        piper.controlJoints(0, 0, 0, 0, 0, 0);
        piper.controlGripper(0, 1000, GripperStatusCode::Enable);

        auto js = piper.getArmJointStates();
        if (js.is_valid)
        {
            std::cout << "actual=[" << js.value.joint_1 << ", " << js.value.joint_2 << ", " << js.value.joint_3 << ", "
                      << js.value.joint_4 << ", " << js.value.joint_5 << ", " << js.value.joint_6 << "] mdeg\r"
                      << std::flush;
        }
        std::this_thread::sleep_for(loop_dt);
    }
    std::cout << "\n";

    // Send a few Disable frames to the gripper before disconnecting (mirror of the gripper-sweep
    // shutdown pattern -- prevents the gripper firmware from latching a fault).
    for (int i = 0; i < 5; ++i)
    {
        piper.controlGripper(0, 0, GripperStatusCode::Disable);
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    piper.disconnectPort(std::chrono::milliseconds{200});
    std::cout << "Done. Joints left enabled and holding zero pose.\n";
    return 0;
}
