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

// Make sure the arm is roughly near zeros before starting the demo.
//
// Continuous real-time joint streaming demo. Drives all six joints with a small-amplitude
// sinusoid and pushes a fresh `controlJoints()` setpoint at a high fixed rate (default 200 Hz)
// -- the same shape of command stream you would emit for continuous real-time controller (e.g.
// ros control or teleoperation input).
//
// The sine is anchored at the current pose: the starting pose is the trough (or peak) of the
// wave, never the centre, so each joint only moves in the direction that is safe to traverse
// from its present position. The per-joint sign pattern (`[+, +, -, +, -, +]`) matches the
// two-pose `piper_joint_ctrl_demo`.
//
// Stays in standard PositionVelocity mode (no MIT). The arm's firmware-side position
// controller tracks the streamed target; the only thing this script does differently from the
// two-pose `piper_joint_ctrl_demo` is to send a smooth target every loop tick.
//
// SIGINT-safe: leaves the joints enabled holding their last commanded pose.
//
// Flags:
//   --speed <0..100>     speed cap percentage forwarded to MoveJ (default 30)
//   --rate-hz <N>        command stream rate in Hz (default 200)
//   --amp-rad <R>        sinusoid amplitude per joint in radians (default 0.05 ~= 2.9 deg)
//   --freq-hz <F>        sinusoid frequency in Hz (default 0.25 -> 4 s period)
//   --duration-s <N>     total run duration (default: forever)

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
    int rate_hz = 200;
    double amp_rad = 0.05;
    double freq_hz = 0.25;
    int duration_s = 0;

    for (int i = 1; i < argc; ++i)
    {
        std::string a = argv[i];
        if (a == "--speed" && i + 1 < argc)
            speed_pct = std::atoi(argv[++i]);
        else if (a == "--rate-hz" && i + 1 < argc)
            rate_hz = std::atoi(argv[++i]);
        else if (a == "--amp-rad" && i + 1 < argc)
            amp_rad = std::atof(argv[++i]);
        else if (a == "--freq-hz" && i + 1 < argc)
            freq_hz = std::atof(argv[++i]);
        else if (a == "--duration-s" && i + 1 < argc)
            duration_s = std::atoi(argv[++i]);
        else
        {
            std::cerr << "usage: piper_joint_sine_demo [--speed 0..100] [--rate-hz N] "
                         "[--amp-rad R] [--freq-hz F] [--duration-s N]\n";
            return 1;
        }
    }

    if (speed_pct < 0)
        speed_pct = 0;
    if (speed_pct > 100)
        speed_pct = 100;
    if (rate_hz < 10)
        rate_hz = 10;
    if (rate_hz > 500)
        rate_hz = 500;

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

    // Drive all joints to zero before starting the sine cycle, so the wave is anchored at a
    // known, predictable pose. Stream zero targets at 50 Hz for a few seconds, same pattern as
    // `piper_go_zero`.
    {
        std::cout << "Driving all joints to zero before starting the cycle...\n";
        const auto zero_dt = std::chrono::milliseconds(20);
        const auto zero_duration = std::chrono::seconds(3);
        const auto zero_t0 = std::chrono::steady_clock::now();
        while (std::chrono::steady_clock::now() - zero_t0 < zero_duration)
        {
            if (g_stop_requested.load())
            {
                piper.disconnectPort(std::chrono::milliseconds{200});
                return 0;
            }
            piper.controlJoints(0, 0, 0, 0, 0, 0);
            std::this_thread::sleep_for(zero_dt);
        }
    }

    // Sample the live starting pose. This is what the sine wave is anchored to -- each joint
    // will oscillate from this pose toward its safe direction and back, never crossing through
    // the start pose into the opposite direction.
    StateSnapshot<ArmMsgJointValues> js0 = piper.getArmJointStates();
    {
        auto wait_start = std::chrono::steady_clock::now();
        while (!js0.is_valid)
        {
            if (g_stop_requested.load())
            {
                piper.disconnectPort(std::chrono::milliseconds{200});
                return 0;
            }
            if (std::chrono::steady_clock::now() - wait_start > std::chrono::seconds(2))
            {
                std::cerr << "Timed out waiting for joint feedback before sampling start pose.\n";
                piper.disconnectPort(std::chrono::milliseconds{200});
                return 1;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
            js0 = piper.getArmJointStates();
        }
    }
    const int32_t start_md[6] = {
        js0.value.joint_1, js0.value.joint_2, js0.value.joint_3,
        js0.value.joint_4, js0.value.joint_5, js0.value.joint_6,
    };
    // Per-joint safe-direction signs, matching `piper_joint_ctrl_demo`.
    const int signs[6] = {+1, +1, -1, +1, -1, +1};

    std::cout << "Streaming sinusoidal joint targets at " << rate_hz << " Hz "
              << "(amp=" << amp_rad << " rad, freq=" << freq_hz << " Hz, "
              << "anchored on current pose). Ctrl-C to stop.\n";
    std::cout << "Start pose (md): [" << start_md[0] << ", " << start_md[1] << ", " << start_md[2] << ", "
              << start_md[3] << ", " << start_md[4] << ", " << start_md[5] << "]\n";

    const int32_t amp_md = radToMilliDeg(amp_rad);

    const auto t_start = std::chrono::steady_clock::now();
    const auto loop_dt = std::chrono::microseconds(1'000'000 / rate_hz);
    auto next_tick = t_start;

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

        const double t_s = std::chrono::duration<double>(now - t_start).count();
        const double w = 2.0 * M_PI * freq_hz;
        // (1 - cos)/2 maps to [0, 1] starting at 0. Same fundamental frequency as sin().
        const double half = 0.5 * (1.0 - std::cos(w * t_s));
        const int32_t excursion = static_cast<int32_t>(std::lround(amp_md * half));

        int32_t targets[6];
        for (int j = 0; j < 6; ++j)
            targets[j] = start_md[j] + signs[j] * excursion;

        piper.controlJoints(targets[0], targets[1], targets[2], targets[3], targets[4], targets[5]);

        auto js = piper.getArmJointStates();
        if (js.is_valid)
        {
            std::cout << std::fixed << std::setprecision(0) << "t=" << std::setprecision(2) << t_s << "s"
                      << " | target=[" << targets[0] << ", " << targets[1] << ", " << targets[2] << ", " << targets[3]
                      << ", " << targets[4] << ", " << targets[5] << "]"
                      << " | actual=[" << js.value.joint_1 << ", " << js.value.joint_2 << ", " << js.value.joint_3
                      << ", " << js.value.joint_4 << ", " << js.value.joint_5 << ", " << js.value.joint_6 << "] md\r"
                      << std::flush;
        }

        next_tick += loop_dt;
        std::this_thread::sleep_until(next_tick);
    }
    std::cout << "\n";

    piper.disconnectPort(std::chrono::milliseconds{200});
    std::cout << "Done. Joints left enabled and holding last commanded pose.\n";
    return 0;
}
