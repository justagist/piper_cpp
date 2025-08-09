#include "piper_cpp/interface/piper_interface_v2.h"
#include "piper_cpp/types/types.h" // for the ArmMsg* structs
#include <chrono>
#include <iomanip>
#include <iostream>
#include <thread>

int main()
{
    // 1) Create interface on can0:
    piper_cpp::PiperInterfaceV2 piper("can0" /*, judge_flag, auto_init, dh_offset, ... */);

    // 2) Try to open port, start threads, send init queries:
    if (!piper.connectPort(
            /*can_init=*/false,
            /*piper_init=*/true,
            /*start_threads=*/true
        ))
    {
        std::cerr << "Failed to open CAN port\n";
        return 1;
    }

    std::cout << "Connected. Reading states for 10 seconds...\n";

    // 3) Run a 10 s loop, polling every 500 ms:
    for (int i = 0; i < 20; ++i)
    {
        // a) Arm status
        auto statusSnap = piper.getArmStatus();
        std::cout << std::fixed << std::setprecision(6) << "[Status]   t=" << statusSnap.timestamp
                  << "  hz=" << statusSnap.hz << "  ctrl_mode=" << int(statusSnap.value.ctrl_mode)
                  << "  err_code=" << statusSnap.value.err_code << "\n";

        // b) End‐pose
        auto poseSnap = piper.getArmEndPose();
        std::cout << std::fixed << std::setprecision(6) << "[EndPose]  t=" << poseSnap.timestamp
                  << "  hz=" << poseSnap.hz << "  X=" << poseSnap.value.X_axis << "  Y=" << poseSnap.value.Y_axis
                  << "  Z=" << poseSnap.value.Z_axis << "  RX=" << poseSnap.value.RX_axis
                  << "  RY=" << poseSnap.value.RY_axis << "  RZ=" << poseSnap.value.RZ_axis << "\n";

        // c) Joint angles
        auto jointsSnap = piper.getArmJointStates();
        std::cout << std::fixed << std::setprecision(6) << "[Joints]   t=" << jointsSnap.timestamp
                  << "  hz=" << jointsSnap.hz << "  j1=" << jointsSnap.value.joint_1
                  << "  j2=" << jointsSnap.value.joint_2 << "  j3=" << jointsSnap.value.joint_3
                  << "  j4=" << jointsSnap.value.joint_4 << "  j5=" << jointsSnap.value.joint_5
                  << "  j6=" << jointsSnap.value.joint_6 << "\n";

        // d) gripper states
        auto gripperSnap = piper.getArmGripperStates();
        std::cout << std::fixed << std::setprecision(6) << "[Gripper]  t=" << gripperSnap.timestamp
                  << "  hz=" << gripperSnap.hz << "  angle=" << gripperSnap.value.grippers_angle
                  << " effort=" << gripperSnap.value.grippers_effort
                  << " foc:" << gripperSnap.value.foc_status.toString() << "\n";

        // e) high spd feedback
        auto high_spd_fb = piper.getArmHighSpeedFeedbacks();
        std::cout << std::fixed << std::setprecision(6) << "[HighSpd]  t=" << high_spd_fb.timestamp
                  << "  hz=" << high_spd_fb.hz;
        for (size_t i = 0; i < high_spd_fb.value.size(); ++i)
        {
            std::cout << "  j" << (i + 1) << "=" << high_spd_fb.value[i].toString();
        }
        std::cout << "\n";

        // f) low spd feedback
        auto low_spd_fb = piper.getArmLowSpeedFeedbacks();
        std::cout << std::fixed << std::setprecision(6) << "[LowSpd]  t=" << low_spd_fb.timestamp
                  << "  hz=" << low_spd_fb.hz;
        for (size_t i = 0; i < low_spd_fb.value.size(); ++i)
        {
            std::cout << "  j" << (i + 1) << "=" << low_spd_fb.value[i].toString();
        }
        std::cout << "\n";

        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }

    // 4) Clean up
    piper.disconnectPort(std::chrono::milliseconds{200});
    std::cout << "Disconnected.\n";
    return 0;
}
