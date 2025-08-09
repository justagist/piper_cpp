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
            /*can_init=*/true,
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
        // 1) Arm status
        auto status = piper.getArmStatus();
        std::cout << std::fixed << std::setprecision(6) << status.toString() << std::endl;

        // 2) End‐pose
        auto pose = piper.getArmEndPose();
        std::cout << std::fixed << std::setprecision(6) << pose.toString() << std::endl;

        // 3) Joint angles
        auto joints = piper.getArmJointStates();
        std::cout << std::fixed << std::setprecision(6) << joints.toString() << std::endl;

        // 4) gripper states
        auto gripper = piper.getGripperStates();
        std::cout << std::fixed << std::setprecision(6) << gripper.toString() << std::endl;

        // 5) high spd feedback
        auto high_spd_fb = piper.getArmHighSpeedFeedbacks();
        std::cout << std::fixed << std::setprecision(6) << high_spd_fb.toString() << std::endl;

        // 6) low spd feedback
        auto low_spd_fb = piper.getArmLowSpeedFeedbacks();
        std::cout << std::fixed << std::setprecision(6) << low_spd_fb.toString() << std::endl;

        // 7) current end velocity and acceleration
        auto end_vel_acc = piper.getArmCurrentEndVelAcc();
        std::cout << std::fixed << std::setprecision(6) << end_vel_acc.toString() << std::endl;

        // 8) crash protection rating fb
        auto crash_protection_rating_fb = piper.getArmCrashProtectionRating();
        std::cout << std::fixed << std::setprecision(6) << crash_protection_rating_fb.toString() << std::endl;

        // 9) gripper teaching pendant fb
        auto gripper_teaching_pendant_fb = piper.getGripperTeachingPendantParamFeedback();
        std::cout << std::fixed << std::setprecision(6) << gripper_teaching_pendant_fb.toString() << std::endl;

        // 10) current motor angle limit max speed
        auto current_motor_angle_limit_max_spd = piper.getArmCurrentMotorAngleLimitMaxSpd();
        std::cout << std::fixed << std::setprecision(6) << current_motor_angle_limit_max_spd.toString() << std::endl;

        // 11) current motor max acc limit
        auto current_motor_max_acc_limit = piper.getArmCurrentMotorMaxAccLimit();
        std::cout << std::fixed << std::setprecision(6) << current_motor_max_acc_limit.toString() << std::endl;

        // 12) all motor angle limit max speed
        auto all_motor_angle_limit_max_spd = piper.getArmAllMotorAngleLimitMaxSpd();
        std::cout << std::fixed << std::setprecision(6) << all_motor_angle_limit_max_spd.toString() << std::endl;

        // 13) all motor max acc limit
        auto all_motor_max_acc_limit = piper.getArmAllMotorMaxAccLimit();
        std::cout << std::fixed << std::setprecision(6) << all_motor_max_acc_limit.toString() << std::endl;

        // 14) joint ctrl msgs
        auto joint_ctrl_msgs = piper.getArmJointCtrlMsgs();
        std::cout << std::fixed << std::setprecision(6) << joint_ctrl_msgs.toString() << std::endl;

        // 15) gripper ctrl msgs
        auto gripper_ctrl_msgs = piper.getArmGripperCtrlMsgs();
        std::cout << std::fixed << std::setprecision(6) << gripper_ctrl_msgs.toString() << std::endl;

        // 16) motion ctrl code 151 (motion control 2)
        auto motion_ctrl_code_151 = piper.getArmMotionCtrlCode151();
        std::cout << std::fixed << std::setprecision(6) << motion_ctrl_code_151.toString() << std::endl;

        // 17) calculated fb FK
        auto fb_fk = piper.getCalculatedFeedbackFK();
        for (const auto& fk : fb_fk)
        {
            std::cout << std::fixed << std::setprecision(6) << "FK: " << fk[0] << ", " << fk[1] << ", " << fk[2] << ", "
                      << fk[3] << ", " << fk[4] << ", " << fk[5] << std::endl;
        }

        // 18) calculated Ctrl FK
        auto ctrl_fk = piper.getCalculatedControlFK();
        for (const auto& fk : ctrl_fk)
        {
            std::cout << std::fixed << std::setprecision(6) << "Ctrl FK: " << fk[0] << ", " << fk[1] << ", " << fk[2]
                      << ", " << fk[3] << ", " << fk[4] << ", " << fk[5] << std::endl;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }

    // 4) Clean up
    piper.disconnectPort(std::chrono::milliseconds{200});
    std::cout << "Disconnected.\n";
    return 0;
}
