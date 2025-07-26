#pragma once

#include "piper_cpp/types/core.h"
#include "piper_cpp/types/feedback/arm_status.h"
#include "piper_cpp/types/feedback/end_effector.h"
#include "piper_cpp/types/feedback/gripper.h"
#include "piper_cpp/types/feedback/joints.h"
#include "piper_cpp/types/feedback/motors.h"

#include <array>
#include <cstdint>
#include <optional>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

namespace piper_cpp
{

struct PiperMessage
{
    static constexpr size_t raw_data_len = 8;
    static constexpr size_t num_joints = 6;

    uint16_t can_id = 0;
    uint8_t raw_data[raw_data_len] = {0}; // Raw data from CAN frame
    ArmMsgType type = ArmMsgType::Unknown;
    double timestamp = 0.0; // Timestamp in seconds

    // ----------- Feedback messages -----------
    ArmMsgFeedbackStatus arm_status_msgs;
    ArmMsgFeedbackEndPose arm_end_pose;
    ArmMsgFeedbackJointStates arm_joint_feedback;
    ArmMsgFeedbackAllJointVelAcc all_joint_vel_acc;
    ArmMsgFeedbackGripper gripper_feedback;
    ArmMsgFeedbackGripperTeachingPendantParam arm_gripper_teaching_param_feedback;
    ArmMsgFeedbackCurrentEndVelAccParam arm_feedback_current_end_vel_acc_param;
    ArmMsgFeedbackCurrentMotorAngleLimitMaxSpd arm_feedback_current_motor_angle_limit_max_spd;
    ArmMsgFeedbackAllCurrentMotorAngleLimitMaxSpd all_motor_angle_limit_max_spd;
    ArmMsgFeedbackCurrentMotorMaxAccLimit arm_feedback_current_motor_max_acc_limit;
    ArmMsgFeedbackAllCurrentMotorMaxAccLimit all_motor_max_acc_limit;
    ArmMsgFeedbackCrashProtectionRating arm_crash_protection_rating_feedback;

    // High-speed feedback, 6 joints
    std::array<ArmMsgFeedbackHighSpd, num_joints> high_spd_feedbacks;
    // Low-speed feedback, 6 joints
    std::array<ArmMsgFeedbackLowSpd, num_joints> low_spd_feedbacks;
    // Per-joint velocity/acceleration feedback (for "joint_vel_acc" type) — 6 slots for joint 1~6
    std::array<ArmMsgFeedbackJointVelAcc, num_joints> joint_vel_accs;

    // ------------ Transmit messages ------------
    // TODO: Add transmit types as C++ structs.
    // For example:
    // ArmMsgMotionCtrl_1 arm_motion_ctrl_1;
    // ArmMsgMotionCtrl_2 arm_motion_ctrl_2;
    // ArmMsgMotionCtrlCartesian arm_motion_ctrl_cartesian;
    // ArmMsgJointCtrl arm_joint_ctrl;
    // ArmMsgCircularPatternCoordNumUpdateCtrl arm_circular_ctrl;
    // ArmMsgGripperCtrl arm_gripper_ctrl;
    // ArmMsgJointMitCtrl arm_joint_mit_ctrl;
    // ArmMsgMasterSlaveModeConfig arm_ms_config;
    // ArmMsgMotorEnableDisableConfig arm_motor_enable;
    // ArmMsgSearchMotorMaxAngleSpdAccLimit arm_search_motor_max_angle_spd_acc_limit;
    // ArmMsgMotorAngleLimitMaxSpdSet arm_motor_angle_limit_max_spd_set;
    // ArmMsgJointConfig arm_joint_config;
    // ArmMsgInstructionResponseConfig arm_set_instruction_response;
    // ArmMsgParamEnquiryAndConfig arm_param_enquiry_and_config;
    // ArmMsgEndVelAccParamConfig arm_end_vel_acc_param_config;
    // ArmMsgCrashProtectionRatingConfig arm_crash_protection_rating_config;
    // ArmMsgGripperTeachingPendantParamConfig arm_gripper_teaching_param_config;

    std::vector<uint8_t> firmware_data; // for firmware binary

    std::string toString() const
    {
        std::ostringstream oss;
        oss << "Type: " << static_cast<int>(type) << "\n"; // fallback for enum value if no string conversion

        // Feedback messages (type-dependent)
        switch (type)
        {
        case ArmMsgType::StatusFeedback:
            oss << "Arm Status: " << arm_status_msgs.toString() << "\n";
            break;
        case ArmMsgType::JointFeedback12:
        case ArmMsgType::JointFeedback34:
        case ArmMsgType::JointFeedback56:
            oss << "Joint Feed: " << arm_joint_feedback.toString() << "\n";
            break;
        case ArmMsgType::GripperFeedback:
            oss << "Gripper Feed: " << gripper_feedback.toString() << "\n";
            break;
        case ArmMsgType::EndPoseFeedback1:
        case ArmMsgType::EndPoseFeedback2:
        case ArmMsgType::EndPoseFeedback3:
            oss << "End Pose Feed: " << arm_end_pose.toString() << "\n";
            break;

        // Per-joint High Spd Feedback
        case ArmMsgType::HighSpdFeed1:
        case ArmMsgType::HighSpdFeed2:
        case ArmMsgType::HighSpdFeed3:
        case ArmMsgType::HighSpdFeed4:
        case ArmMsgType::HighSpdFeed5:
        case ArmMsgType::HighSpdFeed6:
        {
            size_t idx = static_cast<size_t>(type) - static_cast<size_t>(ArmMsgType::HighSpdFeed1);
            oss << "High Spd Feedback [" << idx + 1 << "]: " << high_spd_feedbacks[idx].toString() << "\n";
            break;
        }
        // Per-joint Low Spd Feedback
        case ArmMsgType::LowSpdFeed1:
        case ArmMsgType::LowSpdFeed2:
        case ArmMsgType::LowSpdFeed3:
        case ArmMsgType::LowSpdFeed4:
        case ArmMsgType::LowSpdFeed5:
        case ArmMsgType::LowSpdFeed6:
        {
            size_t idx = static_cast<size_t>(type) - static_cast<size_t>(ArmMsgType::LowSpdFeed1);
            oss << "Low Spd Feedback [" << idx + 1 << "]: " << low_spd_feedbacks[idx].toString() << "\n";
            break;
        }
        default:
            oss << "(Other type or not yet supported toString)\n";
            break;
        }
        return oss.str();
    }
};

} // namespace piper_cpp
