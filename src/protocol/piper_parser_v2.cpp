#include "piper_cpp/protocol/piper_parser_v2.h"
#include <algorithm>
#include <cstring>

namespace piper_cpp
{

// Utility for endian conversion, assumes network (big-endian) order.
static int32_t bytesToInt32(const uint8_t* data)
{
    return (int32_t(data[0]) << 24) | (int32_t(data[1]) << 16) | (int32_t(data[2]) << 8) | int32_t(data[3]);
}
static int16_t bytesToInt16(const uint8_t* data) { return (int16_t)((uint16_t(data[0]) << 8) | uint16_t(data[1])); }
static uint16_t bytesToUint16(const uint8_t* data) { return (uint16_t(data[0]) << 8) | uint16_t(data[1]); }
static int8_t bytesToInt8(const uint8_t* data) { return static_cast<int8_t>(data[0]); }
static uint8_t bytesToUint8(const uint8_t* data) { return data[0]; }

bool PiperParserV2::decodeMessage(const struct can_frame& rx_frame, double timestamp, PiperMessage& msg)
{
    uint32_t can_id = rx_frame.can_id & CAN_EFF_MASK;
    const uint8_t* data = rx_frame.data;
    auto can_enum_id = static_cast<CanIDPiper>(can_id);
    msg.can_id = can_id;
    try
    {
        msg.type = canIdToMsgType(can_enum_id);
    }
    catch (...)
    {
        msg.type = ArmMsgType::Unknown;
        return false;
    }
    std::copy(std::begin(rx_frame.data), std::end(rx_frame.data), std::begin(msg.raw_data));
    msg.timestamp = timestamp;

    // ----------------- Main switch -----------------
    switch (can_enum_id)
    {
    // ----- Basic feedback -----
    case CanIDPiper::ARM_STATUS_FEEDBACK:
    {
        msg.type = ArmMsgType::StatusFeedback;
        ArmMsgFeedbackStatus status;
        status.ctrl_mode = static_cast<ArmMsgFeedbackStatusEnum::CtrlMode>(data[0]);
        status.arm_status = static_cast<ArmMsgFeedbackStatusEnum::ArmStatus>(data[1]);
        status.mode_feed = static_cast<ArmMsgFeedbackStatusEnum::ModeFeed>(data[2]);
        status.teach_status = static_cast<ArmMsgFeedbackStatusEnum::TeachingState>(data[3]);
        status.motion_status = static_cast<ArmMsgFeedbackStatusEnum::MotionStatus>(data[4]);
        status.trajectory_num = data[5];
        uint16_t err_code = (data[6] << 8) | data[7];
        status.setErrCode(err_code);
        msg.arm_status_msgs = status;
        break;
    }
    case CanIDPiper::ARM_END_POSE_FEEDBACK_1:
        msg.type = ArmMsgType::EndPoseFeedback1;
        msg.arm_end_pose.X_axis = bytesToInt32(&data[0]);
        msg.arm_end_pose.Y_axis = bytesToInt32(&data[4]);
        break;
    case CanIDPiper::ARM_END_POSE_FEEDBACK_2:
        msg.type = ArmMsgType::EndPoseFeedback2;
        msg.arm_end_pose.Z_axis = bytesToInt32(&data[0]);
        msg.arm_end_pose.RX_axis = bytesToInt32(&data[4]);
        break;
    case CanIDPiper::ARM_END_POSE_FEEDBACK_3:
        msg.type = ArmMsgType::EndPoseFeedback3;
        msg.arm_end_pose.RY_axis = bytesToInt32(&data[0]);
        msg.arm_end_pose.RZ_axis = bytesToInt32(&data[4]);
        break;

    // ----- Joint angle feedback -----
    case CanIDPiper::ARM_JOINT_FEEDBACK_12:
        msg.type = ArmMsgType::JointFeedback12;
        msg.arm_joint_feedback.joint_1 = bytesToInt32(&data[0]);
        msg.arm_joint_feedback.joint_2 = bytesToInt32(&data[4]);
        break;
    case CanIDPiper::ARM_JOINT_FEEDBACK_34:
        msg.type = ArmMsgType::JointFeedback34;
        msg.arm_joint_feedback.joint_3 = bytesToInt32(&data[0]);
        msg.arm_joint_feedback.joint_4 = bytesToInt32(&data[4]);
        break;
    case CanIDPiper::ARM_JOINT_FEEDBACK_56:
        msg.type = ArmMsgType::JointFeedback56;
        msg.arm_joint_feedback.joint_5 = bytesToInt32(&data[0]);
        msg.arm_joint_feedback.joint_6 = bytesToInt32(&data[4]);
        break;

    // ----- Gripper feedback -----
    case CanIDPiper::ARM_GRIPPER_FEEDBACK:
        msg.type = ArmMsgType::GripperFeedback;
        msg.gripper_feedback.grippers_angle = bytesToInt32(&data[0]);
        msg.gripper_feedback.grippers_effort = bytesToInt16(&data[4]);
        msg.gripper_feedback.status_code = data[6];
        msg.gripper_feedback.updateFocStatus();
        break;

    // ----- High-speed driver feedback (per joint, 1~6) -----
    case CanIDPiper::ARM_INFO_HIGH_SPD_FEEDBACK_1:
    case CanIDPiper::ARM_INFO_HIGH_SPD_FEEDBACK_2:
    case CanIDPiper::ARM_INFO_HIGH_SPD_FEEDBACK_3:
    case CanIDPiper::ARM_INFO_HIGH_SPD_FEEDBACK_4:
    case CanIDPiper::ARM_INFO_HIGH_SPD_FEEDBACK_5:
    case CanIDPiper::ARM_INFO_HIGH_SPD_FEEDBACK_6:
    {
        size_t idx = static_cast<size_t>(can_enum_id) - static_cast<size_t>(CanIDPiper::ARM_INFO_HIGH_SPD_FEEDBACK_1);
        if (idx < msg.high_spd_feedbacks.size())
        {
            msg.type = static_cast<ArmMsgType>(static_cast<int>(ArmMsgType::HighSpdFeed1) + idx);
            auto& high = msg.high_spd_feedbacks[idx];
            high.can_id = can_id;
            high.motor_speed = bytesToInt16(&data[0]);
            high.current = bytesToInt16(&data[2]);
            high.pos = bytesToInt32(&data[4]);
            high.calcEffort(); // If needed
        }
        break;
    }

    // ----- Low-speed driver feedback (per joint, 1~6) -----
    case CanIDPiper::ARM_INFO_LOW_SPD_FEEDBACK_1:
    case CanIDPiper::ARM_INFO_LOW_SPD_FEEDBACK_2:
    case CanIDPiper::ARM_INFO_LOW_SPD_FEEDBACK_3:
    case CanIDPiper::ARM_INFO_LOW_SPD_FEEDBACK_4:
    case CanIDPiper::ARM_INFO_LOW_SPD_FEEDBACK_5:
    case CanIDPiper::ARM_INFO_LOW_SPD_FEEDBACK_6:
    {
        size_t idx = static_cast<size_t>(can_enum_id) - static_cast<size_t>(CanIDPiper::ARM_INFO_LOW_SPD_FEEDBACK_1);
        if (idx < msg.low_spd_feedbacks.size())
        {
            msg.type = static_cast<ArmMsgType>(static_cast<int>(ArmMsgType::LowSpdFeed1) + idx);
            auto& low = msg.low_spd_feedbacks[idx];
            low.can_id = can_id;
            low.vol = bytesToUint16(&data[0]);
            low.foc_temp = bytesToInt16(&data[2]);
            low.motor_temp = bytesToInt8(&data[4]);
            low.foc_status_code = data[5];
            low.updateStatusFromCode();
            low.bus_current = bytesToUint16(&data[6]);
        }
        break;
    }

    // ----- Joint Vel/Acc Feedback (per joint) -----
    case CanIDPiper::ARM_FEEDBACK_JOINT_VEL_ACC_1:
    case CanIDPiper::ARM_FEEDBACK_JOINT_VEL_ACC_2:
    case CanIDPiper::ARM_FEEDBACK_JOINT_VEL_ACC_3:
    case CanIDPiper::ARM_FEEDBACK_JOINT_VEL_ACC_4:
    case CanIDPiper::ARM_FEEDBACK_JOINT_VEL_ACC_5:
    case CanIDPiper::ARM_FEEDBACK_JOINT_VEL_ACC_6:
    {
        size_t idx = static_cast<size_t>(can_enum_id) - static_cast<size_t>(CanIDPiper::ARM_FEEDBACK_JOINT_VEL_ACC_1);
        if (idx < msg.joint_vel_accs.size())
        {
            msg.type = static_cast<ArmMsgType>(static_cast<int>(ArmMsgType::FeedbackJointVelAcc1) + idx);
            auto& v = msg.joint_vel_accs[idx];
            v.can_id = can_id;
            v.end_linear_vel = bytesToUint16(&data[0]);
            v.end_angular_vel = bytesToUint16(&data[2]);
            v.end_linear_acc = bytesToUint16(&data[4]);
            v.end_angular_acc = bytesToUint16(&data[6]);
        }
        break;
    }

    // ----- End Vel/Acc Feedback -----
    case CanIDPiper::ARM_FEEDBACK_CURRENT_END_VEL_ACC_PARAM:
        msg.type = ArmMsgType::FeedbackCurrentEndVelAccParam;
        msg.arm_feedback_current_end_vel_acc_param.end_max_linear_vel = bytesToUint16(&data[0]);
        msg.arm_feedback_current_end_vel_acc_param.end_max_angular_vel = bytesToUint16(&data[2]);
        msg.arm_feedback_current_end_vel_acc_param.end_max_linear_acc = bytesToUint16(&data[4]);
        msg.arm_feedback_current_end_vel_acc_param.end_max_angular_acc = bytesToUint16(&data[6]);
        break;

    // ----- Crash Protection Rating -----
    case CanIDPiper::ARM_CRASH_PROTECTION_RATING_FEEDBACK:
        msg.type = ArmMsgType::CrashProtectionRatingFeedback;
        msg.arm_crash_protection_rating_feedback.joint_1_protection_level = data[0];
        msg.arm_crash_protection_rating_feedback.joint_2_protection_level = data[1];
        msg.arm_crash_protection_rating_feedback.joint_3_protection_level = data[2];
        msg.arm_crash_protection_rating_feedback.joint_4_protection_level = data[3];
        msg.arm_crash_protection_rating_feedback.joint_5_protection_level = data[4];
        msg.arm_crash_protection_rating_feedback.joint_6_protection_level = data[5];
        break;

    // ----- Gripper/Teaching Pendant Param Feedback -----
    case CanIDPiper::ARM_GRIPPER_TEACHING_PENDANT_PARAM_FEEDBACK:
        msg.type = ArmMsgType::GripperTeachingPendantParamFeedback;
        msg.arm_gripper_teaching_param_feedback.teaching_range_per = data[0];
        msg.arm_gripper_teaching_param_feedback.max_range_config = data[1];
        msg.arm_gripper_teaching_param_feedback.teaching_friction = data[2];
        break;

    // ----- Motor Angle/Speed/Accel Limit Feedback -----
    case CanIDPiper::ARM_FEEDBACK_CURRENT_MOTOR_ANGLE_LIMIT_MAX_SPD:
        msg.type = ArmMsgType::FeedbackCurrentMotorAngleLimitMaxSpd;
        msg.arm_feedback_current_motor_angle_limit_max_spd.motor_num = data[0];
        msg.arm_feedback_current_motor_angle_limit_max_spd.max_angle_limit = bytesToInt16(&data[1]);
        msg.arm_feedback_current_motor_angle_limit_max_spd.min_angle_limit = bytesToInt16(&data[3]);
        msg.arm_feedback_current_motor_angle_limit_max_spd.max_joint_spd = bytesToInt16(&data[5]);
        break;
    case CanIDPiper::ARM_FEEDBACK_CURRENT_MOTOR_MAX_ACC_LIMIT:
        msg.type = ArmMsgType::FeedbackCurrentMotorMaxAccLimit;
        msg.arm_feedback_current_motor_max_acc_limit.joint_motor_num = data[0];
        msg.arm_feedback_current_motor_max_acc_limit.max_joint_acc = bytesToInt16(&data[1]);
        break;

        // ----- Control/Transmit messages (TODO) -----
        // case CanIDPiper::ARM_MOTION_CTRL_2:
        // case CanIDPiper::ARM_JOINT_CTRL_12:
        // case CanIDPiper::ARM_JOINT_CTRL_34:
        // case CanIDPiper::ARM_JOINT_CTRL_56:
        // case CanIDPiper::ARM_GRIPPER_CTRL:
        //     // TODO: Implement parsing for transmit/control messages when their C++ structs are defined.
        //     break;

        // ----- Firmware read -----
        // case CanIDPiper::ARM_FIRMWARE_READ:
        //     // TODO: Add a std::vector<uint8_t> or similar for firmware_data and assign it here.
        //     break;

    default:
        return true;
    }
    return true;
}

} // namespace piper_cpp
