#include "piper_cpp/protocol/piper_parser_v2.h"
#include <algorithm>
#include <cstring>

namespace piper_cpp
{

// Utility for endian conversion, assumes network (big-endian) order.
static int32_t bytesToInt32(const uint8_t *data)
{
    uint32_t val = (uint32_t(data[0]) << 24) | (uint32_t(data[1]) << 16) | (uint32_t(data[2]) << 8) | uint32_t(data[3]);
    return static_cast<int32_t>(val); // This cast will sign-extend if MSB is set
}

static int16_t bytesToInt16(const uint8_t *data)
{
    uint16_t val = (uint16_t(data[0]) << 8) | uint16_t(data[1]);
    return static_cast<int16_t>(val); // This cast will sign-extend if MSB is set
}

// For unsigned 16-bit
static uint16_t bytesToUint16(const uint8_t *data) { return (uint16_t(data[0]) << 8) | uint16_t(data[1]); }

// For signed 8-bit (this is safe, but explicit)
static int8_t bytesToInt8(const uint8_t *data) { return static_cast<int8_t>(data[0]); }

// For unsigned 8-bit
static uint8_t bytesToUint8(const uint8_t *data) { return data[0]; }

// Split a value into unsigned bytes (big-endian)
inline void int16_to_bytes(uint16_t val, uint8_t *out)
{
    out[0] = (val >> 8) & 0xFF;
    out[1] = (val) & 0xFF;
}

inline void uint16_to_bytes(uint16_t value, uint8_t *out)
{
    out[0] = (value >> 8) & 0xFF;
    out[1] = value & 0xFF;
}

inline void int32_to_bytes(int32_t val, uint8_t *out)
{
    out[0] = (val >> 24) & 0xFF;
    out[1] = (val >> 16) & 0xFF;
    out[2] = (val >> 8) & 0xFF;
    out[3] = (val) & 0xFF;
}
// For uint32_t
inline void uint32_to_bytes(uint32_t val, uint8_t *out)
{
    out[0] = (val >> 24) & 0xFF;
    out[1] = (val >> 16) & 0xFF;
    out[2] = (val >> 8) & 0xFF;
    out[3] = (val) & 0xFF;
}
// For 8-bit
inline void int8_to_bytes(uint8_t val, uint8_t *out) { out[0] = val & 0xFF; }

bool PiperParserV2::decodeMessage(const struct can_frame &rx_frame, double timestamp, PiperMessage &msg)
{
    uint32_t can_id = rx_frame.can_id & CAN_EFF_MASK;
    const uint8_t *data = rx_frame.data;
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
        msg.gripper_feedback.status_code = bytesToUint8(&data[6]);
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
            auto &high = msg.high_spd_feedbacks[idx];
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
            auto &low = msg.low_spd_feedbacks[idx];
            low.can_id = can_id;
            low.vol = bytesToUint16(&data[0]);
            low.foc_temp = bytesToInt16(&data[2]);
            low.motor_temp = bytesToInt8(&data[4]);
            low.foc_status_code = bytesToUint8(&data[5]);
            low.updateStatusFromCode();
            low.bus_current = bytesToUint16(&data[6]);
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
    // ----- Motion Control 2 -----
    case CanIDPiper::ARM_MOTION_CTRL_2:
    {
        msg.type = ArmMsgType::MotionCtrl2;
        // Example structure; update to match your C++ struct
        msg.arm_motion_ctrl_2.ctrl_mode = data[0];
        msg.arm_motion_ctrl_2.move_mode = data[1];
        msg.arm_motion_ctrl_2.move_spd_rate_ctrl = data[2];
        msg.arm_motion_ctrl_2.mit_mode = data[3];
        msg.arm_motion_ctrl_2.residence_time = data[4];
        break;
    }

    // ----- Joint Control 12 -----
    case CanIDPiper::ARM_JOINT_CTRL_12:
    {
        msg.type = ArmMsgType::JointCtrl12;
        msg.arm_joint_ctrl.joint_1 = bytesToInt32(&data[0]);
        msg.arm_joint_ctrl.joint_2 = bytesToInt32(&data[4]);
        break;
    }

    // ----- Joint Control 34 -----
    case CanIDPiper::ARM_JOINT_CTRL_34:
    {
        msg.type = ArmMsgType::JointCtrl34;
        msg.arm_joint_ctrl.joint_3 = bytesToInt32(&data[0]);
        msg.arm_joint_ctrl.joint_4 = bytesToInt32(&data[4]);
        break;
    }

    // ----- Joint Control 56 -----
    case CanIDPiper::ARM_JOINT_CTRL_56:
    {
        msg.type = ArmMsgType::JointCtrl56;
        msg.arm_joint_ctrl.joint_5 = bytesToInt32(&data[0]);
        msg.arm_joint_ctrl.joint_6 = bytesToInt32(&data[4]);
        break;
    }

    // ----- Gripper Control -----
    case CanIDPiper::ARM_GRIPPER_CTRL:
    {
        msg.type = ArmMsgType::GripperCtrl;
        msg.gripper_ctrl.grippers_angle = bytesToInt32(&data[0]);
        msg.gripper_ctrl.grippers_effort = bytesToInt16(&data[4]);
        msg.gripper_ctrl.status_code = bytesToUint8(&data[6]);
        msg.gripper_ctrl.set_zero = bytesToUint8(&data[7]);
        break;
    }

    // ----- Firmware Read -----
    case CanIDPiper::ARM_FIRMWARE_READ:
    {
        msg.type = ArmMsgType::FirmwareRead;
        // Firmware data is just raw; store in vector for easy use
        msg.firmware_data.assign(data, data + PiperMessage::raw_data_len);
        break;
    }

    // ----- Instruction Response (0x476) -----
    case CanIDPiper::ARM_INSTRUCTION_RESPONSE_CONFIG:
    {
        msg.type = ArmMsgType::InstructionResponseConfig;
        msg.arm_set_instruction_response.instruction_index = data[0];
        msg.arm_set_instruction_response.zero_config_success_flag = data[1];
        break;
    }

    default:
        return false;
    }
    return true;
}

// Main function
bool PiperParserV2::encodeMessage(const PiperMessage &msg, struct can_frame &tx_frame)
{
    using namespace std;
    bool ret = true;
    memset(tx_frame.data, 0, 8);

    // Always set CAN ID based on type for robustness
    tx_frame.can_id = msg.can_id;

    switch (msg.type)
    {
    // --- Motion Ctrl 1 ---
    case ArmMsgType::MotionCtrl1:
        tx_frame.data[0] = msg.arm_motion_ctrl_1.emergency_stop;
        tx_frame.data[1] = msg.arm_motion_ctrl_1.track_ctrl;
        tx_frame.data[2] = msg.arm_motion_ctrl_1.grag_teach_ctrl;
        // [3..7] zero
        break;

    // --- Motion Ctrl 2 ---
    case ArmMsgType::MotionCtrl2:
        tx_frame.data[0] = msg.arm_motion_ctrl_2.ctrl_mode;
        tx_frame.data[1] = msg.arm_motion_ctrl_2.move_mode;
        tx_frame.data[2] = msg.arm_motion_ctrl_2.move_spd_rate_ctrl;
        tx_frame.data[3] = msg.arm_motion_ctrl_2.mit_mode;
        tx_frame.data[4] = msg.arm_motion_ctrl_2.residence_time;
        tx_frame.data[5] = msg.arm_motion_ctrl_2.installation_pos;
        // [6..7] zero
        break;

    // --- Cartesian Control (1/2/3) ---
    case ArmMsgType::MotionCtrlCartesian1:
        int32_to_bytes(msg.arm_motion_ctrl_cartesian.X_axis, &tx_frame.data[0]);
        int32_to_bytes(msg.arm_motion_ctrl_cartesian.Y_axis, &tx_frame.data[4]);
        break;
    case ArmMsgType::MotionCtrlCartesian2:
        int32_to_bytes(msg.arm_motion_ctrl_cartesian.Z_axis, &tx_frame.data[0]);
        int32_to_bytes(msg.arm_motion_ctrl_cartesian.RX_axis, &tx_frame.data[4]);
        break;
    case ArmMsgType::MotionCtrlCartesian3:
        int32_to_bytes(msg.arm_motion_ctrl_cartesian.RY_axis, &tx_frame.data[0]);
        int32_to_bytes(msg.arm_motion_ctrl_cartesian.RZ_axis, &tx_frame.data[4]);
        break;

    // --- Joint Ctrl 12/34/56 ---
    case ArmMsgType::JointCtrl12:
        int32_to_bytes(msg.arm_joint_ctrl.joint_1, &tx_frame.data[0]);
        int32_to_bytes(msg.arm_joint_ctrl.joint_2, &tx_frame.data[4]);
        break;
    case ArmMsgType::JointCtrl34:
        int32_to_bytes(msg.arm_joint_ctrl.joint_3, &tx_frame.data[0]);
        int32_to_bytes(msg.arm_joint_ctrl.joint_4, &tx_frame.data[4]);
        break;
    case ArmMsgType::JointCtrl56:
        int32_to_bytes(msg.arm_joint_ctrl.joint_5, &tx_frame.data[0]);
        int32_to_bytes(msg.arm_joint_ctrl.joint_6, &tx_frame.data[4]);
        break;

    // --- Circular Pattern Coord Num Update Ctrl ---
    case ArmMsgType::CircularPatternCoordNumUpdateCtrl:
        tx_frame.data[0] = msg.arm_circular_ctrl.instruction_num;
        // [1..7] zero
        break;

    // --- Gripper Control ---
    case ArmMsgType::GripperCtrl:
        int32_to_bytes(msg.gripper_ctrl.grippers_angle, &tx_frame.data[0]);
        uint16_to_bytes(msg.gripper_ctrl.grippers_effort, &tx_frame.data[4]);
        tx_frame.data[6] = msg.gripper_ctrl.status_code;
        tx_frame.data[7] = msg.gripper_ctrl.set_zero;
        break;

    // --- Master-Slave Mode Config ---
    case ArmMsgType::MasterSlaveModeConfig:
        tx_frame.data[0] = msg.arm_ms_config.linkage_config;
        tx_frame.data[1] = msg.arm_ms_config.feedback_offset;
        tx_frame.data[2] = msg.arm_ms_config.ctrl_offset;
        tx_frame.data[3] = msg.arm_ms_config.linkage_offset;
        // [4..7] zero
        break;

    // --- Motor Enable/Disable Config ---
    case ArmMsgType::MotorEnableDisableConfig:
        tx_frame.data[0] = msg.arm_motor_enable.motor_num;
        tx_frame.data[1] = msg.arm_motor_enable.enable_flag;
        // [2..7] zero
        break;

    // --- Search Motor Max Angle/Speed/Acc Limit ---
    case ArmMsgType::SearchMotorMaxAngleSpdAccLimit:
        tx_frame.data[0] = msg.arm_search_motor_max_angle_spd_acc_limit.motor_num;
        tx_frame.data[1] = msg.arm_search_motor_max_angle_spd_acc_limit.search_content;
        // [2..7] zero
        break;

    // --- Motor Angle Limit/Max Spd Set ---
    case ArmMsgType::MotorAngleLimitMaxSpdSet:
        tx_frame.data[0] = msg.arm_motor_angle_limit_max_spd_set.motor_num;
        int16_to_bytes(msg.arm_motor_angle_limit_max_spd_set.max_angle_limit, &tx_frame.data[1]);
        int16_to_bytes(msg.arm_motor_angle_limit_max_spd_set.min_angle_limit, &tx_frame.data[3]);
        uint16_to_bytes(msg.arm_motor_angle_limit_max_spd_set.max_joint_spd, &tx_frame.data[5]);
        // [7] zero
        break;

    // --- Joint Config ---
    case ArmMsgType::JointConfig:
        tx_frame.data[0] = msg.arm_joint_config.joint_motor_num;
        tx_frame.data[1] = msg.arm_joint_config.set_motor_current_pos_as_zero;
        tx_frame.data[2] = msg.arm_joint_config.acc_param_config_is_effective_or_not;
        uint16_to_bytes(msg.arm_joint_config.max_joint_acc, &tx_frame.data[3]);
        tx_frame.data[5] = msg.arm_joint_config.clear_joint_err;
        // [6..7] zero
        break;

    // --- Instruction Response Config ---
    case ArmMsgType::InstructionResponseConfig:
        tx_frame.data[0] = msg.arm_set_instruction_response.instruction_index;
        tx_frame.data[1] = msg.arm_set_instruction_response.zero_config_success_flag;
        // [2..7] zero
        break;

    // --- Param Enquiry and Config ---
    case ArmMsgType::ParamEnquiryAndConfig:
        tx_frame.data[0] = msg.arm_param_enquiry_and_config.param_enquiry;
        tx_frame.data[1] = msg.arm_param_enquiry_and_config.param_setting;
        tx_frame.data[2] = msg.arm_param_enquiry_and_config.data_feedback_0x48x;
        tx_frame.data[3] = msg.arm_param_enquiry_and_config.end_load_param_setting_effective;
        tx_frame.data[4] = msg.arm_param_enquiry_and_config.set_end_load;
        // [5..7] zero
        break;

    // --- End Vel/Acc Param Config ---
    case ArmMsgType::EndVelAccParamConfig:
        uint16_to_bytes(msg.arm_end_vel_acc_param_config.end_max_linear_vel, &tx_frame.data[0]);
        uint16_to_bytes(msg.arm_end_vel_acc_param_config.end_max_angular_vel, &tx_frame.data[2]);
        uint16_to_bytes(msg.arm_end_vel_acc_param_config.end_max_linear_acc, &tx_frame.data[4]);
        uint16_to_bytes(msg.arm_end_vel_acc_param_config.end_max_angular_acc, &tx_frame.data[6]);
        break;

    // --- Crash Protection Rating Config ---
    case ArmMsgType::CrashProtectionRatingConfig:
        tx_frame.data[0] = msg.arm_crash_protection_rating_config.joint_1_protection_level;
        tx_frame.data[1] = msg.arm_crash_protection_rating_config.joint_2_protection_level;
        tx_frame.data[2] = msg.arm_crash_protection_rating_config.joint_3_protection_level;
        tx_frame.data[3] = msg.arm_crash_protection_rating_config.joint_4_protection_level;
        tx_frame.data[4] = msg.arm_crash_protection_rating_config.joint_5_protection_level;
        tx_frame.data[5] = msg.arm_crash_protection_rating_config.joint_6_protection_level;
        // [6..7] zero
        break;

    // --- Gripper Teaching Pendant Param Config ---
    case ArmMsgType::GripperTeachingPendantParamConfig:
        tx_frame.data[0] = msg.arm_gripper_teaching_param_config.teaching_range_per;
        tx_frame.data[1] = msg.arm_gripper_teaching_param_config.max_range_config;
        tx_frame.data[2] = msg.arm_gripper_teaching_param_config.teaching_friction;
        // [3..7] zero
        break;

    // --- MIT Single Joint Control (1~6) ---
    case ArmMsgType::JointMitCtrl1:
    case ArmMsgType::JointMitCtrl2:
    case ArmMsgType::JointMitCtrl3:
    case ArmMsgType::JointMitCtrl4:
    case ArmMsgType::JointMitCtrl5:
    case ArmMsgType::JointMitCtrl6:
    {
        size_t joint_idx = static_cast<size_t>(msg.type) - static_cast<size_t>(ArmMsgType::JointMitCtrl1);
        const ArmMsgJointMitCtrl &mit = msg.arm_joint_mit_ctrl.motors[joint_idx];

        tx_frame.data[0] = (mit.pos_ref >> 8) & 0xFF;
        tx_frame.data[1] = mit.pos_ref & 0xFF;
        tx_frame.data[2] = (mit.vel_ref >> 4) & 0xFF;
        tx_frame.data[3] = ((mit.vel_ref & 0xF) << 4) | ((mit.kp >> 8) & 0x0F);
        tx_frame.data[4] = mit.kp & 0xFF;
        tx_frame.data[5] = (mit.kd >> 4) & 0xFF;
        tx_frame.data[6] = ((mit.kd & 0xF) << 4) | ((mit.t_ref >> 4) & 0x0F);

        uint8_t crc = (tx_frame.data[0] ^ tx_frame.data[1] ^ tx_frame.data[2] ^ tx_frame.data[3] ^ tx_frame.data[4] ^
                       tx_frame.data[5] ^ tx_frame.data[6]) &
                      0x0F;
        tx_frame.data[7] = ((mit.t_ref & 0x0F) << 4) | (crc & 0x0F);
        break;
    }

    // --- Firmware Read ---
    case ArmMsgType::FirmwareRead:
        for (size_t i = 0; i < std::min(msg.firmware_data.size(), size_t(8)); ++i)
            tx_frame.data[i] = msg.firmware_data[i];
        for (size_t i = msg.firmware_data.size(); i < 8; ++i)
            tx_frame.data[i] = 0;
        break;

    // --- Unknown / Not Supported ---
    default:
        ret = false;
        break;
    }

    return ret;
}

} // namespace piper_cpp
