#pragma once

#include <cstdint>
#include <optional>
#include <stdexcept>
#include <string>
#include <unordered_map>

namespace piper_cpp
{

// Message type enum (corresponding to ArmMsgType)
enum class ArmMsgType : uint16_t
{
    Unknown = 0x00,

    // Feedback
    StatusFeedback,
    EndPoseFeedback1,
    EndPoseFeedback2,
    EndPoseFeedback3,
    JointFeedback12,
    JointFeedback34,
    JointFeedback56,
    GripperFeedback,
    HighSpdFeed1,
    HighSpdFeed2,
    HighSpdFeed3,
    HighSpdFeed4,
    HighSpdFeed5,
    HighSpdFeed6,
    LowSpdFeed1,
    LowSpdFeed2,
    LowSpdFeed3,
    LowSpdFeed4,
    LowSpdFeed5,
    LowSpdFeed6,

    // Transmit
    MotionCtrl1,
    MotionCtrl2,
    MotionCtrlCartesian1,
    MotionCtrlCartesian2,
    MotionCtrlCartesian3,
    JointCtrl12,
    JointCtrl34,
    JointCtrl56,
    CircularPatternCoordNumUpdateCtrl,
    GripperCtrl,

    JointMitCtrl1,
    JointMitCtrl2,
    JointMitCtrl3,
    JointMitCtrl4,
    JointMitCtrl5,
    JointMitCtrl6,

    MasterSlaveModeConfig,
    MotorEnableDisableConfig,
    SearchMotorMaxAngleSpdAccLimit,
    FeedbackCurrentMotorAngleLimitMaxSpd,
    MotorAngleLimitMaxSpdSet,
    JointConfig,
    InstructionResponseConfig,
    ParamEnquiryAndConfig,
    FeedbackCurrentEndVelAccParam,
    EndVelAccParamConfig,
    CrashProtectionRatingConfig,
    CrashProtectionRatingFeedback,
    FeedbackCurrentMotorMaxAccLimit,
    GripperTeachingPendantParamConfig,
    GripperTeachingPendantParamFeedback,

    FeedbackJointVelAcc1,
    FeedbackJointVelAcc2,
    FeedbackJointVelAcc3,
    FeedbackJointVelAcc4,
    FeedbackJointVelAcc5,
    FeedbackJointVelAcc6,

    LightCtrl,
    CanUpdateSilentModeConfig,
    FirmwareRead
};

// CAN ID enum (corresponding to CanIDPiper)
enum class CanIDPiper : uint16_t
{
    ARM_STATUS_FEEDBACK = 0x2A1,
    ARM_END_POSE_FEEDBACK_1 = 0x2A2,
    ARM_END_POSE_FEEDBACK_2 = 0x2A3,
    ARM_END_POSE_FEEDBACK_3 = 0x2A4,
    ARM_JOINT_FEEDBACK_12 = 0x2A5,
    ARM_JOINT_FEEDBACK_34 = 0x2A6,
    ARM_JOINT_FEEDBACK_56 = 0x2A7,
    ARM_GRIPPER_FEEDBACK = 0x2A8,

    ARM_MOTION_CTRL_1 = 0x150,
    ARM_MOTION_CTRL_2 = 0x151,
    ARM_MOTION_CTRL_CARTESIAN_1 = 0x152,
    ARM_MOTION_CTRL_CARTESIAN_2 = 0x153,
    ARM_MOTION_CTRL_CARTESIAN_3 = 0x154,
    ARM_JOINT_CTRL_12 = 0x155,
    ARM_JOINT_CTRL_34 = 0x156,
    ARM_JOINT_CTRL_56 = 0x157,
    ARM_CIRCULAR_PATTERN_COORD_NUM_UPDATE_CTRL = 0x158,
    ARM_GRIPPER_CTRL = 0x159,

    ARM_JOINT_MIT_CTRL_1 = 0x15A,
    ARM_JOINT_MIT_CTRL_2 = 0x15B,
    ARM_JOINT_MIT_CTRL_3 = 0x15C,
    ARM_JOINT_MIT_CTRL_4 = 0x15D,
    ARM_JOINT_MIT_CTRL_5 = 0x15E,
    ARM_JOINT_MIT_CTRL_6 = 0x15F,

    ARM_MASTER_SLAVE_MODE_CONFIG = 0x470,
    ARM_MOTOR_ENABLE_DISABLE_CONFIG = 0x471,
    ARM_SEARCH_MOTOR_MAX_SPD_ACC_LIMIT = 0x472,
    ARM_FEEDBACK_CURRENT_MOTOR_ANGLE_LIMIT_MAX_SPD = 0x473,
    ARM_MOTOR_ANGLE_LIMIT_MAX_SPD_SET = 0x474,
    ARM_JOINT_CONFIG = 0x475,
    ARM_INSTRUCTION_RESPONSE_CONFIG = 0x476,
    ARM_PARAM_ENQUIRY_AND_CONFIG = 0x477,
    ARM_FEEDBACK_CURRENT_END_VEL_ACC_PARAM = 0x478,
    ARM_END_VEL_ACC_PARAM_CONFIG = 0x479,
    ARM_CRASH_PROTECTION_RATING_CONFIG = 0x47A,
    ARM_CRASH_PROTECTION_RATING_FEEDBACK = 0x47B,
    ARM_FEEDBACK_CURRENT_MOTOR_MAX_ACC_LIMIT = 0x47C,
    ARM_GRIPPER_TEACHING_PENDANT_PARAM_CONFIG = 0x47D,
    ARM_GRIPPER_TEACHING_PENDANT_PARAM_FEEDBACK = 0x47E,

    ARM_FEEDBACK_JOINT_VEL_ACC_1 = 0x481,
    ARM_FEEDBACK_JOINT_VEL_ACC_2 = 0x482,
    ARM_FEEDBACK_JOINT_VEL_ACC_3 = 0x483,
    ARM_FEEDBACK_JOINT_VEL_ACC_4 = 0x484,
    ARM_FEEDBACK_JOINT_VEL_ACC_5 = 0x485,
    ARM_FEEDBACK_JOINT_VEL_ACC_6 = 0x486,

    ARM_LIGHT_CTRL = 0x121,

    ARM_INFO_HIGH_SPD_FEEDBACK_1 = 0x251,
    ARM_INFO_HIGH_SPD_FEEDBACK_2 = 0x252,
    ARM_INFO_HIGH_SPD_FEEDBACK_3 = 0x253,
    ARM_INFO_HIGH_SPD_FEEDBACK_4 = 0x254,
    ARM_INFO_HIGH_SPD_FEEDBACK_5 = 0x255,
    ARM_INFO_HIGH_SPD_FEEDBACK_6 = 0x256,

    ARM_INFO_LOW_SPD_FEEDBACK_1 = 0x261,
    ARM_INFO_LOW_SPD_FEEDBACK_2 = 0x262,
    ARM_INFO_LOW_SPD_FEEDBACK_3 = 0x263,
    ARM_INFO_LOW_SPD_FEEDBACK_4 = 0x264,
    ARM_INFO_LOW_SPD_FEEDBACK_5 = 0x265,
    ARM_INFO_LOW_SPD_FEEDBACK_6 = 0x266,

    ARM_CAN_UPDATE_SILENT_MODE_CONFIG = 0x422,
    ARM_FIRMWARE_READ = 0x4AF
};

// Mapping functions
ArmMsgType canIdToMsgType(CanIDPiper can_id);
CanIDPiper msgTypeToCanId(ArmMsgType type);

std::string toString(CanIDPiper id);
std::string toString(ArmMsgType type);

} // namespace piper_cpp
