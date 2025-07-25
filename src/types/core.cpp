#include "piper_cpp/types/core.h"
#include <stdexcept>

namespace piper_cpp
{

CanIDPiper msgTypeToCanId(ArmMsgType type)
{
    switch (type)
    {
    case ArmMsgType::StatusFeedback:
        return CanIDPiper::ARM_STATUS_FEEDBACK;
    case ArmMsgType::EndPoseFeedback1:
        return CanIDPiper::ARM_END_POSE_FEEDBACK_1;
    case ArmMsgType::EndPoseFeedback2:
        return CanIDPiper::ARM_END_POSE_FEEDBACK_2;
    case ArmMsgType::EndPoseFeedback3:
        return CanIDPiper::ARM_END_POSE_FEEDBACK_3;
    case ArmMsgType::JointFeedback12:
        return CanIDPiper::ARM_JOINT_FEEDBACK_12;
    case ArmMsgType::JointFeedback34:
        return CanIDPiper::ARM_JOINT_FEEDBACK_34;
    case ArmMsgType::JointFeedback56:
        return CanIDPiper::ARM_JOINT_FEEDBACK_56;
    case ArmMsgType::GripperFeedback:
        return CanIDPiper::ARM_GRIPPER_FEEDBACK;
    case ArmMsgType::HighSpdFeed1:
        return CanIDPiper::ARM_INFO_HIGH_SPD_FEEDBACK_1;
    case ArmMsgType::HighSpdFeed2:
        return CanIDPiper::ARM_INFO_HIGH_SPD_FEEDBACK_2;
    case ArmMsgType::HighSpdFeed3:
        return CanIDPiper::ARM_INFO_HIGH_SPD_FEEDBACK_3;
    case ArmMsgType::HighSpdFeed4:
        return CanIDPiper::ARM_INFO_HIGH_SPD_FEEDBACK_4;
    case ArmMsgType::HighSpdFeed5:
        return CanIDPiper::ARM_INFO_HIGH_SPD_FEEDBACK_5;
    case ArmMsgType::HighSpdFeed6:
        return CanIDPiper::ARM_INFO_HIGH_SPD_FEEDBACK_6;
    case ArmMsgType::LowSpdFeed1:
        return CanIDPiper::ARM_INFO_LOW_SPD_FEEDBACK_1;
    case ArmMsgType::LowSpdFeed2:
        return CanIDPiper::ARM_INFO_LOW_SPD_FEEDBACK_2;
    case ArmMsgType::LowSpdFeed3:
        return CanIDPiper::ARM_INFO_LOW_SPD_FEEDBACK_3;
    case ArmMsgType::LowSpdFeed4:
        return CanIDPiper::ARM_INFO_LOW_SPD_FEEDBACK_4;
    case ArmMsgType::LowSpdFeed5:
        return CanIDPiper::ARM_INFO_LOW_SPD_FEEDBACK_5;
    case ArmMsgType::LowSpdFeed6:
        return CanIDPiper::ARM_INFO_LOW_SPD_FEEDBACK_6;
    case ArmMsgType::MotionCtrl1:
        return CanIDPiper::ARM_MOTION_CTRL_1;
    case ArmMsgType::MotionCtrl2:
        return CanIDPiper::ARM_MOTION_CTRL_2;
    case ArmMsgType::MotionCtrlCartesian1:
        return CanIDPiper::ARM_MOTION_CTRL_CARTESIAN_1;
    case ArmMsgType::MotionCtrlCartesian2:
        return CanIDPiper::ARM_MOTION_CTRL_CARTESIAN_2;
    case ArmMsgType::MotionCtrlCartesian3:
        return CanIDPiper::ARM_MOTION_CTRL_CARTESIAN_3;
    case ArmMsgType::JointCtrl12:
        return CanIDPiper::ARM_JOINT_CTRL_12;
    case ArmMsgType::JointCtrl34:
        return CanIDPiper::ARM_JOINT_CTRL_34;
    case ArmMsgType::JointCtrl56:
        return CanIDPiper::ARM_JOINT_CTRL_56;
    case ArmMsgType::CircularPatternCoordNumUpdateCtrl:
        return CanIDPiper::ARM_CIRCULAR_PATTERN_COORD_NUM_UPDATE_CTRL;
    case ArmMsgType::GripperCtrl:
        return CanIDPiper::ARM_GRIPPER_CTRL;
    case ArmMsgType::JointMitCtrl1:
        return CanIDPiper::ARM_JOINT_MIT_CTRL_1;
    case ArmMsgType::JointMitCtrl2:
        return CanIDPiper::ARM_JOINT_MIT_CTRL_2;
    case ArmMsgType::JointMitCtrl3:
        return CanIDPiper::ARM_JOINT_MIT_CTRL_3;
    case ArmMsgType::JointMitCtrl4:
        return CanIDPiper::ARM_JOINT_MIT_CTRL_4;
    case ArmMsgType::JointMitCtrl5:
        return CanIDPiper::ARM_JOINT_MIT_CTRL_5;
    case ArmMsgType::JointMitCtrl6:
        return CanIDPiper::ARM_JOINT_MIT_CTRL_6;
    case ArmMsgType::MasterSlaveModeConfig:
        return CanIDPiper::ARM_MASTER_SLAVE_MODE_CONFIG;
    case ArmMsgType::MotorEnableDisableConfig:
        return CanIDPiper::ARM_MOTOR_ENABLE_DISABLE_CONFIG;
    case ArmMsgType::SearchMotorMaxAngleSpdAccLimit:
        return CanIDPiper::ARM_SEARCH_MOTOR_MAX_SPD_ACC_LIMIT;
    case ArmMsgType::FeedbackCurrentMotorAngleLimitMaxSpd:
        return CanIDPiper::ARM_FEEDBACK_CURRENT_MOTOR_ANGLE_LIMIT_MAX_SPD;
    case ArmMsgType::MotorAngleLimitMaxSpdSet:
        return CanIDPiper::ARM_MOTOR_ANGLE_LIMIT_MAX_SPD_SET;
    case ArmMsgType::JointConfig:
        return CanIDPiper::ARM_JOINT_CONFIG;
    case ArmMsgType::InstructionResponseConfig:
        return CanIDPiper::ARM_INSTRUCTION_RESPONSE_CONFIG;
    case ArmMsgType::ParamEnquiryAndConfig:
        return CanIDPiper::ARM_PARAM_ENQUIRY_AND_CONFIG;
    case ArmMsgType::FeedbackCurrentEndVelAccParam:
        return CanIDPiper::ARM_FEEDBACK_CURRENT_END_VEL_ACC_PARAM;
    case ArmMsgType::EndVelAccParamConfig:
        return CanIDPiper::ARM_END_VEL_ACC_PARAM_CONFIG;
    case ArmMsgType::CrashProtectionRatingConfig:
        return CanIDPiper::ARM_CRASH_PROTECTION_RATING_CONFIG;
    case ArmMsgType::CrashProtectionRatingFeedback:
        return CanIDPiper::ARM_CRASH_PROTECTION_RATING_FEEDBACK;
    case ArmMsgType::FeedbackCurrentMotorMaxAccLimit:
        return CanIDPiper::ARM_FEEDBACK_CURRENT_MOTOR_MAX_ACC_LIMIT;
    case ArmMsgType::GripperTeachingPendantParamConfig:
        return CanIDPiper::ARM_GRIPPER_TEACHING_PENDANT_PARAM_CONFIG;
    case ArmMsgType::GripperTeachingPendantParamFeedback:
        return CanIDPiper::ARM_GRIPPER_TEACHING_PENDANT_PARAM_FEEDBACK;
    case ArmMsgType::FeedbackJointVelAcc1:
        return CanIDPiper::ARM_FEEDBACK_JOINT_VEL_ACC_1;
    case ArmMsgType::FeedbackJointVelAcc2:
        return CanIDPiper::ARM_FEEDBACK_JOINT_VEL_ACC_2;
    case ArmMsgType::FeedbackJointVelAcc3:
        return CanIDPiper::ARM_FEEDBACK_JOINT_VEL_ACC_3;
    case ArmMsgType::FeedbackJointVelAcc4:
        return CanIDPiper::ARM_FEEDBACK_JOINT_VEL_ACC_4;
    case ArmMsgType::FeedbackJointVelAcc5:
        return CanIDPiper::ARM_FEEDBACK_JOINT_VEL_ACC_5;
    case ArmMsgType::FeedbackJointVelAcc6:
        return CanIDPiper::ARM_FEEDBACK_JOINT_VEL_ACC_6;
    case ArmMsgType::LightCtrl:
        return CanIDPiper::ARM_LIGHT_CTRL;
    case ArmMsgType::CanUpdateSilentModeConfig:
        return CanIDPiper::ARM_CAN_UPDATE_SILENT_MODE_CONFIG;
    case ArmMsgType::FirmwareRead:
        return CanIDPiper::ARM_FIRMWARE_READ;
    case ArmMsgType::Unknown:
        throw std::invalid_argument("Unknown ArmMsgType has no valid CAN ID");
    }
    throw std::invalid_argument("Unhandled ArmMsgType in msgTypeToCanId");
}

ArmMsgType canIdToMsgType(CanIDPiper id)
{
    switch (id)
    {
    case CanIDPiper::ARM_STATUS_FEEDBACK:
        return ArmMsgType::StatusFeedback;
    case CanIDPiper::ARM_END_POSE_FEEDBACK_1:
        return ArmMsgType::EndPoseFeedback1;
    case CanIDPiper::ARM_END_POSE_FEEDBACK_2:
        return ArmMsgType::EndPoseFeedback2;
    case CanIDPiper::ARM_END_POSE_FEEDBACK_3:
        return ArmMsgType::EndPoseFeedback3;
    case CanIDPiper::ARM_JOINT_FEEDBACK_12:
        return ArmMsgType::JointFeedback12;
    case CanIDPiper::ARM_JOINT_FEEDBACK_34:
        return ArmMsgType::JointFeedback34;
    case CanIDPiper::ARM_JOINT_FEEDBACK_56:
        return ArmMsgType::JointFeedback56;
    case CanIDPiper::ARM_GRIPPER_FEEDBACK:
        return ArmMsgType::GripperFeedback;
    case CanIDPiper::ARM_INFO_HIGH_SPD_FEEDBACK_1:
        return ArmMsgType::HighSpdFeed1;
    case CanIDPiper::ARM_INFO_HIGH_SPD_FEEDBACK_2:
        return ArmMsgType::HighSpdFeed2;
    case CanIDPiper::ARM_INFO_HIGH_SPD_FEEDBACK_3:
        return ArmMsgType::HighSpdFeed3;
    case CanIDPiper::ARM_INFO_HIGH_SPD_FEEDBACK_4:
        return ArmMsgType::HighSpdFeed4;
    case CanIDPiper::ARM_INFO_HIGH_SPD_FEEDBACK_5:
        return ArmMsgType::HighSpdFeed5;
    case CanIDPiper::ARM_INFO_HIGH_SPD_FEEDBACK_6:
        return ArmMsgType::HighSpdFeed6;
    case CanIDPiper::ARM_INFO_LOW_SPD_FEEDBACK_1:
        return ArmMsgType::LowSpdFeed1;
    case CanIDPiper::ARM_INFO_LOW_SPD_FEEDBACK_2:
        return ArmMsgType::LowSpdFeed2;
    case CanIDPiper::ARM_INFO_LOW_SPD_FEEDBACK_3:
        return ArmMsgType::LowSpdFeed3;
    case CanIDPiper::ARM_INFO_LOW_SPD_FEEDBACK_4:
        return ArmMsgType::LowSpdFeed4;
    case CanIDPiper::ARM_INFO_LOW_SPD_FEEDBACK_5:
        return ArmMsgType::LowSpdFeed5;
    case CanIDPiper::ARM_INFO_LOW_SPD_FEEDBACK_6:
        return ArmMsgType::LowSpdFeed6;
    case CanIDPiper::ARM_MOTION_CTRL_1:
        return ArmMsgType::MotionCtrl1;
    case CanIDPiper::ARM_MOTION_CTRL_2:
        return ArmMsgType::MotionCtrl2;
    case CanIDPiper::ARM_MOTION_CTRL_CARTESIAN_1:
        return ArmMsgType::MotionCtrlCartesian1;
    case CanIDPiper::ARM_MOTION_CTRL_CARTESIAN_2:
        return ArmMsgType::MotionCtrlCartesian2;
    case CanIDPiper::ARM_MOTION_CTRL_CARTESIAN_3:
        return ArmMsgType::MotionCtrlCartesian3;
    case CanIDPiper::ARM_JOINT_CTRL_12:
        return ArmMsgType::JointCtrl12;
    case CanIDPiper::ARM_JOINT_CTRL_34:
        return ArmMsgType::JointCtrl34;
    case CanIDPiper::ARM_JOINT_CTRL_56:
        return ArmMsgType::JointCtrl56;
    case CanIDPiper::ARM_CIRCULAR_PATTERN_COORD_NUM_UPDATE_CTRL:
        return ArmMsgType::CircularPatternCoordNumUpdateCtrl;
    case CanIDPiper::ARM_GRIPPER_CTRL:
        return ArmMsgType::GripperCtrl;
    case CanIDPiper::ARM_JOINT_MIT_CTRL_1:
        return ArmMsgType::JointMitCtrl1;
    case CanIDPiper::ARM_JOINT_MIT_CTRL_2:
        return ArmMsgType::JointMitCtrl2;
    case CanIDPiper::ARM_JOINT_MIT_CTRL_3:
        return ArmMsgType::JointMitCtrl3;
    case CanIDPiper::ARM_JOINT_MIT_CTRL_4:
        return ArmMsgType::JointMitCtrl4;
    case CanIDPiper::ARM_JOINT_MIT_CTRL_5:
        return ArmMsgType::JointMitCtrl5;
    case CanIDPiper::ARM_JOINT_MIT_CTRL_6:
        return ArmMsgType::JointMitCtrl6;
    case CanIDPiper::ARM_MASTER_SLAVE_MODE_CONFIG:
        return ArmMsgType::MasterSlaveModeConfig;
    case CanIDPiper::ARM_MOTOR_ENABLE_DISABLE_CONFIG:
        return ArmMsgType::MotorEnableDisableConfig;
    case CanIDPiper::ARM_SEARCH_MOTOR_MAX_SPD_ACC_LIMIT:
        return ArmMsgType::SearchMotorMaxAngleSpdAccLimit;
    case CanIDPiper::ARM_FEEDBACK_CURRENT_MOTOR_ANGLE_LIMIT_MAX_SPD:
        return ArmMsgType::FeedbackCurrentMotorAngleLimitMaxSpd;
    case CanIDPiper::ARM_MOTOR_ANGLE_LIMIT_MAX_SPD_SET:
        return ArmMsgType::MotorAngleLimitMaxSpdSet;
    case CanIDPiper::ARM_JOINT_CONFIG:
        return ArmMsgType::JointConfig;
    case CanIDPiper::ARM_INSTRUCTION_RESPONSE_CONFIG:
        return ArmMsgType::InstructionResponseConfig;
    case CanIDPiper::ARM_PARAM_ENQUIRY_AND_CONFIG:
        return ArmMsgType::ParamEnquiryAndConfig;
    case CanIDPiper::ARM_FEEDBACK_CURRENT_END_VEL_ACC_PARAM:
        return ArmMsgType::FeedbackCurrentEndVelAccParam;
    case CanIDPiper::ARM_END_VEL_ACC_PARAM_CONFIG:
        return ArmMsgType::EndVelAccParamConfig;
    case CanIDPiper::ARM_CRASH_PROTECTION_RATING_CONFIG:
        return ArmMsgType::CrashProtectionRatingConfig;
    case CanIDPiper::ARM_CRASH_PROTECTION_RATING_FEEDBACK:
        return ArmMsgType::CrashProtectionRatingFeedback;
    case CanIDPiper::ARM_FEEDBACK_CURRENT_MOTOR_MAX_ACC_LIMIT:
        return ArmMsgType::FeedbackCurrentMotorMaxAccLimit;
    case CanIDPiper::ARM_GRIPPER_TEACHING_PENDANT_PARAM_CONFIG:
        return ArmMsgType::GripperTeachingPendantParamConfig;
    case CanIDPiper::ARM_GRIPPER_TEACHING_PENDANT_PARAM_FEEDBACK:
        return ArmMsgType::GripperTeachingPendantParamFeedback;
    case CanIDPiper::ARM_FEEDBACK_JOINT_VEL_ACC_1:
        return ArmMsgType::FeedbackJointVelAcc1;
    case CanIDPiper::ARM_FEEDBACK_JOINT_VEL_ACC_2:
        return ArmMsgType::FeedbackJointVelAcc2;
    case CanIDPiper::ARM_FEEDBACK_JOINT_VEL_ACC_3:
        return ArmMsgType::FeedbackJointVelAcc3;
    case CanIDPiper::ARM_FEEDBACK_JOINT_VEL_ACC_4:
        return ArmMsgType::FeedbackJointVelAcc4;
    case CanIDPiper::ARM_FEEDBACK_JOINT_VEL_ACC_5:
        return ArmMsgType::FeedbackJointVelAcc5;
    case CanIDPiper::ARM_FEEDBACK_JOINT_VEL_ACC_6:
        return ArmMsgType::FeedbackJointVelAcc6;
    case CanIDPiper::ARM_LIGHT_CTRL:
        return ArmMsgType::LightCtrl;
    case CanIDPiper::ARM_CAN_UPDATE_SILENT_MODE_CONFIG:
        return ArmMsgType::CanUpdateSilentModeConfig;
    case CanIDPiper::ARM_FIRMWARE_READ:
        return ArmMsgType::FirmwareRead;
    }
    throw std::invalid_argument("Unhandled CanIDPiper in canIdToMsgType");
}

std::string toString(ArmMsgType type)
{
    switch (type)
    {
#define CASE(t)                                                                                                        \
    case ArmMsgType::t:                                                                                                \
        return #t
        CASE(Unknown);
        CASE(StatusFeedback);
        CASE(EndPoseFeedback1);
        CASE(EndPoseFeedback2);
        CASE(EndPoseFeedback3);
        CASE(JointFeedback12);
        CASE(JointFeedback34);
        CASE(JointFeedback56);
        CASE(GripperFeedback);
        CASE(HighSpdFeed1);
        CASE(HighSpdFeed2);
        CASE(HighSpdFeed3);
        CASE(HighSpdFeed4);
        CASE(HighSpdFeed5);
        CASE(HighSpdFeed6);
        CASE(LowSpdFeed1);
        CASE(LowSpdFeed2);
        CASE(LowSpdFeed3);
        CASE(LowSpdFeed4);
        CASE(LowSpdFeed5);
        CASE(LowSpdFeed6);
        CASE(MotionCtrl1);
        CASE(MotionCtrl2);
        CASE(MotionCtrlCartesian1);
        CASE(MotionCtrlCartesian2);
        CASE(MotionCtrlCartesian3);
        CASE(JointCtrl12);
        CASE(JointCtrl34);
        CASE(JointCtrl56);
        CASE(CircularPatternCoordNumUpdateCtrl);
        CASE(GripperCtrl);
        CASE(JointMitCtrl1);
        CASE(JointMitCtrl2);
        CASE(JointMitCtrl3);
        CASE(JointMitCtrl4);
        CASE(JointMitCtrl5);
        CASE(JointMitCtrl6);
        CASE(MasterSlaveModeConfig);
        CASE(MotorEnableDisableConfig);
        CASE(SearchMotorMaxAngleSpdAccLimit);
        CASE(FeedbackCurrentMotorAngleLimitMaxSpd);
        CASE(MotorAngleLimitMaxSpdSet);
        CASE(JointConfig);
        CASE(InstructionResponseConfig);
        CASE(ParamEnquiryAndConfig);
        CASE(FeedbackCurrentEndVelAccParam);
        CASE(EndVelAccParamConfig);
        CASE(CrashProtectionRatingConfig);
        CASE(CrashProtectionRatingFeedback);
        CASE(FeedbackCurrentMotorMaxAccLimit);
        CASE(GripperTeachingPendantParamConfig);
        CASE(GripperTeachingPendantParamFeedback);
        CASE(FeedbackJointVelAcc1);
        CASE(FeedbackJointVelAcc2);
        CASE(FeedbackJointVelAcc3);
        CASE(FeedbackJointVelAcc4);
        CASE(FeedbackJointVelAcc5);
        CASE(FeedbackJointVelAcc6);
        CASE(LightCtrl);
        CASE(CanUpdateSilentModeConfig);
        CASE(FirmwareRead);
#undef CASE
    }
    return "Unknown";
}

std::string toString(CanIDPiper id)
{
    switch (id)
    {
#define CASE(t)                                                                                                        \
    case CanIDPiper::t:                                                                                                \
        return #t
        CASE(ARM_STATUS_FEEDBACK);
        CASE(ARM_END_POSE_FEEDBACK_1);
        CASE(ARM_END_POSE_FEEDBACK_2);
        CASE(ARM_END_POSE_FEEDBACK_3);
        CASE(ARM_JOINT_FEEDBACK_12);
        CASE(ARM_JOINT_FEEDBACK_34);
        CASE(ARM_JOINT_FEEDBACK_56);
        CASE(ARM_GRIPPER_FEEDBACK);
        CASE(ARM_MOTION_CTRL_1);
        CASE(ARM_MOTION_CTRL_2);
        CASE(ARM_MOTION_CTRL_CARTESIAN_1);
        CASE(ARM_MOTION_CTRL_CARTESIAN_2);
        CASE(ARM_MOTION_CTRL_CARTESIAN_3);
        CASE(ARM_JOINT_CTRL_12);
        CASE(ARM_JOINT_CTRL_34);
        CASE(ARM_JOINT_CTRL_56);
        CASE(ARM_CIRCULAR_PATTERN_COORD_NUM_UPDATE_CTRL);
        CASE(ARM_GRIPPER_CTRL);
        CASE(ARM_JOINT_MIT_CTRL_1);
        CASE(ARM_JOINT_MIT_CTRL_2);
        CASE(ARM_JOINT_MIT_CTRL_3);
        CASE(ARM_JOINT_MIT_CTRL_4);
        CASE(ARM_JOINT_MIT_CTRL_5);
        CASE(ARM_JOINT_MIT_CTRL_6);
        CASE(ARM_MASTER_SLAVE_MODE_CONFIG);
        CASE(ARM_MOTOR_ENABLE_DISABLE_CONFIG);
        CASE(ARM_SEARCH_MOTOR_MAX_SPD_ACC_LIMIT);
        CASE(ARM_FEEDBACK_CURRENT_MOTOR_ANGLE_LIMIT_MAX_SPD);
        CASE(ARM_MOTOR_ANGLE_LIMIT_MAX_SPD_SET);
        CASE(ARM_JOINT_CONFIG);
        CASE(ARM_INSTRUCTION_RESPONSE_CONFIG);
        CASE(ARM_PARAM_ENQUIRY_AND_CONFIG);
        CASE(ARM_FEEDBACK_CURRENT_END_VEL_ACC_PARAM);
        CASE(ARM_END_VEL_ACC_PARAM_CONFIG);
        CASE(ARM_CRASH_PROTECTION_RATING_CONFIG);
        CASE(ARM_CRASH_PROTECTION_RATING_FEEDBACK);
        CASE(ARM_FEEDBACK_CURRENT_MOTOR_MAX_ACC_LIMIT);
        CASE(ARM_GRIPPER_TEACHING_PENDANT_PARAM_CONFIG);
        CASE(ARM_GRIPPER_TEACHING_PENDANT_PARAM_FEEDBACK);
        CASE(ARM_FEEDBACK_JOINT_VEL_ACC_1);
        CASE(ARM_FEEDBACK_JOINT_VEL_ACC_2);
        CASE(ARM_FEEDBACK_JOINT_VEL_ACC_3);
        CASE(ARM_FEEDBACK_JOINT_VEL_ACC_4);
        CASE(ARM_FEEDBACK_JOINT_VEL_ACC_5);
        CASE(ARM_FEEDBACK_JOINT_VEL_ACC_6);
        CASE(ARM_LIGHT_CTRL);
        CASE(ARM_INFO_HIGH_SPD_FEEDBACK_1);
        CASE(ARM_INFO_HIGH_SPD_FEEDBACK_2);
        CASE(ARM_INFO_HIGH_SPD_FEEDBACK_3);
        CASE(ARM_INFO_HIGH_SPD_FEEDBACK_4);
        CASE(ARM_INFO_HIGH_SPD_FEEDBACK_5);
        CASE(ARM_INFO_HIGH_SPD_FEEDBACK_6);
        CASE(ARM_INFO_LOW_SPD_FEEDBACK_1);
        CASE(ARM_INFO_LOW_SPD_FEEDBACK_2);
        CASE(ARM_INFO_LOW_SPD_FEEDBACK_3);
        CASE(ARM_INFO_LOW_SPD_FEEDBACK_4);
        CASE(ARM_INFO_LOW_SPD_FEEDBACK_5);
        CASE(ARM_INFO_LOW_SPD_FEEDBACK_6);
        CASE(ARM_CAN_UPDATE_SILENT_MODE_CONFIG);
        CASE(ARM_FIRMWARE_READ);
#undef CASE
    }
    return "Unknown";
}

} // namespace piper_cpp
