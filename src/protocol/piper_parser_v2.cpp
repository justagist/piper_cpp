#include "piper_cpp/protocol/piper_parser_v2.h"
#include <cstring>

namespace piper_cpp
{

static int32_t bytesToInt32(const uint8_t* data)
{
    int32_t result;
    std::memcpy(&result, data, sizeof(int32_t));
    return __builtin_bswap32(result); // For big-endian like in Python
}

static int16_t bytesToInt16(const uint8_t* data)
{
    int16_t result;
    std::memcpy(&result, data, sizeof(int16_t));
    return __builtin_bswap16(result);
}

bool PiperParserV2::decodeMessage(const struct can_frame& rx_frame, double timestamp, PiperMessage& msg)
{
    uint32_t can_id = rx_frame.can_id & CAN_EFF_MASK;
    const uint8_t* data = rx_frame.data;

    // Map CAN ID to ArmMsgType using your generated switch/case functions
    try
    {
        auto can_enum_id = static_cast<CanIDPiper>(can_id);
        msg.can_id = can_id;
        msg.type = canIdToMsgType(can_enum_id);
    }
    catch (...)
    {
        msg.type = ArmMsgType::Unknown;
        return false;
    }

    msg.raw_data[0] = data[0];
    msg.raw_data[1] = data[1];
    msg.raw_data[2] = data[2];
    msg.raw_data[3] = data[3];
    msg.raw_data[4] = data[4];
    msg.raw_data[5] = data[5];
    msg.raw_data[6] = data[6];
    msg.raw_data[7] = data[7];

    msg.timestamp = timestamp; // <<--- Set timestamp here!

    /// TODO: Fill PiperMessage fields based on message type
    switch (msg.type)
    {
    case ArmMsgType::StatusFeedback:
    {
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
    // case ArmMsgType::EndPoseFeedback1:
    //     msg.arm_end_pose.X_axis = bytesToInt32(&data[0]);
    //     msg.arm_end_pose.Y_axis = bytesToInt32(&data[4]);
    //     break;
    // case ArmMsgType::EndPoseFeedback2:
    //     msg.arm_end_pose.Z_axis = bytesToInt32(&data[0]);
    //     msg.arm_end_pose.RX_axis = bytesToInt32(&data[4]);
    //     break;
    // case ArmMsgType::EndPoseFeedback3:
    //     msg.arm_end_pose.RY_axis = bytesToInt32(&data[0]);
    //     msg.arm_end_pose.RZ_axis = bytesToInt32(&data[4]);
    //     break;
    // case ArmMsgType::JointFeedback12:
    //     msg.arm_joint_feedback.joint_1 = bytesToInt32(&data[0]);
    //     msg.arm_joint_feedback.joint_2 = bytesToInt32(&data[4]);
    //     break;
    // case ArmMsgType::JointFeedback34:
    //     msg.arm_joint_feedback.joint_3 = bytesToInt32(&data[0]);
    //     msg.arm_joint_feedback.joint_4 = bytesToInt32(&data[4]);
    //     break;
    // case ArmMsgType::JointFeedback56:
    //     msg.arm_joint_feedback.joint_5 = bytesToInt32(&data[0]);
    //     msg.arm_joint_feedback.joint_6 = bytesToInt32(&data[4]);
    //     break;
    // case ArmMsgType::GripperFeedback:
    //     msg.gripper_feedback.grippers_angle = bytesToInt32(&data[0]);
    //     msg.gripper_feedback.grippers_effort = bytesToInt16(&data[4]);
    //     msg.gripper_feedback.status_code = data[6];
    //     break;
    default:
        /// TODO: once all cases are handled, remove this and return false
        return true;
    }

    return true;
}

} // namespace piper_cpp
