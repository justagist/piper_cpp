#include "piper_cpp/protocol/piper_parser_v3.h"
#include "piper_codec.h"
#include <cstring>
#include <initializer_list>
#include <optional>

namespace piper_cpp
{

namespace
{

constexpr uint8_t kMoveMWire = 0x06;
constexpr uint8_t kUnknownMode = 0xFF;
// Bit 2 of the 0x159 status byte selects angle mode; bits 0-1 keep their V2 meaning.
constexpr uint8_t kGripperAngleBit = 0x04;
// The official SDK's V3 gripper command rejects efforts above 5 N*m.
constexpr uint16_t kMaxGripperEffort = 5000;

bool isOneOf(uint8_t value, std::initializer_list<uint8_t> allowed)
{
    for (uint8_t a : allowed)
        if (value == a)
            return true;
    return false;
}

/// V3 moved MOVE M from 0x04 to 0x06 and left 0x04 undefined; every other mode keeps its V2
/// number. Used for both the 0x151 echo and the 0x2A1 mode feedback.
uint8_t moveModeFromWire(uint8_t wire)
{
    if (wire == kMoveMWire)
        return static_cast<uint8_t>(MoveMode::MoveM);
    if (wire == static_cast<uint8_t>(MoveMode::MoveM) || wire > static_cast<uint8_t>(MoveMode::MoveCPV))
        return kUnknownMode;
    return wire;
}

/// Map the semantic 0x151 fields onto V3 wire bytes, rejecting what the official V3 message
/// class rejects. Its changelog also lists ctrl_mode 0x02, 0x05, 0x06 and 0xFF, but the class
/// refuses them, so this does too.
bool motionCtrl2ToWire(ArmMsgMotionCtrl_2 &ctrl)
{
    if (!isOneOf(ctrl.ctrl_mode, {0x00, 0x01, 0x03, 0x04, 0x07}) || ctrl.move_spd_rate_ctrl > 100 ||
        !isOneOf(ctrl.mit_mode, {0x00, 0xAD, 0xFF}) ||
        ctrl.installation_pos > static_cast<uint8_t>(InstallationPos::HorizontalInverted) ||
        ctrl.move_mode > static_cast<uint8_t>(MoveMode::MoveCPV))
        return false;
    if (ctrl.move_mode == static_cast<uint8_t>(MoveMode::MoveM))
        ctrl.move_mode = kMoveMWire;
    return true;
}

/// The V3 status byte for `ctrl`, or nothing for a value the official `GripperCtrl_V3()`
/// refuses. Its message class has these range checks commented out; the interface method
/// enforces them, and so does this.
std::optional<uint8_t> gripperStatusToWire(const ArmMsgGripperCtrl &ctrl)
{
    if (ctrl.status_code > static_cast<uint8_t>(GripperStatusCode::EnableAndClearError) ||
        ctrl.grippers_effort > kMaxGripperEffort || !isOneOf(ctrl.set_zero, {0x00, 0xAE}))
        return std::nullopt;
    switch (ctrl.mode)
    {
    case GripperMode::Width:
        return ctrl.status_code;
    case GripperMode::Angle:
        return ctrl.status_code | kGripperAngleBit;
    default:
        return std::nullopt;
    }
}

} // namespace

bool PiperParserV3::decodeMessage(const struct can_frame &rx_frame, double timestamp, PiperMessage &msg)
{
    switch (rx_frame.can_id)
    {
    case static_cast<canid_t>(CanIDPiper::ARM_IK_JOINT_FEEDBACK_12):
    case static_cast<canid_t>(CanIDPiper::ARM_IK_JOINT_FEEDBACK_34):
    case static_cast<canid_t>(CanIDPiper::ARM_IK_JOINT_FEEDBACK_56):
    {
        if (!acceptFrame(rx_frame, timestamp, msg))
            return false;
        auto &ik = msg.arm_ik_joint_feedback;
        int32_t *pair[] = {&ik.joint_1, &ik.joint_3, &ik.joint_5};
        const size_t part = static_cast<size_t>(msg.type) - static_cast<size_t>(ArmMsgType::IKJointFeedback12);
        pair[part][0] = bytesToInt32(&rx_frame.data[0]);
        pair[part][1] = bytesToInt32(&rx_frame.data[4]);
        return true;
    }
    default:
        break;
    }

    if (!PiperParserV2::decodeMessage(rx_frame, timestamp, msg))
        return false;
    const uint8_t *data = rx_frame.data;
    switch (msg.type)
    {
    case ArmMsgType::StatusFeedback:
        msg.arm_status_msgs.mode_feed = static_cast<ArmMsgFeedbackStatusEnum::ModeFeed>(moveModeFromWire(data[2]));
        break;
    case ArmMsgType::MotionCtrl2:
        msg.arm_motion_ctrl_2.move_mode = moveModeFromWire(data[1]);
        break;
    case ArmMsgType::GripperFeedback:
        // A 7-byte frame carries no mode byte; read it as width, as the SDK does.
        if (rx_frame.can_dlc < 8)
            msg.gripper_feedback.mode = GripperMode::Width;
        else
            msg.gripper_feedback.mode = data[7] == 0x00   ? GripperMode::Width
                                        : data[7] == 0x01 ? GripperMode::Angle
                                                          : GripperMode::Unknown;
        break;
    case ArmMsgType::GripperCtrl:
        if (data[6] <= (static_cast<uint8_t>(GripperStatusCode::EnableAndClearError) | kGripperAngleBit))
        {
            msg.gripper_ctrl.status_code = data[6] & ~kGripperAngleBit;
            msg.gripper_ctrl.mode = (data[6] & kGripperAngleBit) ? GripperMode::Angle : GripperMode::Width;
        }
        else
        {
            msg.gripper_ctrl.mode = GripperMode::Unknown; // status_code keeps the raw byte
        }
        break;
    default:
        break;
    }
    return true;
}

bool PiperParserV3::encodeMessage(const PiperMessage &msg, struct can_frame &tx_frame)
{
    if (msg.type == ArmMsgType::MotionCtrl2)
    {
        ArmMsgMotionCtrl_2 wire = msg.arm_motion_ctrl_2;
        if (!motionCtrl2ToWire(wire))
            return false;
        std::memset(tx_frame.data, 0, 8);
        writeMotionCtrl2(wire, tx_frame.data);
    }
    else if (msg.type == ArmMsgType::GripperCtrl)
    {
        const auto status = gripperStatusToWire(msg.gripper_ctrl);
        if (!status)
            return false;
        writeGripperCtrl(msg.gripper_ctrl, *status, tx_frame.data);
    }
    else if (isJointMitCtrl(msg.type))
    {
        const size_t joint_idx = static_cast<size_t>(msg.type) - static_cast<size_t>(ArmMsgType::JointMitCtrl1);
        const ArmMsgJointMitCtrl &mit = msg.arm_joint_mit_ctrl.motors[joint_idx];
        writeMitGains(mit, tx_frame.data);
        tx_frame.data[6] |= (mit.t_ref >> 8) & 0x0F;
        tx_frame.data[7] = mit.t_ref & 0xFF;
    }
    else
    {
        return PiperParserV2::encodeMessage(msg, tx_frame);
    }
    setCommandHeader(msg.type, tx_frame);
    return true;
}

ArmMsgJointMitCtrl PiperParserV3::packJointMit(double pos_rad, double vel, double kp, double kd, double torque) const
{
    constexpr auto q = Quantisation::Truncate;
    ArmMsgJointMitCtrl mit;
    mit.pos_ref = static_cast<uint16_t>(floatToUintRange(pos_rad, -12.5, 12.5, 16, q));
    mit.vel_ref = static_cast<uint16_t>(floatToUintRange(vel, -45.0, 45.0, 12, q));
    mit.kp = static_cast<uint16_t>(floatToUintRange(kp, 0.0, 500.0, 12, q));
    mit.kd = static_cast<uint16_t>(floatToUintRange(kd, -5.0, 5.0, 12, q));
    mit.t_ref = static_cast<uint16_t>(floatToUintRange(torque, -16.0, 16.0, 12, q));
    mit.crc = 0;
    return mit;
}

} // namespace piper_cpp
