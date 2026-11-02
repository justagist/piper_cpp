// Golden-frame tests for the V3 wire protocol. Expected bytes are worked out by hand from the
// V3 frame tables (assets/V3/CHANGELOG-V3.MD in piper_sdk), independently of both this
// library's encoder and the Python SDK, so a mistake shared by the two cannot hide here.

#include "piper_cpp/protocol/piper_parser_v3.h"
#include "test_support.h"

#include <gtest/gtest.h>

#include <array>
#include <linux/can.h>

using piper_cpp::ArmMsgType;
using piper_cpp::GripperMode;
using piper_cpp::PiperMessage;
using piper_cpp::PiperParserV2;
using piper_cpp::PiperParserV3;
using test_support::frame;
using test_support::jointPairFrame;
using ModeFeed = piper_cpp::ArmMsgFeedbackStatusEnum::ModeFeed;
using Bytes = std::array<uint8_t, 8>;

namespace
{

Bytes payload(const can_frame& f)
{
    Bytes b{};
    std::copy(f.data, f.data + 8, b.begin());
    return b;
}

can_frame encodeMit(PiperParserV3& parser, int joint, double pos, double vel, double kp, double kd, double torque)
{
    PiperMessage msg;
    msg.type = static_cast<ArmMsgType>(static_cast<int>(ArmMsgType::JointMitCtrl1) + joint - 1);
    msg.arm_joint_mit_ctrl.motors[joint - 1] = parser.packJointMit(pos, vel, kp, kd, torque);
    can_frame tx{};
    EXPECT_TRUE(parser.encodeMessage(msg, tx));
    return tx;
}

can_frame encodeMotionCtrl2(PiperParserV3& parser, uint8_t ctrl, uint8_t move, uint8_t spd, uint8_t install, bool& ok)
{
    PiperMessage msg;
    msg.type = ArmMsgType::MotionCtrl2;
    msg.arm_motion_ctrl_2 = ArmMsgMotionCtrl_2{ctrl, move, spd, 0x00, 0, install};
    can_frame tx{};
    ok = parser.encodeMessage(msg, tx);
    return tx;
}

} // namespace

// Code = trunc((x - min) * (2^bits - 1) / (max - min)). Worked per field:
//   pos 0     -> 12.5 * 65535 / 25 = 32767.5 -> 0x7FFF     pos -1 -> 11.5 * 65535 / 25 = 30146.1 -> 0x75C2
//   vel 0     -> 45 * 4095 / 90    = 2047.5  -> 0x7FF      vel 22.5 -> 67.5 * 4095 / 90 = 3071.25 -> 0xBFF
//   kp 10     -> 10 * 4095 / 500   = 81.9    -> 0x051      kd 0.8 -> 5.8 * 4095 / 10    = 2375.1  -> 0x947
//   torque 0  -> 16 * 4095 / 32    = 2047.5  -> 0x7FF      torque -8 -> 8 * 4095 / 32  = 1023.75 -> 0x3FF
//   torque 1  -> 17 * 4095 / 32    = 2175.47 -> 0x87F
// Layout: pos[15:8] pos[7:0] vel[11:4] vel[3:0]|kp[11:8] kp[7:0] kd[11:4] kd[3:0]|t[11:8] t[7:0].
TEST(PiperParserV3, MitPacksHandComputedCodesOnEveryJoint)
{
    PiperParserV3 parser;
    struct Case
    {
        double pos, vel, kp, kd, torque;
        Bytes expected;
    };
    const Case cases[] = {
        {0.0,   0.0,   10.0,  0.8,  0.0,   {0x7F, 0xFF, 0x7F, 0xF0, 0x51, 0x94, 0x77, 0xFF}},
        {0.0,   0.0,   10.0,  0.8,  -8.0,  {0x7F, 0xFF, 0x7F, 0xF0, 0x51, 0x94, 0x73, 0xFF}},
        {0.0,   0.0,   10.0,  0.8,  1.0,   {0x7F, 0xFF, 0x7F, 0xF0, 0x51, 0x94, 0x78, 0x7F}},
        {-1.0,  22.5,  10.0,  0.8,  0.0,   {0x75, 0xC2, 0xBF, 0xF0, 0x51, 0x94, 0x77, 0xFF}},
        {12.5,  45.0,  500.0, 5.0,  16.0,  {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}},
        {-12.5, -45.0, 0.0,   -5.0, -16.0, {0, 0, 0, 0, 0, 0, 0, 0}                        },
        // Out of range saturates at the ends instead of wrapping into the next field.
        {13.0,  50.0,  600.0, 6.0,  17.0,  {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}},
        {-13.0, -50.0, -1.0,  -6.0, -17.0, {0, 0, 0, 0, 0, 0, 0, 0}                        },
    };
    for (int joint = 1; joint <= 6; ++joint)
    {
        for (const auto& c : cases)
        {
            const auto tx = encodeMit(parser, joint, c.pos, c.vel, c.kp, c.kd, c.torque);
            EXPECT_EQ(tx.can_id, 0x159u + joint);
            EXPECT_EQ(tx.can_dlc, 8);
            EXPECT_EQ(payload(tx), c.expected) << "joint " << joint << " torque " << c.torque;
        }
    }
}

TEST(PiperParserV3, MitTorqueUsesTwelveBitsAndNoCrc)
{
    PiperParserV3 parser;
    PiperMessage msg;
    msg.type = ArmMsgType::JointMitCtrl1;
    msg.arm_joint_mit_ctrl.motors[0] = {0x1234, 0xABC, 0xDEF, 0x345, 0x678, 0};
    can_frame tx{};
    ASSERT_TRUE(parser.encodeMessage(msg, tx));
    EXPECT_EQ(payload(tx), (Bytes{0x12, 0x34, 0xAB, 0xCD, 0xEF, 0x34, 0x56, 0x78}));
}

TEST(PiperParserV3, MotionCtrl2SendsV3ModeNumbers)
{
    PiperParserV3 parser;
    bool ok = false;
    // MOVE M is 0x04 as a MoveMode but 0x06 on the V3 wire; the other modes keep their numbers.
    auto tx = encodeMotionCtrl2(parser, 0x01, 0x04, 100, 0x00, ok);
    ASSERT_TRUE(ok);
    EXPECT_EQ(tx.can_id, 0x151u);
    EXPECT_EQ(payload(tx), (Bytes{0x01, 0x06, 0x64, 0x00, 0x00, 0x00, 0x00, 0x00}));
    for (uint8_t mode : {0x00, 0x01, 0x02, 0x03, 0x05})
    {
        tx = encodeMotionCtrl2(parser, 0x01, mode, 50, 0x00, ok);
        ASSERT_TRUE(ok);
        EXPECT_EQ(tx.data[1], mode);
    }
    tx = encodeMotionCtrl2(parser, 0x01, 0x01, 50, 0x04, ok);
    ASSERT_TRUE(ok) << "horizontal inverted is a V3 installation position";
    EXPECT_EQ(tx.data[5], 0x04);
}

TEST(PiperParserV3, MotionCtrl2RejectsValuesV3DoesNotDefine)
{
    PiperParserV3 parser;
    bool ok = true;
    encodeMotionCtrl2(parser, 0x01, 0x06, 50, 0x00, ok);
    EXPECT_FALSE(ok) << "0x06 is a wire byte, not a MoveMode";
    encodeMotionCtrl2(parser, 0x01, 0x01, 50, 0x05, ok);
    EXPECT_FALSE(ok) << "installation position";
    encodeMotionCtrl2(parser, 0x02, 0x01, 50, 0x00, ok);
    EXPECT_FALSE(ok) << "control mode";
    encodeMotionCtrl2(parser, 0x01, 0x01, 101, 0x00, ok);
    EXPECT_FALSE(ok) << "speed percentage";
}

TEST(PiperParserV3, GripperCommandCarriesModeInStatusBit2)
{
    PiperParserV3 parser;
    PiperMessage msg;
    msg.type = ArmMsgType::GripperCtrl;
    // -45 deg = -45000 = 0xFFFF5038; effort 1000 = 0x03E8; Enable (0x01) | angle bit (0x04).
    msg.gripper_ctrl = ArmMsgGripperCtrl{-45000, 1000, 0x01, 0x00, GripperMode::Angle};
    can_frame tx{};
    ASSERT_TRUE(parser.encodeMessage(msg, tx));
    EXPECT_EQ(tx.can_id, 0x159u);
    EXPECT_EQ(payload(tx), (Bytes{0xFF, 0xFF, 0x50, 0x38, 0x03, 0xE8, 0x05, 0x00}));

    msg.gripper_ctrl = ArmMsgGripperCtrl{30000, 1000, 0x03, 0x00, GripperMode::Width};
    ASSERT_TRUE(parser.encodeMessage(msg, tx));
    EXPECT_EQ(payload(tx), (Bytes{0x00, 0x00, 0x75, 0x30, 0x03, 0xE8, 0x03, 0x00}));

    msg.gripper_ctrl = ArmMsgGripperCtrl{0, 0, 0x04, 0x00, GripperMode::Width};
    EXPECT_FALSE(parser.encodeMessage(msg, tx)) << "the mode belongs in `mode`, not the status code";
    msg.gripper_ctrl = ArmMsgGripperCtrl{0, 0, 0x01, 0x00, GripperMode::Unknown};
    EXPECT_FALSE(parser.encodeMessage(msg, tx));
}

TEST(PiperParserV3, StatusModeFeedbackMapsToProtocolIndependentModes)
{
    PiperParserV3 parser;
    PiperMessage msg;
    auto modeFeed = [&](uint8_t wire)
    {
        EXPECT_TRUE(parser.decodeMessage(frame(0x2A1, {0x01, 0x00, wire, 0, 0, 0, 0, 0}), 0.0, msg));
        return msg.arm_status_msgs.mode_feed;
    };
    EXPECT_EQ(modeFeed(0x06), ModeFeed::MOVE_M);
    EXPECT_EQ(modeFeed(0x05), ModeFeed::MOVE_CPV);
    EXPECT_EQ(modeFeed(0x00), ModeFeed::MOVE_P);
    EXPECT_EQ(modeFeed(0x04), ModeFeed::UNKNOWN) << "0x04 is not MOVE M under V3";
    EXPECT_EQ(modeFeed(0x07), ModeFeed::UNKNOWN);
}

TEST(PiperParserV3, MotionCtrl2EchoReadsBackAsMoveMode)
{
    PiperParserV3 parser;
    PiperMessage msg;
    ASSERT_TRUE(parser.decodeMessage(frame(0x151, {0x01, 0x06, 0x32, 0xAD, 0x00, 0x04, 0, 0}), 0.0, msg));
    EXPECT_EQ(msg.arm_motion_ctrl_2.move_mode, static_cast<uint8_t>(piper_cpp::MoveMode::MoveM));
    EXPECT_EQ(msg.arm_motion_ctrl_2.installation_pos, 0x04);
}

TEST(PiperParserV3, GripperFeedbackReportsMode)
{
    PiperParserV3 parser;
    PiperMessage msg;
    // 30000 (0x7530), effort -1000 (0xFC18), status 0xC1: enabled, homed, low voltage.
    ASSERT_TRUE(parser.decodeMessage(frame(0x2A8, {0, 0, 0x75, 0x30, 0xFC, 0x18, 0xC1, 0x01}), 0.0, msg));
    EXPECT_EQ(msg.gripper_feedback.grippers_angle, 30000);
    EXPECT_EQ(msg.gripper_feedback.grippers_effort, -1000);
    EXPECT_TRUE(msg.gripper_feedback.foc_status.homing_status);
    EXPECT_EQ(msg.gripper_feedback.mode, GripperMode::Angle);

    ASSERT_TRUE(parser.decodeMessage(frame(0x2A8, {0, 0, 0x75, 0x30, 0, 0, 0, 0x00}), 0.0, msg));
    EXPECT_EQ(msg.gripper_feedback.mode, GripperMode::Width);
    ASSERT_TRUE(parser.decodeMessage(frame(0x2A8, {0, 0, 0x75, 0x30, 0, 0, 0, 0x02}), 0.0, msg));
    EXPECT_EQ(msg.gripper_feedback.mode, GripperMode::Unknown);

    // A seven-byte frame has no mode byte; the SDK reads it as width, and so does this parser.
    msg.gripper_feedback.mode = GripperMode::Angle;
    ASSERT_TRUE(parser.decodeMessage(frame(0x2A8, {0, 0, 0x75, 0x30, 0, 0, 0}), 0.0, msg));
    EXPECT_EQ(msg.gripper_feedback.mode, GripperMode::Width);
}

TEST(PiperParserV3, GripperCommandEchoSplitsStatusAndMode)
{
    PiperParserV3 parser;
    PiperMessage msg;
    ASSERT_TRUE(parser.decodeMessage(frame(0x159, {0, 0, 0x30, 0x39, 0, 0, 0x07, 0x00}), 0.0, msg));
    EXPECT_EQ(msg.gripper_ctrl.status_code, 0x03);
    EXPECT_EQ(msg.gripper_ctrl.mode, GripperMode::Angle);

    ASSERT_TRUE(parser.decodeMessage(frame(0x159, {0, 0, 0x30, 0x39, 0, 0, 0x08, 0x00}), 0.0, msg));
    EXPECT_EQ(msg.gripper_ctrl.mode, GripperMode::Unknown);
    EXPECT_EQ(msg.gripper_ctrl.status_code, 0x08) << "an undefined status byte is kept as received";
}

TEST(PiperParserV3, IkJointFeedbackDecodesSignedPairs)
{
    PiperParserV3 parser;
    PiperMessage msg;
    ASSERT_TRUE(parser.decodeMessage(jointPairFrame(0x2AA, 10000, -20000), 1.5, msg));
    EXPECT_EQ(msg.type, ArmMsgType::IKJointFeedback12);
    EXPECT_EQ(msg.arm_ik_joint_feedback.joint_1, 10000);
    EXPECT_EQ(msg.arm_ik_joint_feedback.joint_2, -20000);
    EXPECT_DOUBLE_EQ(msg.timestamp, 1.5);
    ASSERT_TRUE(parser.decodeMessage(jointPairFrame(0x2AB, INT32_MIN, INT32_MAX), 0.0, msg));
    EXPECT_EQ(msg.arm_ik_joint_feedback.joint_3, INT32_MIN);
    EXPECT_EQ(msg.arm_ik_joint_feedback.joint_4, INT32_MAX);
    ASSERT_TRUE(parser.decodeMessage(jointPairFrame(0x2AC, 0, -1), 0.0, msg));
    EXPECT_EQ(msg.type, ArmMsgType::IKJointFeedback56);
    EXPECT_EQ(msg.arm_ik_joint_feedback.joint_6, -1);
}

TEST(PiperParserV3, RejectsMalformedIkFrames)
{
    PiperParserV3 parser;
    PiperMessage msg;
    auto shortFrame = jointPairFrame(0x2AA, 1, 2);
    shortFrame.can_dlc = 7;
    EXPECT_FALSE(parser.decodeMessage(shortFrame, 0.0, msg));
    auto extended = jointPairFrame(0x2AA, 1, 2);
    extended.can_id |= CAN_EFF_FLAG;
    EXPECT_FALSE(parser.decodeMessage(extended, 0.0, msg));
    EXPECT_EQ(msg.type, ArmMsgType::Unknown);
}

TEST(PiperParserV3, UnchangedMessagesMatchV2)
{
    PiperParserV2 v2;
    PiperParserV3 v3;
    PiperMessage msg;
    msg.type = ArmMsgType::JointCtrl34;
    msg.arm_joint_ctrl = {1, 2, -30000, 40000, 5, 6};
    can_frame a{}, b{};
    ASSERT_TRUE(v2.encodeMessage(msg, a));
    ASSERT_TRUE(v3.encodeMessage(msg, b));
    EXPECT_EQ(a.can_id, b.can_id);
    EXPECT_EQ(payload(a), payload(b));

    PiperMessage fa, fb;
    const auto feedback = frame(0x251, {0, 0x10, 0xFF, 0x9C, 0, 0, 0x01, 0x00});
    ASSERT_TRUE(v2.decodeMessage(feedback, 0.0, fa));
    ASSERT_TRUE(v3.decodeMessage(feedback, 0.0, fb));
    EXPECT_EQ(fa.high_spd_feedbacks[0].current, fb.high_spd_feedbacks[0].current);
    EXPECT_EQ(fa.high_spd_feedbacks[0].pos, fb.high_spd_feedbacks[0].pos);
}
