// Golden-frame tests for the V2 wire protocol: what the decoder accepts, how it interprets
// signed fields, and that the encoder produces complete frames on its own.

#include "piper_cpp/protocol/piper_parser_v2.h"
#include "test_support.h"

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <linux/can.h>

using piper_cpp::ArmMsgType;
using piper_cpp::PiperMessage;
using piper_cpp::PiperParserV2;
using test_support::frame;

TEST(PiperParserV2, RejectsFramesThatAreNotStandardData)
{
    PiperParserV2 parser;
    PiperMessage msg;
    const auto joints = frame(0x2A5, {0, 0, 0, 1, 0, 0, 0, 2});
    ASSERT_TRUE(parser.decodeMessage(joints, 0.0, msg));

    // Flags, and a reserved bit between the 11-bit ID and the flags, which masking would hide.
    for (canid_t bit : {CAN_RTR_FLAG, CAN_EFF_FLAG, CAN_ERR_FLAG, canid_t{0x800}})
    {
        can_frame f = joints;
        f.can_id |= bit;
        EXPECT_FALSE(parser.decodeMessage(f, 0.0, msg)) << "bit 0x" << std::hex << bit;
        EXPECT_EQ(msg.type, ArmMsgType::Unknown);
    }
}

TEST(PiperParserV2, RejectsDlcAboveClassicFrameSize)
{
    PiperParserV2 parser;
    PiperMessage msg;
    // Firmware chunks are copied by DLC, so an oversize DLC would read past the buffer.
    auto f = frame(0x4AF, {'S', '-', 'V', '1', '.', '5', '-', '2'});
    f.can_dlc = 9;
    EXPECT_FALSE(parser.decodeMessage(f, 0.0, msg));
}

TEST(PiperParserV2, RejectsPayloadShorterThanTheFieldsRead)
{
    PiperParserV2 parser;
    PiperMessage msg;

    // Joint feedback reads all eight bytes.
    EXPECT_FALSE(parser.decodeMessage(frame(0x2A5, {}), 0.0, msg));
    EXPECT_FALSE(parser.decodeMessage(frame(0x2A5, {0, 0, 0, 1, 0, 0, 0}), 0.0, msg));
    EXPECT_TRUE(parser.decodeMessage(frame(0x2A5, {0, 0, 0, 1, 0, 0, 0, 2}), 0.0, msg));

    // Gripper feedback reads seven, so a seven-byte frame is complete and six is not.
    EXPECT_FALSE(parser.decodeMessage(frame(0x2A8, {0, 0, 0, 1, 0, 0}), 0.0, msg));
    EXPECT_TRUE(parser.decodeMessage(frame(0x2A8, {0, 0, 0, 1, 0, 0, 0}), 0.0, msg));

    // Firmware text arrives in chunks of any length; only the bytes on the bus are kept, in
    // both the typed field and the raw copy.
    msg.raw_data[3] = 0xAA; // stale byte from a previous decode of the same message object
    ASSERT_TRUE(parser.decodeMessage(frame(0x4AF, {'S', '-', 'V'}), 0.0, msg));
    EXPECT_EQ(msg.firmware_data, (std::vector<uint8_t>{'S', '-', 'V'}));
    EXPECT_EQ(msg.raw_data[2], 'V');
    EXPECT_EQ(msg.raw_data[3], 0);
}

TEST(PiperParserV2, MotorCurrentIsSigned)
{
    PiperParserV2 parser;
    PiperMessage msg;
    // Joint 1 high-speed feedback: speed 0, current -100 mA (0xFF9C), position 0.
    ASSERT_TRUE(parser.decodeMessage(frame(0x251, {0, 0, 0xFF, 0x9C, 0, 0, 0, 0}), 0.0, msg));
    EXPECT_EQ(msg.high_spd_feedbacks[0].current, -100);
    EXPECT_LT(msg.high_spd_feedbacks[0].effort, 0.0f) << "derived torque must keep the sign of the current";
}

TEST(PiperParserV2, EncoderDerivesIdAndDlcFromType)
{
    PiperParserV2 parser;
    PiperMessage msg;
    msg.type = ArmMsgType::JointCtrl12;
    msg.arm_joint_ctrl.joint_1 = 1000;

    can_frame tx{};
    ASSERT_TRUE(parser.encodeMessage(msg, tx));
    EXPECT_EQ(tx.can_id, 0x155u);
    EXPECT_EQ(tx.can_dlc, 8);
    EXPECT_EQ(tx.data[0], 0x00); // 1000 = 0x000003E8, big-endian
    EXPECT_EQ(tx.data[1], 0x00);
    EXPECT_EQ(tx.data[2], 0x03);
    EXPECT_EQ(tx.data[3], 0xE8);
}

TEST(PiperParserV2, MotionCtrl2RoundTripsInstallationPos)
{
    PiperParserV2 parser;
    PiperMessage msg;
    msg.type = ArmMsgType::MotionCtrl2;
    msg.arm_motion_ctrl_2.move_spd_rate_ctrl = 30;
    msg.arm_motion_ctrl_2.installation_pos = 3;

    can_frame tx{};
    ASSERT_TRUE(parser.encodeMessage(msg, tx));
    EXPECT_EQ(tx.can_id, 0x151u);

    PiperMessage back;
    ASSERT_TRUE(parser.decodeMessage(tx, 0.0, back));
    EXPECT_EQ(back.arm_motion_ctrl_2.move_spd_rate_ctrl, 30);
    EXPECT_EQ(back.arm_motion_ctrl_2.installation_pos, 3);
}

// V2 MIT packing as it was before V3 existed: round to nearest, 8-bit torque over [-8, 8] and a
// CRC nibble. Code = round((x - min) * (2^bits - 1) / (max - min)), worked per field:
//   pos 0 -> 32767.5 -> 0x8000   vel 0 -> 2047.5 -> 0x800   kp 10 -> 81.9 -> 0x052
//   kd 0.8 -> 2375.1 -> 0x947    torque 0 -> 127.5 -> 0x80   torque -4 -> 63.75 -> 0x40
// Byte 6 is kd[3:0]|t[7:4], byte 7 is t[3:0]|crc, crc = (byte0 ^ ... ^ byte6) & 0xF.
TEST(PiperParserV2, MitPackingIsUnchangedByV3)
{
    PiperParserV2 parser;
    struct Case
    {
        double torque;
        std::array<uint8_t, 8> expected;
    };
    const Case cases[] = {
        {0.0,  {0x80, 0x00, 0x80, 0x00, 0x52, 0x94, 0x78, 0x0E}}, // crc 80^00^80^00^52^94^78 = BE
        {-4.0, {0x80, 0x00, 0x80, 0x00, 0x52, 0x94, 0x74, 0x02}}, // crc ... ^74 = B2
    };
    for (int joint = 1; joint <= 6; ++joint)
    {
        for (const auto& c : cases)
        {
            PiperMessage msg;
            msg.type = static_cast<ArmMsgType>(static_cast<int>(ArmMsgType::JointMitCtrl1) + joint - 1);
            msg.arm_joint_mit_ctrl.motors[joint - 1] = parser.packJointMit(0.0, 0.0, 10.0, 0.8, c.torque);
            can_frame tx{};
            ASSERT_TRUE(parser.encodeMessage(msg, tx));
            EXPECT_EQ(tx.can_id, 0x159u + joint);
            EXPECT_TRUE(std::equal(c.expected.begin(), c.expected.end(), tx.data)) << "joint " << joint;
        }
    }
    // V2 saturates torque at +-8, so 16 encodes as 8 would.
    EXPECT_EQ(parser.packJointMit(0, 0, 10, 0.8, 16.0).t_ref, 0xFF);
}

TEST(PiperParserV2, KeepsV2ModeNumbersAndRefusesV3OnlyValues)
{
    PiperParserV2 parser;
    PiperMessage msg;
    can_frame tx{};
    msg.type = ArmMsgType::MotionCtrl2;
    msg.arm_motion_ctrl_2.move_mode = static_cast<uint8_t>(piper_cpp::MoveMode::MoveM);
    ASSERT_TRUE(parser.encodeMessage(msg, tx));
    EXPECT_EQ(tx.data[1], 0x04) << "V2 MOVE M stays 0x04";

    msg.arm_motion_ctrl_2.installation_pos = static_cast<uint8_t>(piper_cpp::InstallationPos::HorizontalInverted);
    EXPECT_FALSE(parser.encodeMessage(msg, tx)) << "0x04 is an invalid installation position in V2";

    msg.type = ArmMsgType::GripperCtrl;
    msg.gripper_ctrl.mode = piper_cpp::GripperMode::Angle;
    EXPECT_FALSE(parser.encodeMessage(msg, tx)) << "V2 would read an angle as a width";

    // Byte 7 is not a mode under V2, whatever it holds.
    ASSERT_TRUE(parser.decodeMessage(frame(0x2A8, {0, 0, 0x75, 0x30, 0, 0, 0, 0x01}), 0.0, msg));
    EXPECT_EQ(msg.gripper_feedback.mode, piper_cpp::GripperMode::Width);

    // Feedback bytes stay as received: V2 has no MOVE M at 0x06 and no IK feedback.
    ASSERT_TRUE(parser.decodeMessage(frame(0x2A1, {0x01, 0x00, 0x06, 0, 0, 0, 0, 0}), 0.0, msg));
    EXPECT_EQ(static_cast<uint8_t>(msg.arm_status_msgs.mode_feed), 0x06);
    EXPECT_FALSE(parser.decodeMessage(frame(0x2AA, {0, 0, 0, 1, 0, 0, 0, 2}), 0.0, msg));
    EXPECT_EQ(msg.type, ArmMsgType::Unknown);
}
