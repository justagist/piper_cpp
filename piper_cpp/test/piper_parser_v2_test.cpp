// Golden-frame tests for the V2 wire protocol: what the decoder accepts, how it interprets
// signed fields, and that the encoder produces complete frames on its own.

#include "piper_cpp/protocol/piper_parser_v2.h"

#include <gtest/gtest.h>

#include <algorithm>
#include <initializer_list>
#include <linux/can.h>

using piper_cpp::ArmMsgType;
using piper_cpp::PiperMessage;
using piper_cpp::PiperParserV2;

namespace
{

can_frame frame(uint32_t id, std::initializer_list<uint8_t> bytes)
{
    can_frame f{};
    f.can_id = id;
    f.can_dlc = static_cast<uint8_t>(bytes.size());
    std::copy(bytes.begin(), bytes.end(), f.data);
    return f;
}

} // namespace

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
    EXPECT_EQ(tx.data[3], 0xE8); // 1000 = 0x000003E8, big-endian
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
