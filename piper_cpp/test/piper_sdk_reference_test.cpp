// Differential tests against the official Python piper_sdk. sdk_reference/sdk_reference_frames.h
// holds the frames the SDK (pinned commit, see kSdkCommit) sends for a set of public command
// calls and what it decodes from a set of feedback frames; these tests make the same calls
// through this library's public API and compare CAN ID, payload length and payload bytes.
//
// Not compared, on purpose: V2 MIT through the interface (V2 here rounds to nearest where the
// SDK truncates, and clamps where the SDK does not), and V2 0x151 values the SDK rejects but
// V2 here has always passed through. Both keep V2's existing wire behaviour.

#include "piper_cpp/interface/piper_interface_v3.h"
#include "piper_cpp/protocol/piper_parser_v3.h"
#include "sdk_reference/sdk_reference_frames.h"
#include "test_support.h"

#include <gtest/gtest.h>

#include <memory>
#include <sstream>
#include <string>

using namespace piper_cpp;
using test_support::makeBus;
using test_support::receiveFrame;

namespace
{

constexpr auto kQuiet = std::chrono::milliseconds(20);

std::string describe(const sdk_reference::CommandCase& c)
{
    std::ostringstream oss;
    oss << "V" << c.protocol << " " << c.api << "(";
    for (double a : c.args)
        oss << a << ",";
    oss << ")";
    return oss.str();
}

/// The SDK's V3 move modes (0x151 arguments and 0x2A1 feedback) use V3 wire numbers; this
/// library uses protocol-independent modes, where MOVE M is 0x04 in both versions.
uint8_t modeFromSdk(int protocol, int64_t sdk_value)
{
    if (protocol == 3 && sdk_value == 0x06)
        return static_cast<uint8_t>(MoveMode::MoveM);
    return static_cast<uint8_t>(sdk_value);
}

/// Make the call `c` names through the public API of `piper`.
bool call(PiperInterfaceV2& piper, const sdk_reference::CommandCase& c)
{
    const double* a = c.args;
    auto u8 = [&](int i) { return static_cast<uint8_t>(a[i]); };
    auto i16 = [&](int i) { return static_cast<int16_t>(a[i]); };
    auto i32 = [&](int i) { return static_cast<int32_t>(a[i]); };
    const std::string api = c.api;

    if (api == "controlJointMit")
        return piper.controlJointMit(u8(0), a[1], a[2], a[3], a[4], a[5]);
    if (api == "sendMotionCtrl2")
        return piper.sendMotionCtrl2(
            static_cast<CtrlMode>(u8(0)), static_cast<MoveMode>(modeFromSdk(c.protocol, u8(1))), u8(2),
            static_cast<MitMode>(u8(3)), u8(4), static_cast<InstallationPos>(u8(5))
        );
    if (api == "controlGripper")
        return piper.controlGripper(i32(0), static_cast<uint16_t>(a[1]), static_cast<GripperStatusCode>(u8(2)));
    if (api == "setGripperZero")
        return piper.setGripperZero();
    if (api == "controlGripperV3")
    {
        // The SDK packs the mode into the status code; here they are separate.
        auto& v3 = dynamic_cast<PiperInterfaceV3&>(piper);
        const auto status = static_cast<GripperStatusCode>(u8(2) & 0x03);
        return (u8(2) & 0x04) ? v3.controlGripperAngle(i32(0), static_cast<uint16_t>(a[1]), status)
                              : v3.controlGripperWidth(i32(0), static_cast<uint16_t>(a[1]), status);
    }
    if (api == "controlJoints")
        return piper.controlJoints(i32(0), i32(1), i32(2), i32(3), i32(4), i32(5));
    if (api == "controlEndPose")
        return piper.controlEndPose(i32(0), i32(1), i32(2), i32(3), i32(4), i32(5));
    if (api == "enableMotor")
        return piper.enableMotor(static_cast<MotorIndex>(u8(0)));
    if (api == "disableMotor")
        return piper.disableMotor(static_cast<MotorIndex>(u8(0)));
    if (api == "emergencyStop")
        return piper.emergencyStop(static_cast<EmergencyStop>(u8(0)));
    if (api == "moveCAxisUpdate")
        return piper.moveCAxisUpdate(static_cast<MoveCInstructionPoint>(u8(0)));
    if (api == "sendJointConfig")
        return piper.sendJointConfig(
            static_cast<MotorIndex>(u8(0)), static_cast<ConfigEffective>(u8(1)), static_cast<ConfigEffective>(u8(2)),
            static_cast<uint16_t>(a[3]), static_cast<ConfigEffective>(u8(4))
        );
    if (api == "setMotorAngleLimitsMaxSpeed")
        return piper.setMotorAngleLimitsMaxSpeed(static_cast<MotorIndex>(u8(0)), i16(1), i16(2), i16(3));
    if (api == "setEndVelAccLimits")
        return piper.setEndVelAccLimits(i16(0), i16(1), i16(2), i16(3));
    if (api == "setCrashProtectionLevels")
        return piper.setCrashProtectionLevels(u8(0), u8(1), u8(2), u8(3), u8(4), u8(5));
    if (api == "sendArmParamEnquiryAndConfig")
        return piper.sendArmParamEnquiryAndConfig(
            static_cast<ParamEnquiry>(u8(0)), static_cast<ParamSetting>(u8(1)), static_cast<PeriodicFeedback48x>(u8(2)),
            static_cast<ConfigEffective>(u8(3)), static_cast<EndLoad>(u8(4))
        );
    if (api == "setGripperTeachingPendantParam")
        return piper.setGripperTeachingPendantParam(u8(0), u8(1), u8(2));
    if (api == "setMasterSlaveConfig")
        return piper.setMasterSlaveConfig(
            static_cast<LinkageConfig>(u8(0)), static_cast<CanIdOffset>(u8(1)), static_cast<CanIdOffset>(u8(2)),
            static_cast<CanIdOffset>(u8(3))
        );
    ADD_FAILURE() << "no C++ call for " << c.api;
    return false;
}

void expectFrame(const can_frame& actual, const sdk_reference::Frame& expected, const std::string& what)
{
    EXPECT_EQ(actual.can_id, expected.can_id) << what;
    ASSERT_EQ(actual.can_dlc, expected.dlc) << what;
    for (int i = 0; i < expected.dlc; ++i)
        EXPECT_EQ(int(actual.data[i]), int(expected.data[i])) << what << " byte " << i;
}

} // namespace

TEST(PiperSdkReference, PublicCommandsSendTheSdkFrames)
{
    for (const auto& c : sdk_reference::kCommandCases)
    {
        if (std::string(c.api) == "encodeJointMit")
            continue;
        const auto what = describe(c);
        auto run = [&](auto bus)
        {
            ASSERT_TRUE(bus.piper->connectPort(/*can_init=*/false, /*piper_init=*/false, /*start_threads=*/false));
            const bool sent = call(*bus.piper, c);
            if (c.rejected)
            {
                EXPECT_FALSE(sent) << what << ": the SDK rejects this call";
                EXPECT_FALSE(receiveFrame(bus.peer, kQuiet)) << what;
                return;
            }
            EXPECT_TRUE(sent) << what;
            for (int i = 0; i < c.num_frames; ++i)
            {
                const auto tx = receiveFrame(bus.peer);
                ASSERT_TRUE(tx) << what << " frame " << i;
                expectFrame(*tx, c.frames[i], what);
            }
            EXPECT_FALSE(receiveFrame(bus.peer, kQuiet)) << what << ": more frames than the SDK sends";
        };
        if (c.protocol == 3)
            run(makeBus<PiperInterfaceV3>());
        else
            run(makeBus<PiperInterfaceV2>());
    }
}

TEST(PiperSdkReference, MitFieldsPackLikeTheSdk)
{
    for (const auto& c : sdk_reference::kCommandCases)
    {
        if (std::string(c.api) != "encodeJointMit")
            continue;
        std::unique_ptr<PiperParserBase> parser;
        if (c.protocol == 3)
            parser = std::make_unique<PiperParserV3>();
        else
            parser = std::make_unique<PiperParserV2>();
        const int joint = static_cast<int>(c.args[0]);
        PiperMessage msg;
        msg.type = static_cast<ArmMsgType>(static_cast<int>(ArmMsgType::JointMitCtrl1) + joint - 1);
        auto& mit = msg.arm_joint_mit_ctrl.motors[joint - 1];
        mit.pos_ref = static_cast<uint16_t>(c.args[1]);
        mit.vel_ref = static_cast<uint16_t>(c.args[2]);
        mit.kp = static_cast<uint16_t>(c.args[3]);
        mit.kd = static_cast<uint16_t>(c.args[4]);
        mit.t_ref = static_cast<uint16_t>(c.args[5]);
        can_frame tx{};
        ASSERT_TRUE(parser->encodeMessage(msg, tx));
        expectFrame(tx, c.frames[0], describe(c));
    }
}

TEST(PiperSdkReference, FeedbackDecodesLikeTheSdk)
{
    PiperParserV2 v2;
    PiperParserV3 v3;
    for (const auto& c : sdk_reference::kDecodeCases)
    {
        can_frame f{};
        f.can_id = c.frame.can_id;
        f.can_dlc = c.frame.dlc;
        std::copy(c.frame.data, c.frame.data + 8, f.data);
        PiperMessage msg;
        const std::string kind = c.kind;
        const std::string what = "V" + std::to_string(c.protocol) + " " + kind;
        ASSERT_TRUE(c.protocol == 3 ? v3.decodeMessage(f, 0.0, msg) : v2.decodeMessage(f, 0.0, msg)) << what;
        const int64_t* x = c.fields;
        if (kind == "status")
        {
            const auto& s = msg.arm_status_msgs;
            EXPECT_EQ(int(s.ctrl_mode), x[0]) << what;
            EXPECT_EQ(int(s.arm_status), x[1]) << what;
            EXPECT_EQ(int(s.mode_feed), modeFromSdk(c.protocol, x[2])) << what;
            EXPECT_EQ(int(s.teach_status), x[3]) << what;
            EXPECT_EQ(int(s.motion_status), x[4]) << what;
            EXPECT_EQ(int(s.trajectory_num), x[5]) << what;
            EXPECT_EQ(int(s.err_code), x[6]) << what;
        }
        else if (kind == "gripper")
        {
            const auto& g = msg.gripper_feedback;
            EXPECT_EQ(g.grippers_angle, x[0]) << what;
            EXPECT_EQ(g.grippers_effort, x[1]) << what;
            EXPECT_EQ(g.status_code, x[2]) << what;
            if (x[3] >= 0) // the V2 SDK reports no mode
            {
                EXPECT_EQ(int(g.mode), x[3]) << what;
            }
        }
        else if (kind == "ik")
        {
            const auto& ik = msg.arm_ik_joint_feedback;
            const int32_t* pair = f.can_id == 0x2AA ? &ik.joint_1 : f.can_id == 0x2AB ? &ik.joint_3 : &ik.joint_5;
            EXPECT_EQ(pair[0], x[0]) << what;
            EXPECT_EQ(pair[1], x[1]) << what;
        }
        else if (kind == "joint")
        {
            EXPECT_EQ(msg.arm_joint_feedback.joint_3, x[0]) << what;
            EXPECT_EQ(msg.arm_joint_feedback.joint_4, x[1]) << what;
        }
        else if (kind == "high_spd")
        {
            const auto& h = msg.high_spd_feedbacks[2];
            EXPECT_EQ(h.motor_speed, x[0]) << what;
            EXPECT_EQ(h.current, x[1]) << what;
            EXPECT_EQ(h.pos, x[2]) << what;
        }
        else if (kind == "low_spd")
        {
            const auto& lo = msg.low_spd_feedbacks[4];
            EXPECT_EQ(lo.vol, x[0]) << what;
            EXPECT_EQ(lo.foc_temp, x[1]) << what;
            EXPECT_EQ(lo.motor_temp, x[2]) << what;
            EXPECT_EQ(lo.foc_status_code, x[3]) << what;
            EXPECT_EQ(lo.bus_current, x[4]) << what;
        }
        else
        {
            ADD_FAILURE() << "unknown kind " << kind;
        }
    }
}
