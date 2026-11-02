// Drives PiperInterfaceV3 (and PiperInterfaceV2 where the two must differ) through a
// socketpair in place of the CAN bus, checking what the public API puts on the wire and how
// V3 feedback reaches the state getters.

#include "piper_cpp/interface/piper_interface_v3.h"
#include "test_support.h"

#include <gtest/gtest.h>

#include <cmath>
#include <limits>

using namespace std::chrono_literals;
using piper_cpp::GripperMode;
using piper_cpp::GripperStatusCode;
using piper_cpp::InstallationPos;
using piper_cpp::MoveMode;
using piper_cpp::PiperInterfaceV2;
using piper_cpp::PiperInterfaceV3;
using std::chrono::steady_clock;
using test_support::frame;
using test_support::jointPairFrame;
using test_support::makeBus;
using test_support::receiveFrame;
using test_support::sendFrame;
using test_support::waitFor;
using ModeFeed = piper_cpp::ArmMsgFeedbackStatusEnum::ModeFeed;

namespace
{

constexpr auto kQuiet = 50ms; // long enough for a frame that was sent to arrive

template <typename Interface> void connectWithoutReadThread(test_support::Bus<Interface>& bus)
{
    ASSERT_TRUE(bus.piper->connectPort(/*can_init=*/false, /*piper_init=*/false, /*start_threads=*/false));
}

} // namespace

TEST(PiperInterfaceV3, ReportsItsProtocol)
{
    EXPECT_EQ(makeBus<PiperInterfaceV3>().piper->getProtocolVersion(), 3);
    EXPECT_EQ(makeBus<PiperInterfaceV2>().piper->getProtocolVersion(), 2);
}

TEST(PiperInterfaceV3, MoveMIsSentAndReadBackAsTheSameMode)
{
    auto bus = makeBus<PiperInterfaceV3>();
    ASSERT_TRUE(bus.piper->connectPort(/*can_init=*/false, /*piper_init=*/false, /*start_threads=*/true));

    ASSERT_TRUE(bus.piper->setMoveMode(MoveMode::MoveM));
    const auto tx = receiveFrame(bus.peer);
    ASSERT_TRUE(tx);
    EXPECT_EQ(tx->can_id, 0x151u);
    EXPECT_EQ(tx->data[1], 0x06);
    EXPECT_EQ(bus.piper->getCommandedMotionCtrlState().move_mode, static_cast<uint8_t>(MoveMode::MoveM));

    const auto deadline = steady_clock::now() + 2s;
    sendFrame(bus.peer, frame(0x2A1, {0x01, 0x00, 0x06, 0, 0, 0, 0, 0}), deadline);
    sendFrame(bus.peer, frame(0x151, {0x01, 0x06, 0x32, 0xAD, 0x00, 0x00, 0, 0}), deadline);
    ASSERT_TRUE(
        waitFor([&] { return bus.piper->getArmStatus().is_valid && bus.piper->getArmMotionCtrlCode151().is_valid; })
    );
    EXPECT_EQ(bus.piper->getArmStatus().value.mode_feed, ModeFeed::MOVE_M);
    EXPECT_EQ(bus.piper->getArmMotionCtrlCode151().value.move_mode, static_cast<uint8_t>(MoveMode::MoveM));
}

TEST(PiperInterfaceV3, OnlyV3SendsHorizontalInvertedInstallation)
{
    auto v3 = makeBus<PiperInterfaceV3>();
    connectWithoutReadThread(v3);
    ASSERT_TRUE(v3.piper->setInstallationPos(InstallationPos::HorizontalInverted));
    const auto tx = receiveFrame(v3.peer);
    ASSERT_TRUE(tx);
    EXPECT_EQ(tx->data[5], 0x04);

    auto v2 = makeBus<PiperInterfaceV2>();
    connectWithoutReadThread(v2);
    EXPECT_FALSE(v2.piper->setInstallationPos(InstallationPos::HorizontalInverted));
    EXPECT_FALSE(receiveFrame(v2.peer, kQuiet)) << "V2 must not send a value it defines as invalid";

    // The refused value must not stay in the commanded state, or every later 0x151 setter
    // would resend it and fail too.
    EXPECT_EQ(v2.piper->getCommandedMotionCtrlState().installation_pos, 0x00);
    ASSERT_TRUE(v2.piper->setMoveMode(MoveMode::MoveP));
    const auto next = receiveFrame(v2.peer);
    ASSERT_TRUE(next);
    EXPECT_EQ(next->data[1], 0x00);
    EXPECT_EQ(next->data[5], 0x00);
}

TEST(PiperInterfaceV3, GripperCommandsSelectTheMode)
{
    auto bus = makeBus<PiperInterfaceV3>();
    connectWithoutReadThread(bus);

    ASSERT_TRUE(bus.piper->controlGripperAngle(-45000, 1000));
    auto tx = receiveFrame(bus.peer);
    ASSERT_TRUE(tx);
    EXPECT_EQ(tx->data[6], 0x05) << "enable | angle bit";

    ASSERT_TRUE(bus.piper->controlGripperWidth(30000, 1000, GripperStatusCode::EnableAndClearError));
    tx = receiveFrame(bus.peer);
    ASSERT_TRUE(tx);
    EXPECT_EQ(tx->data[6], 0x03);

    // The V2 call keeps meaning width mode under V3.
    ASSERT_TRUE(bus.piper->controlGripper(30000, 1000));
    tx = receiveFrame(bus.peer);
    ASSERT_TRUE(tx);
    EXPECT_EQ(tx->data[6], 0x01);

    EXPECT_FALSE(bus.piper->controlGripperAngle(0, 5001));
    EXPECT_FALSE(bus.piper->controlGripperWidth(0, 5001));
    EXPECT_FALSE(bus.piper->controlGripper(0, 5001)) << "the same width frame, so the same limit";
    EXPECT_FALSE(receiveFrame(bus.peer, kQuiet)) << "a rejected effort must not reach the bus";

    // V2 keeps sending what it always sent.
    auto v2 = makeBus<PiperInterfaceV2>();
    connectWithoutReadThread(v2);
    EXPECT_TRUE(v2.piper->controlGripper(0, 5001));
}

TEST(PiperInterfaceV3, GripperFeedbackLimitFollowsTheMode)
{
    // With SDK gripper limits on, a width is clamped to the width range (default 0-70 mm) and
    // an angle to the angle range (default +-180 deg).
    auto bus = makeBus<PiperInterfaceV3>(/*dh_offset=*/true, /*sdk_joint_limit=*/false, /*sdk_gripper_limit=*/true);
    ASSERT_TRUE(bus.piper->connectPort(/*can_init=*/false, /*piper_init=*/false, /*start_threads=*/true));
    const auto deadline = steady_clock::now() + 2s;

    // 200 deg = 200000 = 0x00030D40, angle mode.
    sendFrame(bus.peer, frame(0x2A8, {0x00, 0x03, 0x0D, 0x40, 0, 0, 0x40, 0x01}), deadline);
    ASSERT_TRUE(waitFor([&] { return bus.piper->getGripperStates().is_valid; }));
    EXPECT_EQ(bus.piper->getGripperStates().value.mode, GripperMode::Angle);
    EXPECT_EQ(bus.piper->getGripperStates().value.grippers_angle, 180000);

    // The same value in width mode is 200 mm, clamped to 70 mm.
    sendFrame(bus.peer, frame(0x2A8, {0x00, 0x03, 0x0D, 0x40, 0, 0, 0x40, 0x00}), deadline);
    ASSERT_TRUE(waitFor([&] { return bus.piper->getGripperStates().value.mode == GripperMode::Width; }));
    EXPECT_EQ(bus.piper->getGripperStates().value.grippers_angle, 70000);
}

TEST(PiperInterfaceV3, IkJointStatesNeedAllThreeFramesAndGoStale)
{
    auto bus = makeBus<PiperInterfaceV3>();
    ASSERT_TRUE(bus.piper->connectPort(/*can_init=*/false, /*piper_init=*/false, /*start_threads=*/true));
    const auto deadline = steady_clock::now() + 5s;

    sendFrame(bus.peer, jointPairFrame(0x2AA, 1000, -2000), deadline);
    sendFrame(bus.peer, jointPairFrame(0x2AB, 3000, -4000), deadline);
    ASSERT_TRUE(waitFor([&] { return bus.piper->getArmIKJointStates().value.joint_4 == -4000; }));
    EXPECT_FALSE(bus.piper->getArmIKJointStates().is_valid) << "joints 5-6 have not arrived";

    sendFrame(bus.peer, jointPairFrame(0x2AC, 5000, -6000), deadline);
    ASSERT_TRUE(waitFor([&] { return bus.piper->getArmIKJointStates().is_valid; }));
    const auto ik = bus.piper->getArmIKJointStates();
    EXPECT_TRUE(ik.is_healthy);
    EXPECT_EQ(ik.value.joint_1, 1000);
    EXPECT_EQ(ik.value.joint_6, -6000);
    EXPECT_FALSE(bus.piper->getArmJointStates().is_valid) << "IK solutions are not measured joint feedback";

    // A stream that stops is reported as unhealthy once the 1 s timeout passes.
    ASSERT_TRUE(waitFor([&] { return !bus.piper->getArmIKJointStates().is_healthy; }, 3s));

    bus.piper->disconnectPort();
    EXPECT_FALSE(bus.piper->getArmIKJointStates().is_valid) << "a closed session's IK feedback must not look current";
}

TEST(PiperInterfaceV3, MitRejectsNonFiniteInputsWithoutSending)
{
    constexpr double nan = std::numeric_limits<double>::quiet_NaN();
    constexpr double inf = std::numeric_limits<double>::infinity();
    auto v3 = makeBus<PiperInterfaceV3>();
    auto v2 = makeBus<PiperInterfaceV2>();
    connectWithoutReadThread(v3);
    connectWithoutReadThread(v2);
    for (PiperInterfaceV2* piper : {static_cast<PiperInterfaceV2*>(v3.piper.get()), v2.piper.get()})
    {
        EXPECT_FALSE(piper->controlJointMit(1, nan, 0, 10, 0.8, 0));
        EXPECT_FALSE(piper->controlJointMit(2, 0, inf, 10, 0.8, 0));
        EXPECT_FALSE(piper->controlJointMit(3, 0, 0, -inf, 0.8, 0));
        EXPECT_FALSE(piper->controlJointMit(4, 0, 0, 10, nan, 0));
        EXPECT_FALSE(piper->controlJointMit(6, 0, 0, 10, 0.8, nan));
        EXPECT_FALSE(piper->controlJointMit(7, 0, 0, 10, 0.8, 0)) << "joint index";
    }
    EXPECT_FALSE(receiveFrame(v3.peer, kQuiet));
    EXPECT_FALSE(receiveFrame(v2.peer, kQuiet));
}

TEST(PiperInterfaceV3, MitTorqueRangeDependsOnProtocol)
{
    // 12 N*m is inside V3's [-16, 16] but beyond V2's [-8, 8], where it saturates.
    auto v3 = makeBus<PiperInterfaceV3>();
    auto v2 = makeBus<PiperInterfaceV2>();
    connectWithoutReadThread(v3);
    connectWithoutReadThread(v2);
    ASSERT_TRUE(v3.piper->controlJointMit(2, 0, 0, 10, 0.8, 12.0));
    ASSERT_TRUE(v2.piper->controlJointMit(2, 0, 0, 10, 0.8, 12.0));
    const auto a = receiveFrame(v3.peer);
    const auto b = receiveFrame(v2.peer);
    ASSERT_TRUE(a && b);
    // V3: 28 * 4095 / 32 = 3583.125 -> 0xDFF. V2: saturated at 0xFF.
    EXPECT_EQ(a->can_id, 0x15Bu);
    EXPECT_EQ(a->data[6] & 0x0F, 0x0D);
    EXPECT_EQ(a->data[7], 0xFF);
    EXPECT_EQ(b->data[6] & 0x0F, 0x0F);
    EXPECT_EQ(b->data[7] >> 4, 0x0F);
}
