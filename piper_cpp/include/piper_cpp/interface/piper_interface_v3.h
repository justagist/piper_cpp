#pragma once

#include "piper_cpp/interface/piper_interface_v2.h"

namespace piper_cpp
{

/// Piper arm interface speaking protocol V3, for firmware V1.8-8 and newer. Every
/// `PiperInterfaceV2` method is available and keeps its meaning; the protocol differences are
/// handled by the V3 parser:
///   - `MoveMode::MoveM` is sent as 0x06, and MOVE M feedback (0x06) reads back as
///     `ModeFeed::MOVE_M`.
///   - `InstallationPos::HorizontalInverted` can be sent.
///   - `controlJointMit()` accepts torque in [-16, 16] and packs it into 12 bits.
///   - The gripper can be driven by width or by angle; feedback reports which.
///   - IK joint feedback is available.
///
/// @warning Tested against the official piper_sdk V3 encoder and decoder only, not on a
/// physical arm.
class PiperInterfaceV3 : public PiperInterfaceV2
{
public:
    /// Same parameters as the matching `PiperInterfaceV2` constructor.
    explicit PiperInterfaceV3(
        std::string can_name, bool judge_flag = true, bool auto_init = true, bool dh_offset = true,
        bool sdk_joint_limit = false, bool sdk_gripper_limit = false
    );

    /// Same parameters as the matching `PiperInterfaceV2` constructor.
    explicit PiperInterfaceV3(
        std::unique_ptr<StdCanInterface> can, bool dh_offset = true, bool sdk_joint_limit = false,
        bool sdk_gripper_limit = false
    );

    /// Joint angles from the arm's own inverse kinematics (joint_1..joint_6, 0.001 deg).
    /// Reassembled from three CAN frames (0x2AA-0x2AC). The arm only starts publishing them
    /// after it has received an end-pose command in `MoveMode::MoveP`; until then `is_valid`
    /// stays false.
    StateSnapshot<ArmMsgJointValues> getArmIKJointStates() const { return StateSnapshot(arm_ik_joint_states_); }

    /// Command the gripper to a jaw opening. Same frame as `controlGripper()`.
    ///
    /// @param width_um      Target jaw opening in 0.001 mm.
    /// @param effort        Maximum gripper torque in 0.001 N*m, range 0-5000.
    /// @param status_code   Enable/disable + error-clear command.
    /// @return true on successful CAN send; false if `effort` is above 5000 (nothing is sent).
    bool controlGripperWidth(
        int32_t width_um, uint16_t effort = 1000, GripperStatusCode status_code = GripperStatusCode::Enable
    );

    /// Command the gripper to a jaw angle. The firmware version that introduced angle mode is
    /// not documented upstream.
    ///
    /// @param angle_millideg  Target jaw angle in 0.001 deg.
    /// @param effort          Maximum gripper torque in 0.001 N*m, range 0-5000.
    /// @param status_code     Enable/disable + error-clear command.
    /// @return true on successful CAN send; false if `effort` is above 5000 (nothing is sent).
    bool controlGripperAngle(
        int32_t angle_millideg, uint16_t effort = 1000, GripperStatusCode status_code = GripperStatusCode::Enable
    );
};

} // namespace piper_cpp
