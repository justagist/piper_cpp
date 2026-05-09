#pragma once
#include <array>
#include <cstdint>
#include <sstream>
#include <string>

namespace piper_cpp
{

/// Where the arm should be listening for motion commands. Use `CanCmd` for normal SDK use --
/// the others are for AgileX's higher-level Ethernet / Wi-Fi control stacks and offline
/// trajectory playback respectively.
enum class CtrlMode : uint8_t
{
    Standby = 0x00,           ///< Standby; the arm ignores motion commands until taken out of standby.
    CanCmd = 0x01,            ///< CAN command control -- the default for direct SDK use.
    Ethernet = 0x03,          ///< Ethernet control (handled by AgileX's stack, not this SDK).
    WiFi = 0x04,              ///< Wi-Fi control (handled by AgileX's stack, not this SDK).
    OfflineTrajectory = 0x07, ///< Replay a trajectory previously stored on the arm.
};

/// Interpolation type for subsequent motion commands. Pick `MoveJ` when sending joint targets,
/// `MoveL`/`MoveP` when sending end-pose targets, etc.
enum class MoveMode : uint8_t
{
    MoveP = 0x00,   ///< Point-to-point in cartesian space.
    MoveJ = 0x01,   ///< Joint-space interpolation -- pair with joint targets.
    MoveL = 0x02,   ///< Linear cartesian -- pair with end-pose targets.
    MoveC = 0x03,   ///< Circular cartesian.
    MoveM = 0x04,   ///< MIT torque/position-mix mode. Requires firmware V1.5-2 or newer.
    MoveCPV = 0x05, ///< Continuous position + velocity. Requires firmware V1.8-1 or newer.
};

/// MIT control mode toggle. Leave as `PositionVelocity` for standard SDK control; only switch
/// to `MIT` when you intend to send MIT (joint torque + position) commands.
enum class MitMode : uint8_t
{
    PositionVelocity = 0x00, ///< Standard position-velocity control (default).
    MIT = 0xAD,              ///< MIT torque/position-mix control.
    Invalid = 0xFF,          ///< Sentinel; do not send.
};

/// How the arm is physically mounted in the world. Calibrates gravity compensation, so
/// getting it right matters for the arm's ability to hold its pose. Typically a one-time
/// commissioning step. Requires firmware V1.5-2 or newer.
enum class InstallationPos : uint8_t
{
    Invalid = 0x00,           ///< Sentinel; firmware will reject. Useful as a "leave the current setting" placeholder.
    HorizontalUpright = 0x01, ///< Mounted upright on a horizontal surface, cabling toward the rear.
    SideMountLeft = 0x02,     ///< Side-mounted on the left.
    SideMountRight = 0x03,    ///< Side-mounted on the right.
};

/// In a leader/follower (a.k.a. master/slave) setup, which role this arm should play.
/// For single-arm use, you almost always want `MotionOutputArm`.
enum class LinkageConfig : uint8_t
{
    Invalid = 0x00, ///< No-op.
    TeachingInputArm =
        0xFA, ///< Leader/teaching arm: gravity-compensated free-drive, broadcasts state, ignores motion commands.
    MotionOutputArm = 0xFC, ///< Follower/motion-output arm: torque-controlled, accepts motion commands. The mode you
                            ///< want for normal SDK control.
};

/// In a dual-arm setup (two arms on the same CAN bus), shifts this arm's CAN IDs into a
/// different range so the two arms don't collide. Always `None` for single-arm setups.
enum class CanIdOffset : uint8_t
{
    None = 0x00,     ///< No offset / restore default.
    ShiftToB = 0x10, ///< Shift base IDs into the "B" range.
    ShiftToC = 0x20, ///< Shift base IDs into the "C" range.
};

/// Gripper enable/disable + error-clear command. Sent on every gripper-control frame.
/// @note The gripper firmware auto-disables roughly 1 second after the last `Enable` is
/// received, so this byte must be repeated continuously (>1 Hz) in your control loop to keep
/// the gripper active. A one-shot enable is not enough.
enum class GripperStatusCode : uint8_t
{
    Disable = 0x00, ///< Disable the gripper -- its position-control loop stops and commanded angles have no effect.
    Enable = 0x01,  ///< Enable the gripper position-control loop.
    DisableAndClearError = 0x02, ///< Disable + clear any latched error.
    EnableAndClearError = 0x03,  ///< Enable + clear any latched error.
};

/// One-shot "set current jaw position as zero reference" command for the gripper.
/// @note Required at least once per power-up before the gripper will act on position
/// commands -- until then, the gripper feedback's `homing_status` bit stays clear and the
/// firmware ignores commanded angles. Use `PiperInterfaceV2::setGripperZero()` rather than
/// poking this field directly.
enum class GripperSetZero : uint8_t
{
    NoOp = 0x00,    ///< No-op; the value used on regular per-cycle frames.
    SetZero = 0xAE, ///< Latch the current jaw position as the zero reference.
};

} // namespace piper_cpp

// 1. Circular Pattern Coordinate Number Update Control
struct ArmMsgCircularPatternCoordNumUpdateCtrl
{
    // Byte 0: instruction_num [0x00, 0x01, 0x02, 0x03]
    uint8_t instruction_num = 0;

    std::string toString() const
    {
        std::ostringstream oss;
        oss << "ArmMsgCircularPatternCoordNumUpdateCtrl(\n"
            << "  instruction_num: " << int(instruction_num) << "\n"
            << ")";
        return oss.str();
    }
};

// 2. Gripper Control
struct ArmMsgGripperCtrl
{
    // Bytes 0-3: int32_t grippers_angle (unit: 0.001°)
    int32_t grippers_angle = 0;
    // Bytes 4-5: uint16_t grippers_effort (unit: 0.001N·m, 0-5000)
    uint16_t grippers_effort = 0;
    // Byte 6: status_code [0x00, 0x01, 0x02, 0x03]
    uint8_t status_code = 0;
    // Byte 7: set_zero [0x00, 0xAE]
    uint8_t set_zero = 0;

    std::string toString() const
    {
        std::ostringstream oss;
        oss << "ArmMsgGripperCtrl(\n"
            << "  grippers_angle: " << grippers_angle << " (" << grippers_angle * 0.001 << " deg),\n"
            << "  grippers_effort: " << grippers_effort << " (" << grippers_effort * 0.001 << " Nm),\n"
            << "  status_code: " << int(status_code) << ",\n"
            << "  set_zero: " << int(set_zero) << "\n"
            << ")";
        return oss.str();
    }
};

// 3. Joint Config
struct ArmMsgJointConfig
{
    // Byte 0: joint_motor_num [1~7] (1~6 joint, 7 all)
    uint8_t joint_motor_num = 7;
    // Byte 1: set_motor_current_pos_as_zero [0x00, 0xAE]
    uint8_t set_motor_current_pos_as_zero = 0;
    // Byte 2: acc_param_config_is_effective_or_not [0x00, 0xAE]
    uint8_t acc_param_config_is_effective_or_not = 0;
    // Bytes 3-4: max_joint_acc (unit: 0.01rad/s^2, or 0x7FFF for invalid)
    uint16_t max_joint_acc = 500;
    // Byte 5: clear_joint_err [0x00, 0xAE]
    uint8_t clear_joint_err = 0;
    // Bytes 6-7: reserved
    uint8_t reserved6 = 0;
    uint8_t reserved7 = 0;

    std::string toString() const
    {
        std::ostringstream oss;
        oss << "ArmMsgJointConfig(\n"
            << "  joint_motor_num: " << int(joint_motor_num) << ",\n"
            << "  set_motor_current_pos_as_zero: " << int(set_motor_current_pos_as_zero) << ",\n"
            << "  acc_param_config_is_effective_or_not: " << int(acc_param_config_is_effective_or_not) << ",\n"
            << "  max_joint_acc: " << max_joint_acc << " (" << (max_joint_acc * 0.01) << " rad/s^2),\n"
            << "  clear_joint_err: " << int(clear_joint_err) << "\n"
            << ")";
        return oss.str();
    }
};

// 4. MIT Single Joint Control
struct ArmMsgJointMitCtrl
{
    uint16_t pos_ref = 0;
    uint16_t vel_ref = 0;
    uint16_t kp = 10; // Use integer, not float. No scaling!
    uint16_t kd = 0;  // Use integer, not float. No scaling!
    uint16_t t_ref = 0;
    uint8_t crc = 0;

    std::string toString() const
    {
        std::ostringstream oss;
        oss << "ArmMsgJointMitCtrl(\n"
            << "  pos_ref: " << pos_ref << ",\n"
            << "  vel_ref: " << vel_ref << ",\n"
            << "  kp: " << kp << ",\n"
            << "  kd: " << kd << ",\n"
            << "  t_ref: " << t_ref << ",\n"
            << "  crc: " << int(crc) << "\n"
            << ")";
        return oss.str();
    }
};

// 5. All Joint MIT Control (per-joint array)
struct ArmMsgAllJointMitCtrl
{
    static constexpr size_t num_joints = 6;
    std::array<ArmMsgJointMitCtrl, num_joints> motors;

    std::string toString() const
    {
        std::ostringstream oss;
        for (size_t i = 0; i < num_joints; ++i)
            oss << "Joint " << (i + 1) << ": " << motors[i].toString() << "\n";
        return oss.str();
    }
};

// 6. Master/Slave Mode Config
struct ArmMsgMasterSlaveModeConfig
{
    uint8_t linkage_config = 0;
    uint8_t feedback_offset = 0;
    uint8_t ctrl_offset = 0;
    uint8_t linkage_offset = 0;

    std::string toString() const
    {
        std::ostringstream oss;
        oss << "ArmMsgMasterSlaveModeConfig(\n"
            << "  linkage_config: " << int(linkage_config) << ",\n"
            << "  feedback_offset: " << int(feedback_offset) << ",\n"
            << "  ctrl_offset: " << int(ctrl_offset) << ",\n"
            << "  linkage_offset: " << int(linkage_offset) << "\n"
            << ")";
        return oss.str();
    }
};

// 7. Motion Ctrl 1
struct ArmMsgMotionCtrl_1
{
    uint8_t emergency_stop = 0;
    uint8_t track_ctrl = 0;
    uint8_t grag_teach_ctrl = 0;
    uint8_t trajectory_index = 0;
    uint16_t name_index = 0; // Byte4:NameIndex_H, Byte5:NameIndex_L
    uint16_t crc16 = 0;      // Byte6:crc16_H, Byte7:crc16_L

    std::string toString() const
    {
        std::ostringstream oss;
        oss << "ArmMsgMotionCtrl_1(\n"
            << "  emergency_stop: " << int(emergency_stop) << ",\n"
            << "  track_ctrl: " << int(track_ctrl) << ",\n"
            << "  grag_teach_ctrl: " << int(grag_teach_ctrl) << ",\n"
            << "  trajectory_index: " << int(trajectory_index) << ",\n"
            << "  name_index: " << name_index << ",\n"
            << "  crc16: " << crc16 << "\n"
            << ")";
        return oss.str();
    }
};

// 8. Motion Ctrl 2
struct ArmMsgMotionCtrl_2
{
    uint8_t ctrl_mode = 0x01;
    uint8_t move_mode = 0x01;
    uint8_t move_spd_rate_ctrl = 50;
    uint8_t mit_mode = 0x00;
    uint8_t residence_time = 0;
    uint8_t installation_pos = 0x00;

    std::string toString() const
    {
        std::ostringstream oss;
        oss << "ArmMsgMotionCtrl_2(\n"
            << "  ctrl_mode: " << int(ctrl_mode) << ",\n"
            << "  move_mode: " << int(move_mode) << ",\n"
            << "  move_spd_rate_ctrl: " << int(move_spd_rate_ctrl) << ",\n"
            << "  mit_mode: " << int(mit_mode) << ",\n"
            << "  residence_time: " << int(residence_time) << ",\n"
            << "  installation_pos: " << int(installation_pos) << "\n"
            << ")";
        return oss.str();
    }
};

// 9. Motor Enable/Disable Config
struct ArmMsgMotorEnableDisableConfig
{
    uint8_t motor_num = 0xFF;
    uint8_t enable_flag = 0x01;

    std::string toString() const
    {
        std::ostringstream oss;
        oss << "ArmMsgMotorEnableDisableConfig(\n"
            << "  motor_num: " << int(motor_num) << ",\n"
            << "  enable_flag: " << int(enable_flag) << "\n"
            << ")";
        return oss.str();
    }
};

// 10. Param Enquiry And Config
struct ArmMsgParamEnquiryAndConfig
{
    uint8_t param_enquiry = 0x00;
    uint8_t param_setting = 0x00;
    uint8_t data_feedback_0x48x = 0x00;
    uint8_t end_load_param_setting_effective = 0x00;
    uint8_t set_end_load = 0x03;

    std::string toString() const
    {
        std::ostringstream oss;
        oss << "ArmMsgParamEnquiryAndConfig(\n"
            << "  param_enquiry: " << int(param_enquiry) << ",\n"
            << "  param_setting: " << int(param_setting) << ",\n"
            << "  data_feedback_0x48x: " << int(data_feedback_0x48x) << ",\n"
            << "  end_load_param_setting_effective: " << int(end_load_param_setting_effective) << ",\n"
            << "  set_end_load: " << int(set_end_load) << "\n"
            << ")";
        return oss.str();
    }
};

// 11. Search Motor Max Angle/Spd/Acc Limit
struct ArmMsgSearchMotorMaxAngleSpdAccLimit
{
    uint8_t motor_num = 1;
    // 0x01: Query motor angle/max speed.
    // 0x02: Query motor max acceleration limit.
    uint8_t search_content = 0x01;

    std::string toString() const
    {
        std::ostringstream oss;
        oss << "ArmMsgSearchMotorMaxAngleSpdAccLimit(\n"
            << "  motor_num: " << int(motor_num) << ",\n"
            << "  search_content: " << int(search_content) << "\n"
            << ")";
        return oss.str();
    }
};

// 12. Instruction Response Config
struct ArmMsgInstructionResponseConfig
{
    uint8_t instruction_index = 0;
    uint8_t zero_config_success_flag = 0;

    std::string toString() const
    {
        std::ostringstream oss;
        oss << "ArmMsgInstructionResponseConfig(\n"
            << "  instruction_index: " << int(instruction_index) << ",\n"
            << "  zero_config_success_flag: " << int(zero_config_success_flag) << "\n"
            << ")";
        return oss.str();
    }
};
