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

/// Emergency-stop / resume command on the 0x150 message.
enum class EmergencyStop : uint8_t
{
    NoOp = 0x00,   ///< No-op.
    Stop = 0x01,   ///< Trigger emergency stop -- the arm immediately cuts torque.
    Resume = 0x02, ///< Resume from a previous emergency stop.
};

/// Trajectory control sub-command on the 0x150 message.
enum class TrackCtrl : uint8_t
{
    Disable = 0x00,             ///< Disable trajectory control.
    PausePlan = 0x01,           ///< Pause the current planned trajectory.
    ContinueTrajectory = 0x02,  ///< Continue execution of the current trajectory.
    ClearCurrent = 0x03,        ///< Clear the current trajectory.
    ClearAll = 0x04,            ///< Clear all stored trajectories.
    GetCurrentPlanned = 0x05,   ///< Request the currently planned trajectory.
    Terminate = 0x06,           ///< Terminate execution.
    StartTransmission = 0x07,   ///< Begin trajectory transmission.
    EndTransmission = 0x08,     ///< End trajectory transmission.
};

/// Drag-teach sub-command on the 0x150 message.
enum class DragTeachCtrl : uint8_t
{
    Disable = 0x00,            ///< Disable drag-teach.
    StartRecord = 0x01,        ///< Enter drag-teach mode and start recording the trajectory.
    EndRecord = 0x02,          ///< End recording and exit drag-teach mode.
    Replay = 0x03,             ///< Replay the recorded trajectory.
    Pause = 0x04,              ///< Pause replay.
    Continue = 0x05,           ///< Continue replay after pause.
    Terminate = 0x06,          ///< Terminate replay.
    MoveToTrajectoryStart = 0x07, ///< Move the arm to the start of the recorded trajectory.
};

/// Coordinate-point sequence number for MoveC (circular pattern). After switching the arm to
/// MoveC mode, send three end-pose commands paired with `StartPoint` / `MidPoint` / `EndPoint`
/// to define the circle.
enum class MoveCInstructionPoint : uint8_t
{
    Invalid = 0x00,
    StartPoint = 0x01,
    MidPoint = 0x02,
    EndPoint = 0x03,
};

/// Per-motor index used by the motor enable/disable, joint config, etc. messages.
/// 1-6 address an individual joint motor; 7 (or 0xFF) is the broadcast / "all motors".
enum class MotorIndex : uint8_t
{
    Joint1 = 0x01,
    Joint2 = 0x02,
    Joint3 = 0x03,
    Joint4 = 0x04,
    Joint5 = 0x05,
    Joint6 = 0x06,
    AllJoints = 0x07,
    Broadcast = 0xFF,
};

/// Param-enquiry sub-command on the 0x477 message. Triggers a one-shot reply on a related
/// feedback ID.
enum class ParamEnquiry : uint8_t
{
    None = 0x00,
    EndVelAcc = 0x01,                ///< Reply on 0x478: end-effector velocity/acceleration limits.
    CrashProtectionLevel = 0x02,     ///< Reply on 0x47B: per-joint crash protection level.
    CurrentTrajectoryIndex = 0x03,   ///< Current trajectory index.
    GripperTeachingPendantParam = 0x04, ///< Reply on 0x47E: gripper / teaching-pendant params (firmware V1.5-2+).
};

/// Param-setting sub-command on the 0x477 message.
enum class ParamSetting : uint8_t
{
    None = 0x00,
    EndVelAccToInitial = 0x01,        ///< Reset end-effector velocity/acceleration to initial values.
    JointLimitsAndMaxSpdAndAccToDefault = 0x02, ///< Reset joint limits, max speed, max accel to defaults.
};

/// Periodic-feedback enable/disable for the 0x48x message family on the 0x477 command.
enum class PeriodicFeedback48x : uint8_t
{
    None = 0x00,
    Disable = 0x01,
    Enable = 0x02,
};

/// End-load setting on the 0x477 command.
enum class EndLoad : uint8_t
{
    NoLoad = 0x00,
    HalfLoad = 0x01,
    FullLoad = 0x02,
    Invalid = 0x03,
};

/// Mode for the request-master-arm-move-to-home (`0x191`) command. Firmware V1.7-4+.
enum class MasterArmHomeMode : uint8_t
{
    RestoreMasterSlaveMode = 0,     ///< Resume normal master/slave operation.
    MasterArmReturnToZero = 1,      ///< Master arm only goes to home.
    BothMasterAndSlaveReturnToZero = 2, ///< Master and follower both go to home.
};

/// "Effective" flag used on several config messages: 0x00 means "no change", 0xAE means
/// "apply this field's value". Wrapped as a typed enum so the API surface is `bool`-like
/// without conflating with the `uint8_t` enum-class wire bytes.
enum class ConfigEffective : uint8_t
{
    NoChange = 0x00,
    Apply = 0xAE,
};

} // namespace piper_cpp

/// MoveC (circular pattern) coordinate-point update payload. Wraps the start/mid/end
/// instruction sent after each `controlEndPose()` while in MoveC mode.
struct ArmMsgCircularPatternCoordNumUpdateCtrl
{
    uint8_t instruction_num = 0; ///< Sequence number; see `MoveCInstructionPoint`.

    std::string toString() const
    {
        std::ostringstream oss;
        oss << "ArmMsgCircularPatternCoordNumUpdateCtrl(\n"
            << "  instruction_num: " << int(instruction_num) << "\n"
            << ")";
        return oss.str();
    }
};

/// Gripper command payload (CAN ID 0x159). Sent on every cycle of the gripper control loop.
struct ArmMsgGripperCtrl
{
    int32_t grippers_angle = 0;  ///< Target jaw stroke, 0.001 mm.
    uint16_t grippers_effort = 0; ///< Max gripper torque, 0.001 N*m. Range 0-5000.
    uint8_t status_code = 0;      ///< Enable/disable + error-clear command; see `GripperStatusCode`.
    uint8_t set_zero = 0;         ///< One-shot set-zero flag; see `GripperSetZero`.

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

/// Joint configuration payload (CAN ID 0x475). Combines per-joint zero-setting, max-acceleration
/// configuration, and error-clearing into one frame. Use `ConfigEffective::Apply` (`0xAE`) on the
/// "effective" / "set" flags to mark the corresponding action as active for this frame.
struct ArmMsgJointConfig
{
    uint8_t joint_motor_num = 7;                       ///< Motor index 1-6, or 7 for all motors.
    uint8_t set_motor_current_pos_as_zero = 0;         ///< 0xAE to latch the current joint position as zero.
    uint8_t acc_param_config_is_effective_or_not = 0;  ///< 0xAE to apply the `max_joint_acc` field below.
    uint16_t max_joint_acc = 500;                      ///< Max joint acceleration, 0.01 rad/s^2. 0x7FFF = leave unchanged.
    uint8_t clear_joint_err = 0;                       ///< 0xAE to clear any latched error code on this joint.
    uint8_t reserved6 = 0;                             ///< Wire byte 6 (reserved).
    uint8_t reserved7 = 0;                             ///< Wire byte 7 (reserved).

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

/// Per-joint MIT control payload (CAN IDs 0x15A-0x15F). Fields are pre-packed integers in the
/// protocol's fixed-point representation; use `PiperInterfaceV2::controlJointMit()` to send
/// natural floats and let the helper handle the float-to-uint mapping.
struct ArmMsgJointMitCtrl
{
    uint16_t pos_ref = 0; ///< Position reference (16-bit packed; unsigned mapping over [-12.5, 12.5] rad).
    uint16_t vel_ref = 0; ///< Velocity reference (12-bit packed; mapped over [-45, 45]).
    uint16_t kp = 10;     ///< Position-error gain (12-bit packed; mapped over [0, 500]).
    uint16_t kd = 0;      ///< Velocity-error gain (12-bit packed; mapped over [-5, 5]).
    uint16_t t_ref = 0;   ///< Feed-forward torque (8-bit packed; mapped over [-8, 8]).
    uint8_t crc = 0;      ///< Optional CRC byte (firmware-dependent; 0 if unused).

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

/// Aggregate of MIT command frames for all six joints. Each frame is sent independently on
/// the bus; this struct just bundles them so callers can compose multi-joint MIT commands.
struct ArmMsgAllJointMitCtrl
{
    static constexpr size_t num_joints = 6;
    std::array<ArmMsgJointMitCtrl, num_joints> motors; ///< One MIT command per joint, indexed 0..5.

    std::string toString() const
    {
        std::ostringstream oss;
        for (size_t i = 0; i < num_joints; ++i)
            oss << "Joint " << (i + 1) << ": " << motors[i].toString() << "\n";
        return oss.str();
    }
};

/// Leader/follower (master/slave) mode configuration payload (CAN ID 0x470).
struct ArmMsgMasterSlaveModeConfig
{
    uint8_t linkage_config = 0;  ///< Role for this arm; see `LinkageConfig`.
    uint8_t feedback_offset = 0; ///< Feedback CAN-ID offset; see `CanIdOffset`. Use 0 for single-arm.
    uint8_t ctrl_offset = 0;     ///< Control CAN-ID offset; see `CanIdOffset`. Use 0 for single-arm.
    uint8_t linkage_offset = 0;  ///< Linkage-mode target CAN-ID offset. Use 0 for single-arm.

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

/// Motion control 1 payload (CAN ID 0x150). Bundles emergency-stop, trajectory control, and
/// drag-teach control fields. Most users hit this through `emergencyStop()` / `resetPiper()`.
struct ArmMsgMotionCtrl_1
{
    uint8_t emergency_stop = 0;   ///< Emergency stop / resume; see `EmergencyStop`.
    uint8_t track_ctrl = 0;       ///< Trajectory control sub-command; see `TrackCtrl`.
    uint8_t grag_teach_ctrl = 0;  ///< Drag-teach sub-command; see `DragTeachCtrl`. (Field name carries Piper's original typo of "drag".)
    uint8_t trajectory_index = 0; ///< Trajectory index for transmit/replay sub-commands.
    uint16_t name_index = 0;      ///< Trajectory name index (high byte first on the wire).
    uint16_t crc16 = 0;           ///< Optional CRC over the frame; firmware-dependent (high byte first).

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

/// Motion control 2 payload (CAN ID 0x151). Bundles the arm's motion-mode configuration:
/// control source, movement type, speed cap, MIT toggle, residence time, and mounting
/// orientation. Most users hit this through the granular setters or `sendMotionCtrl2()`.
struct ArmMsgMotionCtrl_2
{
    uint8_t ctrl_mode = 0x01;        ///< Top-level control source; see `CtrlMode`. Default `CanCmd`.
    uint8_t move_mode = 0x01;        ///< Interpolation type; see `MoveMode`. Default `MoveJ`.
    uint8_t move_spd_rate_ctrl = 50; ///< Speed cap percentage in [0, 100]. Default 50.
    uint8_t mit_mode = 0x00;         ///< MIT mode toggle; see `MitMode`. Default `PositionVelocity`.
    uint8_t residence_time = 0;      ///< Hold time at offline-trajectory waypoints (s). 0-254; 255 terminates.
    uint8_t installation_pos = 0x00; ///< Physical mounting orientation; see `InstallationPos`. 0 = leave unchanged.

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

/// Motor enable/disable command payload (CAN ID 0x471). Hit through `enableRobot()` /
/// `disableRobot()` (broadcast) or `enableMotor()` / `disableMotor()` (per-motor).
struct ArmMsgMotorEnableDisableConfig
{
    uint8_t motor_num = 0xFF;   ///< Motor index 1-6 (one joint), 7 (all joints), or 0xFF (broadcast).
    uint8_t enable_flag = 0x01; ///< 0x01 = disable, 0x02 = enable.

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

/// Parameter-query / -config payload (CAN ID 0x477). Multi-purpose: each field independently
/// triggers a different firmware behaviour. See the corresponding enums.
struct ArmMsgParamEnquiryAndConfig
{
    uint8_t param_enquiry = 0x00;                    ///< Query sub-command; see `ParamEnquiry`.
    uint8_t param_setting = 0x00;                    ///< Setting sub-command; see `ParamSetting`.
    uint8_t data_feedback_0x48x = 0x00;              ///< Periodic-feedback toggle; see `PeriodicFeedback48x`.
    uint8_t end_load_param_setting_effective = 0x00; ///< 0xAE to apply `set_end_load` below.
    uint8_t set_end_load = 0x03;                     ///< End-load setting; see `EndLoad`. Default Invalid (no change).

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

/// "Search motor limits" query payload, fired from `piperInit()` once per joint to populate
/// the per-motor angle/speed/acceleration caches. CAN ID 0x472.
struct ArmMsgSearchMotorMaxAngleSpdAccLimit
{
    uint8_t motor_num = 1;         ///< Motor index 1-6.
    uint8_t search_content = 0x01; ///< 0x01 = query angle / max speed; 0x02 = query max acceleration.

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

/// Instruction-response acknowledgment from the arm (CAN ID 0x476). Returned after set-zero,
/// clear-error, and similar one-shot commands; carries the index of the responded instruction
/// and a success flag.
struct ArmMsgInstructionResponseConfig
{
    uint8_t instruction_index = 0;        ///< Low byte of the instruction this is responding to (e.g. 0x71 for a 0x471 reply).
    uint8_t zero_config_success_flag = 0; ///< 0x01 if the zero/config operation succeeded; 0x00 otherwise.

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
