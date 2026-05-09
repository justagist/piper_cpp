#pragma once

#include <array>
#include <atomic>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

#include "piper_cpp/fk.h"
#include "piper_cpp/interface/timed_freq_state.h"
#include "piper_cpp/piper_params.h"
#include "piper_cpp/protocol/piper_parser_base.h"
#include "piper_cpp/std_can_interface.h"
#include "piper_cpp/types/types.h"

namespace piper_cpp
{
class PiperInterfaceV2
{
public:
    /// Constructor
    PiperInterfaceV2(
        std::string can_name, bool judge_flag = true, bool auto_init = true, bool dh_offset = true,
        bool sdk_joint_limit = false, bool sdk_gripper_limit = false
    );

    ~PiperInterfaceV2();

    /// Open the CAN port, start parsing threads, and optionally send init queries.
    /// Returns true if the port was opened successfully (or was already open).
    bool connectPort(bool can_init = false, bool piper_init = true, bool start_threads = true);

    /// Gracefully stop threads and close the CAN port.
    void disconnectPort(std::chrono::milliseconds timeout = std::chrono::milliseconds{100});

    /// Send the three "search" commands for joint limits, acc limits, firmware.
    void piperInit();

    StateSnapshot<ArmMsgFeedbackStatus> getArmStatus() const { return StateSnapshot(arm_status_); }

    StateSnapshot<ArmMsgEndPose> getArmEndPose() const { return StateSnapshot(arm_end_pose_); }

    StateSnapshot<ArmMsgJointValues> getArmJointStates() const { return StateSnapshot(arm_joint_states_); }

    StateSnapshot<ArmMsgFeedbackGripper> getGripperStates() const { return StateSnapshot(arm_gripper_msgs_); }

    StateSnapshot<ArmMsgFeedbackHighSpdArray> getArmHighSpeedFeedbacks() const
    {
        return StateSnapshot(arm_high_spd_fb_);
    }

    StateSnapshot<ArmMsgFeedbackLowSpdArray> getArmLowSpeedFeedbacks() const { return StateSnapshot(arm_low_spd_fb_); }

    StateSnapshot<ArmMsgCurrentEndVelAccParam> getArmCurrentEndVelAcc() const
    {
        return StateSnapshot(arm_current_end_vel_acc_);
    }

    StateSnapshot<ArmMsgCrashProtectionRatingConfig> getArmCrashProtectionRating() const
    {
        return StateSnapshot(arm_crash_protection_rating_fb_);
    }

    StateSnapshot<ArmMsgGripperTeachingPendantParam> getGripperTeachingPendantParamFeedback() const
    {
        return StateSnapshot(arm_gripper_teaching_pendant_fb_);
    }

    StateSnapshot<ArmMsgCurrentMotorAngleLimitMaxSpd> getArmCurrentMotorAngleLimitMaxSpd() const
    {
        return StateSnapshot(arm_current_motor_angle_limit_max_spd_);
    }

    StateSnapshot<ArmMsgFeedbackCurrentMotorMaxAccLimit> getArmCurrentMotorMaxAccLimit() const
    {
        return StateSnapshot(arm_current_motor_max_acc_limit_);
    }

    StateSnapshot<ArmMsgAllCurrentMotorAngleLimitMaxSpd> getArmAllMotorAngleLimitMaxSpd() const
    {
        return StateSnapshot(arm_all_motor_angle_limit_max_spd_);
    }

    StateSnapshot<ArmMsgFeedbackAllCurrentMotorMaxAccLimit> getArmAllMotorMaxAccLimit() const
    {
        return StateSnapshot(arm_all_motor_max_acc_limit_);
    }

    StateSnapshot<ArmMsgJointValues> getArmJointCtrlMsgs() const { return StateSnapshot(arm_joint_ctrl_msgs_); }

    StateSnapshot<ArmMsgGripperCtrl> getArmGripperCtrlMsgs() const { return StateSnapshot(arm_gripper_ctrl_); }

    StateSnapshot<ArmMsgMotionCtrl_2> getArmMotionCtrlCode151() const
    {
        return StateSnapshot(arm_motion_ctrl_code_151_);
    }

    StateSnapshot<ArmMsgInstructionResponseConfig> getInstructionResponse() const
    {
        return StateSnapshot(arm_instruction_response_);
    }

    /// Raw firmware-info bytes that the arm has streamed in response to the firmware-version
    /// query. The query is fired once automatically by `piperInit()` / `connectPort()`, so this
    /// buffer is populated within a few hundred milliseconds of opening the port.
    /// For most use cases prefer `getFirmwareVersion()` -- this raw accessor is for inspection.
    std::vector<uint8_t> getFirmwareData() const
    {
        std::lock_guard<std::mutex> lock(firmware_data_mutex_);
        return firmware_data_;
    }

    /// Parsed firmware-version string. Looks for the `"S-V"` marker in the streamed firmware
    /// data and returns the 8-byte tag that follows it, e.g. `"S-V1.7-4"`.
    ///
    /// Returns an empty optional if the marker hasn't been received yet (e.g. you've called
    /// this within the first ~100 ms of `connectPort` and the bytes haven't arrived). If you
    /// need to re-query (after a firmware-update on the arm, say), call
    /// `requestFirmwareVersion()` and read this again a moment later.
    std::optional<std::string> getFirmwareVersion() const;

    /// Re-send the firmware-version query (CAN ID `0x4AF`). The reply trickles in over the next
    /// few frames into the firmware-data buffer; read it via `getFirmwareVersion()`. Normally
    /// not needed because `connectPort` already fires this once.
    bool requestFirmwareVersion();

    std::vector<std::array<double, 6>> getCalculatedFeedbackFK() const;

    std::vector<std::array<double, 6>> getCalculatedControlFK() const;

    StateSnapshot<PiperHealth> getArmHealth() const { return StateSnapshot(arm_health_); }

    bool isHealthy() const { return arm_health_.isValid() && arm_health_.isHealthy(); }

    bool isEnabled() const
    {
        auto lowspd_fb = getArmLowSpeedFeedbacks();
        if (lowspd_fb.is_valid)
        {
            const auto &fb = lowspd_fb.value;
            return std::all_of(
                fb.low_spd_feedbacks.begin(), fb.low_spd_feedbacks.end(),
                [](const ArmMsgFeedbackLowSpd &msg) { return msg.foc_status.driver_enable_status; }
            );
        }
        return false;
    }

    PiperParamManager &getParameterManager() const { return PiperParamManager::instance(); }

    /// Enable the arm: powers on all six joint motors plus the gripper motor and switches them
    /// into closed-loop torque control. After this call, each joint should start reporting
    /// `driver_enable_status=true` in its low-speed feedback within a few frames; once they do,
    /// the arm holds its current pose against gravity instead of falling.
    ///
    /// @note This does **not** start the gripper's position-control loop. The gripper has its
    /// own enable inside the gripper-control message -- see `controlGripper()`.
    /// @note Sends the motor enable/disable broadcast (CAN ID `0x471`, motor_num=7, enable=0x02).
    /// @return true on successful CAN send; does not block on confirmation.
    bool enableRobot();

    /// Disable the arm: cuts torque to all six joint motors and the gripper motor. The arm
    /// will fall under gravity unless physically supported -- catch the wrist or set it on
    /// its rest before calling.
    ///
    /// @note Sends the motor enable/disable broadcast (CAN ID `0x471`, motor_num=7, disable=0x01).
    /// @return true on successful CAN send; does not block on confirmation.
    bool disableRobot();

    // ---- Arm motion-mode configuration ------------------------------------------------------
    //
    // These methods configure how the arm interprets motion commands (pose targets, joint
    // targets, etc.). They all share a single configuration message on the wire (CAN ID
    // `0x151`, "MotionCtrl_2" in the protocol) that bundles six fields:
    //   - control source (CAN, Ethernet, Wi-Fi, ...)
    //   - movement type (joint-space, linear cartesian, circular, MIT, ...)
    //   - speed cap (% of joint max)
    //   - MIT mode toggle
    //   - offline-trajectory waypoint hold time
    //   - physical mounting orientation
    //
    // Every send to the arm overwrites all six fields, so to make per-field updates safe
    // (so changing the movement type doesn't accidentally reset speed back to a default),
    // this class keeps a private cached copy of the last commanded state. The granular
    // setters below mutate one field of the cache under a lock and re-send the whole
    // message; `sendMotionCtrl2` is the all-at-once escape hatch.

    /// Set the top-level control source -- where the arm should be listening for commands
    /// (CAN bus, Ethernet, Wi-Fi, etc.). Use `CtrlMode::CanCmd` for normal SDK use.
    ///
    /// Updates only the control-source field; movement type, speed, MIT mode, residence time,
    /// and mounting orientation keep their last commanded values.
    bool setCtrlMode(CtrlMode ctrl_mode);

    /// Set the interpolation type for subsequent motion commands -- joint-space, linear
    /// cartesian, circular, MIT, etc. Switch this when you change between, e.g., joint
    /// targets and end-pose targets. Updates only this field; everything else is preserved.
    bool setMoveMode(MoveMode move_mode);

    /// Set the speed cap as a percentage of each joint's maximum speed. Useful for slowing
    /// the arm during testing or commanding fast motions in production.
    ///
    /// @param percentage    `[0, 100]`. Values outside this range are clamped to 100.
    bool setSpeedPercentage(uint8_t percentage);

    /// Toggle MIT control mode. Use `MitMode::PositionVelocity` for standard SDK control,
    /// `MitMode::MIT` when you intend to use MIT (torque/position-mix) joint commands.
    bool setMitMode(MitMode mit_mode);

    /// Set the hold time at each waypoint of an offline trajectory. Only meaningful when
    /// the arm is in `CtrlMode::OfflineTrajectory`; ignored otherwise.
    ///
    /// @param seconds   0-254 seconds. 255 is a sentinel that terminates the trajectory.
    bool setResidenceTime(uint8_t seconds);

    /// Tell the arm how it is physically mounted (upright, side-mount left, side-mount right).
    /// This calibrates gravity compensation, so getting it right matters for hold-against-gravity.
    /// Typically a one-time commissioning step. Requires firmware V1.5-2 or newer.
    bool setInstallationPos(InstallationPos installation_pos);

    /// Set all six motion-mode fields at once in a single CAN frame. Use this when you want
    /// to change multiple fields atomically; otherwise prefer the per-field setters above.
    /// Replaces the cached state entirely.
    ///
    /// @param ctrl_mode             Top-level control source. Use `CtrlMode::CanCmd` for normal SDK use.
    /// @param move_mode             Interpolation type for subsequent motion commands.
    /// @param move_spd_rate_ctrl    Speed cap percentage in `[0, 100]` (clamped to 100 if higher).
    /// @param mit_mode              MIT-mode toggle. Use `MitMode::PositionVelocity` unless explicitly using MIT
    /// control.
    /// @param residence_time        Offline-trajectory waypoint hold time, in seconds (0-254). 255 terminates the
    /// trajectory.
    /// @param installation_pos      Physical mounting orientation; affects gravity comp. `Invalid` leaves the
    /// firmware's last setting untouched.
    /// @return true on successful CAN send.
    bool sendMotionCtrl2(
        CtrlMode ctrl_mode = CtrlMode::CanCmd, MoveMode move_mode = MoveMode::MoveJ, uint8_t move_spd_rate_ctrl = 50,
        MitMode mit_mode = MitMode::PositionVelocity, uint8_t residence_time = 0,
        InstallationPos installation_pos = InstallationPos::Invalid
    );

    /// Read back the motion-mode state this interface last *commanded*. Distinct from
    /// `getArmMotionCtrlCode151()`, which returns what was last *received* on the bus
    /// (i.e. the arm's actual current setting).
    ArmMsgMotionCtrl_2 getCommandedMotionCtrlState() const;

    /// Configure this arm's role in a leader/follower (a.k.a. master/slave) setup. Two arms
    /// connected on the same CAN bus can be paired so one acts as a teaching/leader arm
    /// (gravity-compensated free-drive, the human moves it by hand) and the other as a
    /// motion-output/follower arm (torque-controlled, mirrors the leader's motion).
    ///
    /// For a single-arm setup you almost always want `LinkageConfig::MotionOutputArm` -- see
    /// the `setAsSlaveArm()` shortcut.
    ///
    /// The setting is stored in the arm's firmware and persists across reboots.
    ///
    /// @warning Switching this arm out of leader mode requires a **power cycle** before motion
    /// commands take effect again.
    /// @note Sends the master/slave mode config message (CAN ID `0x470`).
    ///
    /// @param linkage_config    Role for this arm. Use `MotionOutputArm` for normal SDK control.
    /// @param feedback_offset   Shifts this arm's feedback CAN IDs to a different range. Used in
    ///                          dual-arm setups so two arms don't collide on the bus. Leave `None`
    ///                          for single-arm.
    /// @param ctrl_offset       Same idea, for this arm's control CAN IDs. Leave `None` for single-arm.
    /// @param linkage_offset    Same idea, for the linkage-mode control target. Leave `None` for single-arm.
    /// @return true on successful CAN send.
    bool setMasterSlaveConfig(
        LinkageConfig linkage_config, CanIdOffset feedback_offset = CanIdOffset::None,
        CanIdOffset ctrl_offset = CanIdOffset::None, CanIdOffset linkage_offset = CanIdOffset::None
    );

    /// Configure this arm as the leader/teaching arm. The arm releases its joints into
    /// gravity-compensated free-drive (you can move it by hand) and broadcasts its state on
    /// the bus, but ignores all motion commands. Use this when the arm is acting as a teleop
    /// input device for another arm to follow.
    ///
    /// @warning After calling this, motion commands will be silently ignored until the arm is
    /// reconfigured back via `setAsSlaveArm()` **and** power-cycled. The setting is persistent.
    /// @return true on successful CAN send.
    bool setAsMasterArm() { return setMasterSlaveConfig(LinkageConfig::TeachingInputArm); }

    /// Configure this arm as the follower/motion-output arm. This is the mode you want for
    /// normal SDK control -- joints become torque-controlled and the arm accepts pose, joint,
    /// and gripper commands. State feedback continues to be published.
    ///
    /// @warning If the arm was previously in leader mode, it must be **power-cycled** after
    /// this call before commands take effect.
    /// @return true on successful CAN send.
    bool setAsSlaveArm() { return setMasterSlaveConfig(LinkageConfig::MotionOutputArm); }

    /// Command the gripper to a target jaw position with a maximum effort. This is the
    /// per-cycle command -- call it continuously (>1 Hz) in your control loop with
    /// `status_code = Enable`, otherwise the gripper firmware will auto-disable about a
    /// second after your last frame.
    ///
    /// @note The gripper firmware ignores `grippers_angle` until the gripper has been zeroed
    /// at least once per power-up. Call `setGripperZero()` first if your gripper isn't homed
    /// (check `getGripperStates().value.foc_status.homing_status`).
    ///
    /// @param grippers_angle    Target jaw stroke in micrometres (0.001 mm units). e.g. 30000 = 30 mm.
    /// @param grippers_effort   Maximum gripper torque in 0.001 N*m. Range 0-5000 (0-5 N*m).
    /// @param status_code       Gripper enable/disable + error-clear command. Default `Enable`.
    /// @return true on successful CAN send.
    bool controlGripper(
        int32_t grippers_angle = 0, uint16_t grippers_effort = 1000,
        GripperStatusCode status_code = GripperStatusCode::Enable
    );

    /// Latch the gripper's current jaw position as the zero reference. Required at least once
    /// per power-up before `controlGripper()` will actually move the jaws -- the firmware blocks
    /// position commands until this has been done. Position the jaws (typically fully closed)
    /// **before** calling.
    ///
    /// One-shot: send this once, then go back to `controlGripper()` calls in your loop.
    /// @return true on successful CAN send.
    bool setGripperZero();

    /// Command target angles for all six joints. The arm splits the targets across three CAN
    /// frames (joints 1+2, 3+4, 5+6); this method sends all three in sequence.
    ///
    /// Pair this with `setMoveMode(MoveMode::MoveJ)` (or `sendMotionCtrl2(...)` once at startup);
    /// the arm uses its joint-space interpolator to move toward each commanded target. Like the
    /// gripper, the arm only acts on these commands when the motion-mode is set to `CanCmd` --
    /// see the motion-mode section above.
    ///
    /// @param joint_1_millideg  Joint 1 target in 0.001 degrees (e.g. 30000 = 30 degrees).
    /// @param joint_2_millideg  Joint 2 target in 0.001 degrees.
    /// @param joint_3_millideg  Joint 3 target in 0.001 degrees.
    /// @param joint_4_millideg  Joint 4 target in 0.001 degrees.
    /// @param joint_5_millideg  Joint 5 target in 0.001 degrees.
    /// @param joint_6_millideg  Joint 6 target in 0.001 degrees.
    /// @return true if all three CAN frames were sent successfully.
    bool controlJoints(
        int32_t joint_1_millideg, int32_t joint_2_millideg, int32_t joint_3_millideg, int32_t joint_4_millideg,
        int32_t joint_5_millideg, int32_t joint_6_millideg
    );

    /// Command an end-effector pose target in cartesian space. Splits across three CAN frames
    /// (X+Y, Z+RX, RY+RZ). Pair with `setMoveMode(MoveMode::MoveP)` (point-to-point),
    /// `MoveMode::MoveL` (linear), or `MoveMode::MoveC` (circular -- requires a sequence of
    /// `moveCAxisUpdate(StartPoint/MidPoint/EndPoint)` calls).
    ///
    /// @param x_um   Target X in 0.001 mm.
    /// @param y_um   Target Y in 0.001 mm.
    /// @param z_um   Target Z in 0.001 mm.
    /// @param rx_md  Rotation about X in 0.001 degrees.
    /// @param ry_md  Rotation about Y in 0.001 degrees.
    /// @param rz_md  Rotation about Z in 0.001 degrees.
    /// @return true if all three CAN frames were sent successfully.
    bool controlEndPose(int32_t x_um, int32_t y_um, int32_t z_um, int32_t rx_md, int32_t ry_md, int32_t rz_md);

    /// MIT joint control for a single joint. Sends one of the per-joint MIT command frames.
    /// Pair with `setMitMode(MitMode::MIT)` and `setMoveMode(MoveMode::MoveM)`. All scalar
    /// inputs are floats in physical units; the encoder converts to the protocol's packed
    /// fixed-point representation internally.
    ///
    /// @param motor_num   Joint index 1-6.
    /// @param pos_rad     Target position in radians, range [-12.5, 12.5].
    /// @param vel         Velocity setpoint, range [-45.0, 45.0].
    /// @param kp          Position-error gain, reference 10.0, range [0.0, 500.0].
    /// @param kd          Velocity-error gain, reference 0.8, range [-5.0, 5.0].
    /// @param torque      Feed-forward torque, range [-8.0, 8.0].
    /// @return true on successful CAN send. Returns false if motor_num is out of range.
    bool controlJointMit(uint8_t motor_num, double pos_rad, double vel, double kp, double kd, double torque);

    // ---- Safety / mode commands (CAN ID 0x150 -- "MotionCtrl_1") ----------------------------

    /// Send the 0x150 motion control 1 message: emergency-stop, trajectory control, and drag-
    /// teach control in one frame. For the common single-purpose calls use the convenience
    /// methods `emergencyStop()` and `resetPiper()` below.
    bool sendMotionCtrl1(
        EmergencyStop emergency_stop = EmergencyStop::NoOp, TrackCtrl track_ctrl = TrackCtrl::Disable,
        DragTeachCtrl drag_teach_ctrl = DragTeachCtrl::Disable
    );

    /// Trigger emergency stop, or resume from a previous emergency stop.
    /// @warning `EmergencyStop::Stop` cuts torque immediately; the arm will fall under gravity.
    bool emergencyStop(EmergencyStop mode = EmergencyStop::Stop);

    /// Reset the arm: immediately drops power to all joints (the arm will fall under gravity)
    /// and clears all error and internal-state flags. Equivalent to sending a power cycle in
    /// software. After reset you'll need to `enableRobot()` and re-establish motion mode.
    bool resetPiper();

    // ---- Circular-pattern coordinate update -------------------------------------------------

    /// MoveC mode coordinate-point update. Use after `setMoveMode(MoveMode::MoveC)` to define
    /// the three points of a circular path: send `controlEndPose(StartXYZ...)` followed by
    /// `moveCAxisUpdate(StartPoint)`, then mid, then end. The arm interpolates a circle through
    /// the three.
    bool moveCAxisUpdate(MoveCInstructionPoint point);

    // ---- Per-motor enable/disable -----------------------------------------------------------

    /// Enable a specific motor. Pass `MotorIndex::AllJoints` (the default) to broadcast --
    /// equivalent to `enableRobot()`.
    bool enableMotor(MotorIndex motor = MotorIndex::AllJoints);

    /// Disable a specific motor. Pass `MotorIndex::AllJoints` (the default) to broadcast --
    /// equivalent to `disableRobot()`. The motor's joint will fall under gravity.
    bool disableMotor(MotorIndex motor = MotorIndex::AllJoints);

    // ---- Joint config (CAN ID 0x475) --------------------------------------------------------

    /// Send the 0x475 joint config message: any combination of "set current pos as zero",
    /// configure max acceleration, or clear joint error.
    ///
    /// @param motor              Which joint motor (1-6 or AllJoints).
    /// @param set_zero           Latch the current position as the joint's zero reference.
    /// @param acc_effective      If `Apply`, `max_joint_acc` is committed; otherwise the field is ignored.
    /// @param max_joint_acc      Max joint acceleration in 0.01 rad/s^2 (range 0-500). Use 0x7FFF to leave unchanged (firmware V1.5-2+).
    /// @param clear_error        Clear any latched joint error code.
    bool sendJointConfig(
        MotorIndex motor, ConfigEffective set_zero, ConfigEffective acc_effective, uint16_t max_joint_acc,
        ConfigEffective clear_error
    );

    /// Convenience: latch the current position as the given joint's zero reference.
    bool setJointAsCurrentZero(MotorIndex motor);

    /// Convenience: set just the max acceleration for one joint, leaving zero/error state alone.
    /// @param max_joint_acc      Max joint acceleration in 0.01 rad/s^2. Range 0-500 (= 0-5 rad/s^2).
    bool setJointMaxAcc(MotorIndex motor, uint16_t max_joint_acc);

    /// Convenience: clear the latched error code on a single joint.
    bool clearJointError(MotorIndex motor);

    // ---- Motor angle/speed limits (CAN ID 0x474) --------------------------------------------

    /// Set per-motor angle limits and maximum joint speed in one frame. Use 0x7FFF on any
    /// field to leave it unchanged (firmware V1.5-2+).
    /// @param motor              Joint motor (1-6).
    /// @param max_angle_01deg    Max angle limit in 0.1 degrees.
    /// @param min_angle_01deg    Min angle limit in 0.1 degrees.
    /// @param max_spd_001rad     Max joint speed in 0.001 rad/s. Range 0-3000.
    bool setMotorAngleLimitsMaxSpeed(
        MotorIndex motor, int16_t max_angle_01deg, int16_t min_angle_01deg, int16_t max_spd_001rad
    );

    /// Convenience: set just the max joint speed for a single motor.
    bool setMotorMaxSpeed(MotorIndex motor, int16_t max_spd_001rad);

    // ---- End-effector velocity/acceleration (CAN ID 0x479) ----------------------------------

    /// Set end-effector velocity/acceleration limits.
    /// @param max_linear_vel_001m_per_s   Max linear velocity in 0.001 m/s.
    /// @param max_angular_vel_001rad_per_s   Max angular velocity in 0.001 rad/s.
    /// @param max_linear_acc_001m_per_s2  Max linear acceleration in 0.001 m/s^2.
    /// @param max_angular_acc_001rad_per_s2  Max angular acceleration in 0.001 rad/s^2.
    bool setEndVelAccLimits(
        int16_t max_linear_vel_001m_per_s, int16_t max_angular_vel_001rad_per_s,
        int16_t max_linear_acc_001m_per_s2, int16_t max_angular_acc_001rad_per_s2
    );

    // ---- Crash protection (CAN ID 0x47A) ----------------------------------------------------

    /// Set per-joint collision-protection sensitivity.
    /// @param j1..j6  Level 0-8 per joint. 0 disables collision detection on that joint;
    /// higher values are more sensitive.
    bool setCrashProtectionLevels(uint8_t j1, uint8_t j2, uint8_t j3, uint8_t j4, uint8_t j5, uint8_t j6);

    // ---- Param enquiry/config (CAN ID 0x477) ------------------------------------------------

    /// Multi-purpose query/config message. Different field combinations request different
    /// firmware behaviours -- see the individual enum values for what each one does.
    bool sendArmParamEnquiryAndConfig(
        ParamEnquiry param_enquiry = ParamEnquiry::None, ParamSetting param_setting = ParamSetting::None,
        PeriodicFeedback48x periodic_feedback = PeriodicFeedback48x::None,
        ConfigEffective end_load_effective = ConfigEffective::NoChange, EndLoad set_end_load = EndLoad::Invalid
    );

    // ---- Gripper teaching-pendant config (CAN ID 0x47D) -------------------------------------

    /// Configure the optional teaching-pendant gripper accessory. Firmware V1.5-2+.
    /// @param teaching_range_per   Travel range coefficient in [100, 200] %.
    /// @param max_range_mm         Max control travel limit in mm. Documented values: 0, 70, 100.
    /// @param teaching_friction    Drag-teach friction coefficient in [1, 10].
    bool setGripperTeachingPendantParam(
        uint8_t teaching_range_per = 100, uint8_t max_range_mm = 70, uint8_t teaching_friction = 1
    );

    // ---- Request master arm to home (CAN ID 0x191) ------------------------------------------

    /// Send the master-arm "go home" request. Firmware V1.7-4+. Used for two-arm leader/follower
    /// setups -- on a single arm in slave mode this command has no effect.
    bool requestMasterArmMoveToHome(MasterArmHomeMode mode);

private:
    /// High‐resolution timestamp in seconds
    static double getCurrentTime()
    {
        using namespace std::chrono;
        return duration<double>(high_resolution_clock::now().time_since_epoch()).count();
    }

    /// Parse a raw CAN frame and update all the TimedFreqState<> members.
    void parseCANFrame(const struct can_frame &, double timestamp);

    bool searchAllMotorLimits();

    bool sendPiperMsg(const PiperMessage &msg);

    bool sendFirmwareQuery();

    std::string can_name_;
    std::unique_ptr<PiperParserBase> parser_;
    std::unique_ptr<PiperForwardKinematics> fk_;
    std::unique_ptr<StdCanInterface> arm_can_;

    bool sdk_joint_limit_{false}, sdk_gripper_limit_{false};

    std::atomic<bool> connected_{false};

    std::thread read_thread_;
    std::atomic<bool> stop_read_{false};

    TimedFreqState<ArmMsgFeedbackStatus> arm_status_;
    TimedFreqState<ArmMsgEndPose> arm_end_pose_;
    TimedFreqState<ArmMsgJointValues> arm_joint_states_;
    TimedFreqState<ArmMsgFeedbackGripper> arm_gripper_msgs_;
    TimedFreqState<ArmMsgFeedbackHighSpdArray> arm_high_spd_fb_;
    TimedFreqState<ArmMsgFeedbackLowSpdArray> arm_low_spd_fb_;
    TimedFreqState<ArmMsgCurrentEndVelAccParam> arm_current_end_vel_acc_;
    TimedFreqState<ArmMsgCrashProtectionRatingConfig> arm_crash_protection_rating_fb_;
    TimedFreqState<ArmMsgGripperTeachingPendantParam> arm_gripper_teaching_pendant_fb_;
    TimedFreqState<ArmMsgCurrentMotorAngleLimitMaxSpd> arm_current_motor_angle_limit_max_spd_;
    TimedFreqState<ArmMsgFeedbackCurrentMotorMaxAccLimit> arm_current_motor_max_acc_limit_;
    TimedFreqState<ArmMsgAllCurrentMotorAngleLimitMaxSpd> arm_all_motor_angle_limit_max_spd_;
    TimedFreqState<ArmMsgFeedbackAllCurrentMotorMaxAccLimit> arm_all_motor_max_acc_limit_;
    TimedFreqState<ArmMsgJointValues> arm_joint_ctrl_msgs_;
    TimedFreqState<ArmMsgGripperCtrl> arm_gripper_ctrl_;
    TimedFreqState<ArmMsgMotionCtrl_2> arm_motion_ctrl_code_151_;
    TimedFreqState<ArmMsgInstructionResponseConfig> arm_instruction_response_;
    TimedFreqState<PiperHealth> arm_health_;

    std::vector<uint8_t> firmware_data_;
    mutable std::mutex firmware_data_mutex_;

    // TX shadow of the last commanded 0x151 state. Granular setters mutate one field under
    // this mutex and re-send the whole message. Initialised to the struct's defaults
    // (CanCmd / MoveJ / 50% / PositionVelocity / 0 / Invalid).
    mutable std::mutex ctrl_151_tx_mutex_;
    ArmMsgMotionCtrl_2 ctrl_151_tx_;
    bool sendCommandedMotionCtrlLocked();

    void updateFeedbackFK();

    void updateCtrlFK();

    // Internal helper threads
    void readLoop();
    void monitorLoop();
};

} // namespace piper_cpp
