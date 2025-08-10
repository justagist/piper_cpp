#pragma once

#include <array>
#include <atomic>
#include <memory>
#include <mutex>
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

    std::vector<uint8_t> getFirmwareData() const
    {
        std::lock_guard<std::mutex> lock(firmware_data_mutex_);
        return firmware_data_;
    }

    std::vector<std::array<double, 6>> getCalculatedFeedbackFK() const;

    std::vector<std::array<double, 6>> getCalculatedControlFK() const;

    StateSnapshot<PiperHealth> getArmHealth() const { return StateSnapshot(arm_health_); }

    bool isHealthy() const { return arm_health_.isValid() && arm_health_.isHealthy(); }

    bool isEnabled() const
    {
        auto lowspd_fb = getArmLowSpeedFeedbacks();
        if (lowspd_fb.is_valid)
        {
            const auto& fb = lowspd_fb.value;
            return std::all_of(
                fb.low_spd_feedbacks.begin(), fb.low_spd_feedbacks.end(),
                [](const ArmMsgFeedbackLowSpd& msg) { return msg.foc_status.driver_enable_status; }
            );
        }
        return false;
    }

    PiperParamManager& getParameterManager() const { return PiperParamManager::instance(); }

private:
    /// High‐resolution timestamp in seconds
    static double getCurrentTime()
    {
        using namespace std::chrono;
        return duration<double>(high_resolution_clock::now().time_since_epoch()).count();
    }

    /// Parse a raw CAN frame and update all the TimedFreqState<> members.
    void parseCANFrame(const struct can_frame&, double timestamp);

    std::string can_name_;
    std::unique_ptr<PiperParserBase> parser_;
    std::unique_ptr<PiperForwardKinematics> fk_;
    std::unique_ptr<StdCanInterface> arm_can_;

    bool sdk_joint_limit_{false}, sdk_gripper_limit_{false};

    std::atomic<bool> connected_{false};

    std::thread read_thread_, monitor_thread_;
    std::atomic<bool> stop_read_{false}, stop_monitor_{false};

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
    TimedFreqState<PiperHealth> arm_health_;

    std::vector<uint8_t> firmware_data_;
    mutable std::mutex firmware_data_mutex_;

    void updateFeedbackFK();

    void updateCtrlFK();

    // Internal helper threads
    void readLoop();
    void monitorLoop();
};

} // namespace piper_cpp