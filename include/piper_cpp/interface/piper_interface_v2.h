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

    /// Turn on/off FK‐calculation in the message loop
    void enableFkCal() { start_fk_cal_.store(true, std::memory_order_relaxed); }
    void disableFkCal() { start_fk_cal_.store(false, std::memory_order_relaxed); }
    bool isCalFk() const { return start_fk_cal_.load(std::memory_order_relaxed); }

    StateSnapShot<ArmMsgFeedbackStatus> getArmStatus() const { return StateSnapShot(arm_status_); }

    StateSnapShot<ArmMsgEndPose> getArmEndPose() const { return StateSnapShot(arm_end_pose_); }

    StateSnapShot<ArmMsgJointValues> getArmJointStates() const { return StateSnapShot(arm_joint_states_); }

    StateSnapShot<ArmMsgFeedbackGripper> getArmGripperStates() const { return StateSnapShot(arm_gripper_msgs_); }

    StateSnapShot<std::array<ArmMsgFeedbackHighSpd, 6>> getArmHighSpeedFeedbacks() const
    {
        return StateSnapShot(arm_high_spd_fb_);
    }

    StateSnapShot<std::array<ArmMsgFeedbackLowSpd, 6>> getArmLowSpeedFeedbacks() const
    {
        return StateSnapShot(arm_low_spd_fb_);
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
    std::atomic<bool> start_fk_cal_{false};

    std::thread read_thread_, monitor_thread_;
    std::atomic<bool> stop_read_{false}, stop_monitor_{false};

    TimedFreqState<ArmMsgFeedbackStatus> arm_status_;
    TimedFreqState<ArmMsgEndPose> arm_end_pose_;
    TimedFreqState<ArmMsgJointValues> arm_joint_states_;
    TimedFreqState<ArmMsgFeedbackGripper> arm_gripper_msgs_;
    TimedFreqState<std::array<ArmMsgFeedbackHighSpd, 6>> arm_high_spd_fb_;
    TimedFreqState<std::array<ArmMsgFeedbackLowSpd, 6>> arm_low_spd_fb_;

    // Internal helper threads
    void readLoop();
    void monitorLoop();
};

} // namespace piper_cpp