#include "piper_cpp/interface/piper_interface_v2.h"
#include "piper_cpp/protocol/piper_parser_v2.h"
#include <cmath>
#include <cstring>

namespace piper_cpp
{

static constexpr double MILLIDEG2RAD = 0.001 * M_PI / 180;

// Implementation of the PiperInterfaceV2 methods
PiperInterfaceV2::PiperInterfaceV2(
    std::string can_name, bool judge_flag, bool auto_init, bool dh_offset, bool sdk_joint_limit, bool sdk_gripper_limit
)
    : PiperInterfaceV2(
          std::make_unique<PiperParserV2>(), std::move(can_name), judge_flag, auto_init, dh_offset, sdk_joint_limit,
          sdk_gripper_limit
      )
{
}

PiperInterfaceV2::PiperInterfaceV2(
    std::unique_ptr<StdCanInterface> can, bool dh_offset, bool sdk_joint_limit, bool sdk_gripper_limit
)
    : PiperInterfaceV2(std::make_unique<PiperParserV2>(), std::move(can), dh_offset, sdk_joint_limit, sdk_gripper_limit)
{
}

PiperInterfaceV2::PiperInterfaceV2(
    std::unique_ptr<PiperParserBase> parser, std::string can_name, bool judge_flag, bool auto_init, bool dh_offset,
    bool sdk_joint_limit, bool sdk_gripper_limit
)
    : can_name_(std::move(can_name)), parser_(std::move(parser)), sdk_joint_limit_(sdk_joint_limit),
      sdk_gripper_limit_(sdk_gripper_limit)
{
    fk_ = std::make_unique<PiperForwardKinematics>(dh_offset);
    arm_can_ = std::make_unique<StdCanInterface>(
        can_name_, 1000000, judge_flag, auto_init,
        [this](const struct can_frame &frame, double timestamp) { parseCANFrame(frame, timestamp); }
    );

    arm_status_.setNumCallers(1);
    arm_end_pose_.setNumCallers(3);     // end pose is updated in 3 parts
    arm_joint_states_.setNumCallers(3); // joint states are updated in 3 parts
    arm_gripper_msgs_.setNumCallers(1);
    arm_high_spd_fb_.setNumCallers(6); // 6 joints for high-speed feedback
    arm_low_spd_fb_.setNumCallers(6);  // 6 joints for low-speed feedback
    arm_current_end_vel_acc_.setNumCallers(1);
    arm_crash_protection_rating_fb_.setNumCallers(1);
    arm_gripper_teaching_pendant_fb_.setNumCallers(1);
    arm_current_motor_angle_limit_max_spd_.setNumCallers(1);
    arm_current_motor_max_acc_limit_.setNumCallers(1);
    arm_all_motor_angle_limit_max_spd_.setNumCallers(6); // one reply per motor
    arm_all_motor_max_acc_limit_.setNumCallers(6);       // one reply per motor
    arm_joint_ctrl_msgs_.setNumCallers(3); // joint control messages are updated in 3 parts
    arm_gripper_ctrl_.setNumCallers(1);    // gripper control messages
    arm_motion_ctrl_code_151_.setNumCallers(1);
    arm_instruction_response_.setNumCallers(1);
    arm_ik_joint_states_.setNumCallers(3); // IK joints are updated in 3 parts

    // Query responses and command echoes arrive once, not periodically, so their age says
    // nothing about the link; only the periodic feedback streams keep the default timeout.
    arm_crash_protection_rating_fb_.setHealthTimeout(kNoHealthTimeout);
    arm_gripper_teaching_pendant_fb_.setHealthTimeout(kNoHealthTimeout);
    arm_current_motor_angle_limit_max_spd_.setHealthTimeout(kNoHealthTimeout);
    arm_current_motor_max_acc_limit_.setHealthTimeout(kNoHealthTimeout);
    arm_all_motor_angle_limit_max_spd_.setHealthTimeout(kNoHealthTimeout);
    arm_all_motor_max_acc_limit_.setHealthTimeout(kNoHealthTimeout);
    arm_joint_ctrl_msgs_.setHealthTimeout(kNoHealthTimeout);
    arm_gripper_ctrl_.setHealthTimeout(kNoHealthTimeout);
    arm_motion_ctrl_code_151_.setHealthTimeout(kNoHealthTimeout);
    arm_instruction_response_.setHealthTimeout(kNoHealthTimeout);

    /// TODO: initialise other members as in python
}

PiperInterfaceV2::PiperInterfaceV2(
    std::unique_ptr<PiperParserBase> parser, std::unique_ptr<StdCanInterface> can, bool dh_offset, bool sdk_joint_limit,
    bool sdk_gripper_limit
)
    : PiperInterfaceV2(
          std::move(parser), "", /*judge_flag=*/false, /*auto_init=*/false, dh_offset, sdk_joint_limit,
          sdk_gripper_limit
      )
{
    arm_can_ = std::move(can);
    arm_can_->setCallback([this](const struct can_frame &frame, double timestamp) { parseCANFrame(frame, timestamp); });
}

PiperInterfaceV2::~PiperInterfaceV2() { disconnectPort(); }
bool PiperInterfaceV2::connectPort(bool can_init, bool piper_init, bool start_threads)
{
    if (connected_.load(std::memory_order_relaxed))
    {
        return true; // Already connected
    }
    if (!arm_can_->init())
    {
        return false; // Failed to initialize CAN interface
    }
    connected_.store(true, std::memory_order_relaxed);
    if (can_init)
    {
        arm_can_->judgeCanInfo();
    }
    /// TODO: Add piper_init logic and threads
    if (piper_init)
    {
        piperInit(); // Send initial queries
    }
    if (start_threads)
    {
        stop_read_.store(false, std::memory_order_relaxed);
        read_thread_ = std::thread(&PiperInterfaceV2::readLoop, this);
    }
    return true;
}

void PiperInterfaceV2::piperInit()
{
    // Send the three "search" commands for joint limits, acc limits, firmware
    /// TODO: Implement the actual commands to sends
    searchAllMotorLimits();
    sendFirmwareQuery();
}

void PiperInterfaceV2::readLoop()
{
    // readCanMessage() blocks until a frame arrives or disconnectPort() interrupts it, so no
    // sleep is needed and queued frames are drained as fast as they are parsed.
    while (!stop_read_.load(std::memory_order_relaxed) && arm_can_->isOpen())
    {
        arm_can_->readCanMessage();
    }
    if (!stop_read_.load(std::memory_order_relaxed))
    {
        std::cerr << "Error: CAN socket closed unexpectedly; read thread stopped.\n";
    }
}

void PiperInterfaceV2::disconnectPort(std::chrono::milliseconds timeout)
{
    if (!connected_.load(std::memory_order_relaxed))
    {
        return; // Already disconnected
    }
    stop_read_.store(true, std::memory_order_relaxed);
    arm_can_->interruptRead();
    if (read_thread_.joinable())
    {
        read_thread_.join();
    }
    arm_can_->close();
    resetStates();
    connected_.store(false, std::memory_order_relaxed);
}

void PiperInterfaceV2::resetStates()
{
    // Called with the read thread stopped, so nothing races these.
    arm_status_.reset();
    arm_end_pose_.reset();
    arm_joint_states_.reset();
    arm_gripper_msgs_.reset();
    arm_high_spd_fb_.reset();
    arm_low_spd_fb_.reset();
    arm_current_end_vel_acc_.reset();
    arm_crash_protection_rating_fb_.reset();
    arm_gripper_teaching_pendant_fb_.reset();
    arm_current_motor_angle_limit_max_spd_.reset();
    arm_current_motor_max_acc_limit_.reset();
    arm_all_motor_angle_limit_max_spd_.reset();
    arm_all_motor_max_acc_limit_.reset();
    arm_joint_ctrl_msgs_.reset();
    arm_gripper_ctrl_.reset();
    arm_motion_ctrl_code_151_.reset();
    arm_instruction_response_.reset();
    arm_ik_joint_states_.reset();
    arm_health_.reset();
    std::lock_guard<std::mutex> lk(firmware_data_mutex_);
    firmware_data_.clear();
}

void PiperInterfaceV2::parseCANFrame(const struct can_frame &frame, double timestamp)
{
    // Parse the CAN frame using the parser
    PiperMessage msg;
    if (parser_->decodeMessage(frame, timestamp, msg))
    {
        auto health = arm_health_.getValue();
        switch (msg.type)
        {
        case ArmMsgType::StatusFeedback:
            arm_status_.set(msg.arm_status_msgs, msg.timestamp);
            health.arm_status = msg.arm_status_msgs;
            break;
        case ArmMsgType::EndPoseFeedback1:
        {
            auto latest_end_pose = arm_end_pose_.getValue();
            latest_end_pose.X_axis = msg.arm_end_pose.X_axis;
            latest_end_pose.Y_axis = msg.arm_end_pose.Y_axis;
            arm_end_pose_.set(latest_end_pose, msg.timestamp, 0);
            break;
        }
        case ArmMsgType::EndPoseFeedback2:
        {
            auto latest_end_pose = arm_end_pose_.getValue();
            latest_end_pose.Z_axis = msg.arm_end_pose.Z_axis;
            latest_end_pose.RX_axis = msg.arm_end_pose.RX_axis;
            arm_end_pose_.set(latest_end_pose, msg.timestamp, 1);
            break;
        }
        case ArmMsgType::EndPoseFeedback3:
        {
            auto latest_end_pose = arm_end_pose_.getValue();
            latest_end_pose.RY_axis = msg.arm_end_pose.RY_axis;
            latest_end_pose.RZ_axis = msg.arm_end_pose.RZ_axis;
            arm_end_pose_.set(latest_end_pose, msg.timestamp, 2);
            break;
        }
        case ArmMsgType::JointFeedback12:
        case ArmMsgType::JointFeedback34:
        case ArmMsgType::JointFeedback56:
        {
            const size_t part = static_cast<size_t>(msg.type) - static_cast<size_t>(ArmMsgType::JointFeedback12);
            updateJointPair(arm_joint_states_, part, msg.arm_joint_feedback, msg.timestamp, sdk_joint_limit_);
            break;
        }
        case ArmMsgType::IKJointFeedback12:
        case ArmMsgType::IKJointFeedback34:
        case ArmMsgType::IKJointFeedback56:
        {
            // IK solutions are the arm's own targets, not measurements, so the SDK limits
            // that guard feedback are not applied.
            const size_t part = static_cast<size_t>(msg.type) - static_cast<size_t>(ArmMsgType::IKJointFeedback12);
            updateJointPair(arm_ik_joint_states_, part, msg.arm_ik_joint_feedback, msg.timestamp, false);
            break;
        }
        case ArmMsgType::GripperFeedback:
        {
            // Update gripper messages
            auto gripper_fb = msg.gripper_feedback;
            gripper_fb.grippers_angle = clampGripperValue(gripper_fb.grippers_angle, gripper_fb.mode);
            arm_gripper_msgs_.set(gripper_fb, msg.timestamp);
            break;
        }
        case ArmMsgType::HighSpdFeed1:
        case ArmMsgType::HighSpdFeed2:
        case ArmMsgType::HighSpdFeed3:
        case ArmMsgType::HighSpdFeed4:
        case ArmMsgType::HighSpdFeed5:
        case ArmMsgType::HighSpdFeed6:
        {
            // Update high-speed feedback for each joint
            size_t joint_index = static_cast<size_t>(msg.type) - static_cast<size_t>(ArmMsgType::HighSpdFeed1);
            if (joint_index < arm_high_spd_fb_.getNumCallers())
            {
                auto high_spd_fb = arm_high_spd_fb_.getValue();
                high_spd_fb.high_spd_feedbacks[joint_index] = msg.high_spd_feedbacks[joint_index];
                arm_high_spd_fb_.set(high_spd_fb, msg.timestamp, joint_index);
            }
            else
            {
                // Handle unexpected joint index
                std::cerr << "Error: Invalid joint index (or joint index could not be inferred correctly) for "
                             "high-speed feedback: "
                          << joint_index << "\n";
            }
            break;
        }
        case ArmMsgType::LowSpdFeed1:
        case ArmMsgType::LowSpdFeed2:
        case ArmMsgType::LowSpdFeed3:
        case ArmMsgType::LowSpdFeed4:
        case ArmMsgType::LowSpdFeed5:
        case ArmMsgType::LowSpdFeed6:
        {
            // Update low speed feedback for each joint
            // Update high-speed feedback for each joint
            size_t joint_index = static_cast<size_t>(msg.type) - static_cast<size_t>(ArmMsgType::LowSpdFeed1);
            if (joint_index < arm_low_spd_fb_.getNumCallers())
            {
                auto low_spd_fb = arm_low_spd_fb_.getValue();
                low_spd_fb.low_spd_feedbacks[joint_index] = msg.low_spd_feedbacks[joint_index];
                arm_low_spd_fb_.set(low_spd_fb, msg.timestamp, joint_index);
            }
            else
            {
                // Handle unexpected joint index
                std::cerr << "Error: Invalid joint index (or joint index could not be inferred correctly) for "
                             "low-speed feedback: "
                          << joint_index << "\n";
            }
            break;
        }
        case ArmMsgType::FeedbackCurrentEndVelAccParam:
        {
            arm_current_end_vel_acc_.set(msg.arm_feedback_current_end_vel_acc_param, msg.timestamp);
            break;
        }
        case ArmMsgType::CrashProtectionRatingFeedback:
        {
            arm_crash_protection_rating_fb_.set(msg.arm_crash_protection_rating_feedback, msg.timestamp);
            break;
        }
        case ArmMsgType::GripperTeachingPendantParamFeedback:
        {
            arm_gripper_teaching_pendant_fb_.set(msg.arm_gripper_teaching_param_feedback, msg.timestamp);
            break;
        }
        case ArmMsgType::FeedbackCurrentMotorAngleLimitMaxSpd:
        {
            // all_motor_angle_limit_max_spd.motor[];
            if (msg.arm_feedback_current_motor_angle_limit_max_spd.motor_num < 1 ||
                msg.arm_feedback_current_motor_angle_limit_max_spd.motor_num > 6)
            {
                // Handle unexpected motor number
                std::cerr << "Error: Invalid motor number for current motor angle limit max speed: "
                          << msg.arm_feedback_current_motor_angle_limit_max_spd.motor_num << "\n";
            }
            else
            {
                arm_current_motor_angle_limit_max_spd_.set(
                    msg.arm_feedback_current_motor_angle_limit_max_spd, msg.timestamp
                );
                auto all_motor_angle_limit_max_spd = arm_all_motor_angle_limit_max_spd_.getValue();
                all_motor_angle_limit_max_spd.motor[msg.arm_feedback_current_motor_angle_limit_max_spd.motor_num - 1] =
                    msg.arm_feedback_current_motor_angle_limit_max_spd;
                arm_all_motor_angle_limit_max_spd_.set(
                    all_motor_angle_limit_max_spd, msg.timestamp,
                    msg.arm_feedback_current_motor_angle_limit_max_spd.motor_num - 1
                );
            }
            break;
        }
        case ArmMsgType::FeedbackCurrentMotorMaxAccLimit:
        {
            if (msg.arm_feedback_current_motor_max_acc_limit.joint_motor_num < 1 ||
                msg.arm_feedback_current_motor_max_acc_limit.joint_motor_num > 6)
            {
                std::cerr << "Error: Invalid motor number for current motor max acceleration limit: "
                          << msg.arm_feedback_current_motor_max_acc_limit.joint_motor_num << "\n";
            }
            else
            {
                arm_current_motor_max_acc_limit_.set(msg.arm_feedback_current_motor_max_acc_limit, msg.timestamp);
                auto all_motor_max_acc_limit = arm_all_motor_max_acc_limit_.getValue();
                all_motor_max_acc_limit.motor[msg.arm_feedback_current_motor_max_acc_limit.joint_motor_num - 1] =
                    msg.arm_feedback_current_motor_max_acc_limit;
                arm_all_motor_max_acc_limit_.set(
                    all_motor_max_acc_limit, msg.timestamp,
                    msg.arm_feedback_current_motor_max_acc_limit.joint_motor_num - 1
                );
            }
            break;
        }
        case ArmMsgType::JointCtrl12:
        case ArmMsgType::JointCtrl34:
        case ArmMsgType::JointCtrl56:
        {
            const size_t part = static_cast<size_t>(msg.type) - static_cast<size_t>(ArmMsgType::JointCtrl12);
            updateJointPair(arm_joint_ctrl_msgs_, part, msg.arm_joint_ctrl, msg.timestamp, sdk_joint_limit_);
            break;
        }
        case ArmMsgType::GripperCtrl:
        {
            auto gripper_ctrl = msg.gripper_ctrl;
            // A width frame carries 0.001 mm and a V3 angle frame 0.001 deg, so clamp with the
            // limit for the frame's own mode.
            gripper_ctrl.grippers_angle = clampGripperValue(gripper_ctrl.grippers_angle, gripper_ctrl.mode);
            arm_gripper_ctrl_.set(gripper_ctrl, msg.timestamp);
            break;
        }
        case ArmMsgType::MotionCtrl2:
        {
            arm_motion_ctrl_code_151_.set(msg.arm_motion_ctrl_2, msg.timestamp);
            break;
        }
        case ArmMsgType::FirmwareRead:
        {
            // The reply to one query is a few frames; anything beyond this is noise, not the tag.
            static constexpr size_t kMaxFirmwareBytes = 256;
            std::lock_guard<std::mutex> lock(firmware_data_mutex_);
            if (firmware_data_.size() < kMaxFirmwareBytes)
                firmware_data_.insert(firmware_data_.end(), msg.firmware_data.begin(), msg.firmware_data.end());
            break;
        }
        case ArmMsgType::InstructionResponseConfig:
        {
            arm_instruction_response_.set(msg.arm_set_instruction_response, msg.timestamp);
            break;
        }

        default:
            // Handle other message types as needed
            break;
        }

        health.last_msg_type = msg.type;
        arm_health_.set(health, msg.timestamp);
    }
}

void PiperInterfaceV2::updateJointPair(
    TimedFreqState<ArmMsgJointValues> &state, size_t part, const ArmMsgJointValues &src, double timestamp, bool clamp
)
{
    static constexpr int32_t ArmMsgJointValues::*kJoints[] = {
        &ArmMsgJointValues::joint_1, &ArmMsgJointValues::joint_2, &ArmMsgJointValues::joint_3,
        &ArmMsgJointValues::joint_4, &ArmMsgJointValues::joint_5, &ArmMsgJointValues::joint_6,
    };
    auto &pm = getParameterManager();
    auto values = state.getValue();
    for (size_t j = 2 * part; j < 2 * part + 2; ++j)
    {
        const int32_t v = src.*kJoints[j];
        values.*kJoints[j] = clamp ? pm.clampJointMilliDeg(static_cast<PiperParamManager::Joint>(j), v) : v;
    }
    state.set(values, timestamp, part);
}

int32_t PiperInterfaceV2::clampGripperValue(int32_t value, GripperMode mode) const
{
    if (!sdk_gripper_limit_)
        return value;
    switch (mode)
    {
    case GripperMode::Width:
        return getParameterManager().clampGripperMicrometers(value);
    case GripperMode::Angle:
        return getParameterManager().clampGripperMilliDeg(value);
    default:
        return value; // no limit is known for a mode this library cannot interpret
    }
}

std::vector<std::array<double, 6>> PiperInterfaceV2::getCalculatedFeedbackFK() const
{
    auto joint_fb = getArmJointStates();

    if (!joint_fb.is_valid)
    {
        std::cerr << "Error: Joint feedback is not valid, cannot calculate forward kinematics.\n";
    }

    return fk_->calcFK(
        {joint_fb.value.joint_1 * MILLIDEG2RAD, // Convert from millidegrees to radians
         joint_fb.value.joint_2 * MILLIDEG2RAD, joint_fb.value.joint_3 * MILLIDEG2RAD,
         joint_fb.value.joint_4 * MILLIDEG2RAD, joint_fb.value.joint_5 * MILLIDEG2RAD,
         joint_fb.value.joint_6 * MILLIDEG2RAD}
    );
}

std::vector<std::array<double, 6>> PiperInterfaceV2::getCalculatedControlFK() const
{
    auto joint_ctrl = getArmJointCtrlMsgs();
    if (!joint_ctrl.is_valid)
    {
        std::cerr << "Error: Joint control is not valid, cannot calculate forward kinematics.\n";
    }
    return fk_->calcFK(
        {joint_ctrl.value.joint_1 * MILLIDEG2RAD, // Convert from millidegrees to radians
         joint_ctrl.value.joint_2 * MILLIDEG2RAD, joint_ctrl.value.joint_3 * MILLIDEG2RAD,
         joint_ctrl.value.joint_4 * MILLIDEG2RAD, joint_ctrl.value.joint_5 * MILLIDEG2RAD,
         joint_ctrl.value.joint_6 * MILLIDEG2RAD}
    );
}

bool PiperInterfaceV2::sendPiperMsg(const PiperMessage &msg)
{
    if (!connected_.load(std::memory_order_relaxed))
    {
        std::cerr << "Error: Cannot send message, interface is not connected.\n";
        return false;
    }
    struct can_frame tx
    {
    };
    if (!parser_->encodeMessage(msg, tx))
    {
        std::cerr << "Error: Failed to encode Piper message.\n";
        return false;
    }
    std::vector<uint8_t> data(tx.data, tx.data + tx.can_dlc);
    if (!arm_can_->sendCanMessage(tx.can_id, data))
    {
        std::cerr << "Error: Failed to send CAN message.\n";
        return false;
    }
    return true;
}

bool PiperInterfaceV2::enableRobot()
{
    // Send the enable command to the robot
    PiperMessage msg;
    msg.type = ArmMsgType::MotorEnableDisableConfig;
    msg.arm_motor_enable = ArmMsgMotorEnableDisableConfig({7, 0x02});
    return sendPiperMsg(msg);
}

bool PiperInterfaceV2::disableRobot()
{
    // Send the disable command to the robot
    PiperMessage msg;
    msg.type = ArmMsgType::MotorEnableDisableConfig;
    msg.arm_motor_enable = ArmMsgMotorEnableDisableConfig({7, 0x01});
    return sendPiperMsg(msg);
}

bool PiperInterfaceV2::sendMotionCtrl2Locked(const ArmMsgMotionCtrl_2 &next)
{
    PiperMessage msg;
    msg.type = ArmMsgType::MotionCtrl2;
    msg.arm_motion_ctrl_2 = next;
    if (!sendPiperMsg(msg))
        return false;
    ctrl_151_tx_ = next;
    return true;
}

bool PiperInterfaceV2::sendMotionCtrl2(
    CtrlMode ctrl_mode, MoveMode move_mode, uint8_t move_spd_rate_ctrl, MitMode mit_mode, uint8_t residence_time,
    InstallationPos installation_pos
)
{
    if (move_spd_rate_ctrl > 100)
        move_spd_rate_ctrl = 100;
    std::lock_guard<std::mutex> lock(ctrl_151_tx_mutex_);
    return sendMotionCtrl2Locked(ArmMsgMotionCtrl_2{
        static_cast<uint8_t>(ctrl_mode),
        static_cast<uint8_t>(move_mode),
        move_spd_rate_ctrl,
        static_cast<uint8_t>(mit_mode),
        residence_time,
        static_cast<uint8_t>(installation_pos),
    });
}

bool PiperInterfaceV2::setCtrlMode(CtrlMode ctrl_mode)
{
    std::lock_guard<std::mutex> lock(ctrl_151_tx_mutex_);
    auto next = ctrl_151_tx_;
    next.ctrl_mode = static_cast<uint8_t>(ctrl_mode);
    return sendMotionCtrl2Locked(next);
}

bool PiperInterfaceV2::setMoveMode(MoveMode move_mode)
{
    std::lock_guard<std::mutex> lock(ctrl_151_tx_mutex_);
    auto next = ctrl_151_tx_;
    next.move_mode = static_cast<uint8_t>(move_mode);
    return sendMotionCtrl2Locked(next);
}

bool PiperInterfaceV2::setSpeedPercentage(uint8_t percentage)
{
    if (percentage > 100)
        percentage = 100;
    std::lock_guard<std::mutex> lock(ctrl_151_tx_mutex_);
    auto next = ctrl_151_tx_;
    next.move_spd_rate_ctrl = percentage;
    return sendMotionCtrl2Locked(next);
}

bool PiperInterfaceV2::setMitMode(MitMode mit_mode)
{
    std::lock_guard<std::mutex> lock(ctrl_151_tx_mutex_);
    auto next = ctrl_151_tx_;
    next.mit_mode = static_cast<uint8_t>(mit_mode);
    return sendMotionCtrl2Locked(next);
}

bool PiperInterfaceV2::setResidenceTime(uint8_t seconds)
{
    std::lock_guard<std::mutex> lock(ctrl_151_tx_mutex_);
    auto next = ctrl_151_tx_;
    next.residence_time = seconds;
    return sendMotionCtrl2Locked(next);
}

bool PiperInterfaceV2::setInstallationPos(InstallationPos installation_pos)
{
    std::lock_guard<std::mutex> lock(ctrl_151_tx_mutex_);
    auto next = ctrl_151_tx_;
    next.installation_pos = static_cast<uint8_t>(installation_pos);
    return sendMotionCtrl2Locked(next);
}

ArmMsgMotionCtrl_2 PiperInterfaceV2::getCommandedMotionCtrlState() const
{
    std::lock_guard<std::mutex> lock(ctrl_151_tx_mutex_);
    return ctrl_151_tx_;
}

bool PiperInterfaceV2::setMasterSlaveConfig(
    LinkageConfig linkage_config, CanIdOffset feedback_offset, CanIdOffset ctrl_offset, CanIdOffset linkage_offset
)
{
    PiperMessage msg;
    msg.type = ArmMsgType::MasterSlaveModeConfig;
    msg.arm_ms_config = ArmMsgMasterSlaveModeConfig{
        static_cast<uint8_t>(linkage_config),
        static_cast<uint8_t>(feedback_offset),
        static_cast<uint8_t>(ctrl_offset),
        static_cast<uint8_t>(linkage_offset),
    };
    return sendPiperMsg(msg);
}

bool PiperInterfaceV2::controlGripper(int32_t grippers_angle, uint16_t grippers_effort, GripperStatusCode status_code)
{
    return sendGripperCtrl(GripperMode::Width, grippers_angle, grippers_effort, status_code);
}

bool PiperInterfaceV2::sendGripperCtrl(GripperMode mode, int32_t value, uint16_t effort, GripperStatusCode status_code)
{
    PiperMessage msg;
    msg.type = ArmMsgType::GripperCtrl;
    msg.gripper_ctrl = ArmMsgGripperCtrl{
        value, effort, static_cast<uint8_t>(status_code), static_cast<uint8_t>(GripperSetZero::NoOp), mode,
    };
    return sendPiperMsg(msg);
}

bool PiperInterfaceV2::setGripperZero()
{
    // Mirrors the official piper_set_gripper_zero.py: send with status_code=Disable so the
    // gripper is not actively driving while latching the zero reference. The angle/effort
    // fields are ignored for this one-shot.
    PiperMessage msg;
    msg.type = ArmMsgType::GripperCtrl;
    msg.gripper_ctrl = ArmMsgGripperCtrl{
        0,
        0,
        static_cast<uint8_t>(GripperStatusCode::Disable),
        static_cast<uint8_t>(GripperSetZero::SetZero),
    };
    return sendPiperMsg(msg);
}

bool PiperInterfaceV2::controlJoints(
    int32_t joint_1_millideg, int32_t joint_2_millideg, int32_t joint_3_millideg, int32_t joint_4_millideg,
    int32_t joint_5_millideg, int32_t joint_6_millideg
)
{
    // The arm splits joint targets across three CAN frames: 1+2 -> 0x155, 3+4 -> 0x156, 5+6 -> 0x157.
    // We pack the same `arm_joint_ctrl` struct each time; only the message type changes which
    // determines which two int32 fields the parser writes onto the bus.
    PiperMessage msg;
    msg.arm_joint_ctrl = ArmMsgJointValues{
        joint_1_millideg, joint_2_millideg, joint_3_millideg,
        joint_4_millideg, joint_5_millideg, joint_6_millideg,
    };
    std::lock_guard<std::mutex> lock(multipart_tx_mutex_);
    msg.type = ArmMsgType::JointCtrl12;
    if (!sendPiperMsg(msg))
        return false;
    msg.type = ArmMsgType::JointCtrl34;
    if (!sendPiperMsg(msg))
        return false;
    msg.type = ArmMsgType::JointCtrl56;
    return sendPiperMsg(msg);
}

bool PiperInterfaceV2::controlEndPose(int32_t x_um, int32_t y_um, int32_t z_um, int32_t rx_md, int32_t ry_md, int32_t rz_md)
{
    PiperMessage msg;
    msg.arm_motion_ctrl_cartesian = ArmMsgEndPose{x_um, y_um, z_um, rx_md, ry_md, rz_md};
    std::lock_guard<std::mutex> lock(multipart_tx_mutex_);
    msg.type = ArmMsgType::MotionCtrlCartesian1;
    if (!sendPiperMsg(msg))
        return false;
    msg.type = ArmMsgType::MotionCtrlCartesian2;
    if (!sendPiperMsg(msg))
        return false;
    msg.type = ArmMsgType::MotionCtrlCartesian3;
    return sendPiperMsg(msg);
}

bool PiperInterfaceV2::controlJointMit(
    uint8_t motor_num, double pos_rad, double vel, double kp, double kd, double torque
)
{
    if (motor_num < 1 || motor_num > 6)
    {
        std::cerr << "controlJointMit: motor_num " << int(motor_num) << " out of range [1,6]\n";
        return false;
    }
    // A NaN passes every range check and has no defined integer conversion, so it would put an
    // arbitrary target on the bus.
    if (!std::isfinite(pos_rad) || !std::isfinite(vel) || !std::isfinite(kp) || !std::isfinite(kd) ||
        !std::isfinite(torque))
    {
        std::cerr << "controlJointMit: non-finite input for motor " << int(motor_num) << "\n";
        return false;
    }

    PiperMessage msg;
    const size_t idx = motor_num - 1;
    msg.arm_joint_mit_ctrl.motors[idx] = parser_->packJointMit(pos_rad, vel, kp, kd, torque);
    msg.type = static_cast<ArmMsgType>(static_cast<int>(ArmMsgType::JointMitCtrl1) + idx);
    return sendPiperMsg(msg);
}

bool PiperInterfaceV2::sendMotionCtrl1(EmergencyStop emergency_stop, TrackCtrl track_ctrl, DragTeachCtrl drag_teach_ctrl)
{
    PiperMessage msg;
    msg.type = ArmMsgType::MotionCtrl1;
    msg.arm_motion_ctrl_1 = ArmMsgMotionCtrl_1{};
    msg.arm_motion_ctrl_1.emergency_stop = static_cast<uint8_t>(emergency_stop);
    msg.arm_motion_ctrl_1.track_ctrl = static_cast<uint8_t>(track_ctrl);
    msg.arm_motion_ctrl_1.grag_teach_ctrl = static_cast<uint8_t>(drag_teach_ctrl);
    return sendPiperMsg(msg);
}

bool PiperInterfaceV2::emergencyStop(EmergencyStop mode)
{
    return sendMotionCtrl1(mode, TrackCtrl::Disable, DragTeachCtrl::Disable);
}

bool PiperInterfaceV2::resetPiper()
{
    // Mirrors Python's ResetPiper(): emergency_stop=Resume on the 0x150 message.
    return sendMotionCtrl1(EmergencyStop::Resume, TrackCtrl::Disable, DragTeachCtrl::Disable);
}

bool PiperInterfaceV2::moveCAxisUpdate(MoveCInstructionPoint point)
{
    PiperMessage msg;
    msg.type = ArmMsgType::CircularPatternCoordNumUpdateCtrl;
    msg.arm_circular_ctrl.instruction_num = static_cast<uint8_t>(point);
    return sendPiperMsg(msg);
}

bool PiperInterfaceV2::enableMotor(MotorIndex motor)
{
    PiperMessage msg;
    msg.type = ArmMsgType::MotorEnableDisableConfig;
    msg.arm_motor_enable = ArmMsgMotorEnableDisableConfig{static_cast<uint8_t>(motor), 0x02};
    return sendPiperMsg(msg);
}

bool PiperInterfaceV2::disableMotor(MotorIndex motor)
{
    PiperMessage msg;
    msg.type = ArmMsgType::MotorEnableDisableConfig;
    msg.arm_motor_enable = ArmMsgMotorEnableDisableConfig{static_cast<uint8_t>(motor), 0x01};
    return sendPiperMsg(msg);
}

bool PiperInterfaceV2::sendJointConfig(
    MotorIndex motor, ConfigEffective set_zero, ConfigEffective acc_effective, uint16_t max_joint_acc,
    ConfigEffective clear_error
)
{
    PiperMessage msg;
    msg.type = ArmMsgType::JointConfig;
    msg.arm_joint_config = ArmMsgJointConfig{};
    msg.arm_joint_config.joint_motor_num = static_cast<uint8_t>(motor);
    msg.arm_joint_config.set_motor_current_pos_as_zero = static_cast<uint8_t>(set_zero);
    msg.arm_joint_config.acc_param_config_is_effective_or_not = static_cast<uint8_t>(acc_effective);
    msg.arm_joint_config.max_joint_acc = max_joint_acc;
    msg.arm_joint_config.clear_joint_err = static_cast<uint8_t>(clear_error);
    return sendPiperMsg(msg);
}

bool PiperInterfaceV2::setJointAsCurrentZero(MotorIndex motor)
{
    return sendJointConfig(motor, ConfigEffective::Apply, ConfigEffective::NoChange, 0, ConfigEffective::NoChange);
}

bool PiperInterfaceV2::setJointMaxAcc(MotorIndex motor, uint16_t max_joint_acc)
{
    return sendJointConfig(motor, ConfigEffective::NoChange, ConfigEffective::Apply, max_joint_acc, ConfigEffective::NoChange);
}

bool PiperInterfaceV2::clearJointError(MotorIndex motor)
{
    return sendJointConfig(motor, ConfigEffective::NoChange, ConfigEffective::NoChange, 0, ConfigEffective::Apply);
}

bool PiperInterfaceV2::setMotorAngleLimitsMaxSpeed(
    MotorIndex motor, int16_t max_angle_01deg, int16_t min_angle_01deg, int16_t max_spd
)
{
    PiperMessage msg;
    msg.type = ArmMsgType::MotorAngleLimitMaxSpdSet;
    msg.arm_motor_angle_limit_max_spd_set =
        ArmMsgCurrentMotorAngleLimitMaxSpd{static_cast<uint8_t>(motor), max_angle_01deg, min_angle_01deg, max_spd};
    return sendPiperMsg(msg);
}

bool PiperInterfaceV2::setMotorMaxSpeed(MotorIndex motor, int16_t max_spd)
{
    // 0x7FFF on the angle limit fields tells firmware V1.5-2+ to leave them untouched.
    constexpr int16_t LEAVE = static_cast<int16_t>(0x7FFF);
    return setMotorAngleLimitsMaxSpeed(motor, LEAVE, LEAVE, max_spd);
}

bool PiperInterfaceV2::setEndVelAccLimits(
    int16_t max_linear_vel_001m_per_s, int16_t max_angular_vel_001rad_per_s, int16_t max_linear_acc_001m_per_s2,
    int16_t max_angular_acc_001rad_per_s2
)
{
    PiperMessage msg;
    msg.type = ArmMsgType::EndVelAccParamConfig;
    msg.arm_end_vel_acc_param_config = ArmMsgCurrentEndVelAccParam{
        max_linear_vel_001m_per_s, max_angular_vel_001rad_per_s,
        max_linear_acc_001m_per_s2, max_angular_acc_001rad_per_s2,
    };
    return sendPiperMsg(msg);
}

bool PiperInterfaceV2::setCrashProtectionLevels(uint8_t j1, uint8_t j2, uint8_t j3, uint8_t j4, uint8_t j5, uint8_t j6)
{
    PiperMessage msg;
    msg.type = ArmMsgType::CrashProtectionRatingConfig;
    msg.arm_crash_protection_rating_config = ArmMsgCrashProtectionRatingConfig{j1, j2, j3, j4, j5, j6};
    return sendPiperMsg(msg);
}

bool PiperInterfaceV2::sendArmParamEnquiryAndConfig(
    ParamEnquiry param_enquiry, ParamSetting param_setting, PeriodicFeedback48x periodic_feedback,
    ConfigEffective end_load_effective, EndLoad set_end_load
)
{
    PiperMessage msg;
    msg.type = ArmMsgType::ParamEnquiryAndConfig;
    msg.arm_param_enquiry_and_config = ArmMsgParamEnquiryAndConfig{};
    msg.arm_param_enquiry_and_config.param_enquiry = static_cast<uint8_t>(param_enquiry);
    msg.arm_param_enquiry_and_config.param_setting = static_cast<uint8_t>(param_setting);
    msg.arm_param_enquiry_and_config.data_feedback_0x48x = static_cast<uint8_t>(periodic_feedback);
    msg.arm_param_enquiry_and_config.end_load_param_setting_effective = static_cast<uint8_t>(end_load_effective);
    msg.arm_param_enquiry_and_config.set_end_load = static_cast<uint8_t>(set_end_load);
    return sendPiperMsg(msg);
}

bool PiperInterfaceV2::setGripperTeachingPendantParam(
    uint8_t teaching_range_per, uint8_t max_range_mm, uint8_t teaching_friction
)
{
    PiperMessage msg;
    msg.type = ArmMsgType::GripperTeachingPendantParamConfig;
    msg.arm_gripper_teaching_param_config =
        ArmMsgGripperTeachingPendantParam{teaching_range_per, max_range_mm, teaching_friction};
    return sendPiperMsg(msg);
}

bool PiperInterfaceV2::requestMasterArmMoveToHome(MasterArmHomeMode mode)
{
    if (!connected_.load(std::memory_order_relaxed))
    {
        std::cerr << "requestMasterArmMoveToHome: not connected.\n";
        return false;
    }
    // 0x191 is sent verbatim from the python SDK; not currently routed through the encoder.
    std::vector<uint8_t> payload(8, 0);
    payload[0] = 0x01;
    switch (mode)
    {
    case MasterArmHomeMode::RestoreMasterSlaveMode:
        // {0x01, 0, 0, 0, 0, 0, 0, 0}
        break;
    case MasterArmHomeMode::MasterArmReturnToZero:
        payload[1] = 0x01;
        payload[2] = 0x01;
        break;
    case MasterArmHomeMode::BothMasterAndSlaveReturnToZero:
        payload[2] = 0x01;
        break;
    }
    return arm_can_->sendCanMessage(0x191, payload);
}

bool PiperInterfaceV2::searchAllMotorLimits()
{
    for (size_t motor_num = 1; motor_num <= PiperMessage::num_joints; ++motor_num)
    {
        PiperMessage msg1;
        msg1.type = ArmMsgType::SearchMotorMaxAngleSpdAccLimit;
        msg1.arm_search_motor_max_angle_spd_acc_limit =
            ArmMsgSearchMotorMaxAngleSpdAccLimit({static_cast<uint8_t>(motor_num), 0x01});
        if (!sendPiperMsg(msg1))
        {
            std::cerr << "Error: Failed to send search angle/speed limit for motor " << motor_num << ".\n";
            return false;
        }

        PiperMessage msg2;
        msg2.type = ArmMsgType::SearchMotorMaxAngleSpdAccLimit;
        msg2.arm_search_motor_max_angle_spd_acc_limit =
            ArmMsgSearchMotorMaxAngleSpdAccLimit({static_cast<uint8_t>(motor_num), 0x02});
        if (!sendPiperMsg(msg2))
        {
            std::cerr << "Error: Failed to send search acc limits for motor " << motor_num << ".\n";
            return false;
        }
    }
    return true;
}

bool PiperInterfaceV2::waitForMotorLimits(std::chrono::milliseconds timeout, std::chrono::milliseconds retry_interval)
{
    using clock = std::chrono::steady_clock;
    const auto deadline = clock::now() + timeout;
    // Success must mean replies to these queries, not whatever an earlier session cached.
    arm_all_motor_angle_limit_max_spd_.reset();
    arm_all_motor_max_acc_limit_.reset();
    auto last_sent = clock::now();
    if (!searchAllMotorLimits())
        return false; // not connected or the bus is down: waiting cannot help
    while (!(arm_all_motor_angle_limit_max_spd_.isValid() && arm_all_motor_max_acc_limit_.isValid()))
    {
        if (clock::now() >= deadline)
            return false;
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        if (clock::now() - last_sent >= retry_interval)
        {
            searchAllMotorLimits();
            last_sent = clock::now();
        }
    }
    return true;
}

bool PiperInterfaceV2::sendFirmwareQuery()
{
    if (!connected_.load(std::memory_order_relaxed))
    {
        std::cerr << "Error: Cannot send message, interface is not connected.\n";
        return false;
    }

    std::vector<uint8_t> data{0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    uint16_t can_id = 0x4AF;

    {
        // Each query gets its own reply buffer, so a re-query after a firmware update cannot
        // keep reporting the old tag.
        std::lock_guard<std::mutex> lock(firmware_data_mutex_);
        firmware_data_.clear();
    }
    if (!arm_can_->sendCanMessage(can_id, data))
    {
        std::cerr << "Error: Failed to send firmware query CAN message.\n";
        return false;
    }
    return true;
}

bool PiperInterfaceV2::requestFirmwareVersion() { return sendFirmwareQuery(); }

std::optional<std::string> PiperInterfaceV2::getFirmwareVersion() const
{
    // Mirrors python piper_sdk's GetPiperFirmwareVersion: locate the "S-V" marker in the
    // streamed firmware-info bytes and return the 8-byte tag that follows.
    static constexpr char marker[] = "S-V";
    static constexpr size_t marker_len = sizeof(marker) - 1; // exclude trailing null
    static constexpr size_t version_len = 8;

    std::lock_guard<std::mutex> lock(firmware_data_mutex_);
    if (firmware_data_.size() < marker_len)
        return std::nullopt;

    for (size_t i = 0; i + marker_len <= firmware_data_.size(); ++i)
    {
        if (std::memcmp(firmware_data_.data() + i, marker, marker_len) == 0)
        {
            // The tag spans frames; report it only once it is all here.
            if (i + version_len > firmware_data_.size())
                return std::nullopt;
            return std::string(firmware_data_.begin() + i, firmware_data_.begin() + i + version_len);
        }
    }
    return std::nullopt;
}

} // namespace piper_cpp
