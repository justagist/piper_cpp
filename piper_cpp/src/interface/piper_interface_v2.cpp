#include "piper_cpp/interface/piper_interface_v2.h"
#include "piper_cpp/protocol/piper_parser_v2.h"
#include <cmath>
#include <cstring>

namespace piper_cpp
{

static constexpr double MILLIDEG2RAD = 0.001 * 3.14 / 180;

// Implementation of the PiperInterfaceV2 methods
PiperInterfaceV2::PiperInterfaceV2(
    std::string can_name, bool judge_flag, bool auto_init, bool dh_offset, bool sdk_joint_limit, bool sdk_gripper_limit
)
    : can_name_(std::move(can_name)), sdk_joint_limit_(sdk_joint_limit), sdk_gripper_limit_(sdk_gripper_limit)
{
    // Initialize parser and other members here
    parser_ = std::make_unique<PiperParserV2>();
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
    arm_all_motor_angle_limit_max_spd_.setNumCallers(1);
    arm_all_motor_max_acc_limit_.setNumCallers(1);
    arm_joint_ctrl_msgs_.setNumCallers(3); // joint control messages are updated in 3 parts
    arm_gripper_ctrl_.setNumCallers(1);    // gripper control messages
    arm_motion_ctrl_code_151_.setNumCallers(1);
    arm_instruction_response_.setNumCallers(1);

    /// TODO: initialise other members as in python
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
    while (!stop_read_.load(std::memory_order_relaxed))
    {
        if (arm_can_->readCanMessage())
        {
            // Process the received CAN message
            // The callback will handle parsing and updating state
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1)); // TODO: Adjust sleep duration as needed
    }
}

void PiperInterfaceV2::disconnectPort(std::chrono::milliseconds timeout)
{
    if (!connected_.load(std::memory_order_relaxed))
    {
        return; // Already disconnected
    }
    stop_read_.store(true, std::memory_order_relaxed);
    if (read_thread_.joinable())
    {
        read_thread_.join();
    }
    arm_can_->close();
    connected_.store(false, std::memory_order_relaxed);
}

void PiperInterfaceV2::parseCANFrame(const struct can_frame &frame, double timestamp)
{
    // Parse the CAN frame using the parser
    PiperMessage msg;
    if (parser_->decodeMessage(frame, timestamp, msg))
    {
        auto &pm = getParameterManager();
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
        {
            auto joint_values = arm_joint_states_.getValue();
            if (sdk_joint_limit_)
            {
                joint_values.joint_1 =
                    pm.clampJointMilliDeg(PiperParamManager::Joint::J1, msg.arm_joint_feedback.joint_1);
                joint_values.joint_2 =
                    pm.clampJointMilliDeg(PiperParamManager::Joint::J2, msg.arm_joint_feedback.joint_2);
            }
            else
            {
                joint_values.joint_1 = msg.arm_joint_feedback.joint_1;
                joint_values.joint_2 = msg.arm_joint_feedback.joint_2;
            }
            arm_joint_states_.set(joint_values, msg.timestamp, 0);
            break;
        }
        case ArmMsgType::JointFeedback34:
        {
            auto joint_values = arm_joint_states_.getValue();
            if (sdk_joint_limit_)
            {
                joint_values.joint_3 =
                    pm.clampJointMilliDeg(PiperParamManager::Joint::J3, msg.arm_joint_feedback.joint_3);
                joint_values.joint_4 =
                    pm.clampJointMilliDeg(PiperParamManager::Joint::J4, msg.arm_joint_feedback.joint_4);
            }
            else
            {
                joint_values.joint_3 = msg.arm_joint_feedback.joint_3;
                joint_values.joint_4 = msg.arm_joint_feedback.joint_4;
            }
            arm_joint_states_.set(joint_values, msg.timestamp, 1);
            break;
        }
        case ArmMsgType::JointFeedback56:
        {
            auto joint_values = arm_joint_states_.getValue();
            if (sdk_joint_limit_)
            {
                joint_values.joint_5 =
                    pm.clampJointMilliDeg(PiperParamManager::Joint::J5, msg.arm_joint_feedback.joint_5);
                joint_values.joint_6 =
                    pm.clampJointMilliDeg(PiperParamManager::Joint::J6, msg.arm_joint_feedback.joint_6);
            }
            else
            {
                joint_values.joint_5 = msg.arm_joint_feedback.joint_5;
                joint_values.joint_6 = msg.arm_joint_feedback.joint_6;
            }
            arm_joint_states_.set(joint_values, msg.timestamp, 2);
            break;
        }
        case ArmMsgType::GripperFeedback:
        {
            // Update gripper messages
            auto gripper_fb = msg.gripper_feedback;
            if (sdk_gripper_limit_)
            {
                gripper_fb.grippers_angle = pm.clampGripperMicrometers(gripper_fb.grippers_angle);
            }
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
            arm_current_motor_angle_limit_max_spd_.set(
                msg.arm_feedback_current_motor_angle_limit_max_spd, msg.timestamp
            );
            // all_motor_angle_limit_max_spd.motor[];
            if (msg.arm_feedback_current_motor_angle_limit_max_spd.motor_num > 6)
            {
                // Handle unexpected motor number
                std::cerr << "Error: Invalid motor number for current motor angle limit max speed: "
                          << msg.arm_feedback_current_motor_angle_limit_max_spd.motor_num << "\n";
            }
            else
            {
                auto all_motor_angle_limit_max_spd = arm_all_motor_angle_limit_max_spd_.getValue();
                all_motor_angle_limit_max_spd.motor[msg.arm_feedback_current_motor_angle_limit_max_spd.motor_num - 1] =
                    msg.arm_feedback_current_motor_angle_limit_max_spd;
                arm_all_motor_angle_limit_max_spd_.set(all_motor_angle_limit_max_spd, msg.timestamp);
            }
            break;
        }
        case ArmMsgType::FeedbackCurrentMotorMaxAccLimit:
        {
            arm_current_motor_max_acc_limit_.set(msg.arm_feedback_current_motor_max_acc_limit, msg.timestamp);
            if (msg.arm_feedback_current_motor_max_acc_limit.joint_motor_num > 6)
            {
                std::cerr << "Error: Invalid motor number for current motor max acceleration limit: "
                          << msg.arm_feedback_current_motor_max_acc_limit.joint_motor_num << "\n";
            }
            else
            {
                auto all_motor_max_acc_limit = arm_all_motor_max_acc_limit_.getValue();
                all_motor_max_acc_limit.motor[msg.arm_feedback_current_motor_max_acc_limit.joint_motor_num - 1] =
                    msg.arm_feedback_current_motor_max_acc_limit;
                arm_all_motor_max_acc_limit_.set(all_motor_max_acc_limit, msg.timestamp);
            }
            break;
        }
        case ArmMsgType::JointCtrl12:
        {
            auto joint_ctrl = arm_joint_ctrl_msgs_.getValue();
            if (sdk_joint_limit_)
            {
                joint_ctrl.joint_1 = pm.clampJointMilliDeg(PiperParamManager::Joint::J1, msg.arm_joint_ctrl.joint_1);
                joint_ctrl.joint_2 = pm.clampJointMilliDeg(PiperParamManager::Joint::J2, msg.arm_joint_ctrl.joint_2);
            }
            else
            {
                joint_ctrl.joint_1 = msg.arm_joint_ctrl.joint_1;
                joint_ctrl.joint_2 = msg.arm_joint_ctrl.joint_2;
            }
            arm_joint_ctrl_msgs_.set(joint_ctrl, msg.timestamp, 0);
            break;
        }
        case ArmMsgType::JointCtrl34:
        {
            auto joint_ctrl = arm_joint_ctrl_msgs_.getValue();
            if (sdk_joint_limit_)
            {
                joint_ctrl.joint_3 = pm.clampJointMilliDeg(PiperParamManager::Joint::J3, msg.arm_joint_ctrl.joint_3);
                joint_ctrl.joint_4 = pm.clampJointMilliDeg(PiperParamManager::Joint::J4, msg.arm_joint_ctrl.joint_4);
            }
            else
            {
                joint_ctrl.joint_3 = msg.arm_joint_ctrl.joint_3;
                joint_ctrl.joint_4 = msg.arm_joint_ctrl.joint_4;
            }
            arm_joint_ctrl_msgs_.set(joint_ctrl, msg.timestamp, 1);
            break;
        }
        case ArmMsgType::JointCtrl56:
        {
            auto joint_ctrl = arm_joint_ctrl_msgs_.getValue();
            if (sdk_joint_limit_)
            {
                joint_ctrl.joint_5 = pm.clampJointMilliDeg(PiperParamManager::Joint::J5, msg.arm_joint_ctrl.joint_5);
                joint_ctrl.joint_6 = pm.clampJointMilliDeg(PiperParamManager::Joint::J6, msg.arm_joint_ctrl.joint_6);
            }
            else
            {
                joint_ctrl.joint_5 = msg.arm_joint_ctrl.joint_5;
                joint_ctrl.joint_6 = msg.arm_joint_ctrl.joint_6;
            }
            arm_joint_ctrl_msgs_.set(joint_ctrl, msg.timestamp, 2);
            break;
        }
        case ArmMsgType::GripperCtrl:
        {
            auto gripper_ctrl = msg.gripper_ctrl;
            if (sdk_gripper_limit_)
            {
                /// WARNING: TODO: potential bug here, python docs say this ctrl feedback is in millideg, but is clamped
                /// using the same micrometer clamping done for the gripper state feedback (which is documented to be in
                /// micrometer)
                gripper_ctrl.grippers_angle = pm.clampGripperMicrometers(gripper_ctrl.grippers_angle);
            }
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
            std::lock_guard<std::mutex> lock(firmware_data_mutex_);
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
    std::vector<uint8_t> data(tx.data, tx.data + PiperMessage::raw_data_len);

    auto can_id = static_cast<uint16_t>(msgTypeToCanId(msg.type));

    if (!arm_can_->sendCanMessage(can_id, data))
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

bool PiperInterfaceV2::sendCommandedMotionCtrlLocked()
{
    // Caller must hold ctrl_151_tx_mutex_.
    PiperMessage msg;
    msg.type = ArmMsgType::MotionCtrl2;
    msg.arm_motion_ctrl_2 = ctrl_151_tx_;
    return sendPiperMsg(msg);
}

bool PiperInterfaceV2::sendMotionCtrl2(
    CtrlMode ctrl_mode, MoveMode move_mode, uint8_t move_spd_rate_ctrl, MitMode mit_mode, uint8_t residence_time,
    InstallationPos installation_pos
)
{
    if (move_spd_rate_ctrl > 100)
        move_spd_rate_ctrl = 100;
    std::lock_guard<std::mutex> lock(ctrl_151_tx_mutex_);
    ctrl_151_tx_ = ArmMsgMotionCtrl_2{
        static_cast<uint8_t>(ctrl_mode),
        static_cast<uint8_t>(move_mode),
        move_spd_rate_ctrl,
        static_cast<uint8_t>(mit_mode),
        residence_time,
        static_cast<uint8_t>(installation_pos),
    };
    return sendCommandedMotionCtrlLocked();
}

bool PiperInterfaceV2::setCtrlMode(CtrlMode ctrl_mode)
{
    std::lock_guard<std::mutex> lock(ctrl_151_tx_mutex_);
    ctrl_151_tx_.ctrl_mode = static_cast<uint8_t>(ctrl_mode);
    return sendCommandedMotionCtrlLocked();
}

bool PiperInterfaceV2::setMoveMode(MoveMode move_mode)
{
    std::lock_guard<std::mutex> lock(ctrl_151_tx_mutex_);
    ctrl_151_tx_.move_mode = static_cast<uint8_t>(move_mode);
    return sendCommandedMotionCtrlLocked();
}

bool PiperInterfaceV2::setSpeedPercentage(uint8_t percentage)
{
    if (percentage > 100)
        percentage = 100;
    std::lock_guard<std::mutex> lock(ctrl_151_tx_mutex_);
    ctrl_151_tx_.move_spd_rate_ctrl = percentage;
    return sendCommandedMotionCtrlLocked();
}

bool PiperInterfaceV2::setMitMode(MitMode mit_mode)
{
    std::lock_guard<std::mutex> lock(ctrl_151_tx_mutex_);
    ctrl_151_tx_.mit_mode = static_cast<uint8_t>(mit_mode);
    return sendCommandedMotionCtrlLocked();
}

bool PiperInterfaceV2::setResidenceTime(uint8_t seconds)
{
    std::lock_guard<std::mutex> lock(ctrl_151_tx_mutex_);
    ctrl_151_tx_.residence_time = seconds;
    return sendCommandedMotionCtrlLocked();
}

bool PiperInterfaceV2::setInstallationPos(InstallationPos installation_pos)
{
    std::lock_guard<std::mutex> lock(ctrl_151_tx_mutex_);
    ctrl_151_tx_.installation_pos = static_cast<uint8_t>(installation_pos);
    return sendCommandedMotionCtrlLocked();
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
    PiperMessage msg;
    msg.type = ArmMsgType::GripperCtrl;
    msg.gripper_ctrl = ArmMsgGripperCtrl{
        grippers_angle,
        grippers_effort,
        static_cast<uint8_t>(status_code),
        static_cast<uint8_t>(GripperSetZero::NoOp),
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
    msg.type = ArmMsgType::MotionCtrlCartesian1;
    if (!sendPiperMsg(msg))
        return false;
    msg.type = ArmMsgType::MotionCtrlCartesian2;
    if (!sendPiperMsg(msg))
        return false;
    msg.type = ArmMsgType::MotionCtrlCartesian3;
    return sendPiperMsg(msg);
}

// Float -> N-bit unsigned mapping used by the MIT joint commands. Mirrors piper_sdk's
// FloatToUint(value, min, max, bits): clamps to [min,max], then linearly maps onto
// [0, 2^bits - 1].
static uint32_t floatToUintRange(double value, double min, double max, int bits)
{
    if (value < min)
        value = min;
    if (value > max)
        value = max;
    const double span = max - min;
    const uint32_t max_int = (uint32_t{1} << bits) - 1u;
    return static_cast<uint32_t>(std::lround((value - min) / span * max_int));
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
    // Fixed scaling ranges per the Piper protocol; do not change without firmware reference.
    const uint16_t pos_packed = static_cast<uint16_t>(floatToUintRange(pos_rad, -12.5, 12.5, 16));
    const uint16_t vel_packed = static_cast<uint16_t>(floatToUintRange(vel, -45.0, 45.0, 12));
    const uint16_t kp_packed = static_cast<uint16_t>(floatToUintRange(kp, 0.0, 500.0, 12));
    const uint16_t kd_packed = static_cast<uint16_t>(floatToUintRange(kd, -5.0, 5.0, 12));
    const uint16_t t_packed = static_cast<uint16_t>(floatToUintRange(torque, -8.0, 8.0, 8));

    PiperMessage msg;
    const size_t idx = motor_num - 1;
    msg.arm_joint_mit_ctrl.motors[idx] = ArmMsgJointMitCtrl{pos_packed, vel_packed, kp_packed, kd_packed, t_packed, 0};
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
    MotorIndex motor, int16_t max_angle_01deg, int16_t min_angle_01deg, int16_t max_spd_001rad
)
{
    PiperMessage msg;
    msg.type = ArmMsgType::MotorAngleLimitMaxSpdSet;
    msg.arm_motor_angle_limit_max_spd_set = ArmMsgCurrentMotorAngleLimitMaxSpd{
        static_cast<uint8_t>(motor), max_angle_01deg, min_angle_01deg, max_spd_001rad
    };
    return sendPiperMsg(msg);
}

bool PiperInterfaceV2::setMotorMaxSpeed(MotorIndex motor, int16_t max_spd_001rad)
{
    // 0x7FFF on the angle limit fields tells firmware V1.5-2+ to leave them untouched.
    constexpr int16_t LEAVE = static_cast<int16_t>(0x7FFF);
    return setMotorAngleLimitsMaxSpeed(motor, LEAVE, LEAVE, max_spd_001rad);
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

bool PiperInterfaceV2::sendFirmwareQuery()
{
    if (!connected_.load(std::memory_order_relaxed))
    {
        std::cerr << "Error: Cannot send message, interface is not connected.\n";
        return false;
    }

    std::vector<uint8_t> data{0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    uint16_t can_id = 0x4AF;

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
            const size_t end = std::min(i + version_len, firmware_data_.size());
            return std::string(firmware_data_.begin() + i, firmware_data_.begin() + end);
        }
    }
    return std::nullopt;
}

} // namespace piper_cpp
