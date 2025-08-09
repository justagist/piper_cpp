#include "piper_cpp/interface/piper_interface_v2.h"
#include "piper_cpp/protocol/piper_parser_v2.h"

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
        [this](const struct can_frame& frame, double timestamp) { parseCANFrame(frame, timestamp); }
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
        stop_monitor_.store(false, std::memory_order_relaxed);
        monitor_thread_ = std::thread(&PiperInterfaceV2::monitorLoop, this);
    }
    return true;
}

void PiperInterfaceV2::piperInit()
{
    // Send the three "search" commands for joint limits, acc limits, firmware
    /// TODO: Implement the actual commands to sends
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
        // std::this_thread::sleep_for(std::chrono::milliseconds(100)); // TODO: Adjust sleep duration as needed
    }
}

void PiperInterfaceV2::monitorLoop()
{
    while (!stop_monitor_.load(std::memory_order_relaxed))
    {
        // Monitor the state of the arm, e.g., check for errors or status updates
        // This could involve checking the arm status or other parameters
        std::this_thread::sleep_for(std::chrono::milliseconds(100)); // TODO: Adjust sleep duration as needed
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
    stop_monitor_.store(true, std::memory_order_relaxed);
    if (monitor_thread_.joinable())
    {
        monitor_thread_.join();
    }
    arm_can_->close();
    connected_.store(false, std::memory_order_relaxed);
}

void PiperInterfaceV2::parseCANFrame(const struct can_frame& frame, double timestamp)
{
    // Parse the CAN frame using the parser
    PiperMessage msg;
    if (parser_->decodeMessage(frame, timestamp, msg))
    {
        auto& pm = getParameterManager();
        switch (msg.type)
        {
        case ArmMsgType::StatusFeedback:
            arm_status_.set(msg.arm_status_msgs, msg.timestamp);
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

        default:
            // Handle other message types as needed
            break;
        }
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

} // namespace piper_cpp
