#include "piper_cpp/interface/piper_interface_v2.h"
#include "piper_cpp/protocol/piper_parser_v2.h"

namespace piper_cpp
{

// Implementation of the PiperInterfaceV2 methods
PiperInterfaceV2::PiperInterfaceV2(
    std::string can_name, bool judge_flag, bool auto_init, bool dh_offset, bool sdk_joint_limit, bool sdk_gripper_limit
)
    : can_name_(std::move(can_name)), start_fk_cal_(false)
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
            joint_values.joint_1 = msg.arm_joint_feedback.joint_1;
            joint_values.joint_2 = msg.arm_joint_feedback.joint_2;
            arm_joint_states_.set(joint_values, msg.timestamp, 0);
            break;
        }
        case ArmMsgType::JointFeedback34:
        {
            auto joint_values = arm_joint_states_.getValue();
            joint_values.joint_3 = msg.arm_joint_feedback.joint_3;
            joint_values.joint_4 = msg.arm_joint_feedback.joint_4;
            arm_joint_states_.set(joint_values, msg.timestamp, 1);
            break;
        }
        case ArmMsgType::JointFeedback56:
        {
            auto joint_values = arm_joint_states_.getValue();
            joint_values.joint_5 = msg.arm_joint_feedback.joint_5;
            joint_values.joint_6 = msg.arm_joint_feedback.joint_6;
            arm_joint_states_.set(joint_values, msg.timestamp, 2);
            break;
        }

        default:
            // Handle other message types as needed
            break;
        }
    }
}

} // namespace piper_cpp
