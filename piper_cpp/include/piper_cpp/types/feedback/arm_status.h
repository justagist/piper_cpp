#pragma once

#include <cstdint>
#include <iomanip>
#include <sstream>
#include <string>

namespace piper_cpp
{

namespace ArmMsgFeedbackStatusEnum
{

enum class CtrlMode : uint8_t
{
    STANDBY = 0x00,
    CAN_CTRL = 0x01,
    TEACHING_MODE = 0x02,
    ETHERNET_CONTROL_MODE = 0x03,
    WIFI_CONTROL_MODE = 0x04,
    REMOTE_CONTROL_MODE = 0x05,
    LINKAGE_TEACHING_INPUT_MODE = 0x06,
    OFFLINE_TRAJECTORY_MODE = 0x07
};
inline std::string toString(CtrlMode mode)
{
    switch (mode)
    {
    case CtrlMode::STANDBY:
        return "STANDBY(0x00)";
    case CtrlMode::CAN_CTRL:
        return "CAN_CTRL(0x01)";
    case CtrlMode::TEACHING_MODE:
        return "TEACHING_MODE(0x02)";
    case CtrlMode::ETHERNET_CONTROL_MODE:
        return "ETHERNET_CONTROL_MODE(0x03)";
    case CtrlMode::WIFI_CONTROL_MODE:
        return "WIFI_CONTROL_MODE(0x04)";
    case CtrlMode::REMOTE_CONTROL_MODE:
        return "REMOTE_CONTROL_MODE(0x05)";
    case CtrlMode::LINKAGE_TEACHING_INPUT_MODE:
        return "LINKAGE_TEACHING_INPUT_MODE(0x06)";
    case CtrlMode::OFFLINE_TRAJECTORY_MODE:
        return "OFFLINE_TRAJECTORY_MODE(0x07)";
    default:
        return "UNKNOWN";
    }
}

enum class ArmStatus : uint8_t
{
    NORMAL = 0x00,
    EMERGENCY_STOP = 0x01,
    NO_SOLUTION = 0x02,
    SINGULARITY_POINT = 0x03,
    TARGET_POS_EXCEEDS_LIMIT = 0x04,
    JOINT_COMMUNICATION_ERR = 0x05,
    JOINT_BRAKE_NOT_RELEASED = 0x06,
    COLLISION_OCCURRED = 0x07,
    OVERSPEED_DURING_TEACHING_DRAG = 0x08,
    JOINT_STATUS_ERR = 0x09,
    OTHER_ERR = 0x0A,
    TEACHING_RECORD = 0x0B,
    TEACHING_EXECUTION = 0x0C,
    TEACHING_PAUSE = 0x0D,
    MAIN_CONTROLLER_NTC_OVER_TEMPERATURE = 0x0E,
    RELEASE_RESISTOR_NTC_OVER_TEMPERATURE = 0x0F
};
inline std::string toString(ArmStatus s)
{
    switch (s)
    {
    case ArmStatus::NORMAL:
        return "NORMAL(0x00)";
    case ArmStatus::EMERGENCY_STOP:
        return "EMERGENCY_STOP(0x01)";
    case ArmStatus::NO_SOLUTION:
        return "NO_SOLUTION(0x02)";
    case ArmStatus::SINGULARITY_POINT:
        return "SINGULARITY_POINT(0x03)";
    case ArmStatus::TARGET_POS_EXCEEDS_LIMIT:
        return "TARGET_POS_EXCEEDS_LIMIT(0x04)";
    case ArmStatus::JOINT_COMMUNICATION_ERR:
        return "JOINT_COMMUNICATION_ERR(0x05)";
    case ArmStatus::JOINT_BRAKE_NOT_RELEASED:
        return "JOINT_BRAKE_NOT_RELEASED(0x06)";
    case ArmStatus::COLLISION_OCCURRED:
        return "COLLISION_OCCURRED(0x07)";
    case ArmStatus::OVERSPEED_DURING_TEACHING_DRAG:
        return "OVERSPEED_DURING_TEACHING_DRAG(0x08)";
    case ArmStatus::JOINT_STATUS_ERR:
        return "JOINT_STATUS_ERR(0x09)";
    case ArmStatus::OTHER_ERR:
        return "OTHER_ERR(0x0A)";
    case ArmStatus::TEACHING_RECORD:
        return "TEACHING_RECORD(0x0B)";
    case ArmStatus::TEACHING_EXECUTION:
        return "TEACHING_EXECUTION(0x0C)";
    case ArmStatus::TEACHING_PAUSE:
        return "TEACHING_PAUSE(0x0D)";
    case ArmStatus::MAIN_CONTROLLER_NTC_OVER_TEMPERATURE:
        return "MAIN_CONTROLLER_NTC_OVER_TEMPERATURE(0x0E)";
    case ArmStatus::RELEASE_RESISTOR_NTC_OVER_TEMPERATURE:
        return "RELEASE_RESISTOR_NTC_OVER_TEMPERATURE(0x0F)";
    default:
        return "UNKNOWN";
    }
}

enum class ModeFeed : uint8_t
{
    MOVE_P = 0x00,
    MOVE_J = 0x01,
    MOVE_L = 0x02,
    MOVE_C = 0x03,
    MOVE_M = 0x04,
    MOVE_CPV = 0x05
};
inline std::string toString(ModeFeed m)
{
    switch (m)
    {
    case ModeFeed::MOVE_P:
        return "MOVE_P(0x00)";
    case ModeFeed::MOVE_J:
        return "MOVE_J(0x01)";
    case ModeFeed::MOVE_L:
        return "MOVE_L(0x02)";
    case ModeFeed::MOVE_C:
        return "MOVE_C(0x03)";
    case ModeFeed::MOVE_M:
        return "MOVE_M(0x04)";
    case ModeFeed::MOVE_CPV:
        return "MOVE_CPV(0x05)";
    default:
        return "UNKNOWN";
    }
}

enum class TeachingState : uint8_t
{
    DISABLED = 0x00,
    START_RECORDING = 0x01,
    STOP_RECORDING = 0x02,
    EXECUTE_TRAJECTORY = 0x03,
    PAUSE_EXECUTION = 0x04,
    RESUME_EXECUTION = 0x05,
    TERMINATE_EXECUTION = 0x06,
    MOVE_TO_START = 0x07
};
inline std::string toString(TeachingState t)
{
    switch (t)
    {
    case TeachingState::DISABLED:
        return "DISABLED(0x00)";
    case TeachingState::START_RECORDING:
        return "START_RECORDING(0x01)";
    case TeachingState::STOP_RECORDING:
        return "STOP_RECORDING(0x02)";
    case TeachingState::EXECUTE_TRAJECTORY:
        return "EXECUTE_TRAJECTORY(0x03)";
    case TeachingState::PAUSE_EXECUTION:
        return "PAUSE_EXECUTION(0x04)";
    case TeachingState::RESUME_EXECUTION:
        return "RESUME_EXECUTION(0x05)";
    case TeachingState::TERMINATE_EXECUTION:
        return "TERMINATE_EXECUTION(0x06)";
    case TeachingState::MOVE_TO_START:
        return "MOVE_TO_START(0x07)";
    default:
        return "UNKNOWN";
    }
}

enum class MotionStatus : uint8_t
{
    REACH_TARGET_POS_SUCCESSFULLY = 0x00,
    REACH_TARGET_POS_FAILED = 0x01
};
inline std::string toString(MotionStatus m)
{
    switch (m)
    {
    case MotionStatus::REACH_TARGET_POS_SUCCESSFULLY:
        return "REACH_TARGET_POS_SUCCESSFULLY(0x00)";
    case MotionStatus::REACH_TARGET_POS_FAILED:
        return "REACH_TARGET_POS_FAILED(0x01)";
    default:
        return "UNKNOWN";
    }
}

} // namespace ArmMsgFeedbackStatusEnum

/// Per-joint error-status bitfield carried inside the arm-status feedback (0x2A7). Each flag
/// is set when the corresponding fault is currently active on that joint.
struct ArmErrStatus
{
    bool joint_1_angle_limit = false;
    bool joint_2_angle_limit = false;
    bool joint_3_angle_limit = false;
    bool joint_4_angle_limit = false;
    bool joint_5_angle_limit = false;
    bool joint_6_angle_limit = false;
    bool communication_status_joint_1 = false;
    bool communication_status_joint_2 = false;
    bool communication_status_joint_3 = false;
    bool communication_status_joint_4 = false;
    bool communication_status_joint_5 = false;
    bool communication_status_joint_6 = false;

    void parseFrom(uint16_t err_code)
    {
        communication_status_joint_1 = (err_code & (1 << 0));
        communication_status_joint_2 = (err_code & (1 << 1));
        communication_status_joint_3 = (err_code & (1 << 2));
        communication_status_joint_4 = (err_code & (1 << 3));
        communication_status_joint_5 = (err_code & (1 << 4));
        communication_status_joint_6 = (err_code & (1 << 5));
        joint_1_angle_limit = (err_code & (1 << 8));
        joint_2_angle_limit = (err_code & (1 << 9));
        joint_3_angle_limit = (err_code & (1 << 10));
        joint_4_angle_limit = (err_code & (1 << 11));
        joint_5_angle_limit = (err_code & (1 << 12));
        joint_6_angle_limit = (err_code & (1 << 13));
    }
    std::string toString() const
    {
        std::ostringstream oss;
        oss << " Joint 1 Angle Limit Status: " << joint_1_angle_limit << "\n"
            << " Joint 2 Angle Limit Status: " << joint_2_angle_limit << "\n"
            << " Joint 3 Angle Limit Status: " << joint_3_angle_limit << "\n"
            << " Joint 4 Angle Limit Status: " << joint_4_angle_limit << "\n"
            << " Joint 5 Angle Limit Status: " << joint_5_angle_limit << "\n"
            << " Joint 6 Angle Limit Status: " << joint_6_angle_limit << "\n"
            << " Joint 1 Communication Status: " << communication_status_joint_1 << "\n"
            << " Joint 2 Communication Status: " << communication_status_joint_2 << "\n"
            << " Joint 3 Communication Status: " << communication_status_joint_3 << "\n"
            << " Joint 4 Communication Status: " << communication_status_joint_4 << "\n"
            << " Joint 5 Communication Status: " << communication_status_joint_5 << "\n"
            << " Joint 6 Communication Status: " << communication_status_joint_6 << "\n";
        return oss.str();
    }
};

struct ArmMsgFeedbackStatus
{
    ArmMsgFeedbackStatusEnum::CtrlMode ctrl_mode{};
    ArmMsgFeedbackStatusEnum::ArmStatus arm_status{};
    ArmMsgFeedbackStatusEnum::ModeFeed mode_feed{};
    ArmMsgFeedbackStatusEnum::TeachingState teach_status{};
    ArmMsgFeedbackStatusEnum::MotionStatus motion_status{};
    uint8_t trajectory_num = 0;
    uint16_t err_code = 0;
    ArmErrStatus err_status{};

    void setErrCode(uint16_t value)
    {
        err_code = value;
        err_status.parseFrom(value);
    }

    std::string toString() const
    {
        std::ostringstream oss;
        oss << "Control Mode: " << ArmMsgFeedbackStatusEnum::toString(ctrl_mode) << "\n"
            << "Arm Status: " << ArmMsgFeedbackStatusEnum::toString(arm_status) << "\n"
            << "Mode Feed: " << ArmMsgFeedbackStatusEnum::toString(mode_feed) << "\n"
            << "Teach Status: " << ArmMsgFeedbackStatusEnum::toString(teach_status) << "\n"
            << "Motion Status: " << ArmMsgFeedbackStatusEnum::toString(motion_status) << "\n"
            << "Trajectory Num: " << (int)trajectory_num << "\n"
            << "Error Code: " << std::hex << std::showbase << err_code << "\n"
            << "Error Status: \n"
            << err_status.toString();
        return oss.str();
    }
};

} // namespace piper_cpp
