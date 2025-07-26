#pragma once

#include <cstdint>
#include <iomanip>
#include <sstream>
#include <string>

namespace piper_cpp
{

// End-Effector Pose Feedback
struct ArmMsgFeedbackEndPose
{
    int32_t X_axis{0}; // 0.001mm
    int32_t Y_axis{0};
    int32_t Z_axis{0};
    int32_t RX_axis{0};
    int32_t RY_axis{0};
    int32_t RZ_axis{0};

    std::string toString() const
    {
        std::ostringstream oss;
        oss << "ArmMsgFeedbackEndPose:\n"
            << " X_axis: " << X_axis << "\n"
            << " Y_axis: " << Y_axis << "\n"
            << " Z_axis: " << Z_axis << "\n"
            << " RX_axis: " << RX_axis << "\n"
            << " RY_axis: " << RY_axis << "\n"
            << " RZ_axis: " << RZ_axis;
        return oss.str();
    }
};

// End-Effector Speed/Acceleration Feedback
struct ArmMsgFeedbackCurrentEndVelAccParam
{
    int16_t end_max_linear_vel{0};  // 0.001 m/s
    int16_t end_max_angular_vel{0}; // 0.001 rad/s
    int16_t end_max_linear_acc{0};  // 0.001 m/s²
    int16_t end_max_angular_acc{0}; // 0.001 rad/s²

    std::string toString() const
    {
        std::ostringstream oss;
        oss << "ArmMsgFeedbackCurrentEndVelAccParam(\n"
            << "  end_max_linear_vel: " << end_max_linear_vel << "\n"
            << "  end_max_angular_vel: " << end_max_angular_vel << "\n"
            << "  end_max_linear_acc: " << end_max_linear_acc << "\n"
            << "  end_max_angular_acc: " << end_max_angular_acc << "\n"
            << ")";
        return oss.str();
    }
};

} // namespace piper_cpp
