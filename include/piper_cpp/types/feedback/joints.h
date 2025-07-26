#pragma once

#include <cstdint>
#include <sstream>
#include <string>

namespace piper_cpp
{

struct ArmMsgFeedbackJointVelAcc
{
    uint32_t can_id{0};         // CAN ID (0x481~0x486) for joint index
    int16_t end_linear_vel{0};  // 0.001 m/s
    int16_t end_angular_vel{0}; // 0.001 rad/s
    int16_t end_linear_acc{0};  // 0.001 m/s^2
    int16_t end_angular_acc{0}; // 0.001 rad/s^2

    std::string toString() const
    {
        std::ostringstream oss;
        oss << "ArmMsgFeedbackJointVelAcc(\n"
            << "  can_id: 0x" << std::hex << can_id << std::dec << "\n"
            << "  end_linear_vel: " << end_linear_vel << "\n"
            << "  end_angular_vel: " << end_angular_vel << "\n"
            << "  end_linear_acc: " << end_linear_acc << "\n"
            << "  end_angular_acc: " << end_angular_acc << "\n"
            << ")";
        return oss.str();
    }
};

/// Container for all joints' velocity/acceleration feedback (for convenience)
struct ArmMsgFeedbackAllJointVelAcc
{
    ArmMsgFeedbackJointVelAcc joint[7]; // 1~6 are used; 0 is unused (for alignment with 1-based joint numbers)

    ArmMsgFeedbackAllJointVelAcc()
    {
        for (int i = 0; i < 7; ++i)
            joint[i] = ArmMsgFeedbackJointVelAcc{};
    }

    std::string toString() const
    {
        std::ostringstream oss;
        for (int i = 1; i <= 6; ++i)
            oss << joint[i].toString() << "\n";
        return oss.str();
    }
};

} // namespace piper_cpp
