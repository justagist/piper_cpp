#pragma once

#include <cstdint>
#include <sstream>
#include <string>

namespace piper_cpp
{

struct ArmMsgFeedbackJointStates
{
    // Each value in 0.001 degrees
    int32_t joint_1{0};
    int32_t joint_2{0};
    int32_t joint_3{0};
    int32_t joint_4{0};
    int32_t joint_5{0};
    int32_t joint_6{0};

    std::string toString() const
    {
        std::ostringstream oss;
        oss << "ArmMsgFeedbackJointStates:\n"
            << "  Joint 1: " << joint_1 << "\n"
            << "  Joint 2: " << joint_2 << "\n"
            << "  Joint 3: " << joint_3 << "\n"
            << "  Joint 4: " << joint_4 << "\n"
            << "  Joint 5: " << joint_5 << "\n"
            << "  Joint 6: " << joint_6 << "\n";
        return oss.str();
    }
};

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

struct ArmMsgFeedbackCrashProtectionRating
{
    uint8_t joint_1_protection_level{0};
    uint8_t joint_2_protection_level{0};
    uint8_t joint_3_protection_level{0};
    uint8_t joint_4_protection_level{0};
    uint8_t joint_5_protection_level{0};
    uint8_t joint_6_protection_level{0};
    // Byte 6 and 7: reserved

    std::string toString() const
    {
        std::ostringstream oss;
        oss << "ArmMsgFeedbackCrashProtectionRating(\n"
            << "  joint_1_protection_level: " << int(joint_1_protection_level) << "\n"
            << "  joint_2_protection_level: " << int(joint_2_protection_level) << "\n"
            << "  joint_3_protection_level: " << int(joint_3_protection_level) << "\n"
            << "  joint_4_protection_level: " << int(joint_4_protection_level) << "\n"
            << "  joint_5_protection_level: " << int(joint_5_protection_level) << "\n"
            << "  joint_6_protection_level: " << int(joint_6_protection_level) << "\n"
            << ")";
        return oss.str();
    }
};

} // namespace piper_cpp
