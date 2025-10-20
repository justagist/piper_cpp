#pragma once

#include <cstdint>
#include <sstream>
#include <string>

namespace piper_cpp
{

// joint states/commands
struct ArmMsgJointValues
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
        oss << "ArmMsgJointValues:\n"
            << "  Joint 1: " << joint_1 << "\n"
            << "  Joint 2: " << joint_2 << "\n"
            << "  Joint 3: " << joint_3 << "\n"
            << "  Joint 4: " << joint_4 << "\n"
            << "  Joint 5: " << joint_5 << "\n"
            << "  Joint 6: " << joint_6 << "\n";
        return oss.str();
    }
};

struct ArmMsgCrashProtectionRatingConfig
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
        oss << "ArmMsgCrashProtectionRating(\n"
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

// End-Effector Speed/Acceleration feedback/command
struct ArmMsgCurrentEndVelAccParam
{
    int16_t end_max_linear_vel{0};  // 0.001 m/s
    int16_t end_max_angular_vel{0}; // 0.001 rad/s
    int16_t end_max_linear_acc{0};  // 0.001 m/s²
    int16_t end_max_angular_acc{0}; // 0.001 rad/s²

    std::string toString() const
    {
        std::ostringstream oss;
        oss << "ArmMsgCurrentEndVelAccParam(\n"
            << "  end_max_linear_vel: " << end_max_linear_vel << "\n"
            << "  end_max_angular_vel: " << end_max_angular_vel << "\n"
            << "  end_max_linear_acc: " << end_max_linear_acc << "\n"
            << "  end_max_angular_acc: " << end_max_angular_acc << "\n"
            << ")";
        return oss.str();
    }
};

struct ArmMsgGripperTeachingPendantParam
{
    uint8_t teaching_range_per{0}; // [100,200] %, default 100%
    uint8_t max_range_config{0};   // [0,70,100] mm
    uint8_t teaching_friction{0};  // [1,10]
    // bytes 3~7 are reserved

    std::string toString() const
    {
        std::ostringstream oss;
        oss << "ArmMsgGripperTeachingPendantParam(\n"
            << "  teaching_range_per: " << int(teaching_range_per) << "\n"
            << "  max_range_config: " << int(max_range_config) << "\n"
            << "  teaching_friction: " << int(teaching_friction) << "\n"
            << ")";
        return oss.str();
    }
};

struct ArmMsgCurrentMotorAngleLimitMaxSpd
{
    uint8_t motor_num{0};       // Joint motor number [0..5]
    int16_t max_angle_limit{0}; // Maximum angle limit (0.1 deg)
    int16_t min_angle_limit{0}; // Minimum angle limit (0.1 deg)
    int16_t max_joint_spd{0};   // Maximum joint speed (0.001 rad/s)

    std::string toString() const
    {
        std::ostringstream oss;
        oss << "ArmMsgCurrentMotorAngleLimitMaxSpd(\n"
            << "  motor_num: " << int(motor_num) << "\n"
            << "  max_angle_limit: " << max_angle_limit << "\n"
            << "  min_angle_limit: " << min_angle_limit << "\n"
            << "  max_joint_spd: " << max_joint_spd << "\n"
            << ")";
        return oss.str();
    }
};

// Container for all 6 motors' angle/speed feedback (indexed from 0 to 5)
struct ArmMsgAllCurrentMotorAngleLimitMaxSpd
{
    ArmMsgCurrentMotorAngleLimitMaxSpd motor[6];

    ArmMsgAllCurrentMotorAngleLimitMaxSpd()
    {
        for (int i = 0; i < 6; ++i)
            motor[i] = ArmMsgCurrentMotorAngleLimitMaxSpd{};
    }

    std::string toString() const
    {
        std::ostringstream oss;
        for (int i = 0; i <= 5; ++i)
            oss << motor[i].toString() << "\n";
        return oss.str();
    }
};

// End-Effector Pose
struct ArmMsgEndPose
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
        oss << "ArmMsgEndPose:\n"
            << " X_axis: " << X_axis << "\n"
            << " Y_axis: " << Y_axis << "\n"
            << " Z_axis: " << Z_axis << "\n"
            << " RX_axis: " << RX_axis << "\n"
            << " RY_axis: " << RY_axis << "\n"
            << " RZ_axis: " << RZ_axis;
        return oss.str();
    }
};

} // namespace piper_cpp
