#pragma once

#include <cstdint>
#include <sstream>
#include <string>

namespace piper_cpp
{

/// Six-element joint angle vector. Used both for measured joint feedback (wire ID 0x2A4-0x2A6)
/// and joint targets (0x155-0x157). Values are in 0.001-degree units.
struct ArmMsgJointValues
{
    int32_t joint_1{0}; ///< Joint 1 angle, 0.001 deg.
    int32_t joint_2{0}; ///< Joint 2 angle, 0.001 deg.
    int32_t joint_3{0}; ///< Joint 3 angle, 0.001 deg.
    int32_t joint_4{0}; ///< Joint 4 angle, 0.001 deg.
    int32_t joint_5{0}; ///< Joint 5 angle, 0.001 deg.
    int32_t joint_6{0}; ///< Joint 6 angle, 0.001 deg.

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

/// Per-joint collision-protection levels for the arm. Each field is a level 0-8 where 0
/// disables collision detection on that joint and higher values are more sensitive.
struct ArmMsgCrashProtectionRatingConfig
{
    uint8_t joint_1_protection_level{0}; ///< Joint 1 collision-protection level (0-8).
    uint8_t joint_2_protection_level{0}; ///< Joint 2 collision-protection level (0-8).
    uint8_t joint_3_protection_level{0}; ///< Joint 3 collision-protection level (0-8).
    uint8_t joint_4_protection_level{0}; ///< Joint 4 collision-protection level (0-8).
    uint8_t joint_5_protection_level{0}; ///< Joint 5 collision-protection level (0-8).
    uint8_t joint_6_protection_level{0}; ///< Joint 6 collision-protection level (0-8).
    // Bytes 6-7 of the wire frame are reserved.

    std::string toString() const
    {
        std::ostringstream oss;
        oss << "ArmMsgCrashProtectionRatingConfig(\n"
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

/// End-effector velocity / acceleration limits. Used both as feedback (the values currently
/// active on the arm) and as a command payload to set new limits.
struct ArmMsgCurrentEndVelAccParam
{
    int16_t end_max_linear_vel{0};  ///< Max linear velocity, 0.001 m/s.
    int16_t end_max_angular_vel{0}; ///< Max angular velocity, 0.001 rad/s.
    int16_t end_max_linear_acc{0};  ///< Max linear acceleration, 0.001 m/s^2.
    int16_t end_max_angular_acc{0}; ///< Max angular acceleration, 0.001 rad/s^2.

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

/// Optional teaching-pendant gripper accessory parameters. Firmware V1.5-2+.
struct ArmMsgGripperTeachingPendantParam
{
    uint8_t teaching_range_per{0}; ///< Travel range coefficient, [100, 200] %. Default 100.
    uint8_t max_range_config{0};   ///< Max control travel limit in mm. Documented values: 0, 70, 100.
    uint8_t teaching_friction{0};  ///< Drag-teach friction coefficient, [1, 10].
    // Bytes 3-7 of the wire frame are reserved.

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

/// Per-motor angle-limit and max-speed configuration. Used as feedback (current values) and
/// as a command payload for the motor-angle-limit-max-spd setter.
struct ArmMsgCurrentMotorAngleLimitMaxSpd
{
    uint8_t motor_num{0};       ///< Motor index 1-6.
    int16_t max_angle_limit{0}; ///< Max angle limit, 0.1 deg. 0x7FFF = leave unchanged (firmware V1.5-2+).
    int16_t min_angle_limit{0}; ///< Min angle limit, 0.1 deg. 0x7FFF = leave unchanged.
    int16_t max_joint_spd{0};   ///< Max joint speed, 0.001 rad/s. Range 0-3000. 0x7FFF = leave unchanged.

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

/// Aggregate of all six motors' angle/speed feedback, indexed `motor[0]..motor[5]` for joints
/// 1-6. Populated as the per-motor query replies stream in.
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

/// End-effector pose in cartesian space. Used both for measured feedback (the arm's current
/// pose) and as a command payload (the cartesian target).
struct ArmMsgEndPose
{
    int32_t X_axis{0};  ///< X coordinate, 0.001 mm.
    int32_t Y_axis{0};  ///< Y coordinate, 0.001 mm.
    int32_t Z_axis{0};  ///< Z coordinate, 0.001 mm.
    int32_t RX_axis{0}; ///< Rotation about X (Euler), 0.001 deg.
    int32_t RY_axis{0}; ///< Rotation about Y (Euler), 0.001 deg.
    int32_t RZ_axis{0}; ///< Rotation about Z (Euler), 0.001 deg.

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
