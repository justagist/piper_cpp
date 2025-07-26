#pragma once

#include <cstdint>
#include <sstream>
#include <string>

namespace piper_cpp
{

struct ArmMsgFeedbackCurrentMotorAngleLimitMaxSpd
{
    uint8_t motor_num{0};       // Joint motor number [1..6]
    int16_t max_angle_limit{0}; // Maximum angle limit (0.1 deg)
    int16_t min_angle_limit{0}; // Minimum angle limit (0.1 deg)
    int16_t max_joint_spd{0};   // Maximum joint speed (0.001 rad/s)

    std::string toString() const
    {
        std::ostringstream oss;
        oss << "ArmMsgFeedbackCurrentMotorAngleLimitMaxSpd(\n"
            << "  motor_num: " << int(motor_num) << "\n"
            << "  max_angle_limit: " << max_angle_limit << "\n"
            << "  min_angle_limit: " << min_angle_limit << "\n"
            << "  max_joint_spd: " << max_joint_spd << "\n"
            << ")";
        return oss.str();
    }
};

// Container for all 6 motors' angle/speed feedback (indexed from 1 to 6)
struct ArmMsgFeedbackAllCurrentMotorAngleLimitMaxSpd
{
    ArmMsgFeedbackCurrentMotorAngleLimitMaxSpd motor[7]; // 0 unused, 1~6 valid

    ArmMsgFeedbackAllCurrentMotorAngleLimitMaxSpd()
    {
        for (int i = 0; i < 7; ++i)
            motor[i] = ArmMsgFeedbackCurrentMotorAngleLimitMaxSpd{};
    }

    std::string toString() const
    {
        std::ostringstream oss;
        for (int i = 1; i <= 6; ++i)
            oss << motor[i].toString() << "\n";
        return oss.str();
    }
};

struct ArmMsgFeedbackCurrentMotorMaxAccLimit
{
    uint8_t joint_motor_num{0}; // Joint motor number [1..6]
    int16_t max_joint_acc{0};   // Maximum joint acceleration (0.001 rad/s^2)

    std::string toString() const
    {
        std::ostringstream oss;
        oss << "ArmMsgFeedbackCurrentMotorMaxAccLimit(\n"
            << "  joint_motor_num: " << int(joint_motor_num) << "\n"
            << "  max_joint_acc: " << max_joint_acc << "\n"
            << ")";
        return oss.str();
    }
};

// Container for all 6 motors' acceleration feedback (indexed from 1 to 6)
struct ArmMsgFeedbackAllCurrentMotorMaxAccLimit
{
    ArmMsgFeedbackCurrentMotorMaxAccLimit motor[7];

    ArmMsgFeedbackAllCurrentMotorMaxAccLimit()
    {
        for (int i = 0; i < 7; ++i)
            motor[i] = ArmMsgFeedbackCurrentMotorMaxAccLimit{};
    }

    std::string toString() const
    {
        std::ostringstream oss;
        for (int i = 1; i <= 6; ++i)
            oss << motor[i].toString() << "\n";
        return oss.str();
    }
};

struct ArmMsgFeedbackHighSpd
{
    uint32_t can_id{0};     // CAN ID: 0x251~0x256
    int16_t motor_speed{0}; // Motor speed (0.001 rad/s)
    uint16_t current{0};    // Motor current (0.001 A)
    int32_t pos{0};         // Motor position (rad)
    float effort{0.0f};     // Torque, derived, unit 0.001 N·m

    // Call this function to calculate 'effort' using current and can_id (should be called after setting can_id &
    // current)
    float calcEffort()
    {
        static constexpr float COEFFICIENT_1_3 = 1.18125f;
        static constexpr float COEFFICIENT_4_6 = 0.95844f;
        if (can_id == 0x251 || can_id == 0x252 || can_id == 0x253)
            effort = float(current) * COEFFICIENT_1_3;
        else if (can_id == 0x254 || can_id == 0x255 || can_id == 0x256)
            effort = float(current) * COEFFICIENT_4_6;
        else
            effort = 0.0f;
        return effort;
    }

    std::string toString() const
    {
        std::ostringstream oss;
        oss << "ArmMsgFeedbackHighSpd(\n"
            << "  can_id: 0x" << std::hex << can_id << std::dec << "\n"
            << "  motor_speed: " << motor_speed << "\n"
            << "  current: " << current << "\n"
            << "  pos: " << pos << "\n"
            << "  effort: " << effort << "\n"
            << ")";
        return oss.str();
    }
};

struct ArmMsgFeedbackLowSpd
{
    uint32_t can_id{0};         // CAN ID: 0x261~0x266
    uint16_t vol{0};            // Bus voltage (0.1 V)
    int16_t foc_temp{0};        // Drive temperature (°C)
    int8_t motor_temp{0};       // Motor temperature (°C)
    uint8_t foc_status_code{0}; // Drive status code (bit field, see below)
    uint16_t bus_current{0};    // Bus current (0.001 A)

    // Decoded status bits (filled after parsing foc_status_code)
    struct FOC_Status
    {
        bool voltage_too_low{false};
        bool motor_overheating{false};
        bool driver_overcurrent{false};
        bool driver_overheating{false};
        bool collision_status{false};
        bool driver_error_status{false};
        bool driver_enable_status{false};
        bool stall_status{false};

        void fromCode(uint8_t code)
        {
            voltage_too_low = code & (1 << 0);
            motor_overheating = code & (1 << 1);
            driver_overcurrent = code & (1 << 2);
            driver_overheating = code & (1 << 3);
            collision_status = code & (1 << 4);
            driver_error_status = code & (1 << 5);
            driver_enable_status = code & (1 << 6);
            stall_status = code & (1 << 7);
        }

        std::string toString() const
        {
            std::ostringstream oss;
            oss << "    voltage_too_low : " << voltage_too_low << "\n"
                << "    motor_overheating: " << motor_overheating << "\n"
                << "    driver_overcurrent: " << driver_overcurrent << "\n"
                << "    driver_overheating: " << driver_overheating << "\n"
                << "    collision_status: " << collision_status << "\n"
                << "    driver_error_status: " << driver_error_status << "\n"
                << "    driver_enable_status: " << driver_enable_status << "\n"
                << "    stall_status: " << stall_status << "\n";
            return oss.str();
        }
    } foc_status;

    void updateStatusFromCode() { foc_status.fromCode(foc_status_code); }

    std::string toString() const
    {
        std::ostringstream oss;
        oss << "ArmMsgFeedbackLowSpd(\n"
            << "  can_id: 0x" << std::hex << can_id << std::dec << "\n"
            << "  vol: " << vol << ", " << (vol * 0.1f) << " V,\n"
            << "  foc_temp: " << foc_temp << " C,\n"
            << "  motor_temp: " << int(motor_temp) << " C,\n"
            << "  foc_status: \n"
            << foc_status.toString() << "  bus_current: " << bus_current << ", " << (bus_current * 0.001f) << " A\n"
            << ")";
        return oss.str();
    }
};

} // namespace piper_cpp
