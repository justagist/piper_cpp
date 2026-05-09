#pragma once

#include <cstdint>
#include <sstream>
#include <string>

namespace piper_cpp
{

/// Per-motor max-acceleration feedback. Sent in response to a query.
struct ArmMsgFeedbackCurrentMotorMaxAccLimit
{
    uint8_t joint_motor_num{0}; ///< Motor index 1-6.
    int16_t max_joint_acc{0};   ///< Configured max joint acceleration, 0.001 rad/s^2.

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

/// Aggregate of all six motors' max-acceleration feedback, indexed `motor[0]..motor[5]` for
/// joints 1-6. Filled in as the per-motor query replies stream in during `piperInit()`.
struct ArmMsgFeedbackAllCurrentMotorMaxAccLimit
{
    ArmMsgFeedbackCurrentMotorMaxAccLimit motor[6];

    ArmMsgFeedbackAllCurrentMotorMaxAccLimit()
    {
        for (int i = 0; i < 6; ++i)
            motor[i] = ArmMsgFeedbackCurrentMotorMaxAccLimit{};
    }

    std::string toString() const
    {
        std::ostringstream oss;
        for (int i = 0; i <= 5; ++i)
            oss << motor[i].toString() << "\n";
        return oss.str();
    }
};

/// Per-joint high-speed feedback (CAN IDs 0x251..0x256, one per joint, ~200 Hz).
struct ArmMsgFeedbackHighSpd
{
    uint32_t can_id{0};     ///< Source CAN ID 0x251..0x256, identifying which joint this is.
    int16_t motor_speed{0}; ///< Motor speed, 0.001 rad/s.
    uint16_t current{0};    ///< Motor current, 0.001 A.
    int32_t pos{0};         ///< Raw motor position (encoder units; convert via DH if needed).
    float effort{0.0f};     ///< Derived torque, 0.001 N*m. Populated by `calcEffort()`.

    /// Estimate the joint torque from the motor current using the manufacturer's per-joint
    /// coefficient table. Joints 1-3 and 4-6 use different gear ratios. Call after assigning
    /// `can_id` and `current`; the parser does this automatically when decoding.
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

/// Aggregate of high-speed feedback from all six joints. Each entry is updated independently
/// as 0x251..0x256 frames arrive on the bus.
struct ArmMsgFeedbackHighSpdArray
{
    std::array<ArmMsgFeedbackHighSpd, 6> high_spd_feedbacks; ///< Per-joint feedback, indexed 0..5 for joints 1..6.

    std::string toString() const
    {
        std::ostringstream oss;
        for (size_t i = 0; i < high_spd_feedbacks.size(); ++i)
        {
            oss << "[Joint " << (i + 1) << "] " << high_spd_feedbacks[i].toString() << "\n";
        }
        return oss.str();
    }
};

/// Per-joint low-speed feedback (CAN IDs 0x261..0x266, one per joint, ~40 Hz). Carries
/// thermal/electrical health metrics and the per-bit driver status flags.
struct ArmMsgFeedbackLowSpd
{
    uint32_t can_id{0};         ///< Source CAN ID 0x261..0x266, identifying which joint this is.
    uint16_t vol{0};            ///< Bus voltage, 0.1 V units.
    int16_t foc_temp{0};        ///< Driver IC temperature, deg C.
    int8_t motor_temp{0};       ///< Motor stator temperature, deg C.
    uint8_t foc_status_code{0}; ///< Raw status byte; decoded into `foc_status` by `updateStatusFromCode()`.
    uint16_t bus_current{0};    ///< Driver bus current, 0.001 A.

    /// Per-bit decode of `foc_status_code`. Each fault bit follows the convention
    /// "false = normal, true = fault triggered"; `driver_enable_status` follows
    /// "true = motor energised".
    struct FOC_Status
    {
        bool voltage_too_low{false};      ///< Bit 0: power voltage below threshold.
        bool motor_overheating{false};    ///< Bit 1: motor over-temperature trip.
        bool driver_overcurrent{false};   ///< Bit 2: driver over-current trip.
        bool driver_overheating{false};   ///< Bit 3: driver over-temperature trip.
        bool collision_status{false};     ///< Bit 4: collision-protection trip.
        bool driver_error_status{false};  ///< Bit 5: driver error latched.
        bool driver_enable_status{false}; ///< Bit 6: motor is enabled and torque-controlled.
        bool stall_status{false};         ///< Bit 7: motor stalled (commanded but not moving).

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

/// Aggregate of low-speed feedback from all six joints. Each entry is updated independently
/// as 0x261..0x266 frames arrive on the bus.
struct ArmMsgFeedbackLowSpdArray
{
    std::array<ArmMsgFeedbackLowSpd, 6> low_spd_feedbacks; ///< Per-joint feedback, indexed 0..5 for joints 1..6.

    std::string toString() const
    {
        std::ostringstream oss;
        for (size_t i = 0; i < low_spd_feedbacks.size(); ++i)
        {
            oss << "[Joint " << (i + 1) << "] " << low_spd_feedbacks[i].toString() << "\n";
        }
        return oss.str();
    }
};

} // namespace piper_cpp
