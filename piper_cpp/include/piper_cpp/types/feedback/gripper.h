#pragma once

#include <cstdint>
#include <sstream>
#include <string>

namespace piper_cpp
{

/// Gripper feedback payload (CAN ID 0x2A8). Carries the measured jaw position, applied effort,
/// and a packed status byte that's decoded into per-bit flags below.
struct ArmMsgFeedbackGripper
{
    int32_t grippers_angle{0};  ///< Measured jaw stroke, 0.001 mm.
    int16_t grippers_effort{0}; ///< Measured gripper torque, 0.001 N*m.
    uint8_t status_code{0};     ///< Raw status byte; decoded into `foc_status` by `updateFocStatus()`.

    /// Per-bit decode of `status_code`. All "fault" bits (voltage_too_low, overheating, etc.)
    /// follow the convention "false = normal, true = fault triggered". `driver_enable_status`
    /// and `homing_status` follow "true = active state, false = inactive".
    struct FOCStatus
    {
        bool voltage_too_low = false;     ///< Bit 0: power voltage below threshold.
        bool motor_overheating = false;   ///< Bit 1: motor over-temperature trip.
        bool driver_overcurrent = false;  ///< Bit 2: driver over-current trip.
        bool driver_overheating = false;  ///< Bit 3: driver over-temperature trip.
        bool sensor_status = false;       ///< Bit 4: sensor fault (false = normal).
        bool driver_error_status = false; ///< Bit 5: driver error latched.
        bool driver_enable_status = false; ///< Bit 6: gripper position-control loop active.
        bool homing_status = false;       ///< Bit 7: gripper has been zeroed at least once.

        void fromStatusCode(uint8_t code)
        {
            voltage_too_low = code & (1 << 0);
            motor_overheating = code & (1 << 1);
            driver_overcurrent = code & (1 << 2);
            driver_overheating = code & (1 << 3);
            sensor_status = code & (1 << 4);
            driver_error_status = code & (1 << 5);
            driver_enable_status = code & (1 << 6);
            homing_status = code & (1 << 7);
        }

        std::string toString() const
        {
            std::ostringstream oss;
            oss << "    voltage_too_low: " << voltage_too_low << "\n"
                << "    motor_overheating: " << motor_overheating << "\n"
                << "    driver_overcurrent: " << driver_overcurrent << "\n"
                << "    driver_overheating: " << driver_overheating << "\n"
                << "    sensor_status: " << sensor_status << "\n"
                << "    driver_error_status: " << driver_error_status << "\n"
                << "    driver_enable_status: " << driver_enable_status << "\n"
                << "    homing_status: " << homing_status;
            return oss.str();
        }
    };

    FOCStatus foc_status; ///< Decoded view of `status_code`; populated by `updateFocStatus()`.

    /// Decode `status_code` into the per-bit flags in `foc_status`. The parser calls this
    /// automatically after assigning `status_code`; only invoke directly if you've manually
    /// changed `status_code`.
    void updateFocStatus() { foc_status.fromStatusCode(status_code); }

    std::string toString() const
    {
        std::ostringstream oss;
        oss << "ArmMsgFeedbackGripper(\n"
            << "  grippers_angle: " << grippers_angle << " (" << grippers_angle * 0.001 << " mm)\n"
            << "  grippers_effort: " << grippers_effort << " (" << grippers_effort * 0.001 << " N*m)\n"
            << "  status_code: 0x" << std::hex << int(status_code) << std::dec << "\n"
            << "  FOCStatus:\n"
            << foc_status.toString() << "\n)";
        return oss.str();
    }
};

} // namespace piper_cpp
