#pragma once

#include <cstdint>
#include <sstream>
#include <string>

namespace piper_cpp
{

struct ArmMsgFeedbackGripper
{
    int32_t grippers_angle{0};  // unit: 0.001mm
    int16_t grippers_effort{0}; // unit: 0.001 N*m
    uint8_t status_code{0};     // raw status code byte

    // Nested struct to decode status bits
    struct FOCStatus
    {
        bool voltage_too_low = false;
        bool motor_overheating = false;
        bool driver_overcurrent = false;
        bool driver_overheating = false;
        bool sensor_status = false;
        bool driver_error_status = false;
        bool driver_enable_status = false;
        bool homing_status = false;

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

    FOCStatus foc_status;

    // Call this after setting status_code!
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
