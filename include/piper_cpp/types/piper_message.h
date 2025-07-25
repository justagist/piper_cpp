#pragma once

#include "piper_cpp/types/core.h"
#include "piper_cpp/types/feedback/arm_status.h"

#include <cstdint>
#include <optional>
#include <stdexcept>
#include <string>
#include <unordered_map>

namespace piper_cpp
{

struct PiperMessage
{

    static constexpr size_t raw_data_len = 8;

    uint16_t can_id;
    uint8_t raw_data[raw_data_len] = {0}; // Raw data from CAN frame
    ArmMsgType type = ArmMsgType::Unknown;
    double timestamp; // Timestamp in seconds

    // Additional fields for specific message types
    ArmMsgFeedbackStatus arm_status_msgs; // For status feedback messages
};

} // namespace piper_cpp
