#pragma once

#include "piper_cpp/types/core.h"

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
};

} // namespace piper_cpp
