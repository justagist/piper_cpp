#include "piper_cpp/protocol/piper_parser_v2.h"
#include "piper_cpp/types/types.h"
#include <cstring>

namespace piper_cpp
{

static int32_t bytesToInt32(const uint8_t* data)
{
    int32_t result;
    std::memcpy(&result, data, sizeof(int32_t));
    return __builtin_bswap32(result); // For big-endian like in Python
}

static int16_t bytesToInt16(const uint8_t* data)
{
    int16_t result;
    std::memcpy(&result, data, sizeof(int16_t));
    return __builtin_bswap16(result);
}

bool PiperParserV2::decodeMessage(const struct can_frame& rx_frame, double timestamp, PiperMessage& msg)
{
    uint32_t can_id = rx_frame.can_id & CAN_EFF_MASK;
    const uint8_t* data = rx_frame.data;

    // Map CAN ID to ArmMsgType using your generated switch/case functions
    try
    {
        auto can_enum_id = static_cast<CanIDPiper>(can_id);
        msg.can_id = can_id;
        msg.type = canIdToMsgType(can_enum_id);
    }
    catch (...)
    {
        msg.type = ArmMsgType::Unknown;
        return false;
    }

    msg.raw_data[0] = data[0];
    msg.raw_data[1] = data[1];
    msg.raw_data[2] = data[2];
    msg.raw_data[3] = data[3];
    msg.raw_data[4] = data[4];
    msg.raw_data[5] = data[5];
    msg.raw_data[6] = data[6];
    msg.raw_data[7] = data[7];

    msg.timestamp = timestamp; // <<--- Set timestamp here!

    /// TODO: Fill PiperMessage fields based on message type
    // switch (msg.type)
    // {
    // }

    return true;
}

} // namespace piper_cpp
