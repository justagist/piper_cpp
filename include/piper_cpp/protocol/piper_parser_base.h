#pragma once
#include "piper_cpp/types/types.h"
#include <linux/can.h> // struct can_frame
#include <memory>

namespace piper_cpp
{

class PiperParserBase
{
public:
    virtual ~PiperParserBase() = default;
    virtual bool decodeMessage(const struct can_frame& rx_frame, double timestamp, PiperMessage& msg) = 0;
    virtual uint8_t getProtocolVersion() const = 0;
};

} // namespace piper_cpp
