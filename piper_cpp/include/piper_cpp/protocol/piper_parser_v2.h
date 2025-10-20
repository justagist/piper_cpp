#pragma once
#include "piper_cpp/protocol/piper_parser_base.h"

namespace piper_cpp
{

class PiperParserV2 : public PiperParserBase
{
public:
    uint8_t getProtocolVersion() const override { return 2; }
    bool decodeMessage(const struct can_frame& rx_frame, double timestamp, PiperMessage& msg) override;
    bool encodeMessage(const PiperMessage& msg, struct can_frame& tx_frame) override;
};

} // namespace piper_cpp
