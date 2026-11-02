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

    /// V2 ranges: position [-12.5, 12.5] rad, velocity [-45, 45], Kp [0, 500], Kd [-5, 5] and
    /// torque [-8, 8], the torque packed into 8 bits. Rounds to the nearest code.
    ArmMsgJointMitCtrl packJointMit(double pos_rad, double vel, double kp, double kd, double torque) const override;

protected:
    /// Check that `rx_frame` is a standard data frame with a known ID and enough payload for
    /// that ID, then fill in the ID, type, raw bytes and timestamp. On failure sets the type
    /// to `Unknown` and returns false.
    bool acceptFrame(const struct can_frame& rx_frame, double timestamp, PiperMessage& msg) const;
};

} // namespace piper_cpp
