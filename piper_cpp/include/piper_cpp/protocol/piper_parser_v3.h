#pragma once
#include "piper_cpp/protocol/piper_parser_v2.h"

namespace piper_cpp
{

/// Piper protocol V3, for firmware V1.8-8 and newer. Same frames as V2 except:
///   - 0x151 and 0x2A1: MOVE M is 0x06 on the wire (V2: 0x04); 0x151 also accepts the
///     horizontal-inverted installation position (0x04).
///   - 0x159 and 0x2A8: the gripper value is a width or an angle; the command carries the mode
///     in bit 2 of the status byte, the feedback in byte 7.
///   - 0x15A-0x15F: MIT torque is 12 bits over [-16, 16] and byte 7 has no CRC nibble.
///   - 0x2AA-0x2AC: IK joint feedback.
/// Every other message is handled by the V2 parser unchanged.
class PiperParserV3 : public PiperParserV2
{
public:
    uint8_t getProtocolVersion() const override { return 3; }
    bool decodeMessage(const struct can_frame& rx_frame, double timestamp, PiperMessage& msg) override;
    bool encodeMessage(const PiperMessage& msg, struct can_frame& tx_frame) override;

    /// V3 ranges: position [-12.5, 12.5] rad, velocity [-45, 45], Kp [0, 500], Kd [-5, 5] and
    /// torque [-16, 16], the torque packed into 12 bits. Truncates to the code below, as the
    /// official SDK does.
    ArmMsgJointMitCtrl packJointMit(double pos_rad, double vel, double kp, double kd, double torque) const override;
};

} // namespace piper_cpp
