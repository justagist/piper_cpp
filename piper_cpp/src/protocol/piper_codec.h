#pragma once

// Byte-level helpers shared by the protocol parsers. Every multi-byte Piper field is big-endian.

#include "piper_cpp/types/types.h"

#include <cmath>
#include <cstdint>
#include <linux/can.h>

namespace piper_cpp
{

// Utility for endian conversion, assumes network (big-endian) order.
inline int32_t bytesToInt32(const uint8_t *data)
{
    uint32_t val = (uint32_t(data[0]) << 24) | (uint32_t(data[1]) << 16) | (uint32_t(data[2]) << 8) | uint32_t(data[3]);
    return static_cast<int32_t>(val); // This cast will sign-extend if MSB is set
}

inline int16_t bytesToInt16(const uint8_t *data)
{
    uint16_t val = (uint16_t(data[0]) << 8) | uint16_t(data[1]);
    return static_cast<int16_t>(val); // This cast will sign-extend if MSB is set
}

// For unsigned 16-bit
inline uint16_t bytesToUint16(const uint8_t *data) { return (uint16_t(data[0]) << 8) | uint16_t(data[1]); }

// For signed 8-bit (this is safe, but explicit)
inline int8_t bytesToInt8(const uint8_t *data) { return static_cast<int8_t>(data[0]); }

// For unsigned 8-bit
inline uint8_t bytesToUint8(const uint8_t *data) { return data[0]; }

// Split a value into unsigned bytes (big-endian)
inline void int16_to_bytes(uint16_t val, uint8_t *out)
{
    out[0] = (val >> 8) & 0xFF;
    out[1] = (val) & 0xFF;
}

inline void uint16_to_bytes(uint16_t value, uint8_t *out)
{
    out[0] = (value >> 8) & 0xFF;
    out[1] = value & 0xFF;
}

inline void int32_to_bytes(int32_t val, uint8_t *out)
{
    out[0] = (val >> 24) & 0xFF;
    out[1] = (val >> 16) & 0xFF;
    out[2] = (val >> 8) & 0xFF;
    out[3] = (val) & 0xFF;
}
// For uint32_t
inline void uint32_to_bytes(uint32_t val, uint8_t *out)
{
    out[0] = (val >> 24) & 0xFF;
    out[1] = (val >> 16) & 0xFF;
    out[2] = (val >> 8) & 0xFF;
    out[3] = (val) & 0xFF;
}
// For 8-bit
inline void int8_to_bytes(uint8_t val, uint8_t *out) { out[0] = val & 0xFF; }

/// How a fixed-point mapping resolves a value that falls between two codes.
enum class Quantisation
{
    Nearest,  ///< Round to the nearest code, halves away from zero.
    Truncate, ///< Drop the fraction, as the official Python SDK does.
};

/// Float -> N-bit unsigned mapping used by the MIT joint commands. Mirrors piper_sdk's
/// FloatToUint(value, min, max, bits), but clamps to [min, max] first so an out-of-range value
/// saturates instead of wrapping into the neighbouring bit fields. The value must be finite.
inline uint32_t floatToUintRange(double value, double min, double max, int bits, Quantisation quantisation)
{
    if (value < min)
        value = min;
    if (value > max)
        value = max;
    const double span = max - min;
    const uint32_t max_int = (uint32_t{1} << bits) - 1u;
    if (quantisation == Quantisation::Nearest)
        return static_cast<uint32_t>(std::lround((value - min) / span * max_int));
    // Same operation order as the SDK, so a value on a code boundary truncates to the same code.
    return static_cast<uint32_t>(std::trunc((value - min) * max_int / span));
}

// ---- Frame layouts shared by every protocol version ----
//
// Where a version changes a message, it changes the values in the fields, not where they sit,
// so each layout is written once here and the parsers only differ in what they put into it.

/// Write the 0x151 payload. `wire` holds wire bytes, already mapped for the target protocol.
inline void writeMotionCtrl2(const ArmMsgMotionCtrl_2 &wire, uint8_t *data)
{
    data[0] = wire.ctrl_mode;
    data[1] = wire.move_mode;
    data[2] = wire.move_spd_rate_ctrl;
    data[3] = wire.mit_mode;
    data[4] = wire.residence_time;
    data[5] = wire.installation_pos;
    // [6..7] zero
}

/// Write the 0x159 payload with `status` as the wire status byte.
inline void writeGripperCtrl(const ArmMsgGripperCtrl &ctrl, uint8_t status, uint8_t *data)
{
    int32_to_bytes(ctrl.grippers_angle, &data[0]);
    uint16_to_bytes(ctrl.grippers_effort, &data[4]);
    data[6] = status;
    data[7] = ctrl.set_zero;
}

/// Write the MIT fields both protocols place identically: position, velocity, Kp, Kd and the
/// high nibble of byte 6. The torque bits after them differ per protocol.
inline void writeMitGains(const ArmMsgJointMitCtrl &mit, uint8_t *data)
{
    data[0] = (mit.pos_ref >> 8) & 0xFF;
    data[1] = mit.pos_ref & 0xFF;
    data[2] = (mit.vel_ref >> 4) & 0xFF;
    data[3] = ((mit.vel_ref & 0xF) << 4) | ((mit.kp >> 8) & 0x0F);
    data[4] = mit.kp & 0xFF;
    data[5] = (mit.kd >> 4) & 0xFF;
    data[6] = (mit.kd & 0xF) << 4;
}

/// True for the six per-joint MIT command types.
inline bool isJointMitCtrl(ArmMsgType type)
{
    return type >= ArmMsgType::JointMitCtrl1 && type <= ArmMsgType::JointMitCtrl6;
}

/// Every Piper command is an 8-byte standard frame whose ID follows from the type, so callers
/// only fill in the payload fields.
inline void setCommandHeader(ArmMsgType type, struct can_frame &tx_frame)
{
    tx_frame.can_id = static_cast<canid_t>(msgTypeToCanId(type));
    tx_frame.can_dlc = 8;
}

} // namespace piper_cpp
