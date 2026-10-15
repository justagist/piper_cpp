#pragma once

#include <cmath>
#include <cstdint>
#include <optional>

namespace piper_cpp_ros
{

constexpr double kRadToMd = 1000.0 * 180.0 / M_PI;
constexpr double kMdToRad = 1.0 / kRadToMd;

// piper_cpp's high-speed feedback reports motor_speed in 0.001 rad/s and effort in 0.001 N*m.
constexpr double kMilliToUnit = 1.0e-3;

// Gripper feedback / command: jaw stroke is in 0.001 mm = micrometers (1 um = 1e-6 m).
constexpr double kUmToM = 1.0e-6;
constexpr double kMToUm = 1.0e6;

inline double mdToRad(int32_t md) { return static_cast<double>(md) * kMdToRad; }

/// Joint position command in rad to the arm's 0.001 deg units, or nullopt when the value is not
/// finite or lies beyond a full turn. Real joint limits are the controller's job; this only
/// refuses values that have no sensible wire representation.
inline std::optional<int32_t> jointCommandToMillideg(double rad)
{
    if (!std::isfinite(rad) || std::fabs(rad) > 2.0 * M_PI)
        return std::nullopt;
    return static_cast<int32_t>(std::lround(rad * kRadToMd));
}

/// Gripper stroke command in metres to micrometres, with the same acceptance rule; one metre
/// is far beyond any jaw.
inline std::optional<int32_t> gripperCommandToMicrometres(double m)
{
    if (!std::isfinite(m) || std::fabs(m) > 1.0)
        return std::nullopt;
    return static_cast<int32_t>(std::lround(m * kMToUm));
}

} // namespace piper_cpp_ros
