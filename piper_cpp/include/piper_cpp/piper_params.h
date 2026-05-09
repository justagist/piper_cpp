#pragma once

#include "piper_cpp/version.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <mutex>
#include <shared_mutex>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace piper_cpp
{

class PiperParamManager
{
public:
    static PiperParamManager& instance()
    {
        static PiperParamManager inst;
        return inst;
    }

    // ----- Types -----
    enum class Joint : uint8_t
    {
        J1 = 0,
        J2,
        J3,
        J4,
        J5,
        J6
    };
    using Range = std::array<double, 2>; ///< {min, max}.
    using JointLimits = std::array<Range, 6>;

    struct PiperParams
    {
        JointLimits joint_limits_rad;
        Range gripper_range_m;
        const std::string sdk_version;
    };

    // ----- Resets / snapshots -----
    void resetDefaults()
    {
        std::unique_lock lock(mutex_);
        joint_limits_rad_ = kDefaultsJointRad;
        gripper_range_m_ = kDefaultGripperRangeM;
    }

    PiperParams getPiperParams() const
    {
        std::shared_lock lock(mutex_);
        return PiperParams{joint_limits_rad_, gripper_range_m_, sdk_version_};
    }

    // ----- Version -----
    std::string getSDKVersion() const
    {
        std::shared_lock lock(mutex_);
        return sdk_version_;
    }

    // ----- Joint limits (radians) -----
    Range getJointLimitRad(Joint j) const
    {
        std::shared_lock lock(mutex_);
        return joint_limits_rad_[static_cast<size_t>(j)];
    }
    Range getJointLimitRad(const std::string& name) const { return getJointLimitRad(toJoint(name)); }

    void setJointLimitRad(Joint j, double min_rad, double max_rad)
    {
        if (max_rad < min_rad)
            throw std::invalid_argument("max_val must be >= min_val");
        std::unique_lock lock(mutex_);
        joint_limits_rad_[static_cast<size_t>(j)] = {min_rad, max_rad};
    }
    void setJointLimitRad(const std::string& name, double min_rad, double max_rad)
    {
        setJointLimitRad(toJoint(name), min_rad, max_rad);
    }

    // ----- Gripper range (meters) -----
    Range getGripperRangeM() const
    {
        std::shared_lock lock(mutex_);
        return gripper_range_m_;
    }

    void setGripperRangeM(double min_m, double max_m)
    {
        if (max_m < min_m)
            throw std::invalid_argument("max_val must be >= min_val");
        std::unique_lock lock(mutex_);
        gripper_range_m_ = {min_m, max_m};
    }

    // ----- Helpers -----
    /// Convert a joint name string ("j1".."j6", case-insensitive) to a `Joint` enum value.
    /// @throws std::invalid_argument if the name doesn't match.
    static Joint toJoint(const std::string& name)
    {
        if (name.size() != 2 || (name[0] != 'j' && name[0] != 'J'))
            throw std::invalid_argument("joint_name must be j1..j6");

        const char idx = name[1];
        if (idx < '1' || idx > '6')
            throw std::invalid_argument("joint_name must be j1..j6");

        return static_cast<Joint>(idx - '1');
    }
    /// Clamp a joint angle expressed in milli-degrees (0.001 deg) to the configured limits for
    /// the given joint. Pass `enabled = false` to bypass the clamp and return the input as-is.
    int clampJointMilliDeg(Joint j, int value_mdeg, bool enabled = true) const
    {
        if (!enabled)
            return value_mdeg;
        auto [mn_rad, mx_rad] = getJointLimitRad(j);
        const double rad2deg = 180.0 / std::acos(-1.0);
        const long jmin_mdeg = std::lround(mn_rad * rad2deg * 1000.0);
        const long jmax_mdeg = std::lround(mx_rad * rad2deg * 1000.0);
        return static_cast<int>(std::clamp<long>(value_mdeg, jmin_mdeg, jmax_mdeg));
    }

    /// Clamp a joint angle in radians (the SDK's native unit) to the configured limits for
    /// the given joint. Pass `enabled = false` to bypass the clamp.
    double clampJointRad(Joint j, double value_rad, bool enabled = true) const
    {
        if (!enabled)
            return value_rad;
        auto [mn_rad, mx_rad] = getJointLimitRad(j);
        return std::clamp(value_rad, mn_rad, mx_rad);
    }

    /// Clamp gripper jaw opening expressed in micrometres (0.001 mm) to the configured range.
    /// Pass `enabled = false` to bypass the clamp.
    int clampGripperMicrometers(int value_um, bool enabled = true) const
    {
        if (!enabled)
            return value_um;
        auto [mn_m, mx_m] = getGripperRangeM();
        const long gmin_um = std::lround(mn_m * 1'000'000.0); // m -> µm
        const long gmax_um = std::lround(mx_m * 1'000'000.0);
        return static_cast<int>(std::clamp<long>(value_um, gmin_um, gmax_um));
    }

    /// Clamp gripper jaw opening in metres (native units) to the configured range. Pass
    /// `enabled = false` to bypass the clamp.
    double clampGripperMeters(double value_m, bool enabled = true) const
    {
        if (!enabled)
            return value_m;
        auto [mn_m, mx_m] = getGripperRangeM();
        return std::clamp(value_m, mn_m, mx_m);
    }

private:
    PiperParamManager() { resetDefaults(); }
    // Defaults (radians / meters)
    static constexpr JointLimits kDefaultsJointRad{
        {
         Range{-2.6179, 2.6179},  // j1  [-150°, +150°]
            Range{0.0, 3.14},        // j2  [0°, 180°]
            Range{-2.967, 0.0},      // j3  [-170°, 0°]
            Range{-1.745, 1.745},    // j4  [-100°, +100°]
            Range{-1.22, 1.22},      // j5  [-70°, +70°]
            Range{-2.09439, 2.09439} // j6  [-120°, +120°]
        }
    };
    static constexpr Range kDefaultGripperRangeM{0.0, 0.07}; // 0–70mm

    // Storage
    mutable std::shared_mutex mutex_;
    JointLimits joint_limits_rad_{};
    Range gripper_range_m_{};
    const std::string sdk_version_ = PIPER_CPP_CURRENT_VERSION;
};

} // namespace piper_cpp
