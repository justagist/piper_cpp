#pragma once
#include <array>
#include <cmath>
#include <stdexcept>
#include <tuple>

namespace piper_utils
{

using Mat4 = std::array<double, 16>;
using JointVec = std::array<double, 6>;

static constexpr double PI = 3.14159265358979323846;
static constexpr double RADIAN = 180.0 / PI;

// Supported axes mapping
enum class EulerAxes
{
    sxyz,
    rzyx
};

namespace detail
{
constexpr std::array<int, 4> _NEXT_AXIS{1, 2, 0, 1};
constexpr double _EPS = 1e-10;

// Only support "sxyz" and "rzyx"
inline void axesToTuple(EulerAxes axes, int& firstaxis, int& parity, int& repetition, int& frame)
{
    switch (axes)
    {
    case EulerAxes::sxyz:
        firstaxis = 0;
        parity = 0;
        repetition = 0;
        frame = 0;
        break;
    case EulerAxes::rzyx:
        firstaxis = 0;
        parity = 0;
        repetition = 0;
        frame = 1;
        break;
    default:
        throw std::invalid_argument("Unsupported axes specification");
    }
}
} // namespace detail

/** Normalize quaternion (x, y, z, w) */
inline std::array<double, 4> normalizeQuat(double qx, double qy, double qz, double qw)
{
    double norm = std::sqrt(qx * qx + qy * qy + qz * qz + qw * qw);
    if (norm < detail::_EPS)
        throw std::runtime_error("Zero-norm quaternion");
    return {qx / norm, qy / norm, qz / norm, qw / norm};
}

// Converts 4x4 matrix to euler [x, y, z, roll, pitch, yaw] (deg)
inline std::array<double, 6> matrixToEuler(const Mat4& T)
{
    std::array<double, 6> Pos = {0};
    // Position
    Pos[0] = T[3];
    Pos[1] = T[7];
    Pos[2] = T[11];
    // Orientation
    if (T[8] < -1 + 1e-4)
    {
        Pos[4] = 90.0; // pitch
        Pos[5] = 0;
        Pos[3] = std::atan2(T[1], T[5]) * RADIAN; // roll
    }
    else if (T[8] > 1 - 1e-4)
    {
        Pos[4] = -90.0;
        Pos[5] = 0;
        Pos[3] = -std::atan2(T[1], T[5]) * RADIAN;
    }
    else
    {
        double _bt = std::atan2(-T[8], std::sqrt(T[0] * T[0] + T[4] * T[4]));
        Pos[4] = _bt * RADIAN;
        Pos[5] = std::atan2(T[4] / std::cos(_bt), T[0] / std::cos(_bt)) * RADIAN;
        Pos[3] = std::atan2(T[9] / std::cos(_bt), T[10] / std::cos(_bt)) * RADIAN;
    }
    return Pos;
}

// Matrix multiplication for 4x4 (flattened as 16 elem array)
inline static Mat4 matMultiply(const Mat4& A, const Mat4& B)
{
    Mat4 C{};
    for (int i = 0; i < 4; ++i)
        for (int j = 0; j < 4; ++j)
            for (int k = 0; k < 4; ++k)
                C[4 * i + j] += A[4 * i + k] * B[4 * k + j];
    return C;
}

/** Quaternion to Euler (returns [roll, pitch, yaw], radians) */
inline std::array<double, 3> quatToEuler(double qx, double qy, double qz, double qw, EulerAxes axes = EulerAxes::sxyz)
{
    int firstaxis, parity, repetition, frame;
    detail::axesToTuple(axes, firstaxis, parity, repetition, frame);

    int i = firstaxis;
    int j = detail::_NEXT_AXIS[i + parity];
    int k = detail::_NEXT_AXIS[i - parity + 1];

    auto nq = normalizeQuat(qx, qy, qz, qw);
    qx = nq[0];
    qy = nq[1];
    qz = nq[2];
    qw = nq[3];

    // Build rotation matrix from quaternion
    double M[3][3];
    M[0][0] = 1 - 2 * (qy * qy + qz * qz);
    M[0][1] = 2 * (qx * qy - qz * qw);
    M[0][2] = 2 * (qx * qz + qy * qw);
    M[1][0] = 2 * (qx * qy + qz * qw);
    M[1][1] = 1 - 2 * (qx * qx + qz * qz);
    M[1][2] = 2 * (qy * qz - qx * qw);
    M[2][0] = 2 * (qx * qz - qy * qw);
    M[2][1] = 2 * (qy * qz + qx * qw);
    M[2][2] = 1 - 2 * (qx * qx + qy * qy);

    double ax, ay, az;
    if (repetition)
    {
        double sy = std::sqrt(M[i][j] * M[i][j] + M[i][k] * M[i][k]);
        if (sy > detail::_EPS)
        {
            ax = std::atan2(M[i][j], M[i][k]);
            ay = std::atan2(sy, M[i][i]);
            az = std::atan2(M[j][i], -M[k][i]);
        }
        else
        {
            ax = std::atan2(-M[j][k], M[j][j]);
            ay = std::atan2(sy, M[i][i]);
            az = 0.0;
        }
    }
    else
    {
        double cy = std::sqrt(M[i][i] * M[i][i] + M[j][i] * M[j][i]);
        if (cy > detail::_EPS)
        {
            ax = std::atan2(M[k][j], M[k][k]);
            ay = std::atan2(-M[k][i], cy);
            az = std::atan2(M[j][i], M[i][i]);
        }
        else
        {
            ax = std::atan2(-M[j][k], M[j][j]);
            ay = std::atan2(-M[k][i], cy);
            az = 0.0;
        }
    }

    if (parity)
    {
        ax = -ax;
        ay = -ay;
        az = -az;
    }
    if (frame)
    {
        std::swap(ax, az);
    }
    return {ax, ay, az}; // [roll, pitch, yaw]
}

/** Euler (roll, pitch, yaw) to quaternion [x, y, z, w] */
inline std::array<double, 4> eulerToQuat(double roll, double pitch, double yaw, EulerAxes axes = EulerAxes::sxyz)
{
    int firstaxis, parity, repetition, frame;
    detail::axesToTuple(axes, firstaxis, parity, repetition, frame);
    int i = firstaxis;
    int j = detail::_NEXT_AXIS[i + parity];
    int k = detail::_NEXT_AXIS[i - parity + 1];

    if (frame)
        std::swap(roll, yaw);
    if (parity)
        pitch = -pitch;

    roll *= 0.5;
    pitch *= 0.5;
    yaw *= 0.5;

    double c_roll = std::cos(roll), s_roll = std::sin(roll);
    double c_pitch = std::cos(pitch), s_pitch = std::sin(pitch);
    double c_yaw = std::cos(yaw), s_yaw = std::sin(yaw);

    double cc = c_roll * c_yaw;
    double cs = c_roll * s_yaw;
    double sc = s_roll * c_yaw;
    double ss = s_roll * s_yaw;

    std::array<double, 4> q{0, 0, 0, 0};
    if (repetition)
    {
        q[i] = c_pitch * (cs + sc);
        q[j] = s_pitch * (cc + ss);
        q[k] = s_pitch * (cs - sc);
        q[3] = c_pitch * (cc - ss);
    }
    else
    {
        q[i] = c_pitch * sc - s_pitch * cs;
        q[j] = c_pitch * ss + s_pitch * cc;
        q[k] = c_pitch * cs - s_pitch * sc;
        q[3] = c_pitch * cc + s_pitch * ss;
    }
    if (parity)
        q[j] *= -1;
    return q; // [qx, qy, qz, qw]
}

} // namespace piper_utils
