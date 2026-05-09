#pragma once
#include <array>
#include <cmath>
#include <cstdint>
#include <vector>

#include "piper_cpp/utils.h"

namespace piper_cpp
{

class PiperForwardKinematics
{
public:
    // Constants

    using Mat4 = std::array<double, 16>;
    using JointVec = std::array<double, 6>;
    static constexpr double PI = 3.14159265358979323846;
    static constexpr double RADIAN = 180.0 / PI;

    // Members
    JointVec _a, _alpha, _theta, _d, init_pos;

    PiperForwardKinematics(uint8_t dh_is_offset = 0x01)
    {
        if (dh_is_offset == 0x01)
        {
            _a = {0, 0, 285.03, -21.98, 0, 0};
            _alpha = {0, -PI / 2, 0, PI / 2, -PI / 2, PI / 2};
            _theta = {0, -PI * 172.22 / 180.0, -102.78 / 180.0 * PI, 0, 0, 0};
            _d = {123, 0, 0, 250.75, 0, 91};
            init_pos = {56.128, 0, 213.266, 0, 85.0, 0};
        }
        else
        {
            _a = {0, 0, 285.03, -21.98, 0, 0};
            _alpha = {0, -PI / 2, 0, PI / 2, -PI / 2, PI / 2};
            _theta = {0, -PI * 174.22 / 180.0, -100.78 / 180.0 * PI, 0, 0, 0};
            _d = {123, 0, 0, 250.75, 0, 91};
            init_pos = {55.0, 0, 205.0, 0, 85.0, 0};
        }
    }

    // Main FK function: returns a vector of arrays [x, y, z, roll, pitch, yaw] for each link
    std::vector<std::array<double, 6>> calcFK(const JointVec& cur_j) const
    {
        std::array<Mat4, 6> Ts;
        for (size_t i = 0; i < 6; ++i)
            Ts[i] = linkTransformation(_alpha[i], _a[i], cur_j[i] + _theta[i], _d[i]);

        Mat4 R02 = piper_utils::matMultiply(Ts[0], Ts[1]);
        Mat4 R03 = piper_utils::matMultiply(R02, Ts[2]);
        Mat4 R04 = piper_utils::matMultiply(R03, Ts[3]);
        Mat4 R05 = piper_utils::matMultiply(R04, Ts[4]);
        Mat4 R06 = piper_utils::matMultiply(R05, Ts[5]);

        std::vector<std::array<double, 6>> l_pose;
        l_pose.push_back(piper_utils::matrixToEuler(Ts[0]));
        l_pose.push_back(piper_utils::matrixToEuler(R02));
        l_pose.push_back(piper_utils::matrixToEuler(R03));
        l_pose.push_back(piper_utils::matrixToEuler(R04));
        l_pose.push_back(piper_utils::matrixToEuler(R05));
        l_pose.push_back(piper_utils::matrixToEuler(R06));
        return l_pose;
    }

private:
    // Single link transformation matrix (DH)
    static Mat4 linkTransformation(double alpha, double a, double theta, double d)
    {
        double ca = std::cos(alpha), sa = std::sin(alpha);
        double ct = std::cos(theta), st = std::sin(theta);
        Mat4 T{};
        T[0] = ct;
        T[1] = -st;
        T[2] = 0;
        T[3] = a;
        T[4] = st * ca;
        T[5] = ct * ca;
        T[6] = -sa;
        T[7] = -sa * d;
        T[8] = st * sa;
        T[9] = ct * sa;
        T[10] = ca;
        T[11] = ca * d;
        T[12] = 0;
        T[13] = 0;
        T[14] = 0;
        T[15] = 1;
        return T;
    }
};

} // namespace piper_cpp
