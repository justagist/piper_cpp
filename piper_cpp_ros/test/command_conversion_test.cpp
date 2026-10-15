#include "piper_cpp_ros/command_conversion.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <limits>

using piper_cpp_ros::gripperCommandToMicrometres;
using piper_cpp_ros::jointCommandToMillideg;

TEST(CommandConversion, RejectsValuesWithNoWireRepresentation)
{
    const double nan = std::numeric_limits<double>::quiet_NaN();
    const double inf = std::numeric_limits<double>::infinity();
    EXPECT_FALSE(jointCommandToMillideg(nan));
    EXPECT_FALSE(jointCommandToMillideg(inf));
    EXPECT_FALSE(jointCommandToMillideg(-inf));
    EXPECT_FALSE(jointCommandToMillideg(1e300)) << "would overflow the int32 wire field";
    EXPECT_FALSE(jointCommandToMillideg(7.0)) << "more than a full turn";
    EXPECT_FALSE(gripperCommandToMicrometres(nan));
    EXPECT_FALSE(gripperCommandToMicrometres(5.0)) << "five metres of jaw travel";
}

TEST(CommandConversion, ConvertsPlausibleCommands)
{
    EXPECT_EQ(jointCommandToMillideg(0.0), 0);
    EXPECT_EQ(jointCommandToMillideg(M_PI / 2), 90000);
    EXPECT_EQ(jointCommandToMillideg(-M_PI), -180000);
    EXPECT_EQ(gripperCommandToMicrometres(0.035), 35000);
    EXPECT_EQ(gripperCommandToMicrometres(0.0), 0);
}
