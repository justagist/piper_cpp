#include "piper_cpp/fk.h"

#include <gtest/gtest.h>

using piper_cpp::PiperForwardKinematics;

// The manufacturer's SDK quotes the home pose of the default (offset) DH set in mm and
// degrees; the FK must reproduce it at zero joint angles, which pins the output units. The
// non-offset set's quoted pose is rounded to the millimetre, so it is not asserted.
TEST(ForwardKinematics, ZeroJointsGiveTheHomePoseInMillimetresAndDegrees)
{
    PiperForwardKinematics fk;
    const auto pose = fk.calcFK({0, 0, 0, 0, 0, 0}).back();
    for (int i = 0; i < 6; ++i)
        EXPECT_NEAR(pose[i], fk.init_pos[i], 0.001) << "component " << i;
}

TEST(ForwardKinematics, InputIsRadians)
{
    PiperForwardKinematics fk;
    // A quarter turn of joint 1 swings the home position from +x onto +y.
    const auto pose = fk.calcFK({PiperForwardKinematics::PI / 2, 0, 0, 0, 0, 0}).back();
    EXPECT_NEAR(pose[0], 0.0, 0.01);
    EXPECT_NEAR(pose[1], fk.init_pos[0], 0.01);
    EXPECT_NEAR(pose[2], fk.init_pos[2], 0.01);
}
