#include <gtest/gtest.h>
#include "tread_distance_publisher.h"
#include <cmath>

TEST(TreadDistance, Basic)
{
    double wheelRadius=1, ticksPerRevolution=4, maxTicks=1e4;
    TreadDistance treadDistance(ticksPerRevolution, maxTicks, wheelRadius);
    treadDistance.updateFromNewCount(4);
	EXPECT_EQ(treadDistance.distanceTraveled, 2*M_PI);
	treadDistance.updateFromNewCount(10);
	EXPECT_NE(treadDistance.distanceTraveled, 2.5*2*M_PI);
	EXPECT_EQ(treadDistance.distanceTraveled, 1.5*2*M_PI);
	treadDistance.updateFromNewCount(6);
	EXPECT_EQ(treadDistance.distanceTraveled, -2*M_PI);
}

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
