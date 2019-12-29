#include <gtest/gtest.h>
#include "tread_speed_publisher.h"

TEST(TreadSpeed, Basic)
{
    double wheelRadius=1, ticksPerRevolution=4, maxTicks=1e4;
    TreadSpeed treadSpeed(ticksPerRevolution, maxTicks, wheelRadius);
    treadSpeed.updateFromNewCount(4);
	EXPECT_EQ(treadSpeed.speed, 1);
	treadSpeed.updateFromNewCount(10);
	EXPECT_NE(treadSpeed.speed, 2.5);
	EXPECT_EQ(treadSpeed.speed, 1.5);
	treadSpeed.updateFromNewCount(6);
	EXPECT_EQ(treadSpeed.speed, -1);
	//EXPECT_EQ(treadSpeed.speed, -3);
}

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
