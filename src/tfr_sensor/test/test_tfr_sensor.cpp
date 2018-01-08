#include <gtest/gtest.h>
#include "sensor_helper.h"

TEST(Sensor, SensorHelper)
{
    // Google tests provide no handlers for exceptions
    try
    {
        tfr_sensor::SensorHelper helper;
        EXPECT_STREQ("Sensor System Online", helper.GetEcho().c_str());
    }
    catch (std::exception e)
    {
        // An unhandled exception was thrown; fail the test
        ASSERT_TRUE(false);
    }
}

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}