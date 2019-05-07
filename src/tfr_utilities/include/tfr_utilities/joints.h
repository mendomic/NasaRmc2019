/*
 * Shared codes and enums for teleop transmission
 * */
#ifndef JOINT_H
#define JOINT_H

#include <cstdint>

namespace tfr_utilities
{
    /*
     * All of the joints on the robot
     * */
    enum Joint
    {
        LEFT_TREAD, 
        RIGHT_TREAD, 
        BIN, 
        TURNTABLE, 
        LOWER_ARM,
        UPPER_ARM, 
        SCOOP,
        JOINT_COUNT,
    };
}
#endif
