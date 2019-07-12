#ifndef STATUS_CODE_H
#define STATUS_CODE_H

#include <cstdint>
#include <string>

const int MAX_MESSAGE_SIZE = 100;

/**
 * Definitions of Sub Systems.
 *
 * These codes use the top 8 bits to uniquely identify the sub system a
 * particular status code belongs to and should never be changed.
 */
 
enum class SubSystem: uint16_t
{
    SYS = 0,
    EXC = 0X100,
    LOC = 0X200,
    NAV = 0X400,
    MIN = 0X800,
    DMP = 0X1000
};

/**
 * Defines the list of all status codes used by all Sub Systems.
 *
 * These codes consist of two parts:
 *  - The top 8 bits which identify the sub system
 *  - The bottom 8 bits which identify the status of the sub system
 *
 * When adding a new status code for a sub system, use the system identifier
 * defined in the SubSystem enum class above and then increase the previous 8
 * bit code by one. For example, if adding a new status code for the Executive
 * sub system and the most recently added status code is 0b0000000100000000, the
 * new status code should be defined as 0b0000000100000001.
 *
 * Don't forget to also add a message for any newly added status codes in
 * status_code.cpp.
 */
enum class StatusCode : uint16_t
{
    //System Status Codes
    SYS_OK = 0,
    SYS_MOTOR_TOGGLE = 1,

    //Executive Status Codes
    EXC_OK = 0X100,
    EXC_CONNECT_LOCALIZATION = 0X101,
    EXC_CONNECT_NAVIGATION = 0X102,

    //Localization Status Codes
    LOC_OK = 0X200,

    //Navigation Status Codes
    NAV_OK = 0X400,

    //Mining Status Codes
    MIN_OK = 0X800,

    //Dumping Status Codes
    DMP_OK = 0X1000
};

/**
 * Get the status message from a status code and status data.
 */
std::string getStatusMessage(StatusCode code, float data);

#endif
