// C++11

#include <iostream>
#include <unistd.h>
#include <chrono>
#include <thread>
#include <ros/ros.h>
#include <tfr_msgs/PwmCommand.h>

// The wiringPi library declares its functions directly in the global namespace by default, so we put it inside of a namespace.
namespace wiringpi
{
    #include <wiringPi.h>
    #include <wiringSerial.h>
}

// Note, the envrionment which runs this executable must first set the following environment variable:
// export WIRINGPI_GPIOMEM=1

const std::string UART_DEVICE_NAME = "/dev/ttyS0";
const int UART_BAUD_RATE = 9600;
const int COMMAND_SIZE_BYTES = 2;
const int MOTOR_RIGHT = 0;
const int MOTOR_LEFT  = 1;
int fd;
int right_power_scale = 70;
int left_power_scale = 75;


void serialWriteCallback(const tfr_msgs::PwmCommand & command) {
    //ROS_INFO("In callback");

    int8_t motorPower[COMMAND_SIZE_BYTES] = {0, 0};
    uint8_t * unsignedPower = (uint8_t *)motorPower;
    
    if(command.enabled) {
        ros::param::getCached("~right_power_scale", right_power_scale);
        ros::param::getCached("~left_power_scale", left_power_scale);
        

        motorPower[MOTOR_RIGHT] = command.tread_right * right_power_scale;
        motorPower[MOTOR_LEFT] = -command.tread_left * left_power_scale;
        
        if (command.tread_right != 0 || command.tread_left != 0) {
            ROS_INFO("writing power: %d, %d", motorPower[MOTOR_RIGHT], motorPower[MOTOR_LEFT]);
        }

        write(fd, motorPower, COMMAND_SIZE_BYTES);	
    }
    else {
        ROS_INFO("command not enabled");
        write(fd, motorPower, COMMAND_SIZE_BYTES);
    }
}


int main (int argc, char** argv)
{
    ROS_INFO("starting raspberry pi node");
    ros::init(argc, argv, "rp_control");
    
    //setup serial connection
    if ( wiringpi::wiringPiSetup() == -1 )
    {
        ROS_ERROR("wiringPiSetup() failed.");
    }
    fd = wiringpi::serialOpen(UART_DEVICE_NAME.c_str(), UART_BAUD_RATE);
    if (fd < 0)
    {
        ROS_ERROR("wiringPi failed to open serial device %s.", UART_DEVICE_NAME.c_str());
    }
    else
    {
        ROS_INFO("wiringPi opened serial device %s, baud %d.", UART_DEVICE_NAME.c_str(), UART_BAUD_RATE);
    }
    
    ros::NodeHandle n;
    ros::Subscriber sub_obj = n.subscribe("/motor_output", 4, serialWriteCallback);
    ROS_INFO("About to spin raspberry pi node");
    ros::spin();
    ROS_INFO("Have returned from spin");


    return 0;
}

