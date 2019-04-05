// C++11

#include <iostream>
#include <unistd.h>
#include <chrono>
#include <thread>
#include <ros/ros.h>
#include <tfr_msgs/PwmCommand.h>

// The wiringPi library declares its functions directly in the global namespace. It can go fuck itself.
namespace wiringpi
{
    #include <wiringPi.h>
    #include <wiringSerial.h>
}

// Note, the envrionment which runs this executable must first set the following environment variable:
// export WIRINGPI_GPIOMEM=1

const std::string UART_DEVICE_NAME = "/dev/ttyS0";
const int UART_BAUD_RATE = 9600;
const int right_power_scale = 70;
const int left_power_scale = 75;

/*
void set_motors(const uint8_t right_motor_power, const uint8_t left_motor_power)
{
	const int COMMAND_SIZE_BYTES = 2;
	uint8_t motor_power[COMMAND_SIZE_BYTES] = {right_motor_power, left_motor_power};
	//write(fd, motor_power, COMMAND_SIZE_BYTES);
}
*/

void serialWriteCallback(const tfr_msgs::PwmCommand & command)
{
    //ROS_INFO("In callback");

    int fd = wiringpi::serialOpen(UART_DEVICE_NAME.c_str(),
                                  UART_BAUD_RATE);
    if (fd < 0)
    {
       // ROS_ERROR("wiringPi failed to open serial device " 
        //<< UART_DEVICE_NAME << "." << std::endl);
    }

    //ROS_INFO("wiringPi opened serial device " << UART_DEVICE_NAME 
    //<< ", baud " << UART_BAUD_RATE << "." << std::endl);

    const int COMMAND_SIZE_BYTES = 2;
    uint8_t motor_power[COMMAND_SIZE_BYTES] = {0, 0};

    const int MOTOR_RIGHT = 0;
    const int MOTOR_LEFT  = 1;
    
    if(command.enabled)
    {
    
        //ROS_INFO("writing power");
        	

        write(fd, motor_power, COMMAND_SIZE_BYTES);

        motor_power[MOTOR_RIGHT] = command.tread_right;
        motor_power[MOTOR_LEFT] = command.tread_left;

        write(fd, motor_power, COMMAND_SIZE_BYTES);	
	
        std::this_thread::sleep_for(std::chrono::milliseconds(2000));

        motor_power[MOTOR_RIGHT] = 0;
        motor_power[MOTOR_LEFT]  = 0;

        write(fd, motor_power, COMMAND_SIZE_BYTES);
    }
    else
    {
        ROS_INFO("command not enabled");
        write(fd, motor_power, COMMAND_SIZE_BYTES);
    }
}


int main (int argc, char** argv)
{
    ROS_INFO("starting raspberry pi node");
    ros::init(argc, argv, "rp_control");
    if ( wiringpi::wiringPiSetup() == -1 )
    {
        ROS_ERROR("wiringPiSetup() failed.");
    }   
    ros::NodeHandle n;
    ros::Subscriber sub_obj = n.subscribe("/motor_output", 4, serialWriteCallback);
    ROS_INFO("About to spin raspberry pi node");
    ros::spin();
    ROS_INFO("Have returned from spin");


    return 0;
}

