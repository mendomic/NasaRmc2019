/****************************************************************************************
 * File:    monitor_arm_current.cpp
 * Node:    monitor_arm_current
 * 
 * Purpose: This is a subscriber that will listen to the current output while the arm is moving 
 *			to make sure that it does not exceed the capacity of the controllers
 *			To help tell the digging action server to change the digging queue becausethe current one
 *			is using to much power
 *
 *	TO_DO: not finished yet, might not use for this years compition 
 ****************************************************************************************/

#include "ros/ros.h"
#include "std_msgs/String.h"
#include <tfr_msgs/DiggingAction.h>  // Note: "Action" is appended
#include <tfr_msgs/ArmMoveAction.h>  // Note: "Action" is appended
#include <tfr_control/robot_interface.h>

const float max_turntable_amps = 0.0;
const float max_lower_arm_amps = 0.0;
const float max_upper_arm_amps = 0.0;
const float max_scoop_amps = 0.0;

void monitor_turntable_amps(const std_msgs::Float64 &msg)
{
	float turntable_amps = msg.data;
	if (cmath::fabs(turntable_amps) > max_turntable_amps)
	{
		//TO_DO:
	}
}

void monitor_lower_arm_amps(const std_msgs::Float64 &msg)
{
	float lower_arm_amps = msg.data;
	if (cmath::fabs(lower_arm_amps) > max_lower_arm_amps)
	{
		//TO_DO:
	}
}

void monitor_upper_arm_amps(const std_msgs::Float64 &msg)
{
	float upper_arm_amps = msg.data;
	if (cmath::fabs(upper_arm_amps) > max_upper_arm_amps)
	{
		//TO_DO:
	}
}

void monitor_scoop_amps(const std_msgs::Float64 &msg)
{
	float scoop_amps = msg.data;
	if (cmath::fabs(scoop_amps) > max_scoop_amps)
	{
		//TO_DO:
	}
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "monitor_arm_current");
	ros::NodeHandle n;
    
	ros::Subscriber turntable_sub = n.subscribe("/device4/get_qry_batamps/channel_1", 
		1, monitor_turntable_amps);
	ros::Subscriber lower_arm_sub = n.subscribe("/device12/get_qry_batamps/channel_1", 
		1, monitor_lower_arm_amps);
	ros::Subscriber upper_arm_sub = n.subscribe("/device4/get_qry_batamps/channel_3", 
		1, monitor_upper_arm_amps);
	ros::Subscriber scoop_sub = n.subscribe("/device4/get_qry_batamps/channel_2", 
		1, monitor_scoop_amps);
	
	ros::spin();
	return 1;
		
}