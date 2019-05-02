/****************************************************************************************
 * File:    monitor_arm_current.cpp
 * Node:    monitor_arm_current
 * 
 * Purpose: This is a subscriber that will listen to the current output while the arm is moving 
 *			to make sure that it does not exceed the capacity of the controllers
 ****************************************************************************************/

#include "ros/ros.h"
#include "std_msgs/String.h"
#include <tfr_msgs/DiggingAction.h>  // Note: "Action" is appended
#include <tfr_msgs/ArmMoveAction.h>  // Note: "Action" is appended


int main(int argc, char **argv)
{
	ros::init(argc, argv, "monitor_arm_current");
	ros::NodeHandle n;
    
	ros:Subscriber sub = n.subscribe()
}