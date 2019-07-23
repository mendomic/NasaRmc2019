/****************************************************************************************
 * File:            drivebase_publisher.cpp
 * 
 * Purpose:         This is the implementation file for the DrivebasePublisher class.
 *                  See tfr_control/include/tfr_control/drivebase_publisher.h for details.
 ***************************************************************************************/
#include "drivebase_publisher.h"

namespace tfr_control
{
    DrivebasePublisher::DrivebasePublisher(
        ros::NodeHandle& n, double wheel_radius, double wheel_span) : 
        n{n}, wheel_radius{wheel_radius}, wheel_span{wheel_span}, 
        left_tread_publisher{}, right_tread_publisher{}
    {
        left_tread_publisher = n.advertise<std_msgs::Float64>(
            "left_tread_velocity_controller/command", 5);
        right_tread_publisher = n.advertise<std_msgs::Float64>(
            "right_tread_velocity_controller/command", 5);


        subscriber = n.subscribe("cmd_vel", 100, &DrivebasePublisher::subscriptionCallback, this);
    }

    void DrivebasePublisher::subscriptionCallback(const geometry_msgs::Twist::ConstPtr& msg)
    {
        ros::param::getCached("~wheel_span", wheel_span);
        //ROS_INFO("Wheel Span: %f", wheel_span);

        double left_velocity = msg->linear.x - (wheel_span * msg->angular.z) / 2;
        double right_velocity = msg->linear.x + (wheel_span * msg->angular.z) / 2;
        
        if (msg->linear.x == 0 && msg->angular.z == 0 && msg->linear.y){
            left_velocity = 0.05;
            right_velocity = 0.05;
            if(msg->linear.y < 0){
                left_velocity = -left_velocity;
                right_velocity = -right_velocity;
            }
        }
        

        std_msgs::Float64 left_cmd;
        left_cmd.data = left_velocity;
        std_msgs::Float64 right_cmd;
        right_cmd.data = right_velocity;
        left_tread_publisher.publish(left_cmd);
        right_tread_publisher.publish(right_cmd);
        
        // Debug. Write the setpoints to the parameter server so that other nodes can see them.
        n.setParam("/left_tread_velocity_controller/setpoint", left_cmd.data);
        n.setParam("/right_tread_velocity_controller/setpoint", right_cmd.data);
    }

}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "drivebase");

    ros::NodeHandle n;
    
    double wheel_span, wheel_radius;

    ros::param::param<double>("~wheel_span", wheel_span, 1);
    if (wheel_span <= 0)
    {
        ROS_ERROR("Parameter 'wheel_span' must be a positive value.");
        return 1;
    }

    ros::param::param<double>("~wheel_radius", wheel_radius, 1);
    if (wheel_radius <= 0)
    {
        ROS_ERROR("Parameter 'wheel_radius' must be a positive value.");
        return 1;
    }
    
    tfr_control::DrivebasePublisher publisher(n, wheel_span, wheel_radius);
    
    ROS_INFO("drivebase started");

    ros::spin();
    return 0;
}
