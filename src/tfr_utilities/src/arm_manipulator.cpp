#include <arm_manipulator.h>

ArmManipulator::ArmManipulator(ros::NodeHandle &n):
            trajectory_publisher{n.advertise<trajectory_msgs::JointTrajectory>("/move_arm", 5)},
            scoop_trajectory_publisher{n.advertise<trajectory_msgs::JointTrajectory>("/arm_end_controller/command", 5)}
{ }

void  ArmManipulator::moveArm(const double& turntable, const double& lower_arm ,const double& upper_arm,  const double& scoop )
{
    ROS_INFO_STREAM("Arm manipulator called by: " << ros::this_node::getName() << ". Parameters: " << turntable << ", " << lower_arm << ", " << upper_arm << ", " << scoop << std::endl);

    trajectory_msgs::JointTrajectory trajectory;
    trajectory.header.stamp = ros::Time::now();
    trajectory.joint_names.resize(4);
    trajectory.points.resize(1);
    trajectory.points[0].positions.resize(3);
    trajectory.joint_names[0]="turntable_joint";
    trajectory.joint_names[1]="lower_arm_joint";
    trajectory.joint_names[2]="upper_arm_joint";
    trajectory.joint_names[3]="scoop_joint";
    trajectory.points[0].positions[0] = turntable;
    trajectory.points[0].positions[1] = lower_arm;
    trajectory.points[0].positions[2] = upper_arm;
    trajectory.points[0].positions[3] = scoop;
    trajectory.points[0].time_from_start = ros::Duration(0.06);
    trajectory_publisher.publish(trajectory);
}
