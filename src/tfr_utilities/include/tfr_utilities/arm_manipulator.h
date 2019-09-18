#ifndef ARM_MANIPULATOR_H
#define ARM_MANIPULATOR_H
#include <ros/ros.h>
#include <tfr_msgs/ArmMoveAction.h>
#include <actionlib/server/simple_action_server.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <joints.h>
#include <urdf/model.h>
#include <actionlib/client/simple_action_client.h>

/**
 * Provides a simple method for moving the arm without MoveIt.
 * This is a regular ole' class, just instantiate it and call moveArm.
 * */
class ArmManipulator
{
    public:
        ArmManipulator(ros::NodeHandle &n, bool init_joints=true);
        ~ArmManipulator(){};
        ArmManipulator(const ArmManipulator&) = delete;
        ArmManipulator& operator=(const ArmManipulator&) = delete;
        ArmManipulator(ArmManipulator&&)=delete;
        ArmManipulator& operator=(ArmManipulator&&)=delete;

		/**
         * Moves the arm to the given position.
		 *
		 * Notes:
		 *  - Careful what parameters are passed in, the arm could collide with the robot.
		 *
		 *  - The method is not blocking, so the caller needs to wait for the arm to move.
		 *    See digging_action_server.cpp for example.
         * */
        void moveArm( const double& turntable, const double& lower_arm, const double& upper_arm, const double& scoop);

		/*
		 * Same as moveArm but clamps trajectories to be within the URDF model's joint limits.
		 */
		void moveArmWithLimits(const double& turntable, const double& lower_arm ,const double& upper_arm,  const double& scoop );


        void moveArmWithoutPlanningOrLimits(
            const double& turntable, const double& lower_arm, const double& upper_arm, const double& scoop);
        
    private:
        ros::Publisher trajectory_publisher;
        ros::Publisher scoop_trajectory_publisher;
        actionlib::SimpleActionClient<tfr_msgs::ArmMoveAction> arm_action_client;

		void initializeJointLimits();

		// Return input, restricted to be within the two bounds (inclusive).
		double clamp(double input, double bound_1, double bound_2);

		double lower_limits[tfr_utilities::Joint::JOINT_COUNT] = {0};
		double upper_limits[tfr_utilities::Joint::JOINT_COUNT] = {0};

        trajectory_msgs::JointTrajectory createSinglePointArmTrajectory(const double turntable, const double lower_arm, const double upper_arm);
        trajectory_msgs::JointTrajectory createSinglePointArmEndTrajectory(const double scoop);
 };

#endif
