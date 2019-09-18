#include <arm_manipulator.h>

ArmManipulator::ArmManipulator(ros::NodeHandle &n, bool init_joints):
            arm_action_client{n, "move_arm"},
            trajectory_publisher{n.advertise<trajectory_msgs::JointTrajectory>("/arm_controller/command", 5)},
            scoop_trajectory_publisher{n.advertise<trajectory_msgs::JointTrajectory>("/arm_end_controller/command", 5)}
{
  ROS_INFO("Initializing Arm Manipulator");
	if (init_joints) initializeJointLimits();
  ROS_ERROR("Beep");
}

void ArmManipulator::initializeJointLimits()
{
	// Get the model description
    std::string desc;
    if (!ros::param::get("/robot_description", desc)) {desc = "";}

    if (desc.length() == 0)
    {
        ROS_WARN("robot_description is empty and controller_launcher is using fake values, quitting.");
        return;
    }

    urdf::Model model;
    if (!model.initString(desc))
    {
        ROS_WARN("Couldn't load robot_description.");
        return;
    }

    ROS_INFO("Model loaded successfully, loading joint limits.");
    const int joint_count = 5;
    using namespace tfr_utilities;
    int joint_enum_array[joint_count] = {Joint::BIN, Joint::LOWER_ARM,
      Joint::UPPER_ARM, Joint::SCOOP};
    std::string joint_name_array[joint_count] = {"bin_joint", "lower_arm_joint",
      "upper_arm_joint", "scoop_joint"};
    for (int i = 0; i < joint_count; i++){
      auto joint_name = joint_name_array[i];
      auto joint_ptr = model.getJoint(joint_name);
      if (!joint_ptr){
        ROS_WARN_STREAM("Could not locate joint " << joint_name << " on model to access limits");
        continue;
      }
      auto limits = joint_ptr->limits;
      lower_limits[joint_enum_array[i]] = limits->lower;
      upper_limits[joint_enum_array[i]] = limits->upper;
    }
}

void ArmManipulator::moveArm(const double& turntable, const double& lower_arm ,const double& upper_arm,  const double& scoop )
{
    ROS_INFO_STREAM("Arm manipulator called by: " << ros::this_node::getName() << ". Parameters: " << turntable << ", " << lower_arm << ", " << upper_arm << ", " << scoop << std::endl);

    tfr_msgs::ArmMoveGoal goal{};
    goal.pose.resize(4);

    goal.pose[0] = turntable;
    goal.pose[1] = lower_arm;
    goal.pose[2] = upper_arm;
    goal.pose[3] = scoop;

    arm_action_client.sendGoal(goal);
    while ( !arm_action_client.getState().isDone() && ros::ok())
    {
        if (! ros::ok())
        {
            arm_action_client.cancelAllGoals();
            ROS_INFO("Arm Manip: exiting");
            return;
        }
    }

    if (arm_action_client.getState()!=actionlib::SimpleClientGoalState::SUCCEEDED)
    {
        ROS_INFO("Arm Manip: arm move failed");
        return;
    }
}

// Asynchronous call.
void ArmManipulator::moveArmWithoutPlanningOrLimits(
            const double& turntable, const double& lower_arm, const double& upper_arm, const double& scoop)
{
    ROS_INFO_STREAM("moveArmWithoutPlanningOrLimits() called by: " << ros::this_node::getName() << ". Parameters: " << turntable << ", " << lower_arm << ", " << upper_arm << ", " << scoop << std::endl);

    // Reference: http://wiki.ros.org/pr2_controllers/Tutorials/Moving%20the%20arm%20using%20the%20Joint%20Trajectory%20Action#Creating_the_node

    trajectory_msgs::JointTrajectory arm_traj = createSinglePointArmTrajectory(turntable, lower_arm, upper_arm);
    trajectory_msgs::JointTrajectory arm_end_traj = createSinglePointArmEndTrajectory(scoop);    

    trajectory_publisher.publish(arm_traj);
    scoop_trajectory_publisher.publish(arm_end_traj);

    return;
}

// TODO: Check for NaN input.
double ArmManipulator::clamp(double input, double bound_1, double bound_2)
{
	double lower_bound = 0;
	double upper_bound = 0;

	if ( bound_1 <= bound_2 )	{
		lower_bound = bound_1;
		upper_bound = bound_2;
	} else {
		lower_bound = bound_2;
		upper_bound = bound_1;
	}
	return std::max(std::min(input, upper_bound), lower_bound);
}

trajectory_msgs::JointTrajectory
ArmManipulator::createSinglePointArmTrajectory(
    const double turntable, const double lower_arm, const double upper_arm)
{
    const int NUM_ARM_TRAJ_JOINTS = 3;

    trajectory_msgs::JointTrajectory arm_traj;

    arm_traj.joint_names.resize(NUM_ARM_TRAJ_JOINTS); // very important to resize fisrt, otherwise data will not be populated.
    arm_traj.joint_names[0] = "turntable_joint";
    arm_traj.joint_names[1] = "lower_arm_joint";
    arm_traj.joint_names[2] = "upper_arm_joint";
    
    arm_traj.points.resize(1); // publish a single waypoint. A trajectory can have multiple.
    
    arm_traj.points[0].positions.resize(NUM_ARM_TRAJ_JOINTS);
    arm_traj.points[0].positions[0] = turntable;
    arm_traj.points[0].positions[1] = lower_arm;
    arm_traj.points[0].positions[2] = upper_arm;
    
    // tell the arm to stop at the waypoint
    arm_traj.points[0].velocities.resize(NUM_ARM_TRAJ_JOINTS);
    arm_traj.points[0].velocities[0] = 0;
    arm_traj.points[0].velocities[1] = 0;
    arm_traj.points[0].velocities[2] = 0;

    // Dunno if we need this one.
    arm_traj.points[0].time_from_start = ros::Duration(1.0); 

    arm_traj.header.stamp = ros::Time::now();

    return arm_traj;
}

trajectory_msgs::JointTrajectory
ArmManipulator::createSinglePointArmEndTrajectory(const double scoop)
{
    const int NUM_ARM_END_TRAJ_JOINTS = 1;

    trajectory_msgs::JointTrajectory arm_end_traj;

    arm_end_traj.joint_names.resize(NUM_ARM_END_TRAJ_JOINTS);
    arm_end_traj.joint_names[0] = "scoop_joint";
    
    arm_end_traj.points.resize(1); 
    
    arm_end_traj.points[0].positions.resize(NUM_ARM_END_TRAJ_JOINTS);
    arm_end_traj.points[0].positions[0] = scoop;
    
    arm_end_traj.points[0].velocities.resize(NUM_ARM_END_TRAJ_JOINTS);
    arm_end_traj.points[0].velocities[0] = 0;

    arm_end_traj.points[0].time_from_start = ros::Duration(1.0); 

    arm_end_traj.header.stamp = ros::Time::now();

    return arm_end_traj;
}

void ArmManipulator::moveArmWithLimits(const double& turntable, const double& lower_arm ,const double& upper_arm,  const double& scoop )
{
	double clamp_turntable = turntable; // no joint limits on turntable

	double clamp_lower_arm = clamp(lower_arm,
				lower_limits[static_cast<int>(tfr_utilities::Joint::LOWER_ARM)],
				upper_limits[static_cast<int>(tfr_utilities::Joint::LOWER_ARM)]);

	double clamp_upper_arm = clamp(upper_arm,
				lower_limits[static_cast<int>(tfr_utilities::Joint::UPPER_ARM)],
				upper_limits[static_cast<int>(tfr_utilities::Joint::UPPER_ARM)]);

	double clamp_scoop = clamp(scoop,
				lower_limits[static_cast<int>(tfr_utilities::Joint::SCOOP)],
				upper_limits[static_cast<int>(tfr_utilities::Joint::SCOOP)]);

	moveArm(clamp_turntable, clamp_lower_arm, clamp_upper_arm, clamp_scoop);
}
