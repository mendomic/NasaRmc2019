#include <arm_manipulator.h>

ArmManipulator::ArmManipulator(ros::NodeHandle &n, bool init_joints):
            arm_action_client{n, "move_arm"},
            trajectory_publisher{n.advertise<trajectory_msgs::JointTrajectory>("/move_arm", 5)},
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
