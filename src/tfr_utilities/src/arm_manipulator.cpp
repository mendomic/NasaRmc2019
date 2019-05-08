#include <arm_manipulator.h>

ArmManipulator::ArmManipulator(ros::NodeHandle &n):
            arm_action_client{n, "move_arm"},
            trajectory_publisher{n.advertise<trajectory_msgs::JointTrajectory>("/move_arm", 5)},
            scoop_trajectory_publisher{n.advertise<trajectory_msgs::JointTrajectory>("/arm_end_controller/command", 5)}
{
	initializeJointLimits();
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

void ArmManipulator::initializeJointLimits()
{
	// Get the model description 
    std::string desc;
    if (!ros::param::get("/robot_description", desc)) {desc = "";}

    if (desc.length() == 0) 
    {
        ROS_WARN("robot_description is empty and controller_launcher is using fake values, quitting.");
    }

    urdf::Model model;
    if (!model.initString(desc)) 
    {
        ROS_WARN("Couldn't load robot_description.");
    }

    ROS_INFO("Model loaded successfully, loading joint limits.");
    lower_limits[static_cast<int>(tfr_utilities::Joint::LEFT_BIN)] 
        = model.getJoint("bin_joint")->limits->lower;
    upper_limits[static_cast<int>(tfr_utilities::Joint::LEFT_BIN)] 
        = model.getJoint("bin_joint")->limits->upper;
    lower_limits[static_cast<int>(tfr_utilities::Joint::RIGHT_BIN)] 
        = model.getJoint("bin_joint")->limits->lower;
    upper_limits[static_cast<int>(tfr_utilities::Joint::RIGHT_BIN)] 
        = model.getJoint("bin_joint")->limits->upper;
    lower_limits[static_cast<int>(tfr_utilities::Joint::LOWER_ARM)] 
        = model.getJoint("lower_arm_joint")->limits->lower;
    upper_limits[static_cast<int>(tfr_utilities::Joint::LOWER_ARM)] 
        = model.getJoint("lower_arm_joint")->limits->upper;
    lower_limits[static_cast<int>(tfr_utilities::Joint::UPPER_ARM)] 
        = model.getJoint("upper_arm_joint")->limits->lower;
    upper_limits[static_cast<int>(tfr_utilities::Joint::UPPER_ARM)] 
        = model.getJoint("upper_arm_joint")->limits->upper;
    lower_limits[static_cast<int>(tfr_utilities::Joint::SCOOP)] 
        = model.getJoint("scoop_joint")->limits->lower;
    upper_limits[static_cast<int>(tfr_utilities::Joint::SCOOP)] 
        = model.getJoint("scoop_joint")->limits->upper;
}

// TODO: Check for NaN input.
double ArmManipulator::clamp(double input, double bound_1, double bound_2)
{
	double lower_bound = 0;
	double upper_bound = 0;
	
	if ( bound_1 <= bound_2 )
	{
		lower_bound = bound_1;
		upper_bound = bound_2;
	}
	else
	{
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
