#include <arm_manipulator.h>

ArmManipulator::ArmManipulator(ros::NodeHandle &n):
            trajectory_publisher{n.advertise<trajectory_msgs::JointTrajectory>("/move_arm", 5)},
            scoop_trajectory_publisher{n.advertise<trajectory_msgs::JointTrajectory>("/arm_end_controller/command", 5)}
{
	initializeJointLimits();
}

void ArmManipulator::moveArm(const double& turntable, const double& lower_arm ,const double& upper_arm,  const double& scoop )
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
    lower_limits[static_cast<int>(tfr_utilities::Joint::BIN)] 
        = model.getJoint("bin_joint")->limits->lower;
    upper_limits[static_cast<int>(tfr_utilities::Joint::BIN)] 
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
