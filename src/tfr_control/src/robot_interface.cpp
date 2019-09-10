
/** * controller.cpp * 
 * This class is in charge of handling the physical hardware interface with
 * the robot itself, and is started by the controller_launcher node.
 */
#include "robot_interface.h"
#include <tfr_utilities/joints.h>

using hardware_interface::JointStateHandle;
using hardware_interface::JointHandle;

namespace tfr_control
{
    /*
     * Creates the robot interfaces spins up all the joints and registers them
     * with their relevant interfaces
     * */
    RobotInterface::RobotInterface(ros::NodeHandle &n, bool fakes, 
            const double *lower_lim, const double *upper_lim) :


        brushless_left_tread_vel{n.subscribe("/device8/get_qry_abcntr/channel_1", 5,
                &RobotInterface::setBrushlessLeftEncoder, this)},
        brushless_left_tread_vel_publisher{n.advertise<std_msgs::Int32>("/device8/set_cmd_cango/cmd_cango_1", 1)},
        
        
        brushless_right_tread_vel{n.subscribe("/device8/get_qry_abcntr/channel_2", 5,
                &RobotInterface::setBrushlessRightEncoder, this)},
        brushless_right_tread_vel_publisher{n.advertise<std_msgs::Int32>("/device8/set_cmd_cango/cmd_cango_2", 1)},
        
        
        turntable_subscriber_encoder{n.subscribe("/device4/get_qry_abcntr/channel_1", 5,
                &RobotInterface::readTurntableEncoder, this)},
        turntable_subscriber_amps{n.subscribe("/device4/get_qry_batamps/channel_1", 1,
                &RobotInterface::readTurntableAmps, this)},
        turntable_publisher{n.advertise<std_msgs::Int32>("/device4/set_cmd_cango/cmd_cango_1", 1)},
        
        
        lower_arm_subscriber_encoder{n.subscribe("/device23/get_joint_state", 5, 
                &RobotInterface::readLowerArmEncoder, this)},
        lower_arm_subscriber_amps{n.subscribe("/device12/get_qry_batamps/channel_1", 1,
                &RobotInterface::readLowerArmAmps, this)},
        lower_arm_publisher{n.advertise<sensor_msgs::JointState>("/device23/set_joint_state", 1)},
        
        
        upper_arm_subscriber_encoder{n.subscribe("/device45/get_joint_state", 5,
                &RobotInterface::readUpperArmEncoder, this)},
        upper_arm_subscriber_amps{n.subscribe("/device4/get_qry_batamps/channel_3", 1,
                &RobotInterface::readUpperArmAmps, this)},
        upper_arm_publisher{n.advertise<sensor_msgs::JointState>("/device45/set_joint_state", 1)},
        
        
        scoop_subscriber_encoder{n.subscribe("/device56/get_joint_state", 5,
                &RobotInterface::readScoopEncoder, this)},
        scoop_subscriber_amps{n.subscribe("/device4/get_qry_batamps/channel_2", 1,
                &RobotInterface::readScoopAmps, this)},
        scoop_publisher{n.advertise<sensor_msgs::JointState>("/device56/set_joint_state", 1)},
        
        left_tread_publisher_pid_debug_setpoint{n.advertise<std_msgs::Float64>("/left_tread_velocity_controller/pid_debug/setpoint", 1)},
        left_tread_publisher_pid_debug_state{n.advertise<std_msgs::Float64>("/left_tread_velocity_controller/pid_debug/state", 1)},
        left_tread_publisher_pid_debug_command{n.advertise<std_msgs::Int32>("/left_tread_velocity_controller/pid_debug/command", 1)},
        
        //pwm_publisher{n.advertise<tfr_msgs::PwmCommand>("/motor_output", 15)},
        use_fake_values{fakes}, lower_limits{lower_lim},
        upper_limits{upper_lim}, drivebase_v0{std::make_pair(0,0)},
        last_update{ros::Time::now()},
        enabled{true}
    {
        /*
        // TODO: Enable getting parameters from server instead of hardcoded values in .h file.
        if (    !n.getParam("bin_joint/min", static_cast<int>(bin_encoder_min))
             || !n.getParam("bin_joint/max", static_cast<int>(bin_encoder_max))
             || !n.getParam("turntable_joint/min", static_cast<int>(turntable_encoder_min))
             || !n.getParam("turntable_joint/max", static_cast<int>(turntable_encoder_max))
             || !n.getParam("lower_arm_joint/min", static_cast<int>(arm_lower_encoder_min))
             || !n.getParam("lower_arm_joint/max", static_cast<int>(arm_lower_encoder_max))
             || !n.getParam("upper_arm_joint/min", static_cast<int>(arm_upper_encoder_min))
             || !n.getParam("upper_arm_joint/max", static_cast<int>(arm_upper_encoder_max))
             || !n.getParam("scoop_joint/min", static_cast<int>(arm_end_encoder_min))
             || !n.getParam("scoop_joint/max", static_cast<int>(arm_end_encoder_max))
           )
           {
               ROS_ERROR("tfr_control failed to find all of the encoder limits. Maybe they weren't uploaded to the parameter server by the launch file?");
           }
        */
        
        // Note: the string parameters in these constructors must match the
        // joint names from the URDF, and yaml controller description. 

	// DEBUG: Try to make printing faster
	std::ios_base::sync_with_stdio(false);

        // Connect and register each joint with appropriate interfaces at our
        // layer
        registerJointEffortInterface("left_tread_joint", tfr_utilities::Joint::LEFT_TREAD);
        registerJointEffortInterface("right_tread_joint", tfr_utilities::Joint::RIGHT_TREAD);
        registerJointEffortInterface("bin_joint", tfr_utilities::Joint::BIN); 
        registerJointPositionInterface("turntable_joint", tfr_utilities::Joint::TURNTABLE);
        registerJointPositionInterface("lower_arm_joint", tfr_utilities::Joint::LOWER_ARM);
        registerJointPositionInterface("upper_arm_joint", tfr_utilities::Joint::UPPER_ARM);
        registerJointPositionInterface("scoop_joint", tfr_utilities::Joint::SCOOP);
        //register the interfaces with the controller layer
        registerInterface(&joint_state_interface);
        registerInterface(&joint_effort_interface);
        registerInterface(&joint_position_interface);
        
        
        for (int joint = 0; joint < tfr_utilities::Joint::JOINT_COUNT; joint++)
        {
            velocity_values[joint] = 0;
            effort_values[joint] = 0;
        }
    }


    /*
     * Reads from our hardware and populates from shared memory.  
     *
     * Information that is not explicity needed by our controllers 
     * is written to some safe sensible default (usually 0).
     *
     * */
 void RobotInterface::read() 
    {
        //LEFT_TREAD
        position_values[static_cast<int>(tfr_utilities::Joint::LEFT_TREAD)] = 0;
        velocity_values[static_cast<int>(tfr_utilities::Joint::LEFT_TREAD)] = readBrushlessLeftVel();
        effort_values[static_cast<int>(tfr_utilities::Joint::LEFT_TREAD)] = 0;

        //RIGHT_TREAD
        position_values[static_cast<int>(tfr_utilities::Joint::RIGHT_TREAD)] = 0;
        velocity_values[static_cast<int>(tfr_utilities::Joint::RIGHT_TREAD)] = readBrushlessRightVel();
        effort_values[static_cast<int>(tfr_utilities::Joint::RIGHT_TREAD)] = 0;

        if (!use_fake_values)
        {
            //TURNTABLE
            double turntable_position_double = 
                linear_interp<double>(static_cast<double>(turntable_encoder), static_cast<double>(turntable_encoder_min),
                    turntable_joint_min,
                    static_cast<double>(turntable_encoder_max),
                    turntable_joint_max
                    );
            
            position_values[static_cast<int>(tfr_utilities::Joint::TURNTABLE)] = turntable_position_double;
                //reading_a.arm_turntable_pos + turntable_offset;
            velocity_values[static_cast<int>(tfr_utilities::Joint::TURNTABLE)] = 0; 
            effort_values[static_cast<int>(tfr_utilities::Joint::TURNTABLE)] = 0;

            //LOWER_ARM
            //position_values[static_cast<int>(tfr_utilities::Joint::LOWER_ARM)] = reading_a.arm_lower_pos;
            double lower_arm_position_double = 
                linear_interp<double>(
                    static_cast<double>(lower_arm_encoder),
                    static_cast<double>(arm_lower_encoder_min),
                    arm_lower_joint_min,
                    static_cast<double>(arm_lower_encoder_max),
                    arm_lower_joint_max
                    );
        
            double upper_arm_position_double = 
                linear_interp<double>(
                    static_cast<double>(upper_arm_encoder),
                    static_cast<double>(arm_upper_encoder_min),
                    arm_upper_joint_min,
                    static_cast<double>(arm_upper_encoder_max),
                    arm_upper_joint_max
                    );

            double scoop_position_double = 
                linear_interp<double>(
                    static_cast<double>(scoop_encoder),
                    static_cast<double>(arm_end_encoder_min),
                    arm_end_joint_min,
                    static_cast<double>(arm_end_encoder_max),
                    arm_end_joint_max
                    );

            position_values[static_cast<int>(tfr_utilities::Joint::LOWER_ARM)] = lower_arm_position_double;
            velocity_values[static_cast<int>(tfr_utilities::Joint::LOWER_ARM)] = 0;
            effort_values[static_cast<int>(tfr_utilities::Joint::LOWER_ARM)] = 0;

            //UPPER_ARM
            position_values[static_cast<int>(tfr_utilities::Joint::UPPER_ARM)] = upper_arm_position_double; //reading_a.arm_upper_pos;
            velocity_values[static_cast<int>(tfr_utilities::Joint::UPPER_ARM)] = 0;
            effort_values[static_cast<int>(tfr_utilities::Joint::UPPER_ARM)] = 0;

            //SCOOP
            position_values[static_cast<int>(tfr_utilities::Joint::SCOOP)] = scoop_position_double; //reading_a.arm_scoop_pos;
            velocity_values[static_cast<int>(tfr_utilities::Joint::SCOOP)] = 0;
            effort_values[static_cast<int>(tfr_utilities::Joint::SCOOP)] = 0;

            /*
            ROS_INFO_STREAM("turntable_position: read: encoder: " << turntable_encoder << std::endl);
            ROS_INFO_STREAM("turntable_position: read: " << position_values[static_cast<int>(tfr_utilities::Joint::TURNTABLE)] << std::endl);
            */
            
            /*
            ROS_INFO_STREAM("arm_lower_position: read: encoder: " << lower_arm_encoder << std::endl);
            ROS_INFO_STREAM("arm_lower_position: read: " << position_values[static_cast<int>(tfr_utilities::Joint::LOWER_ARM)] << std::endl);
            */
            
            /*
            ROS_INFO_STREAM("arm_upper_position: read: encoder: " << upper_arm_encoder << std::endl);
            ROS_INFO_STREAM("arm_upper_position: read: " << position_values[static_cast<int>(tfr_utilities::Joint::UPPER_ARM)] << std::endl);
            */
            
            /*
            ROS_INFO_STREAM("scoop_position: read: encoder: " << scoop_encoder << std::endl);
            ROS_INFO_STREAM("scoop_position: read: " << position_values[static_cast<int>(tfr_utilities::Joint::SCOOP)] << std::endl);
            */
            
        }
 
        //BIN
        position_values[static_cast<int>(tfr_utilities::Joint::BIN)] = 0; 
        velocity_values[static_cast<int>(tfr_utilities::Joint::BIN)] = 0;
        effort_values[static_cast<int>(tfr_utilities::Joint::BIN)] = 0;

    }

    /*
     * Writes command values from our controllers to our motors and actuators.
     *
     * Takes in command values from the controllers and these values are scaled
     * to pwm outputs and written to the right place. There are some edge cases
     * for twin actuators, which are controlled as if they are one joint. 
     *
     * The controller gives a command value to move them as one, then we scale
     * our pwm outputs to move them back into sync if they get out of wack.
     * */
    void RobotInterface::write() 
    {
		static double prev_arm_lower_position = std::numeric_limits<double>::quiet_NaN();
		static double prev_arm_upper_position = std::numeric_limits<double>::quiet_NaN();
		static double prev_scoop_position = std::numeric_limits<double>::quiet_NaN();

        double signal;
        if (use_fake_values) //test code  for working with rviz simulator
        {
            adjustFakeJoint(tfr_utilities::Joint::TURNTABLE);
            adjustFakeJoint(tfr_utilities::Joint::LOWER_ARM);
            adjustFakeJoint(tfr_utilities::Joint::UPPER_ARM);
            adjustFakeJoint(tfr_utilities::Joint::SCOOP);
        }
        else  // we are working with the real arm
        {
            bool write_arm_values;
            if (not ros::param::getCached("/write_arm_values", write_arm_values)) {write_arm_values = false;}
            if (write_arm_values){
                //ROS_INFO("Robot Interface: writing arm values");
                
                //TURNTABLE
                int32_t turntable_position =  std::max(std::min(-command_values[static_cast<int>(tfr_utilities::Joint::TURNTABLE)], 1000.0), -1000.0);
                std_msgs::Int32 turntable_position_msg;
                turntable_position_msg.data = turntable_position;
                turntable_publisher.publish(turntable_position_msg);


				// For the Servo Cylinder actuators, only publish a setpoint to them if the setpoint has actually changed. 
				// Testing whether this smoothes out the movement of the actuators.
				
				//LOWER_ARM
				//NOTE we reverse these because actuator is mounted backwards
				double arm_lower_position = linear_interp(command_values[static_cast<int>(tfr_utilities::Joint::LOWER_ARM)], arm_lower_joint_min, arm_lower_encoder_min, arm_lower_joint_max, arm_lower_encoder_max);
				if (arm_lower_position != prev_arm_lower_position)
				{
					sensor_msgs::JointState arm_lower_position_msg;
					arm_lower_position_msg.position.push_back(arm_lower_position);
					lower_arm_publisher.publish(arm_lower_position_msg);
					
					prev_arm_lower_position = arm_lower_position;
				}

				//UPPER_ARM
				double arm_upper_position = linear_interp(command_values[static_cast<int>(tfr_utilities::Joint::UPPER_ARM)], arm_upper_joint_min, arm_upper_encoder_min, arm_upper_joint_max, arm_upper_encoder_max);
				if (arm_upper_position != prev_arm_upper_position)
				{
					sensor_msgs::JointState arm_upper_position_msg;
					arm_upper_position_msg.position.push_back(arm_upper_position);
					upper_arm_publisher.publish(arm_upper_position_msg);
					
					prev_arm_upper_position = arm_upper_position;
				}
            
				//SCOOP
				double scoop_position = linear_interp(command_values[static_cast<int>(tfr_utilities::Joint::SCOOP)], arm_end_joint_min, arm_end_encoder_min, arm_end_joint_max, arm_end_encoder_max);
				if (scoop_position != prev_scoop_position)
				{		
					sensor_msgs::JointState scoop_position_msg;
					scoop_position_msg.position.push_back(scoop_position);
					scoop_publisher.publish(scoop_position_msg);
					
					prev_scoop_position = scoop_position;
				}
            
            } else {
                //ROS_INFO("Robot Interface: not writing arm values");
            }
            
        }
        
        //LEFT_TREAD
        double left_tread_command = command_values[static_cast<int32_t>(tfr_utilities::Joint::LEFT_TREAD)];
        std_msgs::Int32 left_tread_msg;
        left_tread_msg.data = -1 * clamp(static_cast<int32_t>(left_tread_command), -1000, 1000);
	    brushless_left_tread_vel_publisher.publish(left_tread_msg);

        //ROS_INFO_STREAM("left_tread_velocity_value: " << velocity_values[static_cast<int>(tfr_utilities::Joint::LEFT_TREAD)] << std::endl);
	    //ROS_INFO_STREAM("left_tread_command: " << left_tread_command << std::endl);

        
        

	    if (enable_left_tread_pid_debug_output)
	    {
	        double left_tread_setpoint = 0;
            ros::param::get("/left_tread_velocity_controller/setpoint", left_tread_setpoint);
            
            /*
            ROS_INFO_STREAM_NAMED("debugrobotinterface", 
            "left_tread: " << std::setw(8) << std::setprecision(2) << left_tread_setpoint << ", "
            << std::setw(8) << std::setprecision(2) << velocity_values[tfr_utilities::Joint::LEFT_TREAD] << ", "
            << std::setw(8) << left_tread_msg.data);
	        */
	        std_msgs::Float64 left_tread_setpoint_msg;
	        std_msgs::Float64 left_tread_state_msg;
	        
	        left_tread_setpoint_msg.data = left_tread_setpoint;
	        left_tread_state_msg.data = velocity_values[static_cast<int>(tfr_utilities::Joint::LEFT_TREAD)];
	        
	        left_tread_publisher_pid_debug_setpoint.publish(left_tread_setpoint_msg);
	        left_tread_publisher_pid_debug_state.publish(left_tread_state_msg);
	        left_tread_publisher_pid_debug_command.publish(left_tread_msg);
	        
	    }


        //RIGHT_TREAD
        double right_tread_command = command_values[static_cast<int32_t>(tfr_utilities::Joint::RIGHT_TREAD)];
        std_msgs::Int32 right_tread_msg;
        right_tread_msg.data = -1 * clamp(static_cast<int32_t>(right_tread_command), -1000, 1000);
        brushless_right_tread_vel_publisher.publish(right_tread_msg);
        
        //UPKEEP
        last_update = ros::Time::now();
        drivebase_v0.first = velocity_values[static_cast<int>(tfr_utilities::Joint::LEFT_TREAD)];
        drivebase_v0.second = velocity_values[static_cast<int>(tfr_utilities::Joint::RIGHT_TREAD)];
    }
    
    template <typename T>
    T RobotInterface::linear_interp(T x, T x1, T y1, T x2, T y2)
    {
        // line defined by two points: (x1, y1) and (x2, y2)
        T y = ((y2 - y1)/(x2 - x1))*(x - x1) + y1;
        return y;
    }
    
    template <typename T>
    T RobotInterface::clamp(const T input, const T bound_1, const T bound_2)
    {
        T lower_bound;
        T upper_bound;
        
        if (bound_1 < bound_2) {
            lower_bound = bound_1;
            upper_bound = bound_2;        
        } else {
            upper_bound = bound_1;
            lower_bound = bound_2;  
        }      

        return std::max(std::min(input, upper_bound), lower_bound);
    }

    // TODO: Horrible duplication of code should be removed.
    // has hardcoded min joint position in header file
    const int32_t RobotInterface::get_arm_lower_min_int()
    {
        double arm_lower_joint_min_mapped = linear_interp<double>(arm_lower_joint_min * -10,
                arm_lower_joint_min,
                std::numeric_limits<int32_t>::min(),
                arm_lower_joint_max, 
                std::numeric_limits<int32_t>::max());
        int32_t arm_lower_joint_min_mapped_int = static_cast<int32_t>(arm_lower_joint_min_mapped);
        
        return arm_lower_joint_min_mapped_int;
    }
        
    // has hardcoded max joint position in header file
    const int32_t RobotInterface::get_arm_lower_max_int()
    {
        double arm_lower_joint_max_mapped = linear_interp<double>(arm_lower_joint_max * 10,
                arm_lower_joint_min,
                std::numeric_limits<int32_t>::min(),
                arm_lower_joint_max, 
                std::numeric_limits<int32_t>::max());
        int32_t arm_lower_joint_max_mapped_int = static_cast<int32_t>(arm_lower_joint_max_mapped);
        
        return arm_lower_joint_max_mapped_int;
    }
    
    void RobotInterface::setEnabled(bool val)
    {
        enabled = val;
    }

    void RobotInterface::adjustFakeJoint(const tfr_utilities::Joint &j)
    {
        int i = static_cast<int>(j);
        position_values[i] = command_values[i];
        // If this joint has limits, clamp the range down
        if (std::abs(lower_limits[i]) >= 1E-3 || std::abs(upper_limits[i]) >= 1E-3) 
        {
            position_values[i] =
                std::max(std::min(position_values[i],
                            upper_limits[i]), lower_limits[i]);
        }
    }

    /*
     * Resets the commands to a safe neutral state
     * Tells the treads to stop moving, and the arm to hold position
     * */
    void RobotInterface::clearCommands()
    {
        //LEFT_TREAD
        command_values[static_cast<int>(tfr_utilities::Joint::LEFT_TREAD)] = 0;

        //RIGHT_TREAD
        command_values[static_cast<int>(tfr_utilities::Joint::RIGHT_TREAD)] = 0;

        //TURNTABLE
        command_values[static_cast<int>(tfr_utilities::Joint::TURNTABLE)] = 0;

        //LOWER_ARM
        command_values[static_cast<int>(tfr_utilities::Joint::LOWER_ARM)] = 0;
        
        //UPPER_ARM
        command_values[static_cast<int>(tfr_utilities::Joint::UPPER_ARM)] = 0;
        //SCOOP
        command_values[static_cast<int>(tfr_utilities::Joint::SCOOP)] = 0;
        //BIN
        command_values[static_cast<int>(tfr_utilities::Joint::BIN)] = 0;
    }

    /*
     * Retrieves the state of the bin
     * */
    double RobotInterface::getBinState()
    {
        return position_values[static_cast<int>(tfr_utilities::Joint::BIN)];
    }

    /*
     * Retrieved the state of the arm
     * */
    void RobotInterface::getArmState(std::vector<double> &position)
    {
        position.push_back(position_values[static_cast<int>(tfr_utilities::Joint::TURNTABLE)]);
        position.push_back(position_values[static_cast<int>(tfr_utilities::Joint::LOWER_ARM)]);
        position.push_back(position_values[static_cast<int>(tfr_utilities::Joint::UPPER_ARM)]);
        position.push_back(position_values[static_cast<int>(tfr_utilities::Joint::SCOOP)]);
    }

    void RobotInterface::readTurntableEncoder(const std_msgs::Int32 &msg)
    {
        turntable_mutex.lock();
        
        turntable_encoder = msg.data;
        
        turntable_mutex.unlock();
    }
    
    void RobotInterface::readTurntableAmps(const std_msgs::Float64 &msg)
    {
        turntable_amps = msg.data;
    }

    void RobotInterface::readLowerArmEncoder(const sensor_msgs::JointState &msg)
    {
        lower_arm_mutex.lock();
        
        lower_arm_encoder = msg.position[0];
        //ROS_INFO_STREAM("lower_arm_encoder: " << lower_arm_encoder << std::endl);
        
        lower_arm_mutex.unlock();
    }
    
    // TODO: Add mutex to protect all of these arm readings from reading while writing.
    void RobotInterface::readLowerArmAmps(const std_msgs::Float64 &msg)
    {
        lower_arm_amps = msg.data;
        //position_values[static_cast<int>(tfr_utilities::Joint::LOWER_ARM)] = msg.data;
        //ROS_INFO_STREAM("lower_arm_ams: " << position_values[static_cast<int>(tfr_utilities::Joint::LOWER_ARM)] << std::endl);
    }
    
    
    void RobotInterface::readUpperArmEncoder(const sensor_msgs::JointState &msg)
    {
        upper_arm_mutex.lock();
        
        upper_arm_encoder = msg.position[0];
        //ROS_INFO_STREAM("upper_arm_encoder: " << upper_arm_encoder << std::endl);
        
        upper_arm_mutex.unlock();
    }
    
    void RobotInterface::readUpperArmAmps(const std_msgs::Float64 &msg)
    {
        upper_arm_amps = msg.data;
        //position_values[static_cast<int>(tfr_utilities::Joint::UPPER_ARM)] = msg.data;
        //ROS_INFO_STREAM("lower_arm_ams: " << position_values[static_cast<int>(tfr_utilities::Joint::UPPER_ARM)] << std::endl);
    }
    
    
    void RobotInterface::readScoopEncoder(const sensor_msgs::JointState &msg)
    {
        scoop_mutex.lock();
        
        scoop_encoder = msg.position[0];
        //ROS_INFO_STREAM("scoop_encoder: " << scoop_encoder << std::endl);
        
        scoop_mutex.unlock();
    }
    
    void RobotInterface::readScoopAmps(const std_msgs::Float64 &msg)
    {
        scoop_amps = msg.data;
        //position_values[static_cast<int>(tfr_utilities::Joint::LOWER_ARM)] = msg.data;
        //ROS_INFO_STREAM("lower_arm_ams: " << position_values[static_cast<int>(tfr_utilities::Joint::LOWER_ARM)] << std::endl);
    }

    /*
     * Register this joint with each neccessary hardware interface
     * */
    void RobotInterface::registerJointEffortInterface(std::string name, tfr_utilities::Joint joint) 
    {
        auto idx = static_cast<int>(joint);
        //give the joint a state
        JointStateHandle state_handle(name, &position_values[idx],
            &velocity_values[idx], &effort_values[idx]);
        joint_state_interface.registerHandle(state_handle);

        //allow the joint to be commanded
        JointHandle handle(state_handle, &command_values[idx]);
        joint_effort_interface.registerHandle(handle);
    }

    /*
     * Register this joint with each neccessary hardware interface
     * */
     /*
    void RobotInterface::registerBinJoint(std::string name, Joint joint) 
    {
        auto idx = static_cast<int>(joint);
        //give the joint a state
        JointStateHandle state_handle(name, &position_values[idx],
            &velocity_values[idx], &effort_values[idx]);
        joint_state_interface.registerHandle(state_handle);

        //allow the joint to be commanded
        JointHandle handle(state_handle, &command_values[idx]);
        joint_position_interface.registerHandle(handle);
    }
    */


    void RobotInterface::setBrushlessLeftEncoder(const std_msgs::Int32 &msg)
    {
        left_tread_absolute_encoder_current = msg.data;
        left_tread_time_current = ros::Time::now();
    }
    
    void RobotInterface::setBrushlessRightEncoder(const std_msgs::Int32 &msg)
    {
        right_tread_absolute_encoder_current = msg.data;
        right_tread_time_current = ros::Time::now();
    }

    /*
     * Register this joint with each neccessary hardware interface
     * */
    void RobotInterface::registerJointPositionInterface(std::string name, tfr_utilities::Joint joint) 
    {
        auto idx = static_cast<int>(joint);
        //give the joint a state
        JointStateHandle state_handle(name, &position_values[idx],
            &velocity_values[idx], &effort_values[idx]);
        joint_state_interface.registerHandle(state_handle);

        //allow the joint to be commanded
        JointHandle handle(state_handle, &command_values[idx]);
        joint_position_interface.registerHandle(handle);
    }

    // get the ratio of the encoder count to the max encoder count for a revolution
    double RobotInterface::brushlessEncoderCountToRadians(int32_t encoder_count)
    {
        return (2 * pi) * brushlessEncoderCountToRevolutions(encoder_count);
    }

    double RobotInterface::brushlessEncoderCountToRevolutions(int32_t encoder_count)
    {
        return (static_cast<double>(encoder_count) / static_cast<double>(brushless_encoder_count_per_revolution));
    }
   
    // returns the linear speed of the robot (how fast it is moving forwards) in meters / second.
    double RobotInterface::encoderDeltaToLinearSpeed(int32_t encoder_delta, ros::Duration time_delta)
    {
        const double wheel_radius_meters = 0.1524; 
        const double wheel_circumference = 2 * pi * wheel_radius_meters;
        
        const double revolutions = brushlessEncoderCountToRevolutions(encoder_delta);
        
        const double linear_speed_meters_per_sec = (wheel_circumference * revolutions) / time_delta.toSec();
        
        return linear_speed_meters_per_sec;
    }

    
    void RobotInterface::accumulateBrushlessRightVel(const std_msgs::Int32 &msg)
    {
        brushless_right_tread_mutex.lock();

        accumulated_brushless_right_tread_vel = msg.data;
        accumulated_brushless_right_tread_vel_num_updates++;
        accumulated_brushless_right_tread_vel_end_time = ros::Time::now(); // keep this call inside the mutex. The readBrushlessRightVel() call will also update it.

        brushless_right_tread_mutex.unlock();
        
    }
    
    void RobotInterface::accumulateBrushlessLeftVel(const std_msgs::Int32 &msg)
    {
        brushless_left_tread_mutex.lock();

        accumulated_brushless_left_tread_vel = msg.data;
        accumulated_brushless_left_tread_vel_num_updates++;
        accumulated_brushless_left_tread_vel_end_time = ros::Time::now();

        brushless_left_tread_mutex.unlock();
        
        
    }
    
    double RobotInterface::readBrushlessRightVel()
    {
        brushless_right_tread_mutex.lock();

        int32_t encoder_delta = right_tread_absolute_encoder_current - right_tread_absolute_encoder_previous;
        
        ros::Duration time_delta = right_tread_time_current - right_tread_time_previous;
        
        right_tread_absolute_encoder_previous = right_tread_absolute_encoder_current;
        right_tread_time_previous = right_tread_time_current;
        
        brushless_right_tread_mutex.unlock();
        
        const double linear_speed_meters_per_sec = encoderDeltaToLinearSpeed(encoder_delta, time_delta);
        
        return linear_speed_meters_per_sec;
    }
    
    double RobotInterface::readBrushlessLeftVel()
    {
        brushless_left_tread_mutex.lock();

        int32_t encoder_delta = left_tread_absolute_encoder_current - left_tread_absolute_encoder_previous;
        
        ros::Duration time_delta = left_tread_time_current - left_tread_time_previous;
        
        left_tread_absolute_encoder_previous = left_tread_absolute_encoder_current;
        left_tread_time_previous = left_tread_time_current;
        
        brushless_left_tread_mutex.unlock();
        
        const double linear_speed_meters_per_sec = encoderDeltaToLinearSpeed(encoder_delta, time_delta);
        
        return linear_speed_meters_per_sec;
    }
    
    
    void RobotInterface::zeroTurntable()
    {
        /*
        //Grab the neccessary data
        tfr_msgs::ArduinoAReading reading_a;
        if (latest_arduino_a != nullptr)
            reading_a = *latest_arduino_a;
        else 
            return;

        turntable_offset = -reading_a.arm_turntable_pos; 
        */
    }
    

}
