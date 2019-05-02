
/** * controller.cpp * 
 * This class is in charge of handling the physical hardware interface with
 * the robot itself, and is started by the controller_launcher node.
 */
#include "robot_interface.h"

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


		brushless_left_tread_vel{n.subscribe("/device8/get_qry_relcntr/channel_2", 5,
                &RobotInterface::accumulateBrushlessLeftVel, this)},
		brushless_left_tread_vel_publisher{n.advertise<std_msgs::Int32>("/device8/set_cmd_cango/cmd_cango_2", 1)},
		
		
		brushless_right_tread_vel{n.subscribe("/device8/get_qry_relcntr/channel_1", 5,
                &RobotInterface::accumulateBrushlessRightVel, this)},
		brushless_right_tread_vel_publisher{n.advertise<std_msgs::Int32>("/device8/set_cmd_cango/cmd_cango_1", 1)},
		
		
		turntable_subscriber_encoder{n.subscribe("/device4/get_qry_abcntr/channel_1", 5,
                &RobotInterface::readTurntableEncoder, this)},
		turntable_subscriber_amps{n.subscribe("/device4/get_qry_batamps/channel_1", 1,
                &RobotInterface::readTurntableAmps, this)},
		turntable_publisher{n.advertise<std_msgs::Int32>("/device4/set_cmd_cango/cmd_cango_1", 1)},
		
		
		lower_arm_subscriber_encoder{n.subscribe("/device12/get_qry_abcntr/channel_1", 5,
                &RobotInterface::readLowerArmEncoder, this)},
		lower_arm_subscriber_amps{n.subscribe("/device12/get_qry_batamps/channel_1", 1,
                &RobotInterface::readLowerArmAmps, this)},
		lower_arm_publisher{n.advertise<std_msgs::Int32>("/device12/set_cmd_cango/cmd_cango_1", 1)},
		
		
		upper_arm_subscriber_encoder{n.subscribe("/device4/get_qry_abcntr/channel_3", 5,
                &RobotInterface::readUpperArmEncoder, this)},
		upper_arm_subscriber_amps{n.subscribe("/device4/get_qry_batamps/channel_3", 1,
                &RobotInterface::readUpperArmAmps, this)},
		upper_arm_publisher{n.advertise<std_msgs::Int32>("/device4/set_cmd_cango/cmd_cango_3", 1)},
		
		
		scoop_subscriber_encoder{n.subscribe("/device4/get_qry_abcntr/channel_2", 5,
                &RobotInterface::readScoopEncoder, this)},
		scoop_subscriber_amps{n.subscribe("/device4/get_qry_batamps/channel_2", 1,
                &RobotInterface::readScoopAmps, this)},
		scoop_publisher{n.advertise<std_msgs::Int32>("/device4/set_cmd_cango/cmd_cango_2", 1)},
		
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

        // Connect and register each joint with appropriate interfaces at our
        // layer
        registerJointEffortInterface("left_tread_joint", Joint::LEFT_TREAD);
        registerJointEffortInterface("right_tread_joint", Joint::RIGHT_TREAD);
        registerJointEffortInterface("bin_joint", Joint::BIN); 
        registerJointEffortInterface("turntable_joint", Joint::TURNTABLE);
        registerJointEffortInterface("lower_arm_joint", Joint::LOWER_ARM);
        registerJointEffortInterface("upper_arm_joint", Joint::UPPER_ARM);
        registerJointEffortInterface("scoop_joint", Joint::SCOOP);
        //register the interfaces with the controller layer
        registerInterface(&joint_state_interface);
        registerInterface(&joint_effort_interface);
        registerInterface(&joint_position_interface);
		
		
        for (int joint = 0; joint < JOINT_COUNT; joint++)
		{
			velocity_values[static_cast<int>(Joint::RIGHT_TREAD)] = 0;
			effort_values[static_cast<int>(Joint::RIGHT_TREAD)] = 0;
		}
		
    }


    /*
     * Reads from our hardware and populates from shared memory.  
     *
     * Information that is not explicity needed by our controllers 
     * is written to some safe sensible default (usually 0).
     *
     * A couple of our logical joints are controlled by two actuators and read
     * by multiple potentiometers. For the purpose of populating information for
     * control I take the average of the two positions.
     * */
    void RobotInterface::read() 
    {
        //Grab the neccessary data
		/*
        tfr_msgs::ArduinoAReading reading_a;
        tfr_msgs::ArduinoBReading reading_b;
        if (latest_arduino_a != nullptr)
            reading_a = *latest_arduino_a;
        if (latest_arduino_b != nullptr)
            reading_b = *latest_arduino_b;
		*/

        //LEFT_TREAD
        position_values[static_cast<int>(Joint::LEFT_TREAD)] = 0;
        velocity_values[static_cast<int>(Joint::LEFT_TREAD)] = readBrushlessRightVel();
        effort_values[static_cast<int>(Joint::LEFT_TREAD)] = 0;

        //RIGHT_TREAD
        position_values[static_cast<int>(Joint::RIGHT_TREAD)] = 0;
        velocity_values[static_cast<int>(Joint::RIGHT_TREAD)] = readBrushlessLeftVel();
        effort_values[static_cast<int>(Joint::RIGHT_TREAD)] = 0;

        if (!use_fake_values)
        {
            //TURNTABLE
            double turntable_position_double = 
	            linear_interp_double(
		            static_cast<double>(turntable_encoder),
		            static_cast<double>(turntable_encoder_min),
		            turntable_joint_min,
		            static_cast<double>(turntable_encoder_max),
		            turntable_joint_max
		            );
			
			position_values[static_cast<int>(Joint::TURNTABLE)] = turntable_position_double;
                //reading_a.arm_turntable_pos + turntable_offset;
            velocity_values[static_cast<int>(Joint::TURNTABLE)] = 0; 
            effort_values[static_cast<int>(Joint::TURNTABLE)] = 0;

            //LOWER_ARM
            //position_values[static_cast<int>(Joint::LOWER_ARM)] = reading_a.arm_lower_pos;
            double lower_arm_position_double = 
	            linear_interp_double(
		            static_cast<double>(lower_arm_encoder),
		            static_cast<double>(arm_lower_encoder_min),
		            arm_lower_joint_min,
		            static_cast<double>(arm_lower_encoder_max),
		            arm_lower_joint_max
		            );
		
            double upper_arm_position_double = 
	            linear_interp_double(
		            static_cast<double>(upper_arm_encoder),
		            static_cast<double>(arm_upper_encoder_min),
		            arm_upper_joint_min,
		            static_cast<double>(arm_upper_encoder_max),
		            arm_upper_joint_max
		            );

            double scoop_position_double = 
	            linear_interp_double(
		            static_cast<double>(scoop_encoder),
		            static_cast<double>(arm_end_encoder_min),
		            arm_end_joint_min,
		            static_cast<double>(arm_end_encoder_max),
		            arm_end_joint_max
		            );

            position_values[static_cast<int>(Joint::LOWER_ARM)] = lower_arm_position_double;
            velocity_values[static_cast<int>(Joint::LOWER_ARM)] = 0;
            effort_values[static_cast<int>(Joint::LOWER_ARM)] = 0;

            //UPPER_ARM
            position_values[static_cast<int>(Joint::UPPER_ARM)] = upper_arm_position_double; //reading_a.arm_upper_pos;
            velocity_values[static_cast<int>(Joint::UPPER_ARM)] = 0;
            effort_values[static_cast<int>(Joint::UPPER_ARM)] = 0;

            //SCOOP
            position_values[static_cast<int>(Joint::SCOOP)] = scoop_position_double; //reading_a.arm_scoop_pos;
            velocity_values[static_cast<int>(Joint::SCOOP)] = 0;
            effort_values[static_cast<int>(Joint::SCOOP)] = 0;

			ROS_INFO_STREAM("turntable_position: read: encoder: " << turntable_encoder << std::endl);
			ROS_INFO_STREAM("turntable_position: read: " << position_values[static_cast<int>(Joint::TURNTABLE)] << std::endl);
			
			/*
            ROS_INFO_STREAM("arm_lower_position: read: encoder: " << lower_arm_encoder << std::endl);
			ROS_INFO_STREAM("arm_lower_position: read: " << position_values[static_cast<int>(Joint::LOWER_ARM)] << std::endl);
			
			ROS_INFO_STREAM("arm_upper_position: read: encoder: " << upper_arm_encoder << std::endl);
            ROS_INFO_STREAM("arm_upper_position: read: " << position_values[static_cast<int>(Joint::UPPER_ARM)] << std::endl);

			ROS_INFO_STREAM("scoop_position: read: encoder: " << scoop_encoder << std::endl);
            ROS_INFO_STREAM("scoop_position: read: " << position_values[static_cast<int>(Joint::SCOOP)] << std::endl);
			*/
        }
 
        //BIN
        position_values[static_cast<int>(Joint::BIN)] = 0; 
            //(reading_a.bin_left_pos + reading_a.bin_right_pos)/2;
        velocity_values[static_cast<int>(Joint::BIN)] = 0;
        effort_values[static_cast<int>(Joint::BIN)] = 0;

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
        //Grab the neccessary data
        //tfr_msgs::ArduinoAReading reading_a;
        //tfr_msgs::ArduinoBReading reading_b;

        //package for outgoing data
        //tfr_msgs::PwmCommand command;
        /*
		if (latest_arduino_a != nullptr)
            reading_a = *latest_arduino_a;
		*/

        double signal;
        if (use_fake_values) //test code  for working with rviz simulator
        {
            adjustFakeJoint(Joint::TURNTABLE);
            adjustFakeJoint(Joint::LOWER_ARM);
            adjustFakeJoint(Joint::UPPER_ARM);
            adjustFakeJoint(Joint::SCOOP);
        }
        else  // we are working with the real arm
        {
            //TURNTABLE
			int32_t turntable_position = 
				static_cast<int32_t>
			    (
					std::max(
					std::min(
			        linear_interp_double
			        (
			            command_values[static_cast<int>(Joint::TURNTABLE)],
		                0,
		                0,
		                1,
						-1 // This value of -1 is really important. Otherwise the turntable may accelerate when it ought to decelerate and vice versa.
		            ), 1000.0), -1000.0)
		        );
			std_msgs::Int32 turntable_position_msg;
			turntable_position_msg.data = turntable_position;
			turntable_publisher.publish(turntable_position_msg);


            //LOWER_ARM
            //NOTE we reverse these because actuator is mounted backwards
			int32_t arm_lower_position = // command_values[static_cast<int>(Joint::LOWER_ARM)];
			static_cast<int32_t>
                            (
								std::max(
								std::min(
                                linear_interp_double
                                (
                                    command_values[static_cast<int>(Joint::LOWER_ARM)],
                                0,
                                0,
                                1,
                                -1
								), 1000.0), -1000.0)
							);
			std_msgs::Int32 arm_lower_position_msg;
			arm_lower_position_msg.data = arm_lower_position;
			//lower_arm_publisher.publish(arm_lower_position_msg);
			

            //UPPER_ARM
			int32_t arm_upper_position = // command_values[static_cast<int>(Joint::UPPER_ARM)];
			
				static_cast<int32_t>
			    (
					std::max(
					std::min(
			        linear_interp_double
			        (
			            command_values[static_cast<int>(Joint::UPPER_ARM)],
		                0,
		                0,
		                1,
						-1
		            ), 1000.0), -1000.0)
		        );

			std_msgs::Int32 arm_upper_position_msg;
			arm_upper_position_msg.data = arm_upper_position;
			//upper_arm_publisher.publish(arm_upper_position_msg);
			

            //SCOOP
			int32_t scoop_position = // static_cast<int32_t>(command_values[static_cast<int>(Joint::SCOOP)]); //command_values[static_cast<int>(Joint::SCOOP)];
			    
			    static_cast<int32_t>
			    (
					std::max(
					std::min(
			        linear_interp_double
			        (
			            command_values[static_cast<int>(Joint::SCOOP)],
		                0,
		                0,
		                1,
						-1
		            ), 1000.0), -1000.0)
		        );
				
		        
			        
			std_msgs::Int32 scoop_position_msg;
			scoop_position_msg.data = scoop_position;
			//scoop_publisher.publish(scoop_position_msg);
			
			
			ROS_INFO_STREAM("turntable_position: position: write: " << position_values[static_cast<int>(Joint::TURNTABLE)] << std::endl);
			ROS_INFO_STREAM("turntable_position: command: write: " << command_values[static_cast<int>(Joint::TURNTABLE)] << std::endl);
			ROS_INFO_STREAM("turntable_position: effort: write: " << effort_values[static_cast<int>(Joint::TURNTABLE)] << std::endl);
			ROS_INFO_STREAM("turntable_position: velocity: write: " << velocity_values[static_cast<int>(Joint::TURNTABLE)] << std::endl);
			ROS_INFO_STREAM("turntable_position: write: " << turntable_position << std::endl);
			
			
			/*
			ROS_INFO_STREAM("arm_lower_position: position: write: " << position_values[static_cast<int>(Joint::LOWER_ARM)] << std::endl);
			ROS_INFO_STREAM("arm_lower_position: command: write: " << command_values[static_cast<int>(Joint::LOWER_ARM)] << std::endl);
			ROS_INFO_STREAM("arm_lower_position: effort: write: " << effort_values[static_cast<int>(Joint::LOWER_ARM)] << std::endl);
			ROS_INFO_STREAM("arm_lower_position: velocity: write: " << velocity_values[static_cast<int>(Joint::LOWER_ARM)] << std::endl);
			ROS_INFO_STREAM("arm_lower_position: write: " << arm_lower_position << std::endl);
			*/
			
			/*
			ROS_INFO_STREAM("arm_upper_position: position: write: " << position_values[static_cast<int>(Joint::UPPER_ARM)] << std::endl);
			ROS_INFO_STREAM("arm_upper_position: command: write: " << command_values[static_cast<int>(Joint::UPPER_ARM)] << std::endl);
			ROS_INFO_STREAM("arm_upper_position: effort: write: " << effort_values[static_cast<int>(Joint::UPPER_ARM)] << std::endl);
			ROS_INFO_STREAM("arm_upper_position: velocity: write: " << velocity_values[static_cast<int>(Joint::UPPER_ARM)] << std::endl);
			ROS_INFO_STREAM("arm_upper_position: write: " << arm_upper_position << std::endl);
			*/
			
			/*
			ROS_INFO_STREAM("scoop_position: position: write: " << position_values[static_cast<int>(Joint::SCOOP)] << std::endl);
			ROS_INFO_STREAM("scoop_position: command: write: " << command_values[static_cast<int>(Joint::SCOOP)] << std::endl);
			ROS_INFO_STREAM("scoop_position: effort: write: " << effort_values[static_cast<int>(Joint::SCOOP)] << std::endl);
			ROS_INFO_STREAM("scoop_position: velocity: write: " << velocity_values[static_cast<int>(Joint::SCOOP)] << std::endl);
			ROS_INFO_STREAM("scoop_position: write: " << scoop_position << std::endl);
			*/
        }
		
        //LEFT_TREAD
	int left_tread_scale = 1;
	ros::param::getCached("left_tread_scale", left_tread_scale);
	//ROS_INFO("Left tread scale %d", left_tread_scale);
        double left_tread_command = command_values[static_cast<int32_t>(Joint::LEFT_TREAD)];
		//left_tread_command = linear_interp_double(left_tread_command, 0.0, 0.0, 1.0, 1000.0);
		std_msgs::Int32 left_tread_msg;
		left_tread_msg.data = static_cast<int32_t>(left_tread_command * left_tread_scale);
        //brushless_left_tread_vel_publisher.publish(left_tread_msg);

        //RIGHT_TREAD
	int right_tread_scale = 1;
	ros::param::getCached("right_tread_scale", right_tread_scale);
        double right_tread_command = command_values[static_cast<int32_t>(Joint::RIGHT_TREAD)];
		//right_tread_command = linear_interp_double(right_tread_command, 0.0, 0.0, 1.0, 1000.0);
		std_msgs::Int32 right_tread_msg;
		right_tread_msg.data = static_cast<int32_t>(right_tread_command * right_tread_scale);
        //brushless_right_tread_vel_publisher.publish(right_tread_msg);

		//ROS_INFO_STREAM("left_tread_scale: " << left_tread_msg.data << std::endl);
		//ROS_INFO_STREAM("right_tread_scale: " << right_tread_msg.data << std::endl);

        //BIN
		/*
        auto twin_signal = twinAngleToPWM(command_values[static_cast<int>(Joint::BIN)], 0, 0);
					//reading_a.bin_left_pos,
                    //reading_a.bin_right_pos);
					
        command.bin_left = twin_signal.first;
        command.bin_right = twin_signal.second;
		*/
		

        //command.enabled = enabled;
        //pwm_publisher.publish(command);
        
        //UPKEEP
        last_update = ros::Time::now();
        drivebase_v0.first = velocity_values[static_cast<int>(Joint::LEFT_TREAD)];
        drivebase_v0.second = velocity_values[static_cast<int>(Joint::RIGHT_TREAD)];
    }

	double RobotInterface::linear_interp_double(double x, double x1, double y1, double x2, double y2)
	{
		// line defined by two points: (x1, y1) and (x2, y2)
        double y = ((y2 - y1)/(x2 - x1))*(x - x1) + y1;
		return y;
	}
	
	int32_t RobotInterface::linear_interp_int(int32_t x, int32_t x1, int32_t y1, int32_t x2, int32_t y2)
	{
		// line defined by two points: (x1, y1) and (x2, y2)
        int32_t y = ((y2 - y1)/(x2 - x1))*(x - x1) + y1;
		return y;
	}
	
	// TODO: Horrible duplication of code should be removed.
	// has hardcoded min joint position in header file
	const int32_t RobotInterface::get_arm_lower_min_int()
	{
		double arm_lower_joint_min_mapped = linear_interp_double(arm_lower_joint_min * -10,
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
		double arm_lower_joint_max_mapped = linear_interp_double(arm_lower_joint_max * 10,
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

    void RobotInterface::adjustFakeJoint(const Joint &j)
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
        command_values[static_cast<int>(Joint::LEFT_TREAD)] = 0;

        //RIGHT_TREAD
        command_values[static_cast<int>(Joint::RIGHT_TREAD)] = 0;

        //TURNTABLE
        command_values[static_cast<int>(Joint::TURNTABLE)] = 0;

        //LOWER_ARM
        command_values[static_cast<int>(Joint::LOWER_ARM)] = 0;
		
        //UPPER_ARM
        command_values[static_cast<int>(Joint::UPPER_ARM)] = 0;
        //SCOOP
        command_values[static_cast<int>(Joint::SCOOP)] = 0;
        //BIN
        command_values[static_cast<int>(Joint::BIN)] = 0;
    }

    /*
     * Retrieves the state of the bin
     * */
    double RobotInterface::getBinState()
    {
        return position_values[static_cast<int>(Joint::BIN)];
    }

    /*
     * Retrieved the state of the arm
     * */
    void RobotInterface::getArmState(std::vector<double> &position)
    {
        position.push_back(position_values[static_cast<int>(Joint::TURNTABLE)]);
        position.push_back(position_values[static_cast<int>(Joint::LOWER_ARM)]);
        position.push_back(position_values[static_cast<int>(Joint::UPPER_ARM)]);
        position.push_back(position_values[static_cast<int>(Joint::SCOOP)]);
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

	void RobotInterface::readLowerArmEncoder(const std_msgs::Int32 &msg)
	{
		lower_arm_mutex.lock();
		
		lower_arm_encoder = msg.data;
		//ROS_INFO_STREAM("lower_arm_encoder: " << lower_arm_encoder << std::endl);
		
		lower_arm_mutex.unlock();
	}
	
	// TODO: Add mutex to protect all of these arm readings from reading while writing.
	void RobotInterface::readLowerArmAmps(const std_msgs::Float64 &msg)
	{
		lower_arm_amps = msg.data;
		//position_values[static_cast<int>(Joint::LOWER_ARM)] = msg.data;
		//ROS_INFO_STREAM("lower_arm_ams: " << position_values[static_cast<int>(Joint::LOWER_ARM)] << std::endl);
	}
	
	
	void RobotInterface::readUpperArmEncoder(const std_msgs::Int32 &msg)
	{
		upper_arm_mutex.lock();
		
		upper_arm_encoder = msg.data;
		//ROS_INFO_STREAM("upper_arm_encoder: " << upper_arm_encoder << std::endl);
		
		upper_arm_mutex.unlock();
	}
	
	void RobotInterface::readUpperArmAmps(const std_msgs::Float64 &msg)
	{
		upper_arm_amps = msg.data;
		//position_values[static_cast<int>(Joint::UPPER_ARM)] = msg.data;
		//ROS_INFO_STREAM("lower_arm_ams: " << position_values[static_cast<int>(Joint::UPPER_ARM)] << std::endl);
	}
	
	
	void RobotInterface::readScoopEncoder(const std_msgs::Int32 &msg)
	{
		scoop_mutex.lock();
		
		scoop_encoder = msg.data;
		//ROS_INFO_STREAM("scoop_encoder: " << scoop_encoder << std::endl);
		
		scoop_mutex.unlock();
	}
	
	void RobotInterface::readScoopAmps(const std_msgs::Float64 &msg)
	{
		scoop_amps = msg.data;
		//position_values[static_cast<int>(Joint::LOWER_ARM)] = msg.data;
		//ROS_INFO_STREAM("lower_arm_ams: " << position_values[static_cast<int>(Joint::LOWER_ARM)] << std::endl);
	}

    /*
     * Register this joint with each neccessary hardware interface
     * */
    void RobotInterface::registerJointEffortInterface(std::string name, Joint joint) 
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


    /*
     * Register this joint with each neccessary hardware interface
     * */
    void RobotInterface::registerJointPositionInterface(std::string name, Joint joint) 
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
		//static const double pi = boost::math::constants::pi<double>();
		static const double pi = 3.14159265358979;
		return (static_cast<double>(encoder_count) * (2 * pi)) / static_cast<double>(brushless_encoder_count_per_revolution);
	}

    /*
     * Input is angle desired/measured and output is in raw pwm frequency.
     * */
	/*
    double RobotInterface::angleToPWM(const double &desired, const double &actual)
    {
        //we don't anticipate this changing very much keep at method level
        double min_delta = 0.01;
        double max_delta = 0.35;

        double difference = desired - actual;
        if (std::abs(difference) > min_delta)
        {

            int sign = (difference < 0) ? -1 : 1;
            double magnitude = std::min(std::abs(difference)/max_delta, 0.8);           
            return sign*magnitude;
        }
        return 0;
    }
	*/

    /*
     * Input is angle desired/measured of a twin acutuator joint and output is
     * in raw pwm frequency for both of them. The actuator further ahead get's
     * scaled down.
     * */
	/*
    std::pair<double,double> RobotInterface::twinAngleToPWM(const double &desired, 
            const double &actual_left, const double &actual_right)
    {
        //we don't anticipate these changing very much keep at method level
        double  total_angle_tolerance = 0.005,
                individual_angle_tolerance = 0.01,
                scaling_factor = .6, 
                difference = desired - (actual_left + actual_right)/2;
        if (std::abs(difference) > total_angle_tolerance)
        {
            int direction = (difference < 0) ? 1 : -1;
            double delta = actual_left - actual_right;
            double cmd_left = direction, cmd_right = direction;
            if (std::abs(delta) > individual_angle_tolerance)
            {
                if (direction < 0)
                {
                    //extending
                    if (actual_left > actual_right)
                        cmd_left *= scaling_factor;
                    else
                        cmd_right *= scaling_factor;
                }
                else
                {
                    //retracting
                    if (actual_left > actual_right)
                        cmd_right *= scaling_factor;
                    else
                        cmd_left *= scaling_factor;
                }
            }
            return std::make_pair(cmd_left, cmd_right);
        }
        return std::make_pair(0,0);
    }
    */
	
    /*
     * Input is angle desired/measured of turntable and output is in raw pwm frequency.
     * */
	/*
    double RobotInterface::turntableAngleToPWM(const double &desired, const double &actual)
    {
        //we don't anticipate this changing very much keep at method level
        double min_delta = 0.01;
        double max_delta = 0.2;
        double difference = desired - actual;
        if (std::abs(difference) > min_delta)
        {
            int sign = (difference < 0) ? 1 : -1;
            double magnitude = std::min(std::abs(difference)/max_delta, 0.92);           
            return sign*magnitude;
        }
        return 0;
    }
	*/

    /*
     * Takes in a velocity, and converts it to pwm for the drivebase.
     * Velocity is in meters per second, and output is in raw pwm frequency.
     * Scaled to match the values expected by pwm interface
     * NOTE we have a safety limit here of 1 m/s^2 any more and it will snap a
     * shaft
     * */
	 /*
    double RobotInterface::drivebaseVelocityToPWM(const double& v_1, const double& v_0)
    {
        //limit for max velocity
        //we don't anticipate this changing very much keep at method level
        double max_vel = 0.5;
        if (v_1 > 0.05 || v_1 < -0.05)
        {
            int sign = (v_1 < 0) ? -1 : 1;
            //double magnitude = std::min(std::abs(v_1)/max_vel, 0.9);
            double magnitude = 1;
            return sign * magnitude;
        }
        return 0;
    }
	*/

    /*
     * Prevents large pwm changes to avoid brown out
     * */
	/*
    double RobotInterface::scalePWM(const double& pwm_1, const double& pwm_0)
    {
        double sign = ((pwm_1 - pwm_0) > 0) ? 1 : -1;
        double magnitude = std::min(std::abs(pwm_1-pwm_0), 0.01);
        return pwm_0 + sign * magnitude;
    }
	*/

	
    /*
     * Callback for our encoder subscriber
     * */
	/*
    void RobotInterface::readArduinoA(const tfr_msgs::ArduinoAReadingConstPtr &msg)
    {
        latest_arduino_a = msg;
    }
	*/
	
    /*
     * Callback for our encoder subscriber
     * */
	/*
    void RobotInterface::readArduinoB(const tfr_msgs::ArduinoBReadingConstPtr &msg)
    {
        latest_arduino_b = msg;
    }
	*/

    //This DOES work
	void RobotInterface::accumulateBrushlessRightVel(const std_msgs::Int32 &msg)
	{
		brushless_right_tread_mutex.lock();

		accumulated_brushless_right_tread_vel += msg.data;
		accumulated_brushless_right_tread_vel_num_updates++;
		accumulated_brushless_right_tread_vel_end_time = ros::Time::now(); // keep this call inside the mutex. The readBrushlessRightVel() call will also update it.

		brushless_right_tread_mutex.unlock();
		
		//ROS_INFO_STREAM("accumulateBrushlessRightVel: " << accumulated_brushless_right_tread_vel << std::endl);
		
		
	}
	
	//This DOES work
	void RobotInterface::accumulateBrushlessLeftVel(const std_msgs::Int32 &msg)
	{
		brushless_left_tread_mutex.lock();

		accumulated_brushless_left_tread_vel += msg.data;
		accumulated_brushless_left_tread_vel_num_updates++;
		accumulated_brushless_left_tread_vel_end_time = ros::Time::now();

		brushless_left_tread_mutex.unlock();
		
		//ROS_INFO_STREAM("accumulateBrushlessLeftVel: " << accumulated_brushless_left_tread_vel << std::endl);
		
	}
	
	double RobotInterface::readBrushlessRightVel()
	{
		brushless_right_tread_mutex.lock();

		int32_t encoder_count = accumulated_brushless_right_tread_vel;
		int32_t num_updates = accumulated_brushless_right_tread_vel_num_updates;

		accumulated_brushless_right_tread_vel = 0;
		accumulated_brushless_right_tread_vel_num_updates = 0;
		accumulated_brushless_right_tread_vel_start_time = ros::Time::now();
		accumulated_brushless_right_tread_vel_end_time = ros::Time::now();

		brushless_right_tread_mutex.unlock();
		
		return brushlessEncoderCountToRadians(encoder_count) / accumulated_brushless_right_tread_vel_end_time.toSec();
	}
	
	double RobotInterface::readBrushlessLeftVel()
	{
		brushless_left_tread_mutex.lock();

		int32_t encoder_count = accumulated_brushless_left_tread_vel;
		int32_t num_updates = accumulated_brushless_left_tread_vel_num_updates;

		accumulated_brushless_left_tread_vel = 0;
		accumulated_brushless_left_tread_vel_num_updates = 0;
		accumulated_brushless_right_tread_vel_start_time = ros::Time::now();
		accumulated_brushless_right_tread_vel_end_time = ros::Time::now();

		brushless_left_tread_mutex.unlock();
		
		return brushlessEncoderCountToRadians(encoder_count) / accumulated_brushless_left_tread_vel_end_time.toSec();
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
