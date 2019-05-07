/**
 * controller.h
 * 
 * Provides utilites for the hardware layer of the ros control stack.
 * One of its jobs is to go from commanded values, to hardware control, 
 * with a few safety measures. 
 * 
 * It's other job is to put feedback in the appropriate data structures,
 * that are used by the controllers at the control layer to appropriately
 * command the rover
 *
 * It has an an enum of joints, and a class that are detailed below.
 */
#ifndef ROBOT_INTERFACE_H
#define ROBOT_INTERFACE_H

#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int32.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <utility>
#include <algorithm>
#include <tfr_msgs/ArduinoAReading.h>
#include <tfr_msgs/ArduinoBReading.h>
#include <tfr_msgs/PwmCommand.h>
#include <tfr_utilities/control_code.h>
#include <tfr_utilities/joints.h>
#include <vector>
#include <mutex>
#include <limits>

namespace tfr_control {

    /**
     * Contains the lower level interface inbetween user commands coming
     * in from the controller layer, and manages the state of all joints,
     * and sends commands to those joints
     * */
    class RobotInterface : public hardware_interface::RobotHW
    {
    public:

        RobotInterface(ros::NodeHandle &n, bool fakes, const double lower_lim[tfr_utilities::Joint::JOINT_COUNT],
                const double upper_lim[tfr_utilities::Joint::JOINT_COUNT]);
	
        
        /*
         * Reads state from hardware (encoders/potentiometers) and writes it to
         * shared memory 
         * */
        void read();
	
        /*
         * Takes commanded states from shared memory, enforces basic safety
         * contraints, and writes them to hardware
         * */
        void write();
	
	
        /*
         * retrieves the state of the bin
         * */
        double getBinState();
	
        /*
         * retrieves the state of the arm
         * */
        void getArmState(std::vector<double>&);
	
        /*
         * Clears all command values being sent and sets them to safe values
         * stops the treads and commands the arm to hold position.
         * */
        void clearCommands();
	
        void setEnabled(bool val);
	
        void zeroTurntable();
		
		

    private:
        //joint states for Joint state publisher package
        hardware_interface::JointStateInterface joint_state_interface;
        //cmd states for position driven joints
        hardware_interface::PositionJointInterface joint_position_interface;
        //cmd states for velocity driven joints
        hardware_interface::EffortJointInterface joint_effort_interface;

        //reads from arduino encoder publisher
        /*
		ros::Subscriber arduino_a;
        ros::Subscriber arduino_b;
        ros::Publisher pwm_publisher;
		*/
        bool enabled;
        
		/*
		tfr_msgs::ArduinoAReadingConstPtr latest_arduino_a;
        tfr_msgs::ArduinoBReadingConstPtr latest_arduino_b;
		*/
        double turntable_offset;

		// Read the relative velocity counters from the brushless motor controller
		ros::Subscriber brushless_right_tread_vel;
		ros::Subscriber brushless_left_tread_vel;
		
		volatile ros::Subscriber turntable_subscriber_encoder;
		volatile ros::Subscriber turntable_subscriber_amps;
		ros::Publisher  turntable_publisher;
		volatile int32_t turntable_encoder = 0;
		volatile double turntable_amps = 0.0;
		std::mutex turntable_mutex;
		
		volatile ros::Subscriber lower_arm_subscriber_encoder;
		volatile ros::Subscriber lower_arm_subscriber_amps;
		ros::Publisher  lower_arm_publisher;
		volatile int32_t lower_arm_encoder = 0;
		volatile double lower_arm_amps = 0.0;
		std::mutex lower_arm_mutex;
		
		volatile ros::Subscriber upper_arm_subscriber_encoder;
		volatile ros::Subscriber upper_arm_subscriber_amps;
		ros::Publisher  upper_arm_publisher;
		volatile int32_t upper_arm_encoder = 0;
		volatile double upper_arm_amps = 0.0;
		std::mutex upper_arm_mutex;
		
		volatile ros::Subscriber scoop_subscriber_encoder;
		volatile ros::Subscriber scoop_subscriber_amps;
		ros::Publisher  scoop_publisher;
		volatile int32_t scoop_encoder = 0;
		volatile double scoop_amps = 0.0;
		std::mutex scoop_mutex;
		
		void readTurntableEncoder(const std_msgs::Int32 &msg);
		void readTurntableAmps(const std_msgs::Float64 &msg);
		
		void readLowerArmEncoder(const std_msgs::Int32 &msg);
		void readLowerArmAmps(const std_msgs::Float64 &msg);
		
		void readUpperArmEncoder(const std_msgs::Int32 &msg);
		void readUpperArmAmps(const std_msgs::Float64 &msg);
		
		void readScoopEncoder(const std_msgs::Int32 &msg);
		void readScoopAmps(const std_msgs::Float64 &msg);
		
		ros::Publisher brushless_right_tread_vel_publisher;
		ros::Publisher brushless_left_tread_vel_publisher;
		
		
		std::mutex brushless_right_tread_mutex;
		int32_t accumulated_brushless_right_tread_vel = 0;
		int32_t accumulated_brushless_right_tread_vel_num_updates = 0;
		ros::Time accumulated_brushless_right_tread_vel_start_time;
		ros::Time accumulated_brushless_right_tread_vel_end_time;
		
		
		std::mutex brushless_left_tread_mutex;
		int32_t accumulated_brushless_left_tread_vel = 0;
		int32_t accumulated_brushless_left_tread_vel_num_updates = 0;
		ros::Time accumulated_brushless_left_tread_vel_start_time;
		ros::Time accumulated_brushless_left_tread_vel_end_time;
		
		
		void accumulateBrushlessRightVel(const std_msgs::Int32 &msg);
		void accumulateBrushlessLeftVel(const std_msgs::Int32 &msg);
		
		
		double readBrushlessRightVel();
		double readBrushlessLeftVel();
		
		const int32_t brushless_encoder_count_per_revolution = 1280;
		double brushlessEncoderCountToRadians(int32_t encoder_count);
		
		 int32_t bin_encoder_min = 0;
		 int32_t bin_encoder_max = 1000;
		 double bin_joint_min = 0.0;
		 double bin_joint_max = 0.0;
		
		 int32_t turntable_encoder_min = -25760;
		 int32_t turntable_encoder_max = 25760;
		 double turntable_joint_min = -2 * 3.14159265358979;
		 double turntable_joint_max = 2 * 3.14159265358979;
		
		
		/* 
			Arm all the way up (actutator extended):
				encoder: 0
				joint position: 0.1
			Arm all the way down (actuator retracted):
				encoder: 149
				joint position: 1.55
		*/
		 int32_t arm_lower_encoder_min = 0;
		 int32_t arm_lower_encoder_max = 888;
		 double arm_lower_joint_min = 0.104; // lower arm UP
		 double arm_lower_joint_max = 1.55;
		
		 int32_t arm_upper_encoder_min = 836; // arm UP
		 int32_t arm_upper_encoder_max = 0;
		 double arm_upper_joint_min = 0.98; // arm UP
		 double arm_upper_joint_max = 2.4; // arm DOWN, actuator EXTENDED
		
		 int32_t arm_end_encoder_min = 1721;
		 int32_t arm_end_encoder_max = 0;
		 double arm_end_joint_min = -1.16614; // scoop OPEN
		 double arm_end_joint_max = 1.62; // actuator EXTENDED, scoop CLOSED
		
		const int32_t get_arm_lower_min_int();
		const int32_t get_arm_lower_max_int();
		
		double  linear_interp_double(double x, double x1, double y1, double x2, double y2);
		int32_t linear_interp_int(int32_t x, int32_t x1, int32_t y1, int32_t x2, int32_t y2);

        // Populated by controller layer for us to use
        double command_values[tfr_utilities::Joint::JOINT_COUNT]{};

        // Populated by us for controller layer to use
        double position_values[tfr_utilities::Joint::JOINT_COUNT]{};
        // Populated by us for controller layer to use
        double velocity_values[tfr_utilities::Joint::JOINT_COUNT]{};
        // Populated by us for controller layer to use
        double effort_values[tfr_utilities::Joint::JOINT_COUNT]{};
        //used to limit acceleration pull on the drivebase
        std::pair<double, double> drivebase_v0;
        ros::Time last_update;

        
        void registerJointEffortInterface(std::string name, tfr_utilities::Joint joint);
        void registerJointPositionInterface(std::string name, tfr_utilities::Joint joint);
        //void registerBinJoint(std::string name, Joint joint);

		/*
        //callback for publisher
        void readArduinoA(const tfr_msgs::ArduinoAReadingConstPtr &msg);
        //callback for publisher
        void readArduinoB(const tfr_msgs::ArduinoBReadingConstPtr &msg);
		*/
	
	
        /**
         * Gets the PWM appropriate output for an angle joint at the current time
         * */
        //double angleToPWM(const double &desired, const double &measured);

        /**
         * Gets the PWM appropriate output for turntable at the current time
         * */
        //double turntableAngleToPWM(const double &desired, const double &measured);

        /**
         * Gets the PWM appropriate output for turntable at the current time
         * */
        /*
		std::pair<double, double> twinAngleToPWM(const double &desired, 
                const double &measured_left, const double &measured_right);
		*/

        /**
         * Gets the PWM appropriate output for a joint at the current time
         * */
        //double drivebaseVelocityToPWM(const double &v_1, const double &v_0);

        /*
         * Scale the PWM outputs to avoid browning out 
         * */
        //double scalePWM(const double &pwm_1, const double &pwm_0);

        void adjustFakeJoint(const tfr_utilities::Joint &joint);

        // THESE DATA MEMBERS ARE FOR SIMULATION ONLY
        // Holds the lower and upper limits of the URDF model joint
        bool use_fake_values = false;
        const double *lower_limits, *upper_limits;

    };
}

#endif // CONTROLLER_H
