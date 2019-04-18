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
#include <vector>
#include <mutex>
#include <limits>

namespace tfr_control {

    /*
     * All of the joints on the robot
     * */
    enum class Joint 
    {
        LEFT_TREAD, 
        RIGHT_TREAD, 
        BIN, 
        TURNTABLE, 
        LOWER_ARM,
        UPPER_ARM, 
        SCOOP 
    };

    /**
     * Contains the lower level interface inbetween user commands coming
     * in from the controller layer, and manages the state of all joints,
     * and sends commands to those joints
     * */
    class RobotInterface : public hardware_interface::RobotHW
    {
    public:
        
        //Number of joints we need to control in our layer
        static const int JOINT_COUNT = 7;


        RobotInterface(ros::NodeHandle &n, bool fakes, const double lower_lim[JOINT_COUNT],
                const double upper_lim[JOINT_COUNT]);
	
        
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
		ros::Subscriber brushless_a_vel;
		ros::Subscriber brushless_b_vel;
		ros::Subscriber device4_3_subscriber_encoder;
		ros::Subscriber device4_3_subscriber_command;
		
		ros::Publisher brushless_a_vel_publisher;
		ros::Publisher brushless_b_vel_publisher;
		
		ros::Publisher device4_3_publisher;
		
		std::mutex brushless_a_mutex;
		int32_t accumulated_brushless_a_vel = 0;
		int32_t accumulated_brushless_a_vel_num_updates = 0;
		std::mutex brushless_b_mutex;
		int32_t accumulated_brushless_b_vel = 0;
		int32_t accumulated_brushless_b_vel_num_updates = 0;
		
		void accumulateBrushlessAVel(const std_msgs::Int32 &msg);
		void accumulateBrushlessBVel(const std_msgs::Int32 &msg);
		
		int32_t device4_encoder = 0;
		void readDevice4Encoder(const std_msgs::Int32 &msg);
		void readDevice4Command(const std_msgs::Float64 &msg);
		
		int32_t readBrushlessAVel();
		int32_t readBrushlessBVel();
		
		const int32_t arm_lower_encoder_min = 0;
		const int32_t arm_lower_encoder_max = 0;
		const double arm_lower_joint_min = 0.104;
		const double arm_lower_joint_max = 1.55;
		
		const int32_t arm_upper_encoder_min = 0;
		const int32_t arm_upper_encoder_max = 0;
		const double arm_upper_joint_min = 0.0;
		const double arm_upper_joint_max = 0.0;
		
		const int32_t arm_end_encoder_min = 0;
		const int32_t arm_end_encoder_max = 0;
		const double arm_end_joint_min = 0.0;
		const double arm_end_joint_max = 0.0;
		
		const int32_t get_arm_lower_min_int();
		const int32_t get_arm_lower_max_int();
		
		double  linear_interp_double(double x, double x1, double y1, double x2, double y2);
		int32_t linear_interp_int(int32_t x, int32_t x1, int32_t y1, int32_t x2, int32_t y2);

        // Populated by controller layer for us to use
        double command_values[JOINT_COUNT]{};

        // Populated by us for controller layer to use
        double position_values[JOINT_COUNT]{};
        // Populated by us for controller layer to use
        double velocity_values[JOINT_COUNT]{};
        // Populated by us for controller layer to use
        double effort_values[JOINT_COUNT]{};
        //used to limit acceleration pull on the drivebase
        std::pair<double, double> drivebase_v0;
        ros::Time last_update;

        
        void registerJointEffortInterface(std::string name, Joint joint);
        void registerJointPositionIterface(std::string name, Joint joint);
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

        void adjustFakeJoint(const Joint &joint);

        // THESE DATA MEMBERS ARE FOR SIMULATION ONLY
        // Holds the lower and upper limits of the URDF model joint
        bool use_fake_values = false;
        const double *lower_limits, *upper_limits;

    };
}

#endif // CONTROLLER_H