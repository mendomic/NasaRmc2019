#include "master.h"
#include "bridge.h"
#include "logger.h"
#include "joint_state_publisher.h"
#include "joint_state_subscriber.h"
#include "entry_publisher.h"
#include "entry_subscriber.h"

#include <thread>
#include <chrono>
#include <memory>

//#include <ros/package.h> // for looking up the location of the current package, in order to find our EDS files.
//#include <ros>

// Set the name of your CAN bus. "slcan0" is a common bus name
// for the first SocketCAN device on a Linux system.
const std::string busname = "can1";

// Set the baudrate of your CAN bus. Most drivers support the values
// "1M", "500K", "125K", "100K", "50K", "20K", "10K" and "5K".
const std::string baudrate = "250K";

const size_t num_devices_required = 1;

const double loop_rate = 100; // [Hz]
const int slow_loop_rate = 1; // 1 Hz

// CANopen node IDs:
const int IMU_NODE_ID = 120;
const int SERVO_CYLINDER_LOWER_ARM = 23;
const int SERVO_CYLINDER_SPARE 	= 34;
const int SERVO_CYLINDER_UPPER_ARM = 45;
const int SERVO_CYLINDER_SCOOP = 56;


void setupDevice4Topics(kaco::Device& device, kaco::Bridge& bridge, std::string& eds_files_path){
    // Roboteq SDC3260 in Closed Loop Count Position mode.

	auto iosub_4_1_1 = std::make_shared<kaco::EntrySubscriber>(device, "cmd_cango/cmd_cango_1");
	bridge.add_subscriber(iosub_4_1_1);

	auto iopub_4_1_2 = std::make_shared<kaco::EntryPublisher>(device, "qry_motamps/channel_1");
	bridge.add_publisher(iopub_4_1_2, loop_rate);
	
	auto iopub_4_1_3 = std::make_shared<kaco::EntryPublisher>(device, "qry_abcntr/channel_1");
	bridge.add_publisher(iopub_4_1_3, loop_rate);
	
	auto iopub_4_1_4 = std::make_shared<kaco::EntrySubscriber>(device, "cmd_sencntr/counter_1");
	bridge.add_subscriber(iopub_4_1_4);
	
	
	auto iosub_4_2_1 = std::make_shared<kaco::EntrySubscriber>(device, "cmd_cango/cmd_cango_2");
	bridge.add_subscriber(iosub_4_2_1);

	auto iopub_4_2_2 = std::make_shared<kaco::EntryPublisher>(device, "qry_motamps/channel_2");
	bridge.add_publisher(iopub_4_2_2, loop_rate);
	
	auto iopub_4_2_3 = std::make_shared<kaco::EntryPublisher>(device, "qry_abcntr/channel_2");
	bridge.add_publisher(iopub_4_2_3, loop_rate);
	
	auto iopub_4_2_4 = std::make_shared<kaco::EntrySubscriber>(device, "cmd_sencntr/counter_2");
	bridge.add_subscriber(iopub_4_2_4);
	
	
	auto iosub_4_3_1 = std::make_shared<kaco::EntrySubscriber>(device, "cmd_cango/cmd_cango_3");
	bridge.add_subscriber(iosub_4_3_1);

	auto iopub_4_3_2 = std::make_shared<kaco::EntryPublisher>(device, "qry_motamps/channel_3");
	bridge.add_publisher(iopub_4_3_2, loop_rate);
	
	auto iopub_4_3_3 = std::make_shared<kaco::EntryPublisher>(device, "qry_abcntr/channel_3");
	bridge.add_publisher(iopub_4_3_3, loop_rate);
	
	auto iopub_4_3_4 = std::make_shared<kaco::EntrySubscriber>(device, "cmd_sencntr/counter_3");
	bridge.add_subscriber(iopub_4_3_4);
}


//todo
void setupDevice8Topics(kaco::Device& device, kaco::Bridge& bridge, std::string& eds_files_path){
}

//todo
void setupDevice12Topics(kaco::Device& device, kaco::Bridge& bridge, std::string& eds_files_path){
}

//todo
void setupLpmsImuDevice(kaco::Device& device, kaco::Bridge& bridge, std::string& eds_files_path){

// initialize the topics for any Servo Cylinder actuator (must be 5.75" stroke length)
void setupServoCylinderDevice(kaco::Device& device, kaco::Bridge& bridge, std::string& eds_files_path)
{
    
    //device.load_dictionary_from_library();
    
    device.load_dictionary_from_eds(eds_files_path + "SC_MC630R11_v_0_7_OD.eds");
    
    PRINT("Set position mode");
    device.set_entry("modes_of_operation", device.get_constant("profile_position_mode"));

    PRINT("Enable operation");
    device.execute("enable_operation");


	// min: 0 -> 0, 
	// max: 47104 -> 6.28==2pi
    auto jspub = std::make_shared<kaco::JointStatePublisher>(device, 0, 47104); 
    bridge.add_publisher(jspub, loop_rate);
    
    auto jssub = std::make_shared<kaco::JointStateSubscriber>(device, 0, 47104);
    bridge.add_subscriber(jssub);
    
    // read the current torque value
    auto iopub_1 = std::make_shared<kaco::EntryPublisher>(device, "torque_actual_value");
    bridge.add_publisher(iopub_1, loop_rate);
    
    auto iosub_1 = std::make_shared<kaco::EntrySubscriber>(device, "torque_actual_value");
    bridge.add_subscriber(iosub_1);
    
    // read/write the max allowed torque value
    auto iopub_2 = std::make_shared<kaco::EntryPublisher>(device, "max_torque");
    bridge.add_publisher(iopub_2, slow_loop_rate);
    
    auto iosub_2 = std::make_shared<kaco::EntrySubscriber>(device, "max_torque");
    bridge.add_subscriber(iosub_2);
    
    
    // read the current velocity value
    auto iopub_3 = std::make_shared<kaco::EntryPublisher>(device, "velocity_actual_value");
    bridge.add_publisher(iopub_3, loop_rate);
    
    // read/write max speed
    auto iopub_4 = std::make_shared<kaco::EntryPublisher>(device, "profile_velocity");
    bridge.add_publisher(iopub_4, slow_loop_rate);
    
    auto iosub_4 = std::make_shared<kaco::EntrySubscriber>(device, "profile_velocity");
    bridge.add_subscriber(iosub_4);
    
    
    

    
    
    // read/write heartbeat time interval in milliseconds
    auto iopub_5 = std::make_shared<kaco::EntryPublisher>(device, "producer_heartbeat_time");
    bridge.add_publisher(iopub_5, slow_loop_rate);
    
    auto iosub_5 = std::make_shared<kaco::EntrySubscriber>(device, "producer_heartbeat_time");
    bridge.add_subscriber(iosub_5);
    
    
    // profile_acceleration
    auto iopub_6 = std::make_shared<kaco::EntryPublisher>(device, "profile_acceleration");
    bridge.add_publisher(iopub_6, slow_loop_rate);
    
    auto iosub_6 = std::make_shared<kaco::EntrySubscriber>(device, "profile_acceleration");
    bridge.add_subscriber(iosub_6);
    
    
    // profile_deceleration
    auto iopub_7 = std::make_shared<kaco::EntryPublisher>(device, "profile_deceleration");
    bridge.add_publisher(iopub_7, slow_loop_rate);
    
    auto iosub_7 = std::make_shared<kaco::EntrySubscriber>(device, "profile_deceleration");
    bridge.add_subscriber(iosub_7);
}



int main(int argc, char* argv[]) {

	
	kaco::Master master;
	if (!master.start(busname, baudrate)) {
		ERROR("Starting master failed.");
		return EXIT_FAILURE;
	}

	std::this_thread::sleep_for(std::chrono::seconds(1));
	
	while (master.num_devices()<num_devices_required) {
		ERROR("Number of devices found: " << master.num_devices() << ". Waiting for " << num_devices_required << ".");
		//PRINT("Trying to discover more nodes via NMT Node Guarding...");
		//master.core.nmt.discover_nodes();
		std::this_thread::sleep_for(std::chrono::seconds(2));
	}

	// Create bridge
	ros::init(argc, argv, "canopen_bridge");
	kaco::Bridge bridge;

	for (size_t i=0; i<master.num_devices(); ++i) {

		kaco::Device& device = master.get_device(i);
		device.start();

		PRINT("Found device with node ID "<<device.get_node_id()<<": "<<device.get_entry("manufacturer_device_name"));
		std::string eds_files_path;
		if (ros::param::getCached("~eds_files_path", eds_files_path)) {
				PRINT("Great it worked.");
				PRINT(eds_files_path);
		} else {
		    ERROR("tfr_can could not find the private parameter 'eds_files_path'. Make sure this parameter is getting set in the launch file for tfr_can.");
		}
		
		int deviceId = device.get_node_id();

        if (deviceId == SERVO_CYLINDER_LOWER_ARM)
        {
            setupServoCylinderDevice(device, bridge, eds_files_path);
        }
		
		if (deviceId == SERVO_CYLINDER_SPARE)
        {
            setupServoCylinderDevice(device, bridge, eds_files_path);
        }
		
		if (deviceId == SERVO_CYLINDER_UPPER_ARM)
        {
            setupServoCylinderDevice(device, bridge, eds_files_path);
        }
		
		if (deviceId == SERVO_CYLINDER_SCOOP)
        {
            setupServoCylinderDevice(device, bridge, eds_files_path);
        }

		if (deviceId == 4)
		{
			device.load_dictionary_from_eds(eds_files_path + "roboteq_motor_controllers_v60.eds");
			
			ROS_DEBUG_STREAM("tfr_can: case: Device 4" << std::endl);
			
			//device.print_dictionary();
			
			setupDevice4Topics(device, bridge, eds_files_path);
		}
		
		
		else if (deviceId == 8)
		{
			device.load_dictionary_from_eds(eds_files_path + "roboteq_motor_controllers_v60.eds");
			
			ROS_DEBUG_STREAM("tfr_can: case: Device 8" << std::endl);
			
			// Roboteq SBL2360.

			auto iosub_8_1_1 = std::make_shared<kaco::EntrySubscriber>(device, "cmd_cango/cmd_cango_1");
    		bridge.add_subscriber(iosub_8_1_1);

			//auto iopub_8_1_2 = std::make_shared<kaco::EntryPublisher>(device, "qry_relcntr/channel_1");
    		//bridge.add_publisher(iopub_8_1_2, loop_rate);

			auto iopub_8_1_3 = std::make_shared<kaco::EntryPublisher>(device, "qry_motamps/channel_1");
    		bridge.add_publisher(iopub_8_1_3, loop_rate);
			
			//auto iopub_8_1_4 = std::make_shared<kaco::EntryPublisher>(device, "qry_blrspeed/channel_1");
    		//bridge.add_publisher(iopub_8_1_4, loop_rate);
    		
    		auto iopub_8_1_5 = std::make_shared<kaco::EntryPublisher>(device, "qry_abcntr/channel_1");
    		bridge.add_publisher(iopub_8_1_5, loop_rate);
			
			
			auto iosub_8_2_1 = std::make_shared<kaco::EntrySubscriber>(device, "cmd_cango/cmd_cango_2");
    		bridge.add_subscriber(iosub_8_2_1);

			//auto iopub_8_2_2 = std::make_shared<kaco::EntryPublisher>(device, "qry_relcntr/channel_2");
    		//bridge.add_publisher(iopub_8_2_2, loop_rate);

			auto iopub_8_2_3 = std::make_shared<kaco::EntryPublisher>(device, "qry_motamps/channel_2");
    		bridge.add_publisher(iopub_8_2_3, loop_rate);
			
			//auto iopub_8_2_4 = std::make_shared<kaco::EntryPublisher>(device, "qry_blrspeed/channel_2");
    		//bridge.add_publisher(iopub_8_2_4, loop_rate);
    		
    		auto iopub_8_2_5 = std::make_shared<kaco::EntryPublisher>(device, "qry_abcntr/channel_2");
    		bridge.add_publisher(iopub_8_2_5, loop_rate);
			
		}
		
		else if (deviceId == 12)
		{
			device.load_dictionary_from_eds(eds_files_path + "roboteq_motor_controllers_v60.eds");
			
			ROS_DEBUG_STREAM("tfr_can: case: Device 12" << std::endl);
			
			
			auto iosub_12_1_1 = std::make_shared<kaco::EntrySubscriber>(device, "cmd_cango/cmd_cango_1");
    		bridge.add_subscriber(iosub_12_1_1);

			auto iopub_12_1_2 = std::make_shared<kaco::EntryPublisher>(device, "qry_motamps/channel_1");
    		bridge.add_publisher(iopub_12_1_2, loop_rate);
			
			auto iopub_12_1_3 = std::make_shared<kaco::EntryPublisher>(device, "qry_abcntr/channel_1");
    		bridge.add_publisher(iopub_12_1_3, loop_rate);
			
			auto iopub_12_1_4 = std::make_shared<kaco::EntrySubscriber>(device, "cmd_sencntr/counter_1");
			bridge.add_subscriber(iopub_12_1_4);
			
			
			auto iosub_12_2_1 = std::make_shared<kaco::EntrySubscriber>(device, "cmd_cango/cmd_cango_2");
    		bridge.add_subscriber(iosub_12_2_1);

			auto iopub_12_2_2 = std::make_shared<kaco::EntryPublisher>(device, "qry_motamps/channel_2");
    		bridge.add_publisher(iopub_12_2_2, loop_rate);
			
			auto iopub_12_2_3 = std::make_shared<kaco::EntryPublisher>(device, "qry_abcntr/channel_2");
    		bridge.add_publisher(iopub_12_2_3, loop_rate);
			
			auto iopub_12_2_4 = std::make_shared<kaco::EntrySubscriber>(device, "cmd_sencntr/counter_2");
			bridge.add_subscriber(iopub_12_2_4);
			
			
			auto iosub_12_3_1 = std::make_shared<kaco::EntrySubscriber>(device, "cmd_cango/cmd_cango_3");
    		bridge.add_subscriber(iosub_12_3_1);

			auto iopub_12_3_2 = std::make_shared<kaco::EntryPublisher>(device, "qry_motamps/channel_3");
    		bridge.add_publisher(iopub_12_3_2, loop_rate);
			
			auto iopub_12_3_3 = std::make_shared<kaco::EntryPublisher>(device, "qry_abcntr/channel_3");
    		bridge.add_publisher(iopub_12_3_3, loop_rate);
			
			auto iopub_12_3_4 = std::make_shared<kaco::EntrySubscriber>(device, "cmd_sencntr/counter_3");
			bridge.add_subscriber(iopub_12_3_4);
			
		}
		
		else if (deviceId == IMU_NODE_ID)
		{	
			// Remember, we are not allowed to use the magnetometer. It is also disabled in the IMU's control settings, so no messages for that are being published on the can bus.
	
			device.load_dictionary_from_eds(eds_files_path + "LPMS-CU2_32BitDataSettings.eds");
	
			auto iopub_120_1 = std::make_shared<kaco::EntryPublisher>(device, "gyroscope_x");
    		bridge.add_publisher(iopub_120_1, loop_rate);
			
			auto iopub_120_2 = std::make_shared<kaco::EntryPublisher>(device, "gyroscope_y");
    		bridge.add_publisher(iopub_120_2, loop_rate);
			
			auto iopub_120_3 = std::make_shared<kaco::EntryPublisher>(device, "gyroscope_z");
    		bridge.add_publisher(iopub_120_3, loop_rate);
			
			auto iopub_120_4 = std::make_shared<kaco::EntryPublisher>(device, "euler_x");
    		bridge.add_publisher(iopub_120_4, loop_rate);
			
			auto iopub_120_5 = std::make_shared<kaco::EntryPublisher>(device, "euler_y");
    		bridge.add_publisher(iopub_120_5, loop_rate);
			
			auto iopub_120_6 = std::make_shared<kaco::EntryPublisher>(device, "euler_z");
    		bridge.add_publisher(iopub_120_6, loop_rate);
			
			auto iopub_120_7 = std::make_shared<kaco::EntryPublisher>(device, "linear_acceleration_x");
    		bridge.add_publisher(iopub_120_7, loop_rate);
			
			auto iopub_120_8 = std::make_shared<kaco::EntryPublisher>(device, "linear_acceleration_y");
    		bridge.add_publisher(iopub_120_8, loop_rate);
			
			auto iopub_120_9 = std::make_shared<kaco::EntryPublisher>(device, "linear_acceleration_z");
    		bridge.add_publisher(iopub_120_9, loop_rate);
			
			auto iopub_120_10 = std::make_shared<kaco::EntryPublisher>(device, "quaternion_w");
    		bridge.add_publisher(iopub_120_10, loop_rate);
			
			auto iopub_120_11 = std::make_shared<kaco::EntryPublisher>(device, "quaternion_x");
    		bridge.add_publisher(iopub_120_11, loop_rate);
			
			auto iopub_120_12 = std::make_shared<kaco::EntryPublisher>(device, "quaternion_y");
    		bridge.add_publisher(iopub_120_12, loop_rate);
			
			auto iopub_120_13 = std::make_shared<kaco::EntryPublisher>(device, "quaternion_z");
    		bridge.add_publisher(iopub_120_13, loop_rate);
			
			
		}
	}
	PRINT("About to call bridge.run()");
	bridge.run();
	
    master.stop();
    
	return EXIT_SUCCESS;
}
