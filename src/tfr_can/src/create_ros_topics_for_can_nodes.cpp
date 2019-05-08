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

const double loop_rate = 10; // [Hz]

const int IMU_NODE_ID = 120;

template <class t>
t* setEntry<t>(const kaco::Device& device, const & std::string entry_name){
    if device.has_entry(entry_name){
        return std::make_shared<t>(device, "cmd_cango/cmd_cango_1");
    }
    ROS_INFO("CAN Setup Error: Device dict has no entry for %s", entry_name.c_str());
    return nullptr;
}

void setupDevice4Topics(kaco::Device& device, kaco::Bridge& bridge, std::string& eds_files_path){
    // Roboteq SDC3260 in Closed Loop Count Position mode.

	auto iosub_4_1_1 = std::make_shared<kaco::EntrySubscriber>(device, "cmd_cango/cmd_cango_1");
	bridge.add_subscriber(iosub_4_1_1);

	auto iopub_4_1_2 = std::make_shared<kaco::EntryPublisher>(device, "qry_motamps/channel_1");
	bridge.add_publisher(iopub_4_1_2, loop_rate);
	
	auto iopub_4_1_3 = std::make_shared<kaco::EntryPublisher>(device, "qry_abcntr/channel_1");
	bridge.add_publisher(iopub_4_1_3, loop_rate);
	
	auto iosub_4_1_4 = setEntry<kaco::EntrySubscriber>(device, "Cmd_SENCNTR/Counter 1");
	bridge.add_subscriber(iosub_4_1_4);
	
	
	auto iosub_4_2_1 = std::make_shared<kaco::EntrySubscriber>(device, "cmd_cango/cmd_cango_2");
	bridge.add_subscriber(iosub_4_2_1);

	auto iopub_4_2_2 = std::make_shared<kaco::EntryPublisher>(device, "qry_motamps/channel_2");
	bridge.add_publisher(iopub_4_2_2, loop_rate);
	
	auto iopub_4_2_3 = std::make_shared<kaco::EntryPublisher>(device, "qry_abcntr/channel_2");
	bridge.add_publisher(iopub_4_2_3, loop_rate);
	
	auto iosub_4_2_4 = std::make_shared<kaco::EntrySubscriber>(device, "cmd_sencntr/counter_2");
	bridge.add_subscriber(iosub_4_2_4);
	
	
	auto iosub_4_3_1 = std::make_shared<kaco::EntrySubscriber>(device, "cmd_cango/cmd_cango_3");
	bridge.add_subscriber(iosub_4_3_1);

	auto iopub_4_3_2 = std::make_shared<kaco::EntryPublisher>(device, "qry_motamps/channel_3");
	bridge.add_publisher(iopub_4_3_2, loop_rate);
	
	auto iopub_4_3_3 = std::make_shared<kaco::EntryPublisher>(device, "qry_abcntr/channel_3");
	bridge.add_publisher(iopub_4_3_3, loop_rate);
	
	auto iosub_4_3_4 = std::make_shared<kaco::EntrySubscriber>(device, "cmd_sencntr/counter_3");
	bridge.add_subscriber(iosub_4_3_4);
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
		
		device.load_dictionary_from_eds(eds_files_path + "roboteq_motor_controllers_v60.eds");
		
		int deviceId = device.get_node_id();

		if (deviceId == 4)
		{
			ROS_DEBUG_STREAM("tfr_can: case: Device 4" << std::endl);
			
			//device.print_dictionary();
			
			setupDevice4Topics(device, bridge, eds_files_path);
		}
		
		
		else if (deviceId == 8)
		{
			ROS_DEBUG_STREAM("tfr_can: case: Device 8" << std::endl);
			
			// Roboteq SBL2360 in Closed Loop Speed Position mode.

			auto iosub_8_1_1 = std::make_shared<kaco::EntrySubscriber>(device, "cmd_cango/cmd_cango_1");
    		bridge.add_subscriber(iosub_8_1_1);

			auto iopub_8_1_2 = std::make_shared<kaco::EntryPublisher>(device, "qry_relcntr/channel_1");
    		bridge.add_publisher(iopub_8_1_2, loop_rate);

			auto iopub_8_1_3 = std::make_shared<kaco::EntryPublisher>(device, "qry_motamps/channel_1");
    		bridge.add_publisher(iopub_8_1_3, loop_rate);
			
			auto iopub_8_1_4 = std::make_shared<kaco::EntryPublisher>(device, "qry_blrspeed/channel_1");
    		bridge.add_publisher(iopub_8_1_4, loop_rate);
			
			
			auto iosub_8_2_1 = std::make_shared<kaco::EntrySubscriber>(device, "cmd_cango/cmd_cango_2");
    		bridge.add_subscriber(iosub_8_2_1);

			auto iopub_8_2_2 = std::make_shared<kaco::EntryPublisher>(device, "qry_relcntr/channel_2");
    		bridge.add_publisher(iopub_8_2_2, loop_rate);

			auto iopub_8_2_3 = std::make_shared<kaco::EntryPublisher>(device, "qry_motamps/channel_2");
    		bridge.add_publisher(iopub_8_2_3, loop_rate);
			
			auto iopub_8_2_4 = std::make_shared<kaco::EntryPublisher>(device, "qry_blrspeed/channel_2");
    		bridge.add_publisher(iopub_8_2_4, loop_rate);
		}
		
		else if (deviceId == 12)
		{
			ROS_DEBUG_STREAM("tfr_can: case: Device 12" << std::endl);
			
			
			auto iosub_12_1_1 = std::make_shared<kaco::EntrySubscriber>(device, "cmd_cango/cmd_cango_1");
    		bridge.add_subscriber(iosub_12_1_1);

			auto iopub_12_1_2 = std::make_shared<kaco::EntryPublisher>(device, "qry_motamps/channel_1");
    		bridge.add_publisher(iopub_12_1_2, loop_rate);
			
			auto iopub_12_1_3 = std::make_shared<kaco::EntryPublisher>(device, "qry_abcntr/channel_1");
    		bridge.add_publisher(iopub_12_1_3, loop_rate);
			
			auto iopub_12_1_4 = std::make_shared<kaco::EntrySubscriber>(device, "cmd_sencntr/counter_1");
			bridge.add_subscriber(iopub_12_1_4);
			
			auto iopub_12_1_5 = setEntry<kaco::EntryPublisher>(device, "Qry_FEEDBACK/Qry_FEEDBACK 1");
			bridge.add_publisher(iopub_12_1_5);
			
			//left bin
			auto iosub_12_2_1 = std::make_shared<kaco::EntrySubscriber>(device, "cmd_cango/cmd_cango_2");
    		bridge.add_subscriber(iosub_12_2_1);

			auto iopub_12_2_2 = std::make_shared<kaco::EntryPublisher>(device, "qry_motamps/channel_2");
    		bridge.add_publisher(iopub_12_2_2, loop_rate);
			
			auto iopub_12_2_3 = std::make_shared<kaco::EntryPublisher>(device, "qry_abcntr/channel_2");
    		bridge.add_publisher(iopub_12_2_3, loop_rate);
			
			auto iopub_12_2_4 = std::make_shared<kaco::EntrySubscriber>(device, "cmd_sencntr/counter_2");
			bridge.add_subscriber(iopub_12_2_4);
			
			auto iopub_12_2_5 = setEntry<kaco::EntryPublisher>(device, "Qry_FEEDBACK/Qry_FEEDBACK 2");
			bridge.add_publisher(iopub_12_2_5);
			
		    //right bin
			auto iosub_12_3_1 = std::make_shared<kaco::EntrySubscriber>(device, "cmd_cango/cmd_cango_3");
    		bridge.add_subscriber(iosub_12_3_1);

			auto iopub_12_3_2 = std::make_shared<kaco::EntryPublisher>(device, "qry_motamps/channel_3");
    		bridge.add_publisher(iopub_12_3_2, loop_rate);
			
			auto iopub_12_3_3 = std::make_shared<kaco::EntryPublisher>(device, "qry_abcntr/channel_3");
    		bridge.add_publisher(iopub_12_3_3, loop_rate);
			
			auto iopub_12_3_4 = setEntry<kaco::EntrySubscriber>(device, "cmd_sencntr/counter_3");
			bridge.add_subscriber(iopub_12_3_4);
			
			auto iopub_12_2_5 = setEntry<kaco::EntryPublisher>(device, "Qry_FEEDBACK/Qry_FEEDBACK 3");
			bridge.add_publisher(iopub_12_3_5);
			
			
		}
		
		else if (deviceId == IMU_NODE_ID)
		{
			auto iosub_120_1 = std::make_shared<kaco::EntrySubscriber>(device, "cmd_cango/cmd_cango_1");
    		bridge.add_subscriber(iosub_120_1);
		}
	}
	PRINT("About to call bridge.run()");
	bridge.run();
	
    master.stop();
    
	return EXIT_SUCCESS;
}
