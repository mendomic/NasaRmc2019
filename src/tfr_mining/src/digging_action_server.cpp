/****************************************************************************************
 * File:    digging_action_server.cpp
 * Node:    digging_server
 * 
 * Purpose: This is an action server that handles all of the digging subsystem.
 * 
 *          This file includes <tfr_msgs/DiggingAction.h>, which is one of seven headers
 *          built by catkin from `tfr_msgs/action/Digging.action`:
 *              devel/include/tfr_msgs/DiggingAction.h
 *              devel/include/tfr_msgs/DiggingActionFeedback.h
 *              devel/include/tfr_msgs/DiggingActionGoal.h
 *              devel/include/tfr_msgs/DiggingActionResult.h
 *              devel/include/tfr_msgs/DiggingFeedback.h
 *              devel/include/tfr_msgs/DiggingGoal.h
 *              devel/include/tfr_msgs/DiggingResult.h
 * 
 ***************************************************************************************/

#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>  
#include <tfr_msgs/DiggingAction.h>  // Note: "Action" is appended
#include <tfr_msgs/ArmMoveAction.h>  // Note: "Action" is appended
#include <tfr_utilities/arm_manipulator.h>
#include <geometry_msgs/Twist.h>
#include <tfr_utilities/teleop_code.h>
#include <actionlib/client/simple_action_client.h>
#include "digging_queue.h"

typedef actionlib::SimpleActionServer<tfr_msgs::DiggingAction> Server;
typedef actionlib::SimpleActionClient<tfr_msgs::ArmMoveAction> Client;

class DiggingActionServer {
public:
    DiggingActionServer(ros::NodeHandle &nh, ros::NodeHandle &p_nh) :
        priv_nh{p_nh}, queue{priv_nh}, 
        drivebase_publisher{nh.advertise<geometry_msgs::Twist>("cmd_vel", 5)},
        server{nh, "dig", boost::bind(&DiggingActionServer::execute, this, _1),
            false},
        arm_manipulator{nh}

    {
        server.start();
    }

private:

	/*
	 * Description:
	 * Send the robot the number of seconds it is allowed to spend on digging. It will start digging and loop through predefined digging motions repeatedly. It will continue digging for up to (but not exceeding) the provided amount of time. Finally, the robot will move the digging arm back to a safe state.
	 * If the current digging goal is cancelled, or if ROS is shutting down, the arm will be moved back into a safe state.
	 * 
	 * Pre: There must be accurate measurements of the position of the arm. (TODO: Where are these published to?) Digging can start while the arm is turned off-center, but ROS must be aware of this. In other words, as long as the position of the arm is not miscalibrated, it's ok.
	 * 
	 * Post: The arm will be in a centered, safe state.
	 * 
	 * Notes: ROS actionlib seemed to have a bug, and it would take as much as 10 seconds to cancel a digging goal.
	 * 
	 */
    void execute(const tfr_msgs::DiggingGoalConstPtr& goal)
    {
        ROS_INFO("Executing!");
        ROS_INFO("Allowed %d seconds to dig.", goal->diggingTime.sec);
        ros::Time startTime = ros::Time::now();
        ros::Time endTime = startTime + goal->diggingTime;
        ROS_INFO("Starting at %d, ending at %d", startTime.sec, endTime.sec);
		
        Client client("move_arm", true);
        ROS_DEBUG("Waiting for arm action server...");
        client.waitForServer();
        ROS_DEBUG("Connected with arm action server");
        
        std::queue<tfr_mining::DiggingSet> current_queue{queue.sets};

        while (!current_queue.empty())
        {
            ROS_INFO("Time remaining: %f", (endTime - ros::Time::now()).toSec());
            tfr_mining::DiggingSet set = current_queue.front();
            current_queue.pop();
            ros::Time now = ros::Time::now();

            ROS_INFO("starting set");
            
            std::queue<std::vector<double> > current_set{set.states};

            while (!current_set.empty())
            {
                std::vector<double> state = current_set.front();
                current_set.pop();
                
                // Use arm_manipulator, and NOT MoveIt, to send commands to the arm. The actuators will just move to each of the points in the digging queue, there is no trajectory or other points being generated. There is also no collision checking, so be careful.
                ROS_INFO("Moving arm to position: %.2f %.2f %.2f %.2f", state[0], state[1], state[2], state[3]);
                arm_manipulator.moveArmWithoutPlanningOrLimits(state[0], state[1], state[2], state[3]);
                ros::Duration(0.5).sleep();

                ros::Rate rate(10.0);

                if (server.isPreemptRequested() || !ros::ok())
                {
                    ROS_INFO("Preempting digging action server");
                    client.cancelAllGoals();
                    tfr_msgs::DiggingResult result;
                    server.setPreempted(result);
                    return;
                }

                rate.sleep();
            }
        }

        tfr_msgs::DiggingResult result;
        server.setSucceeded(result);
    }


    ros::NodeHandle &priv_nh;
    ros::Publisher drivebase_publisher;
 
    ArmManipulator arm_manipulator;
    tfr_mining::DiggingQueue queue;
    Server server;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "digging_server");
    ros::NodeHandle n;
    ros::NodeHandle p_n("~");

    DiggingActionServer server(n, p_n);
    ros::spin();
    return 0;
}
