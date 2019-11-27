#include <ros/ros.h>
#include <boost/function.hpp>
#include "std_msgs/UInt32.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Float64.h"

class TreadSpeed {
public:
    double speed;

    TreadSpeed(const int ticksPerRevolution, const int maxTicks, const double wheelRadius, const int prevTickCount = 0) :
        speed{ 0 }, prevTickCount{ prevTickCount }, ticksPerRevolution{ ticksPerRevolution }, maxTicks{ maxTicks }, wheelRadius{ wheelRadius } {}

    void updateFromNewCount(const int newCount) {
        auto ticksMoved = calcTickDiff(newCount);
        speed = (wheelRadius * ticksMoved) / ticksPerRevolution;
        prevTickCount = newCount;
    }
    
private:

    int calcTickDiff(const int newCount) {
        // handle rollover from going past maxTicks
        if (prevTickCount > newCount) {
            auto ticksToMax = maxTicks - prevTickCount;
            return ticksToMax + newCount;
        }
        // otherwise its a simple difference
        return newCount - prevTickCount;
    }

    int prevTickCount; // last recorded position of wheel
    const int ticksPerRevolution; // number of ticks counted each revolution of the measured wheel
    const int maxTicks; // number of ticks counted before rolling over back to 0
    const double wheelRadius; // radius of wheel (for which ticks are being counted) in meters
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "tread_speed_publisher");
    ros::NodeHandle n; //NodeHandle is the main access point to communications with the ROS system.

    bool requiredParamsFound = true;
    double wheelRadius, ticksPerRevolution, maxTicks;
    double rate; //rate: how quickly to publish hz.
    requiredParamsFound = requiredParamsFound && ros::param::get("~wheelRadius", wheelRadius);
    requiredParamsFound = requiredParamsFound && ros::param::get("~ticksPerRevolution", ticksPerRevolution);
    requiredParamsFound = requiredParamsFound && ros::param::get("~maxTicks", maxTicks);
    //if (!requiredParamsFound) {//throw an error or something...}
    
    ros::Publisher leftTreadPublisher = n.advertise<std_msgs::Float64>("/left_tread_speed", 15);
    ros::Publisher rightTreadPublisher = n.advertise<std_msgs::Float64>("/right_tread_speed", 15);
    TreadSpeed leftTread(ticksPerRevolution, maxTicks, wheelRadius), rightTread(ticksPerRevolution, maxTicks, wheelRadius);
    boost::function<void(const std_msgs::Int32&)> leftTreadCallback = [&leftTread, &leftTreadPublisher](const std_msgs::Int32& msg) {
        std_msgs::Float64 new_msg;
        leftTread.updateFromNewCount(msg.data);
        new_msg.data = leftTread.speed;
        leftTreadPublisher.publish(new_msg);

    };
    boost::function<void(const std_msgs::Int32&)> rightTreadCallback = [&rightTread, &rightTreadPublisher](const std_msgs::Int32& msg) {
        std_msgs::Float64 new_msg;
        rightTread.updateFromNewCount(msg.data);
        new_msg.data = rightTread.speed;
        rightTreadPublisher.publish(new_msg);

    };
    auto leftTreadCountSub = n.subscribe<std_msgs::Int32>("/left_tread_count", 10, leftTreadCallback);
    auto rightTreadCountSub = n.subscribe<std_msgs::Int32>("/right_tread_count", 10, rightTreadCallback);
    
    ros::param::param<double>("~rate", rate, 10.0);
    ros::Rate loop_rate(rate);
    while (ros::ok()) {
        ros::spinOnce();
        loop_rate.sleep();

    }
    return 0;
}
