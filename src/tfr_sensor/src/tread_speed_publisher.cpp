#include <ros/ros.h>
#include "std_msgs/UInt32.h"
#include "std_msgs/Float64.h"

class TreadSpeed {
public:
    double speed;

    TreadSpeed(const unsigned int ticksPerRevolution, const unsigned int maxTicks, const double wheelRadius, const unsigned int prevTickCount = 0) :
        speed{ 0 }, prevTickCount{ prevTickCount }, ticksPerRevolution{ ticksPerRevolution }, maxTicks{ maxTicks }, wheelRadius{ wheelRadius } {}

    void updateFromNewCount(const unsigned int newCount) {
        auto ticksMoved = calcTickDiff(newCount);
        speed = (wheelRadius * ticksMoved) / ticksPerRevolution;
        prevTickCount = newCount;
    }
    
private:

    unsigned int calcTickDiff(const unsigned int newCount) {
        // handle rollover from going past maxTicks
        if (prevTickCount > newCount) {
            auto ticksToMax = maxTicks - prevTickCount;
            return ticksToMax + newCount;
        }
        // otherwise its a simple difference
        return newCount - prevTickCount;
    }

    unsigned int prevTickCount; // last recorded position of wheel
    const unsigned int ticksPerRevolution; // number of ticks counted each revolution of the measured wheel
    const unsigned int maxTicks; // number of ticks counted before rolling over back to 0
    const double wheelRadius // radius of wheel (for which ticks are being counted) in meters
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "tread_speed_publisher");
    ros::NodeHandle n; //NodeHandle is the main access point to communications with the ROS system.

    bool requiredParamsFound = true;
    double wheelRadius, ticksPerRevolution, maxTicks;
    double rate; //rate: how quickly to publish hz.
    requiredParamsFound = requiredParamsFound && ros::param::getParam<double>("~wheelRadius", wheelRadius);
    requiredParamsFound = requiredParamsFound && ros::param::getParam<double>("~ticksPerRevolution", ticksPerRevolution);
    requiredParamsFound = requiredParamsFound && ros::param::getParam<double>("~maxTicks", maxTicks);
    if (!requiredParamsFound) {//throw an error or something...}
    
    ros::Publisher leftTreadPublisher = n.advertise<std_msgs::Float64>("/left_tread_speed", 15);
    ros::Publisher rightTreadPublisher = n.advertise<std_msgs::Float64>("/right_tread_speed", 15);
    TreadSpeed leftTread(ticksPerRevolution, maxTicks, wheelRadius), rightTread(ticksPerRevolution, maxTicks, wheelRadius);
    auto leftTreadCallback = [&leftTread, &leftTreadPublisher](const std_msgs::UInt32& msg) {
        std_msgs::Float64 new_msg;
        leftTread.updateFromNewCount(msg.data);
        new_msg.data = leftTread.speed;
        leftTreadPublisher.publish(new_msg);

    };
    auto rightTreadCallback = [&rightTread, &rightTreadPublisher](const std_msgs::UInt32& msg) {
        std_msgs::Float64 new_msg;
        rightTread.updateFromNewCount(msg.data);
        new_msg.data = rightTread.speed;
        rightTreadPublisher.publish(new_msg);

    };
    auto leftTreadCountSub = n.subscribe("/left_tread_count", 10, leftTreadCallback);
    auto rightTreadCountSub = n.subscribe("/right_tread_count", 10, rightTreadCallback);
    
    ros::param::param<double>("~rate", rate, 10.0);
    ros::Rate rate(rate);
    while (ros::ok()) {
        ros::spinOnce();
        rate.sleep();

    }
    return 0;
}