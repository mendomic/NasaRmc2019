#include <ros/ros.h>
#include <ros/console.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
class JoyTeleop {
  public:
    JoyTeleop() {
      geometry_msgs::Twist move_cmd{};
      joy_sub = nh.subscribe<sensor_msgs::Joy>("joy", 10, &JoyTeleop::joyCallback, this);
      vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
    };
  private:
    int linear_idx = 1;
    int angular_idx = 0;
    void joyCallback(const sensor_msgs::Joy::ConstPtr& joy) {
      geometry_msgs::Twist twist;
      twist.angular.z = joy->axes[angular_idx];
      twist.linear.x = joy->axes[linear_idx];
      vel_pub.publish(twist);
    };
    ros::Subscriber joy_sub;
    ros::Publisher vel_pub;
    ros::NodeHandle nh;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "joy_teleop");
  JoyTeleop joyTeleop;
  ros::spin();
  return 0;
}
