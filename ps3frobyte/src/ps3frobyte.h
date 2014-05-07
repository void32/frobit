// %Tag(FULLTEXT)%
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <std_msgs/Bool.h>
#include "sensor_msgs/Joy.h"
#include <iostream>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>


class PS3frobyte {
private:
	ros::NodeHandle n;
	std_msgs::Bool deadman_button;
	double z_axis;
	double x_axis;
	geometry_msgs::TwistStamped twist;
	ros::Subscriber sub;
	ros::Publisher twist_pub_;
	ros::Publisher deadman_pub_;
	void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);
public:

	PS3frobyte();
	void updateDeadman();
	void updateVel();
};

