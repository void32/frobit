#include "ps3frobyte.h"

void PS3frobyte::joyCallback(const sensor_msgs::Joy::ConstPtr& joy){
	//Update the deadman button
	deadman_button.data = joy->buttons[11];
	//Update axies
	z_axis = joy->axes[0];
	x_axis = joy->axes[1];
	if(x_axis > 0.755){
		x_axis = 0.755;
	}

	if(x_axis < -0.755){
		x_axis = -0.755;
	}

	//Update left analog-thingy
	z_axis += joy->axes[3];
}

PS3frobyte::PS3frobyte(){
	deadman_button.data = false;
	z_axis = 0;
	x_axis = 0;

	sub = n.subscribe("/joy", 1000, &PS3frobyte::joyCallback, this);
	twist_pub_ = n.advertise<geometry_msgs::TwistStamped>("/fmSignals/cmd_vel", 1000);
	deadman_pub_ = n.advertise<std_msgs::Bool>("/fmSignals/deadman", 1000);
}

void PS3frobyte::updateDeadman(){
	//called every 0.1 sec to publish deadman
	deadman_pub_.publish(deadman_button);
}

void PS3frobyte::updateVel(){
	//called every 0.1 sec to publish cmd_vel
	twist.twist.angular.z = z_axis;
	twist.twist.linear.x = x_axis;

	std::cout << "x: " << x_axis << "\t";
	std::cout << "z: " << z_axis << "\n";
	twist_pub_.publish(twist);
}

