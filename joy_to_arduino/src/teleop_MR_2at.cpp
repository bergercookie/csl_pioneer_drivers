#include <ros/ros.h>
#include <arduino_feedback/arduino_input.h>
#include <sensor_msgs/Joy.h>

//float l_scale=1.1, a_scale=3, l_scale_w=2.2, scale_PWM=255;
float l_scale=0.77, a_scale=1.5, l_scale_w=2.2, scale_PWM=255;
int current_mode=0;

arduino_feedback::arduino_input message;

ros::Publisher vel_pub;
ros::Subscriber joy_sub;

void joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
	if (current_mode==0){
		message.mode=1;
		current_mode=1;
	}
	else if (joy->buttons[0]==1) {
		message.mode=1; current_mode=1;
	}
	else if (joy->buttons[1]==1) {
		message.mode=2; current_mode=2;
	}
	else if (joy->buttons[2]==1) {
		message.mode=3; current_mode=3;
	}

	if (current_mode == 1)
	{
		message.data1 = l_scale * joy->axes[1];
		message.data2 = a_scale * joy->axes[3];
	}
	else if (current_mode == 2)
	{
		message.data1 = l_scale_w * joy->axes[4];
		message.data2 = l_scale_w * joy->axes[1];
	}
	else
	{
		message.data1 = scale_PWM * joy->axes[4];
		message.data2 = scale_PWM * joy->axes[1];
	}
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "teleop_MR");
	ros::NodeHandle nh;

	vel_pub = nh.advertise<arduino_feedback::arduino_input>("arduino_input_2at",10);
	joy_sub = nh.subscribe<sensor_msgs::Joy>("joy", 10, &joyCallback);

	ros::Rate rate(20);
	while (ros::ok()) {
		vel_pub.publish(message);
		ros::spinOnce();
		rate.sleep();
	}
}
