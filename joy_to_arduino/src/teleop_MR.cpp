#include <ros/ros.h>
#include <arduino_feedback/arduino_input.h>
#include <sensor_msgs/Joy.h>


class Teleop_MR
{
public:
	Teleop_MR();

private:
	void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);

	ros::NodeHandle nh_;

	int l_scale_=1.1, a_scale_=1.1, scale_PWM_=255;
	int current_mode=1;
	ros::Publisher vel_pub_;
	ros::Subscriber joy_sub_;

};

Teleop_MR::Teleop_MR()
{

	vel_pub_ = nh_.advertise<arduino_feedback::arduino_input>("arduino_input_2at", 3);

	joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 10, &Teleop_MR::joyCallback, this);

}

void Teleop_MR::joyCallback(const sensor_msgs::Joy:ConstPtr& joy)
{
	arduino_feedback::arduino_input message;
	if (joy.button[1]==true){
		message.mode=1;
		current_mode=1;
	}
	else if (joy.button[2]==true) {
		message.mode=2;
		current_mode=2;
	}
	else if (joy.button[3]==true) {
		message.mode=3;
		current_mode=3;
	}

	if (current_mode == 1) {
		message.linear = l_scale_*joy.axes[2];
		message.angular = a_scale_*joy.axes[1];
	}
	else if (current_mode == 2) {
		message.wheel_speed_reqR = l_scale_*joy.axes[2];
		message.wheel_speed_reqL = l_scale_*joy.axes[4];
	}
	else {
		message.pwm_reqR = scale_PWM_*joy.axes[2];
		message.pwm_reqL = scale_PWM_*joy.axes[4];
	}
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "teleop_MR");
	Teleop_MR teleop_MR;

	ros::spin();
}
