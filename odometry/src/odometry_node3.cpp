#include <ros/ros.h>
#include <arduino_feedback/feedback_int.h>
#include <odometry/odo_msg.h>

float velR, velL, vx, vy, x, y, u, omega, theta;
float theta_last=0, x_last=0, y_last=0, dl_last=0, dtheta_last=0; 
const float d=0.1905;
const float r=0.0975;
float T[3][3];
float T_old[3][3] = {{1.,0.,0.},{0.,1.,0.},{0.,0.,1.}};
float Transf[3][3] = {{0.,0.,0.},{0.,0.,0.},{0.,0.,0.}};

odometry::odo_msg message;

ros::Publisher pub;
ros::Subscriber sub; 

void Callback(const arduino_feedback::feedback_int::ConstPtr& data)
{
	const long enc_res = 9575;
	float dt = float(data->looptime)/1000.;

	long encoderR = data->encoderR;
	long encoderL = data->encoderL;

	float dl = float(encoderR + encoderL)/float(enc_res)/2.*6.28*0.0975;
	float dtheta = float(encoderR - encoderL)/float(enc_res)*6.28*0.0975/2./d;

	theta = theta_last + dtheta;

	velR = encoderR/(float)enc_res/dt*2.*3.14*r;
	velL = encoderL/(float)enc_res/dt*2.*3.14*r;

	u = (velR + velL)/2.;
	omega = (velR - velL)/d/2.;

	Transf[0][0] = cos(dtheta_last);
	Transf[0][1] = -sin(dtheta_last);
	Transf[0][2] = dl_last;
	Transf[1][0] = sin(dtheta_last);
	Transf[1][1] = cos(dtheta_last);

	for (int i=0; i<3; i++)
	{
		for (int j=0; j<3; j++)
		{
			for (int k=0; k<3; k++)
			{
				T[i][j] = T[i][j] + T_old[i][k]*Transf[k][j];
			}
		}
	}

	x = x_last + T[0][0]*dl;
	y = y_last + T[1][0]*dl;

	vx = (x-x_last)/dt;
	vy = (y-y_last)/dt;

	for (int i=0; i<3; i++)
	{
		for (int j=0; j<3; j++)
		{
			T_old[i][j] = T[i][j];
			T[i][j] = 0.;
		}
	}

	x_last = x;
	y_last = y;
	theta_last = theta;
	dl_last = dl;
	dtheta_last = dtheta;

	message.u = u;
	message.omega = omega;
	message.x = x;
	message.y = y;
	message.vx = vx;
	message.vy = vy;
	message.theta = theta;
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "odometry");
	ros::NodeHandle nh;

	pub = nh.advertise<odometry::odo_msg>("odometry",1);
	sub = nh.subscribe<arduino_feedback::feedback_int>("feedback_3dx", 5, &Callback);

	ros::Rate rate(110);

	while (ros::ok())
	{
		pub.publish(message);
		ros::spinOnce();
		rate.sleep();
	}
}
