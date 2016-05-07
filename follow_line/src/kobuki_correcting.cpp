/*
 * kobuki_correcting.cpp
 *
 *  Created on: Feb 4, 2016
 *      Author: javier
 */

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "geometry_msgs/Twist.h"
#include "std_msgs/Float32.h"
#include "follow_ball/Position.h"


class Correcting{

public:
	Correcting()
	{
		shift_sub_ = nh_.subscribe("/Shift" , 1000 , &Correcting::getShift , this);
		cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity" , 1000);
		pub_pos_ = nh_.advertise<follow_ball::Position>("/Position" , 1000);
		shift_ = 0.0;
	}

	void getShift (const std_msgs::Float32 msg)
	{
		shift_ = ((POS_LINE_HOR - msg.data) / MAX_ROWS);
	}

	bool isLeft ()
	{
		return shift_ > 0.0 && shift_ < 0.5;
	}

	bool isRight()
	{
		return shift_ > 0.5 && shift_ < 1.0;
	}

	void corrector()
	{
		geometry_msgs::Twist msg_vel;
		follow_ball::Position pos;

		if (isRight())
			msg_vel.angular.z = -shift_*4;
		else
			msg_vel.angular.z = shift_*4;
		msg_vel.linear.x = 0.2;

		cmd_vel_pub_.publish(msg_vel);
		pos.lineal = msg_vel.linear.x;
		pos.angular = msg_vel.angular.z;
		pub_pos_.publish(pos);
	}

private:

	const static float POS_LINE_HOR = 320.0;
	const static float MAX_ROWS = 640.0;

	ros::NodeHandle nh_;
	ros::Subscriber shift_sub_;
	ros::Publisher cmd_vel_pub_;
	ros::Publisher pub_pos_;

	float shift_;

};

int main(int argc, char **argv)
{

	ros::init(argc, argv, "kobuki_correcting");

	Correcting correcting;

	ros::Rate loop_rate(20);

	while (ros::ok())
	{
		ros::spinOnce();
		correcting.corrector();
		loop_rate.sleep();
	}
	return 0;
}
