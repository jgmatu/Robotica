/*
 * go_ball.cpp
 *
 *  Created on: Feb 18, 2016
 *      Author: javier
 */

#include "ros/ros.h"
#include "follow_ball/Point.h"
#include "geometry_msgs/Twist.h"
#include <math.h>
#include "follow_ball/PIDController.h"
#include "follow_ball/Position.h"


class GoBall{

public:

	GoBall();
	void step();

private:

	ros::NodeHandle n_;

	const static int MIDWIDTH = 400;
	const static int HEIGTH = 640;
	const static int MIDHEIGTH = 320;

	const static float FACVELTURN = 2;
	const static float FACTLINEAL = 1;

	const static float MAX_LEFT =  0.5;
	const static float MAX_RIGHT= -0.5;

	float x_;
	float y_;
	float lasty_;
	bool inside_;
	bool right_;
	bool left_;

	PIDController pidLin_, pidAng_;

	ros::Subscriber sub_point_;
	ros::Publisher pub_vel_;
	ros::Publisher pub_pos_;

	void goBallCallback(const follow_ball::Point::ConstPtr& msg);
	bool isInside ();
	geometry_msgs::Twist ballinsideIm();
	void statusInside ();
	void statusBallLost();
	geometry_msgs::Twist ballLost();

};

	GoBall::GoBall(): pidLin_("LinealPID"  , 0.1 , 1.0 ,  0.0 , 0.5), pidAng_("AngularPID" , 0.1 , 1 , 0.0 , 1.2	)
	{
		sub_point_ = n_.subscribe("/Point", 1000, &GoBall::goBallCallback , this);
		pub_vel_ = n_.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity" , 1000);
		pub_pos_ = n_.advertise<follow_ball::Position>("/Position" , 1000);
		x_= 0;
		y_= 0;
 		inside_ = false;
		right_ = false;
		left_ = false;
		lasty_ = MIDWIDTH;
	}

	void GoBall::goBallCallback(const follow_ball::Point::ConstPtr& msg)
	{
		x_ = msg->x;
		y_ = msg->y;
	}

	bool GoBall::isInside ()
	{
		return x_ != 0.0 || y_ != 0.0;
	}

	geometry_msgs::Twist GoBall::ballinsideIm()
	{
		/* When I have the ball on the image */
		geometry_msgs::Twist cmd;
		float z = 0;
		float x = 0;

		x = (MIDWIDTH - y_) / MIDWIDTH;     /* Domain of linear velocity z (-1,1)  */
		z = (MIDHEIGTH - x_) / MIDHEIGTH;   /* Domain of angular velocity  x (-1,1)  */


		pidLin_.setReference(x);
		pidAng_.setReference(z);

		cmd.linear.x  = pidLin_.getOutput();
		cmd.angular.z = pidAng_.getOutput();

		return cmd;
	}

	void GoBall::statusInside ()
	{
		if (isInside() && !inside_)
			inside_ = true;


		if (!isInside() && inside_)
			inside_ = false;

	}

	void GoBall::statusBallLost()
	{
		if (!inside_ && lasty_ > MIDWIDTH && !right_ && !left_)
		{
			ROS_ERROR("Right");
			right_ = true;
			left_ = false;
		}

		if (inside_ && right_)
		{
			right_ = false;
			ROS_ERROR("BALL FOUND AT RIGHT");
		}

		if (!inside_ && lasty_ < MIDWIDTH && !left_ && !right_)
		{
			ROS_ERROR("Left!");
			left_ = true;
			right_ = false;
		}

		if (inside_ && left_)
		{
			left_ = false;
			ROS_ERROR("BALL FOUND AT LEFT");
		}
	}

	geometry_msgs::Twist GoBall::ballLost()
	{
		geometry_msgs::Twist cmd;

		if (left_)
			pidAng_.setReference(MAX_LEFT);

		if (right_)
			pidAng_.setReference(MAX_RIGHT);

		cmd.angular.z = pidAng_.getOutput();
		cmd.linear.x = 0;
		return cmd;
	}

	void GoBall::step()
	{
		geometry_msgs::Twist cmd;
		follow_ball::Position pos;

		statusInside();
		statusBallLost();

		cmd.linear.x = 0;
		cmd.angular.z = 0;
		if (inside_)
			cmd = ballinsideIm();

	/*	if (!inside_)
			cmd = ballLost();
*/
		pub_vel_.publish(cmd);
		pos.lineal = cmd.linear.x;
		pos.angular = cmd.angular.z;
		pub_pos_.publish(pos);
		lasty_ = y_;
	}

int
main(int argc, char **argv)
{

	ros::init(argc, argv, "bumpgo");


	GoBall goBall;

	ros::Rate loop_rate(20);

	while (ros::ok()){

		goBall.step();

		ros::spinOnce();
		loop_rate.sleep();

	}
	return 0;
}



