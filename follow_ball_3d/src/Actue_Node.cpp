#include <ros/ros.h>
#include <ros/console.h>
#include <time.h>

#include "geometry_msgs/Twist.h"

#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include <math.h>

#include <csuro_tools/PIDController.h>
#include "follow_ball_3d/Position.h"
#include "follow_ball_3d/Obstacles.h"

	const int MAX_OBSTACLES = 12;

	typedef struct {

		float module;
		float angle;

	}VectorT;

	typedef struct {

		int size;
		VectorT obs[MAX_OBSTACLES];

	}ObsVects;

	class Actue_Node {

		public:

			Actue_Node();
			void transforms();

		private:

			static const float ALPHA = 0.78;
			static const float BETHA = 0.3	;
			static const float MAXANG = 0.96;
			static const float MAXDIST = 1.5;
			static const float MINDIST = 0.75;
			static const float INFRONT = 0.2;
			static const float GOALAWAY = 0.4;
			static const float GOALCENTER = 0.2;


			ros::NodeHandle nh;
			ros::Publisher pubVel;
			ros::Publisher pubPos;
			ros::Subscriber subObs;

			tf::TransformListener tflist;
			tf::TransformBroadcaster br;

			PIDController pidAng, pidLin;

			float lastz;

			VectorT vecB;

			bool locatedB;

			void publishvel(float angle, float module);
			void pubposition(float x , float z);
			void searchBall();
			void obsCallBack (const follow_ball_3d::Obstacles::ConstPtr& obs);
			void pubTf(float module , float angle , int numObs);
			bool obsInFrnt(float angle);
			bool goal (float module , float angle);
			bool equalsForces (float module);



			void printObs(ObsVects obsVects);
			void initObsVec (ObsVects obsVects);
			VectorT calcVecObs(ObsVects obsVects);


			tf::StampedTransform subOdBall ();
			tf::StampedTransform subBfOd();
			void pubBfBall(tf::Transform bf2ball);

			void getVecResult(VectorT obsVect);

	};

	Actue_Node::Actue_Node() :  pidLin("LinealPID", 0.0 , 1.0 , 0.0, 0.2) , pidAng("AngularPID", 0.08 , 1, 0.0, 1.8)
								 /*, pidLin("LinealPID", 1.1 , 2.4, 0.0, 0.2)*/
	{
		pubVel = nh.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 1);
		pubPos = nh.advertise<follow_ball_3d::Position>("/Position" , 1);
		subObs = nh.subscribe("/Obstacles", 1000, &Actue_Node::obsCallBack , this);
		lastz = 0.0;

		vecB.module = 0.0;
		vecB.angle = 0.0;

		if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug))
			ros::console::notifyLoggerLevelsChanged();

		locatedB = false;
	}

	void
	Actue_Node::pubTf(float module , float angle , int numObs)
	{
		tf::StampedTransform object;
		char topic_id[256];
		sprintf(topic_id, "obstacle%d", numObs);

		object.child_frame_id_ = topic_id;
		object.frame_id_ = "base_footprint";
		object.stamp_ = ros::Time::now();

		object.setOrigin(tf::Vector3(module*cos(angle) , module*sin(angle) , 0.42));
		tf::Quaternion q;
		q.setRPY(0, 0, 0.1);
		object.setRotation(q);

		try {

			br.sendTransform(object);

		} catch(tf::TransformException &exception) {

			ROS_ERROR("%s", exception.what());

		}
	}

	void
	Actue_Node::printObs(ObsVects obsVects)
	{
		ROS_DEBUG("size : %d" , obsVects.size);
		for (int i = 0 ; i < obsVects.size ; i++)
			ROS_DEBUG("Obstacle : module %f , angle %f " , obsVects.obs[i].module , obsVects.obs[i].angle);
	}

	void
	Actue_Node::initObsVec(ObsVects obsVects)
	{
		for (int i = 0 ; i < obsVects.size ; i++){
			obsVects.obs[i].module = 0.0;
			obsVects.obs[i].angle = 0.0;
		}
	}

	VectorT
	Actue_Node::calcVecObs(ObsVects obsVects)
	{
		VectorT rVecObs;

		rVecObs.angle = 0.0;
		rVecObs.module = 0.0;
		for (int i = 0 ; i < obsVects.size ; i++){
			if (abs(obsVects.obs[i].angle) < MAXANG && obsVects.obs[i].module < MAXDIST){
				/*
				 * Calculate Obstacle Vector total from all obstacles detected in front robot in range damage.
				 *
				 * This follow the vector obstacle profile of VFF.
				 */
				obsVects.obs[i].module = MAXDIST - obsVects.obs[i].module;
				obsVects.obs[i].module = MAXANG - obsVects.obs[i].angle;

				rVecObs.module += obsVects.obs[i].module;
				rVecObs.angle  += obsVects.obs[i].angle;
			}
		}
		return (rVecObs);
	}


	void
	Actue_Node::pubposition(float x , float z)
	{
		follow_ball_3d::Position pos;

		pos.lineal = x;
		pos.angular = z;

		pubPos.publish(pos);
	}


	void Actue_Node::publishvel(float angle, float module)
	{
		geometry_msgs::Twist cmd;

		pidLin.setReference(module);
		pidAng.setReference(angle);

		cmd.linear.x = pidLin.getOutput();
		cmd.angular.z = pidAng.getOutput();

		pubVel.publish(cmd);
		pubposition(cmd.linear.x , cmd.angular.z);
	}

	void Actue_Node::searchBall()
	{
		geometry_msgs::Twist cmd;

		cmd.linear.x = 0.0;
		cmd.angular.z = lastz * 2;

		pubVel.publish(cmd);
	}

	bool Actue_Node::obsInFrnt(float angle)
	{
		return (angle < INFRONT && angle > -INFRONT);
	}

	bool Actue_Node::equalsForces (float module)
	{
		return (module > -0.1 && module < 0.1);
	}

	bool Actue_Node::goal (float module , float angle)
	{
		return (module < GOALAWAY && fabs(angle) < GOALCENTER);
	}

	void
	Actue_Node::getVecResult(VectorT obsVect)
	{
		VectorT vecBall;
		VectorT vResult;

		/* Calculate attraction vector. */
		vecBall.module = vecB.module * ALPHA;
		vecBall.angle =  vecB.angle  * ALPHA;

		/* Default move. Obstacles are repulsive.*/
		obsVect.module = -obsVect.module * BETHA;
		obsVect.angle  = -obsVect.angle  * BETHA * 5;

		if (obsInFrnt(obsVect.angle)) {
			/*
			 * Obstacle in front robot calculate vecObs to avoid obstacle.
			 */
			if (obsVect.angle < INFRONT && obsVect.angle > 0.0)
				obsVect.angle =  0.3;
			else if (obsVect.angle > -INFRONT && obsVect.angle < 0.0)
				obsVect.angle = -0.3;

			/* Turn around all. */
			obsVect.module = 0.0;
		}

		vResult.module = vecBall.module + obsVect.module;
		vResult.angle = vecBall.angle + obsVect.angle;

		if (vResult.module < 0){
			/*
			 * Robot not gone back.
			 */
			vResult.module = 0.1;
		}

		if (equalsForces(vResult.module)){
			/*
			 * Robot must follow to ball.
			 */
			vResult.module = 0.1;
		}

		if (goal(vecBall.module , vecBall.angle)) {
			/*
			 *  Stop robot I have arrived.
			 */
			vResult.module = 0.0;
			vResult.angle = 0.0;
		}

		ROS_INFO  ("VECTOR BALL : module : %f angle : %f " , vecBall.module , vecBall.angle);
		ROS_INFO  ("VECTOR OBSTACLE : module : %f angle : %f " , obsVect.module , obsVect.angle);
		ROS_WARN  ("RESULT VECTOR : module : %f angle : %f" , vResult.module , vResult.angle);

		publishvel(vResult.angle , vResult.module);
	}

	void
	Actue_Node::obsCallBack (const follow_ball_3d::Obstacles::ConstPtr& obs)
	{
		std::vector<float>::const_iterator it1;
		std::vector<float>::const_iterator it2;
		std::vector<int>::const_iterator it3;
		ObsVects obsVects;
		VectorT vecObs;
		VectorT vecResult;

		vecObs.module = 0.0;
		vecObs.angle = 0.0;
		float distMax = 0.0;

		it2 = obs->angles.begin();
		it3 = obs->points.begin();
		int numObs = 0;

		obsVects.size = obs->size;
		for (it1 = obs->modules.begin() ;  it1 != obs->modules.end() ; it1++) {

			obsVects.obs[numObs].module = *it1;
			obsVects.obs[numObs].angle = *it2;

			it2++;
			it3++;
			numObs++;
		}
		printObs(obsVects);

		vecObs = calcVecObs(obsVects);
		getVecResult(vecObs);
	}

	tf::StampedTransform Actue_Node::subOdBall ()
 	{
		tf::StampedTransform od2ball;

		try {

			tflist.lookupTransform("/odom", "/balltfod", ros::Time(0), od2ball);

			ROS_INFO("/odom -> /balltf [%lf, %lf, %lf] %lf ago", od2ball.getOrigin().x(),
						od2ball.getOrigin().y(), od2ball.getOrigin().z(), (ros::Time::now() - od2ball.stamp_).toSec());
			locatedB = true;

		} catch (tf::TransformException& ex) {

			ROS_ERROR("%s", ex.what());
		}
		return (od2ball);
	}

	tf::StampedTransform Actue_Node::subBfOd()
	{
		tf::StampedTransform bf2od;

		try {

			tflist.lookupTransform("/base_footprint", "/odom" , ros::Time(0), bf2od);

			ROS_INFO("/odom -> /balltf [%lf, %lf, %lf] %lf ago", bf2od.getOrigin().x(),
						bf2od.getOrigin().y(), bf2od.getOrigin().z(), (ros::Time::now() - bf2od.stamp_).toSec());

		} catch (tf::TransformException& ex) {

			ROS_ERROR("%s", ex.what());
		}
		return (bf2od);
	}

	void
	Actue_Node::pubBfBall(tf::Transform bf2ball)
	{
		tf::StampedTransform bf2ballp;
		tf::Quaternion q;


		bf2ballp.setOrigin(tf::Vector3(bf2ball.getOrigin().x() , bf2ball.getOrigin().y() , bf2ball.getOrigin().z()));

		bf2ballp.child_frame_id_ = "bf2ball";
		bf2ballp.frame_id_ = "/base_footprint";
		bf2ballp.stamp_ = ros::Time::now();

		q.setRPY(0 , 0 , 0.1);
		bf2ballp.setRotation(q);

		try {

			br.sendTransform(bf2ballp);

		} catch(tf::TransformException &exception) {

			ROS_ERROR("%s", exception.what());

		}
	}

	void
	Actue_Node::transforms()
	{
		tf::StampedTransform bf2od;
		tf::StampedTransform od2ball;
		tf::Transform bf2ball;

		vecB.module = 0.0;
		vecB.angle = 0.0;

		od2ball = subOdBall();
		bf2od = subBfOd();

		bf2ball = bf2od * od2ball;

		pubBfBall(bf2ball);

		if (locatedB){
			vecB.module = sqrt(pow(bf2ball.getOrigin().x(), 2) + pow(bf2ball.getOrigin().y(), 2));
			vecB.angle = atan2(bf2ball.getOrigin().y(), bf2ball.getOrigin().x());
		}
	}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "Actue_Node");
	Actue_Node actue_node;
	ros::Rate loop_rate(20);

	while (ros::ok()) {
		actue_node.transforms();
		ros::spinOnce();
		loop_rate.sleep();
	}
	return (0);
}
