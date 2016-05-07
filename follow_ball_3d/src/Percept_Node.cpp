#include <ros/ros.h>
#include <ros/console.h>

#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types_conversion.h>
#include <pcl/octree/octree_search.h>
#include <pcl/octree/octree.h>
#include <pcl/point_types.h>
#include <pcl/io/ply_io.h>
#include <pcl_ros/transforms.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf/message_filter.h>
#include <message_filters/subscriber.h>

#include <std_msgs/Float32.h>
#include <sensor_msgs/PointCloud2.h>

#include <boost/algorithm/string.hpp>



	class Percept_Node {

		public :

			Percept_Node ();

		private :

			ros::NodeHandle nh;
			ros::Publisher  pclPub;

			tf::TransformBroadcaster br;
			tf::TransformListener tflist;

			tf::MessageFilter<sensor_msgs::PointCloud2>* tfPointCloudSub;
			message_filters::Subscriber<sensor_msgs::PointCloud2>* pclSub;;


			std::string workingFrameId_;
			std::string cameraTopicId_;

			const static int THRESHOLD = 1;

			/*	Percept ball REAL ROBOT DONT ERASE */

/*
			const static int MAX_HUE = 360;
			const static int MIN_HUE = 258;

			const static int MAX_SAT = 255;
			const static int MIN_SAT = 70;

			const static int MAX_INT = 255;
			const static int MIN_INT = 71;
*/
			/*	Percept ball REAL ROBOT DONT ERASE */



//					!!!GOOD¡¡¡

			const static int MAX_HUE = 360;
			const static int MIN_HUE = 343;

			const static int MAX_SAT = 134;
			const static int MIN_SAT = 67;

			const static int MAX_INT = 255;
			const static int MIN_INT = 32;


			/* Percept ball in gazebo robocuplab */
/*
			const static int MAX_HUE = 41;
			const static int MIN_HUE = 10;

			const static int MAX_SAT = 255;
			const static int MIN_SAT = 0;

			const static int MAX_INT = 255;
			const static int MIN_INT = 0;
*/
			bool isValid(const pcl::PointXYZHSV& hsv);
			void cloudCB (const sensor_msgs::PointCloud2::ConstPtr& cloud_in);
			void publishTf (float x , float y , float z);
	};


	Percept_Node::Percept_Node () : workingFrameId_("/odom") , cameraTopicId_("/camera/depth_registered/points")
																/*		  cameraTopicId_("/camera/depth/points") */
	{
		pclPub = nh.advertise<sensor_msgs::PointCloud2>("/cameraout" , 1 , false);

		/* Creo una nube de puntos subscriptora para la transformada respecto a bf. */
		pclSub   = new message_filters::Subscriber<sensor_msgs::PointCloud2> (nh, cameraTopicId_, 5);

		/* Transformada respecto a base foot print. */
		tfPointCloudSub = new tf::MessageFilter<sensor_msgs::PointCloud2> (*pclSub, tflist, workingFrameId_, 5);
		tfPointCloudSub->registerCallback(boost::bind(&Percept_Node::cloudCB, this, _1));

		if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug))
				ros::console::notifyLoggerLevelsChanged();
	}

	bool Percept_Node::isValid(const pcl::PointXYZHSV& hsv)
	{
		pcl::PointXYZHSV hsv_scaled;

		hsv_scaled.h = hsv.h;
		hsv_scaled.s = hsv.s * 100;
		hsv_scaled.v = hsv.v * 100;

		return (hsv_scaled.h > MIN_HUE && hsv_scaled.h < MAX_HUE &&
				hsv_scaled.s > MIN_SAT && hsv_scaled.s < MAX_SAT &&
				hsv_scaled.v > MIN_INT && hsv_scaled.v < MAX_INT);
	}

	void Percept_Node::publishTf (float x , float y , float z)
	{
		tf::StampedTransform ballTf;

		tf::Quaternion q;

		ballTf.child_frame_id_ = "/balltfod";
		ballTf.frame_id_ = "/odom";
		ballTf.stamp_ = ros::Time::now();

		ballTf.setOrigin(tf::Vector3(x ,  y , z));

		q.setRPY(0 , 0 , 0.1);
		ballTf.setRotation(q);

		try {

			br.sendTransform(ballTf);

		} catch(tf::TransformException &exception) {

			ROS_ERROR("%s", exception.what());

		}
	}

	void Percept_Node::cloudCB (const sensor_msgs::PointCloud2::ConstPtr& cloud_in)
	{
		sensor_msgs::PointCloud2 cloud;

		try {

			pcl_ros::transformPointCloud(workingFrameId_, *cloud_in, cloud, tflist);

		} catch(tf::TransformException& ex) {

			ROS_ERROR_STREAM( "Transform error of sensor data: " << ex.what() << ", quitting callback");

		}

		pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcrgb (new pcl::PointCloud<pcl::PointXYZRGB>); /* Create a new pcl points XYZRGB */
		pcl::fromROSMsg(cloud , *pcrgb); /* Pass PointCloud from ROS MSG To pcl points XYZRGB */

		pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcrgb_out(new pcl::PointCloud<pcl::PointXYZRGB>); /* Create and out points pcl XYZRGB  */

		int c = 0;
		float x = 0.0 , y = 0.0 , z = 0.0;
		pcl::PointCloud<pcl::PointXYZRGB>::iterator it;
		for (it=pcrgb->begin() ; it != pcrgb->end() ; it++){
			if (!std::isnan(it->x)){

				pcl::PointXYZHSV hsv;
				pcl::PointXYZRGBtoXYZHSV(*it , hsv);

				if (isValid(hsv)) {
					pcrgb_out->push_back(*it);
					x += it -> x;
					y += it -> y;
					z += it -> z;
					c++;
				}
			}
		}

		if (c > THRESHOLD){
			x = x / static_cast<float>(c);
			y = y / static_cast<float>(c);
			z = z / static_cast<float>(c);

			ROS_DEBUG("TF POINTS  : %f , %f , %f" , x , y , z);
			publishTf(x , y , z);
		}

		ROS_DEBUG("TF POINTS  : %f , %f , %f" , x , y , z);

		sensor_msgs::PointCloud2 cloud_out;
		pcl::toROSMsg(*pcrgb_out , cloud_out);

		cloud_out.header.frame_id = cloud.header.frame_id;
		pclPub.publish(cloud_out);
	}

int
main (int argc , char **argv)
{
	ros::init(argc , argv , "Percept_Node");

	Percept_Node percept;
	ros::spin();
	return (0);
}
