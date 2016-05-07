#include "ros/ros.h"
#include <ros/console.h>

#include <sensor_msgs/LaserScan.h>
#include <tf/transform_listener.h>
#include <tf/message_filter.h>
#include <message_filters/subscriber.h>
#include <tf/tf.h>


#include "follow_ball_3d/Obstacles.h"

	typedef struct
	{
		float moduleB;
		float angleB;
		float moduleE;
		float angleE;
		int points;
	}ObstacleInfo;

	typedef struct Nodo {
	   ObstacleInfo dato;
	   struct Nodo *siguiente;
	} TipoNodo;

	typedef struct List {
		TipoNodo *first;
		TipoNodo *last;
		int numObs;
	}ListObs;

	class Scan_Node {

		public:

			Scan_Node();
			virtual ~Scan_Node();

			void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan_in);
			const std::vector<tf::Stamped<tf::Point> >& getLastScan() const {return (scanBf);};

		private:

			ros::NodeHandle nh;
			ros::Publisher pubObs;

			std::string baseFrameId;
			std::string lastopc;

			tf::TransformListener tfList;

			std::vector<tf::Stamped<tf::Point> > scanBf;

			ObstacleInfo obs;
			ListObs listObs;
			bool inobs;

			tf::MessageFilter<sensor_msgs::LaserScan>* tfScanSub;
			message_filters::Subscriber<sensor_msgs::LaserScan>* scanSub;

			static const float MAXZONE = 2.0;
			static const float STICK = 0.19;
			static const float RELDIST = 0.4;
			static const int POINTS = 18;
			static const int WALL = 200;

			bool maxRange (float module);
			bool minRange (float module);
			bool isobstacle (float module);
			void put(float module , float angle);
			void countObs(float module , float angle);
			void publishObs ();
			void deleteObs();
			bool isObs(float module , float angle);
			void initObs();
	};


	Scan_Node::Scan_Node() : nh(), baseFrameId("base_footprint"), lastopc("/scan"), scanBf()
	{
		scanSub = new message_filters::Subscriber<sensor_msgs::LaserScan> (nh, lastopc, 5);
		tfScanSub = new tf::MessageFilter<sensor_msgs::LaserScan> (*scanSub, tfList, baseFrameId, 5);
		tfScanSub -> registerCallback(boost::bind(&Scan_Node::scanCallback, this, _1));
		pubObs = nh.advertise<follow_ball_3d::Obstacles>("/Obstacles" , 5);

		listObs.first = NULL;
		listObs.last = NULL;
		listObs.numObs = 0;
		inobs = false;
		obs.moduleB = 0.0;
		obs.angleB = 0.0;

		if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug))
			ros::console::notifyLoggerLevelsChanged();
	}

	Scan_Node::~Scan_Node()
	{
		;
	}

	bool Scan_Node::maxRange (float module)
	{
		return (module < MAXZONE);
	}

	bool Scan_Node::minRange (float module)
	{
		return (module > STICK);
	}

	bool Scan_Node::isobstacle (float module)
	{
		return (maxRange(module) && minRange(module));
	}


	void
	Scan_Node::put(float module , float angle)
	{
		TipoNodo *aux = NULL;

		if (listObs.first == NULL){
			listObs.first = new Nodo;
			listObs.first->dato.angleB = angle;
			listObs.first->dato.moduleB = module;
			listObs.first->dato.angleE = angle;
			listObs.first->dato.moduleE = module;
			listObs.first->dato.points = 1;
			listObs.first->siguiente = NULL;
			listObs.last = listObs.first;
			listObs.numObs = 1;
			return;
		}

		aux = new Nodo;
		aux->dato.angleB = angle;
		aux->dato.moduleB = module;
		aux->dato.angleE = angle;
		aux->dato.moduleE = module;
		aux->dato.points = 1;
		aux->siguiente = NULL;

		listObs.last->siguiente = aux;
		listObs.last = aux;
		listObs.numObs += 1;
	}

	bool
	Scan_Node::isObs(float module , float angle)
	{
		return (fabs(module - obs.moduleB) < RELDIST && fabs(angle - obs.angleB) < RELDIST);
	}

	void
	Scan_Node::countObs (float module , float angle)
	{
		if (inobs){
			if (!isObs(module , angle))
				inobs = false;
			 else{
				listObs.last->dato.points += 1;
				listObs.last->dato.moduleE = module;
				listObs.last->dato.angleE = angle;
			 }
		}

		if (!inobs){
			put(module , angle);
			inobs = true;
		}
		obs.angleB = angle;
		obs.moduleB = module;
	}

	void
	Scan_Node::publishObs ()
	{
		follow_ball_3d::Obstacles obstacles;
		TipoNodo *aux = listObs.first;
		int numObs = 0;

		for (int i = 0 ; i < listObs.numObs ; i++){
			if (aux->dato.points > POINTS && aux->dato.points < WALL) {
				obstacles.modules.push_back((aux->dato.moduleB + aux->dato.moduleE )/ 2.0);
				obstacles.angles.push_back((aux->dato.angleB + aux->dato.angleE) / 2.0);
				obstacles.points.push_back(aux->dato.points);
				numObs++;
			}
			aux = aux->siguiente;
		}
		obstacles.size = numObs;
		pubObs.publish(obstacles);
	}

	void
	Scan_Node::deleteObs()
	{
		TipoNodo *aux = NULL;

		while (listObs.first != NULL){
			aux  = listObs.first;
			listObs.first = listObs.first->siguiente;
			free(aux);
		}
		listObs.last = NULL;
		listObs.numObs = 0;
		inobs = false;
	}

	void
	Scan_Node::initObs()
	{
		listObs.first = NULL;
		listObs.last = NULL;
		listObs.numObs = 0;
		obs.moduleB = 0;
		obs.angleB = 0;
		inobs = false;
	}

	void
	Scan_Node::scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan_in)
	{
		float o_t_min = 0.0 ,  o_t_max = 0.0 , o_t_inc = 0.0 , module = 0.0 , angle = 0.0;

		o_t_min = scan_in->angle_min;
		o_t_max = scan_in->angle_max;
		o_t_inc = scan_in->angle_increment;

		int num_points = static_cast<int>(2.0 * o_t_max / o_t_inc);

		tf::Stamped<tf::Point> scan_sensor[num_points];
		scanBf.resize(num_points);

		float rx = 0.0 , ry = 0.0;
		int c = 0;
		initObs();
		for(int i = 0; i < num_points; i++) {

			float theta = o_t_min+i*o_t_inc;
			float r = scan_in->ranges[i];

			scan_sensor[i].setX(r*cos(theta));
			scan_sensor[i].setY(r*sin(theta));
			scan_sensor[i].setZ(0.0);
			scan_sensor[i].setW(1.0);
			scan_sensor[i].stamp_ = scan_in->header.stamp;
			scan_sensor[i].frame_id_ = scan_in->header.frame_id;

			tfList.transformPoint(baseFrameId, scan_sensor[i], scanBf[i]);

			//  Scan Point process
			module = sqrt(pow(scanBf[i].getX() , 2) + pow(scanBf[i].getY() , 2));
			angle = atan2(scanBf[i].getY() , scanBf[i].getX());
			if (isobstacle(module))
				countObs(module , angle);
		}
		publishObs();
		deleteObs();
	}

int
main (int argc , char *argv[])
{
	ros::init(argc , argv , "Scan_Node");
	Scan_Node scan;
	ros::spin();
}



