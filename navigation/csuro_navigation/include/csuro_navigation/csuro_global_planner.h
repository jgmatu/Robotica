#include <ros/ros.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <costmap_2d/costmap_2d_publisher.h>
#include <nav_core/base_global_planner.h>
#include <geometry_msgs/PoseStamped.h>
#include <angles/angles.h>
#include <base_local_planner/world_model.h>
#include <base_local_planner/costmap_model.h>

using std::string;

#ifndef CSURO_GLOBAL_PLANNER_CPP
#define CSURO_GLOBAL_PLANNER_CPP

namespace csuro_global_planner {

class CSUROGlobalPlanner : public nav_core::BaseGlobalPlanner {
public:

	CSUROGlobalPlanner();
	CSUROGlobalPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros);


	void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros);
	bool makePlan(const geometry_msgs::PoseStamped& start,
			const geometry_msgs::PoseStamped& goal,
			std::vector<geometry_msgs::PoseStamped>& plan
	);

private:

	typedef struct {
		int mx;
		int my;
		int cost;
	} TCell;

	typedef struct {
		bool is[2];
		int pos[2];
	} TSize;


	std::list<TCell> cells;

	static const int WALL    = 253;
	static const int UNKNOWN = 255;
	static const int FLOOD   = 170;
	static const int EACH    =   1;
	static const int SPACE   =   0;
	static const int DIAG    =   1;
	static const int DELTA   =   2;
	static const int EMPTY   =   0;
	static const int BORDER  =  70;

	ros::NodeHandle nh_;

	costmap_2d::Costmap2D original_costmap_;
	costmap_2d::Costmap2D gradient_;

	costmap_2d::Costmap2DPublisher gradient_pub_;

	bool isCoordInMap(const costmap_2d::Costmap2D &costmap, const float& x, const float& y);
	bool isCoordInMap(const costmap_2d::Costmap2D &costmap, int x, int y);

	void binaryMap (int i , int j);
	void initMap ();
	bool inMap (int i , int j);
	bool isEmpty (int mx , int my);
	void fillOut (int mx , int my , int cost);
	bool isEndPath (int startCellI , int startCellJ);
	void flood (int goalCellI , int goalCellJ , int startCellI , int startCellJ);
	void min (TCell cell , TCell *minCell);
	void checkH (TCell cell , TSize *horizontal);
	void checkV (TCell cell , TSize *vertical);
	bool isSizes (TSize sizes);
	void initSize (TSize *size);
	int meanSize (TSize size); // We cant return nothing of memory from stack!!.
    TCell optimize (TCell cell);
    bool isCell (int i);
    void print (std::vector<geometry_msgs::PoseStamped>& plan);
    void path (std::vector<geometry_msgs::PoseStamped>& plan , int goalCellI , int goalCellJ , const geometry_msgs::PoseStamped& goal , TCell cell , int* i);
    void updateMap();
    void maskUp(int mx , int my , int cost , bool *border);
    void maskUpR(int mx , int my , int cost,  bool *border);
    void maskRigth (int mx , int my , int cost ,   bool *border);
    void maskDownR (int mx , int my , int cost ,  bool *border);
    void maskDown(int mx , int my , int cost ,  bool *border);
    void maskDownLeft(int mx , int my , int cost ,  bool *border);
    void maskLeft (int mx , int my , int cost ,  bool *border);
    void maskLeftUp (int mx , int my , int cost ,  bool *border);









};
}; //namespace csuro_global_planner
#endif
