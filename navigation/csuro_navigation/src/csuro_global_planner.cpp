#include <pluginlib/class_list_macros.h>
#include <csuro_navigation/csuro_global_planner.h>

//register this planner as a BaseGlobalPlanner plugin
PLUGINLIB_EXPORT_CLASS(csuro_global_planner::CSUROGlobalPlanner, nav_core::BaseGlobalPlanner)

using namespace std;

//Default Constructor
namespace csuro_global_planner {

CSUROGlobalPlanner::CSUROGlobalPlanner ()
: nh_(),
  gradient_(),
  original_costmap_(),
  gradient_pub_(&nh_, &gradient_, "/map", "/csuro_navigation_gradient", true)
{
	initMap();
}

CSUROGlobalPlanner::CSUROGlobalPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros)
: nh_(),
  gradient_(),
  original_costmap_(),
  gradient_pub_(&nh_, &gradient_, "/map", "/csuro_navigation_gradient", true)
{
	initialize(name, costmap_ros);
}


void CSUROGlobalPlanner::binaryMap (int i , int j)
{
	if (gradient_.getCost(i , j) < FLOOD) {
		/*
		 * Empty cells...
		 */
		gradient_.setCost(i , j , EMPTY);
	} else if (gradient_.getCost(i , j) != UNKNOWN) {
		/*
		 * Wall cell except cell unknowns...
		 */
		gradient_.setCost(i , j , WALL);
	}
}


void CSUROGlobalPlanner::initMap ()
{
	for (int i = 0 ; i < gradient_.getSizeInCellsX() ; i++) {
		for (int j = 0 ; j < gradient_.getSizeInCellsY() ; j++) {
			/*
			 * Empty cells, Wall cells and Unknown cells...
			 */
			binaryMap(i , j);
		}
	}
}

void CSUROGlobalPlanner::updateMap()
{
	for (int i = 0 ; i < gradient_.getSizeInCellsX() ; i++) {
			for (int j = 0 ; j < gradient_.getSizeInCellsY() ; j++) {
				if (gradient_.getCost(i , j) != WALL && gradient_.getCost(i , j) != EMPTY && gradient_.getCost(i , j) != UNKNOWN) {
					/*
					 * Set gradient path planning from a value
					 * of gradient to empty path.
					 */
					gradient_.setCost(i , j , EMPTY);
				}
			}
		}
}

void CSUROGlobalPlanner::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros)
{
	ROS_INFO("CSURO Global Planer initializing...");

	original_costmap_ = *costmap_ros->getCostmap();
	gradient_ = *costmap_ros->getCostmap();

	gradient_pub_.publishCostmap();
	initMap();
	ROS_INFO("CSURO Global Planer initialized.");
}


bool CSUROGlobalPlanner::inMap (int i , int j)
{
	return i < gradient_.getSizeInCellsX() && j < gradient_.getSizeInCellsY();
}


bool CSUROGlobalPlanner::isEmpty (int mx , int my)
{
	return gradient_.getCost(mx , my) == EMPTY;
}

void CSUROGlobalPlanner::maskUp(int mx , int my , int cost , bool *border)
{
	TCell cell;


	if (isEmpty(mx , my + 1)) {

		cell.mx = mx;
		cell.my = my + 1;
		cell.cost = cost + DELTA;

		gradient_.setCost(cell.mx  , cell.my , cell.cost);
		cells.push_back(cell);
		ROS_INFO("A");

	} else if (gradient_.getCost(mx , my + 1) == WALL) {

		*border = true;

	}
}

void CSUROGlobalPlanner::maskUpR(int mx , int my , int cost , bool *border)
{
	TCell cell;

	if (isEmpty(mx + 1  , my + 1)) {

		cell.mx = mx + 1;
		cell.my = my + 1;
		cell.cost = cost + DELTA + DIAG;

		gradient_.setCost(cell.mx  , cell.my , cell.cost);
		cells.push_back(cell);
		ROS_INFO("AD");

	} else if (gradient_.getCost(mx + 1 , my + 1) == WALL) {

		*border = true;

	}
}

void CSUROGlobalPlanner::maskRigth (int mx , int my , int cost , bool *border)
{
	TCell cell;

	if (isEmpty(mx + 1  , my)) {

		cell.mx = mx + 1;
		cell.my = my;
		cell.cost = cost + DELTA;

		gradient_.setCost(cell.mx  , cell.my , cell.cost);
		cells.push_back(cell);
		ROS_INFO("D");

	} else if (gradient_.getCost(mx + 1 , my) == WALL) {

		*border = true;

	}

}

void CSUROGlobalPlanner::maskDownR (int mx , int my , int cost , bool *border)
{
	TCell cell;

	if (isEmpty(mx + 1  , my - 1)) {

		cell.mx = mx + 1;
		cell.my = my - 1;
		cell.cost = cost + DELTA + DIAG;

		gradient_.setCost(cell.mx  , cell.my , cell.cost);
		cells.push_back(cell);
		ROS_INFO("BD");
	} else if (gradient_.getCost(mx + 1 , my - 1) == WALL) {

		*border = true;

	}
}

void CSUROGlobalPlanner::maskDown(int mx , int my , int cost , bool *border)
{
	TCell cell;

	if (isEmpty(mx , my - 1)) {

		cell.mx = mx;
		cell.my = my - 1;
		cell.cost = cost + DELTA;

		gradient_.setCost(cell.mx  , cell.my , cell.cost);
		cells.push_back(cell);
		ROS_INFO("B");

	} else if (gradient_.getCost(mx, my - 1) == WALL){

		*border = true;

	}
}

void CSUROGlobalPlanner::maskDownLeft(int mx , int my , int cost , bool *border)
{
	TCell cell;

	if (isEmpty(mx - 1  , my - 1)) {
		cell.mx = mx - 1;
		cell.my = my - 1;
		cell.cost = cost + DELTA + DIAG;

		gradient_.setCost(cell.mx  , cell.my , cell.cost);
		cells.push_back(cell);
		ROS_INFO("BI");

	} else if (gradient_.getCost(mx -1 , my - 1) == WALL){

		*border = true;

	}
}

void CSUROGlobalPlanner::maskLeft (int mx , int my , int cost , bool *border)
{
	TCell cell;

	if (isEmpty(mx - 1, my)) {
		cell.mx = mx - 1;
		cell.my = my;
		cell.cost = cost + DELTA;

		gradient_.setCost(cell.mx  , cell.my , cell.cost);
		cells.push_back(cell);
		ROS_INFO("I");

	} else if (gradient_.getCost(mx -1 , my) == WALL) {

		*border = true;

	}
}

void CSUROGlobalPlanner::maskLeftUp (int mx , int my , int cost , bool *border)
{
	TCell cell;

	if (isEmpty(mx - 1  , my + 1)) {
		cell.mx = mx - 1;
		cell.my = my + 1;
		cell.cost = cost + DELTA + DIAG;

		gradient_.setCost(cell.mx  , cell.my , cell.cost);
		cells.push_back(cell);
		ROS_INFO("AI");

	} else if (gradient_.getCost(mx -1 , my + 1) == WALL) {

		*border = true;

	}
}

void CSUROGlobalPlanner::fillOut (int mx , int my , int cost)
{
	bool border = false;


	maskUp(       mx , my , cost , &border);
	maskUpR(      mx , my , cost , &border);
	maskRigth(    mx , my , cost , &border);
	maskDownR(    mx , my , cost , &border);
	maskDown(     mx , my , cost , &border);
	maskDownLeft( mx , my , cost , &border);
	maskLeft(     mx , my , cost , &border);
	maskLeftUp(   mx , my , cost , &border);

	if (border)
		gradient_.setCost(mx , my  , cost  + BORDER);
}


bool CSUROGlobalPlanner::isEndPath (int startCellI , int startCellJ)
{
	return false;
	return !isEmpty(startCellJ + 1 , startCellI)      && !isEmpty(startCellJ + 1 , startCellI + 1) &&
				!isEmpty(startCellJ , startCellI+1) && !isEmpty(startCellJ-1 , startCellI + 1) &&
				!isEmpty(startCellJ-1,startCellI)   && !isEmpty(startCellJ-1 ,startCellI  - 1) &&
				!isEmpty(startCellJ , startCellI-1) && !isEmpty(startCellJ+1 , startCellI - 1);
}



void CSUROGlobalPlanner::flood (int goalCellI , int goalCellJ , int startCellI , int startCellJ)
{
	TCell cell;

	if (!cells.empty() && !isEndPath(startCellI , startCellJ)) {

		cell = cells.front();
		cells.pop_front();

		fillOut(cell.mx , cell.my , cell.cost);

		flood(goalCellI , goalCellJ , startCellI , startCellJ);
	}

	if (isEndPath(startCellI , startCellJ))
		ROS_WARN("END PATH");
}

void CSUROGlobalPlanner::min (TCell cell , TCell *minCell)
{
	int cost = UNKNOWN;

	// Initialize the minimal cell with Upper cell.
	cost = gradient_.getCost(cell.mx , cell.my + 1);
	if (cell.cost < UNKNOWN) {
		minCell->mx = cell.mx;
		minCell->my = cell.my + 1;
		minCell->cost = cost;
	}

	cost = gradient_.getCost(cell.mx + 1 , cell.my + 1);
	if (cost < minCell->cost){
		minCell->mx = cell.mx + 1;
		minCell->my = cell.my + 1;
		minCell->cost = cost;
	}

	cost = gradient_.getCost(cell.mx + 1 , cell.my);
	if (cost < minCell->cost) {
		minCell->mx = cell.mx + 1;
		minCell->my = cell.my;
		minCell->cost = cost;
	}

	cost = gradient_.getCost(cell.mx + 1 , cell.my - 1);
	if (cost < minCell->cost) {
		minCell->mx = cell.mx + 1;
		minCell->my = cell.my - 1;
		minCell->cost = cost;
	}

	cost = gradient_.getCost(cell.mx , cell.my - 1);
	if (cost < minCell->cost) {
		minCell->mx = cell.mx;
		minCell->my = cell.my - 1;
		minCell->cost = cost;
	}

	cost = gradient_.getCost(cell.mx - 1, cell.my - 1);
	if (cost < minCell->cost) {
		minCell->mx = cell.mx - 1;
		minCell->my = cell.my - 1;
		minCell->cost = cost;
	}

	cost = gradient_.getCost(cell.mx - 1, cell.my);
	if (cost < minCell->cost) {
		minCell->mx = cell.mx - 1;
		minCell->my = cell.my;
		minCell->cost = cost;
	}

	cost = gradient_.getCost(cell.mx - 1, cell.my + 1);
	if (cost < minCell->cost) {
		minCell->mx = cell.mx - 1;
		minCell->my = cell.my + 1;
		minCell->cost = cost;
	}

	if (cost < 0){
		/*
		 * Trace error.
		 */
		minCell->mx = 0;
		minCell->my = 0;
		minCell->cost = UNKNOWN;
	}
}


bool CSUROGlobalPlanner::isCell (int i)
{
	return i%EACH==0;
}


void CSUROGlobalPlanner::print (std::vector<geometry_msgs::PoseStamped>& plan)
{

	ROS_INFO("Plan ========================");
	std::vector<geometry_msgs::PoseStamped>::iterator it;
	for(it=plan.begin(); it<plan.end(); ++it)
	{
		ROS_INFO("%lf %lf", it->pose.position.x, it->pose.position.y);
	}
	ROS_INFO("=============================");

}


void CSUROGlobalPlanner::path (std::vector<geometry_msgs::PoseStamped>& plan , int goalCellI , int goalCellJ , const geometry_msgs::PoseStamped& goal , TCell cell , int *i)
{
	TCell minCell;
	tf::Quaternion goal_quat;
	geometry_msgs::PoseStamped new_goal = goal;
	geometry_msgs::PoseStamped last_goal = goal;

	/* Calc min Cell always the cell know if is wall or not...*/
	minCell.mx = 0;
	minCell.my = 0;
	minCell.cost = UNKNOWN;

	if (isCell(*i))
		 min(cell , &minCell);


	gradient_.mapToWorld(cell.mx , cell.my , last_goal.pose.position.x , last_goal.pose.position.y);
	gradient_.mapToWorld(minCell.mx , minCell.my , new_goal.pose.position.x , new_goal.pose.position.y);

	goal_quat = tf::createQuaternionFromYaw(
			atan2 (new_goal.pose.position.y - last_goal.pose.position.y , new_goal.pose.position.x - last_goal.pose.position.x));

	new_goal.pose.orientation.x = goal_quat.x();
	new_goal.pose.orientation.y = goal_quat.y();
	new_goal.pose.orientation.z = goal_quat.z();
	new_goal.pose.orientation.w = goal_quat.w();

	if (isCell(*i)) {
		ROS_INFO("MINCELL  (x->%d , y->%d , cost -> %d)" , minCell.mx, minCell.my , minCell.cost);
		ROS_INFO("MINCELL  (costMap -> %d)" , gradient_.getCost(minCell.mx , minCell.my));
		plan.push_back(new_goal);
	}

	if (minCell.cost > EMPTY + SPACE + DELTA) {
		*i = *i + 1;
		path(plan , goalCellI , goalCellJ , goal , minCell , i);
	}

}



bool CSUROGlobalPlanner::makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal,  std::vector<geometry_msgs::PoseStamped>& plan)
{

	updateMap();

	ROS_INFO("CSURO Global Planner making plan.");

	// 2D Map points describes in x and y.
	float startX = start.pose.position.x;
	float startY = start.pose.position.y;
	float goalX  = goal.pose.position.x;
	float goalY  = goal.pose.position.y;

	ROS_INFO("New Goal coordinate (%lf, %lf) in %s", goalX, goalY, goal.header.frame_id.c_str());
	ROS_INFO("Coords vals in ([%lf, %lf], [%lf, %lf])",
			original_costmap_.getOriginX(), original_costmap_.getOriginY(),
			original_costmap_.getOriginX() + original_costmap_.getSizeInMetersX(),
			original_costmap_.getOriginY() + original_costmap_.getSizeInMetersY());


	int startCellI , startCellJ;
	int goalCellI , goalCellJ;

	original_costmap_.worldToMapEnforceBounds(goalX, goalY, goalCellI, goalCellJ);
	original_costmap_.worldToMapEnforceBounds(startX, startY, startCellI, startCellJ);

	if(!isCoordInMap(original_costmap_, goalCellI, goalCellJ))
	{
		ROS_ERROR("Coordinate (%lf, %lf) is outside of the map", goalX, goalY);
		return false;
	}


	// Compute gradient.
	TCell goalCell;

	goalCell.mx = goalCellI;
	goalCell.my = goalCellJ;
	goalCell.cost = 0;

	gradient_.setCost (goalCell.mx , goalCell.my , 0);

	cells.push_back(goalCell);
	flood(goalCell.mx , goalCell.my , startCellI , startCellJ);

	gradient_pub_.publishCostmap();


	// Compute path to do. :).
	TCell startCell;

	startCell.mx = startCellI;
	startCell.my = startCellJ;
	startCell.cost = gradient_.getCost(startCell.mx , startCell.my);


	int i;
	i = 0;

	plan.push_back(start);
	path(plan , goalCell.mx , goalCell.my , goal , startCell , &i);

/*
	geometry_msgs::PoseStamped new_goal = goal;
	tf::Quaternion goal_quat = tf::createQuaternionFromYaw(
			atan2 (goal.pose.position.y - start.pose.position.y, goal.pose.position.x - start.pose.position.x ) / 2.0 );

	new_goal.pose.position.x = (goal.pose.position.x + start.pose.position.x) / 2.0;
	new_goal.pose.position.y = (goal.pose.position.y + start.pose.position.y) / 2.0;
	new_goal.pose.orientation.x = goal_quat.x();
	new_goal.pose.orientation.y = goal_quat.y();
	new_goal.pose.orientation.z = goal_quat.z();
	new_goal.pose.orientation.w = goal_quat.w();
	plan.push_back(new_goal);
*/

	plan.push_back(goal);

	print(plan);

	ROS_INFO("CSURO Global Plan done");

	return true;
}

bool CSUROGlobalPlanner::isCoordInMap(const costmap_2d::Costmap2D &costmap, int x, int y)
{
	return x < costmap.getSizeInCellsX() && y < costmap.getSizeInCellsY();
}


};
