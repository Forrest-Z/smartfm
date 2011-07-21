#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/PointStamped.h>
#include <string>
#include <cmath>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/PointCloud.h>

#include "RoutePlanner.hh"
#include <iostream>


using namespace std;

namespace station_path {
const int station_number = 4;

int path_points_01[][2] {{1077, 2158}, {1032, 2199}, {1047, 2221}, {1056, 2269}, {1458, 3072}, {1461, 3081}, {1503, 3096},  {1609, 3118}, {1627, 3201}, {1585, 3255}, {1526, 3243}, {1489, 3206}, {1473, 3137}};
int path_points_02[][2] {{970,2109}, {932,2056}, {887,2020}, {830,2001}, {769,1998}, {728,2006}, {673,2032}, {632,2064}, {598,2100}, {566,2147}, {536,2192}, {517,2239}, {507,2271}, {469,2318}, {412,2334}, {363,2347}, {324, 2331}, {302, 2285}, {300, 2216}, {346, 1722}, {374, 1224}, {386, 1165}, {563, 672}, {726, 344}, {781, 245}, {837, 213}, {894, 217}, {1290, 370}, {1206, 442}, {1170, 432}, {1116, 412}};
int path_points_03[][2] {{970,2109}, {932,2056}, {887,2020}, {830,2001}, {769,1998}, {728,2006}, {673,2032}, {632,2064}, {598,2100}, {566,2147}, {536,2192}, {517,2239}, {507,2271}, {469,2318}, {412,2334}, {363,2347}, {324, 2331}, {302, 2285}, {300, 2216}, {346, 1722}, {374, 1224}, {386, 1165}, {563, 672}, {726, 344}, {781, 245}, {837, 213}, {894, 217}, {1423, 415}, {1856, 563}, {1938, 596}, {1994, 612}, {2010, 638}, {1986, 659}, {1951, 680}, {1923, 680}, {1881, 695}};
int path_points_10[][2] {{1432,3061}, {1093,2349}, {1067,2272}, {1015,2179}, {1036, 2142}, {1064, 2126}, {1076, 2148}};
int path_points_12[][2] {{1432,3061}, {1093,2349}, {1067,2272}, {1015,2179}, {970,2109}, {932,2056}, {887,2020}, {830,2001}, {769,1998}, {728,2006}, {673,2032}, {632,2064}, {598,2100}, {566,2147}, {536,2192}, {517,2239}, {507,2271}, {469,2318}, {412,2334}, {363,2347}, {324, 2331}, {302, 2285}, {300, 2216}, {346, 1722}, {374, 1224}, {386, 1165}, {563, 672}, {726, 344}, {781, 245}, {837, 213}, {894, 217}, {1290, 370}, {1206, 442}, {1170, 432}, {1116, 412}};
int path_points_13[][2] {{1432,3061}, {1093,2349}, {1067,2272}, {1015,2179}, {970,2109}, {932,2056}, {887,2020}, {830,2001}, {769,1998}, {728,2006}, {673,2032}, {632,2064}, {598,2100}, {566,2147}, {536,2192}, {517,2239}, {507,2271}, {469,2318}, {412,2334}, {363,2347}, {324, 2331}, {302, 2285}, {300, 2216}, {346, 1722}, {374, 1224}, {386, 1165}, {563, 672}, {726, 344}, {781, 245}, {837, 213}, {894, 217}, {1423, 415}, {1856, 563}, {1938, 596}, {1994, 612}, {2010, 638}, {1986, 659}, {1951, 680}, {1923, 680}, {1881, 695}};
int path_points_20[][2] {{1054, 348}, {1036, 304}, {1008, 276}, {956,254}, {888, 242}, {843, 229},{813,240}, {793, 263}, {774, 285}, {746, 353}, {687, 458}, {588, 683}, {497, 912}, {410, 1175}, {399, 1229}, {374, 1721}, {325, 2217}, {334, 2270}, {360, 2306}, {440, 2288}, {481, 2249}, {522, 2186}, {558, 2131}, {588, 2084}, {622, 2047}, {663, 2010}, {720, 1986}, {768, 1985}, {833, 1982}, {892, 2008}, {943, 2044}, {983, 2099}, {1009, 2130}, {1053, 2122}, {1077, 2159}};
int path_points_21[][2] {{1054, 348}, {1036, 304}, {1008, 276}, {956,254}, {888, 242}, {843, 229},{813,240}, {793, 263}, {774, 285}, {746, 353}, {687, 458}, {588, 683}, {497, 912}, {410, 1175}, {399, 1229}, {374, 1721}, {325, 2217}, {334, 2270}, {360, 2306}, {440, 2288}, {481, 2249}, {522, 2186}, {558, 2131}, {588, 2084}, {622, 2047}, {663, 2010}, {720, 1986}, {768, 1985}, {833, 1982}, {892, 2008}, {943, 2044}, {983, 2099}, {1009, 2130}, {1053, 2122}, {1077, 2159}, {1032, 2199}, {1047, 2221}, {1056, 2269}, {1458, 3072}, {1461, 3081}, {1503, 3096},  {1609, 3118}, {1627, 3201}, {1585, 3255}, {1526, 3243}, {1489, 3206}, {1473, 3137}};
int path_points_23[][2] {{1054, 348}, {1036, 304}, {1086, 304}, {956,254}, {1423, 415}, {1856, 563}, {1938, 596}, {1994, 612}, {2010, 638}, {1986, 659}, {1951, 680}, {1923, 680}, {1881, 695}};
int path_points_30[][2] {{ 1833, 650}, {1800, 610}, {1789, 573}, {1740, 545}, {1415, 434}, {888, 242}, {843, 229},{813,240}, {793, 263}, {774, 285}, {746, 353}, {687, 458}, {588, 683}, {497, 912}, {410, 1175}, {399, 1229}, {374, 1721}, {325, 2217}, {334, 2270}, {360, 2306}, {440, 2288}, {481, 2249}, {522, 2186}, {558, 2131}, {588, 2084}, {622, 2047}, {663, 2010}, {720, 1986}, {768, 1985}, {833, 1982}, {892, 2008}, {943, 2044}, {983, 2099}, {1009, 2130}, {1053, 2122}, {1077, 2159}};
int path_points_31[][2] {{ 1833, 650}, {1800, 610}, {1789, 573}, {1740, 545}, {1415, 434}, {888, 242}, {843, 229},{813,240}, {793, 263}, {774, 285}, {746, 353}, {687, 458}, {588, 683}, {497, 912}, {410, 1175}, {399, 1229}, {374, 1721}, {325, 2217}, {334, 2270}, {360, 2306}, {440, 2288}, {481, 2249}, {522, 2186}, {558, 2131}, {588, 2084}, {622, 2047}, {663, 2010}, {720, 1986}, {768, 1985}, {833, 1982}, {892, 2008}, {943, 2044}, {983, 2099}, {1009, 2130}, {1053, 2122}, {1077, 2159}, {1032, 2199}, {1047, 2221}, {1056, 2269}, {1458, 3072}, {1461, 3081}, {1503, 3096},  {1609, 3118}, {1627, 3201}, {1585, 3255}, {1526, 3243}, {1489, 3206}, {1473, 3137}};
int path_points_32[][2] {{ 1833, 650}, {1800, 610}, {1789, 573}, {1740, 545}, {1298, 400}, {1206, 442}, {1172, 432}, {1116, 412}};

vector<vector<vector<geometry_msgs::Point> > > station_paths;

void storeIntoStationPaths(int path_points[][2], vector<geometry_msgs::Point> &station_paths, int size)
{
	//this function will convert the points from pixel number to points in the map frame
	double res = 0.1;
	int y_pixels = 3536;
	double distance=0;

	for(unsigned int i=0; i<size;i++)
	{
		geometry_msgs::Point p;
		p.x = path_points[i][0]*res;
		p.y = (3536 - path_points[i][1])*res;
		station_paths.push_back(p);

		if(i>0)
		{
			double x_cur = station_paths[i].x, x_pre = station_paths[i-1].x;
			double y_cur = station_paths[i].y, y_pre = station_paths[i-1].y;
			distance+=sqrt((x_cur-x_pre)*(x_cur-x_pre)+(y_cur-y_pre)*(y_cur-y_pre));
		}
	}
	cout<<"Distance= " <<distance<<endl;
}

void initialize()
{
	station_paths.resize(station_number);
	for(int i=0; i<station_number;i++)
		station_paths[i].resize(station_number);

	storeIntoStationPaths(path_points_01, station_paths[0][1],sizeof(path_points_01)/sizeof(path_points_01[0]));
	storeIntoStationPaths(path_points_02, station_paths[0][2],sizeof(path_points_02)/sizeof(path_points_02[0]));
	storeIntoStationPaths(path_points_03, station_paths[0][3],sizeof(path_points_03)/sizeof(path_points_03[0]));
	storeIntoStationPaths(path_points_10, station_paths[1][0],sizeof(path_points_10)/sizeof(path_points_10[0]));
	storeIntoStationPaths(path_points_12, station_paths[1][2],sizeof(path_points_12)/sizeof(path_points_12[0]));
	storeIntoStationPaths(path_points_13, station_paths[1][3],sizeof(path_points_13)/sizeof(path_points_13[0]));
	storeIntoStationPaths(path_points_20, station_paths[2][0],sizeof(path_points_20)/sizeof(path_points_20[0]));
	storeIntoStationPaths(path_points_21, station_paths[2][1],sizeof(path_points_21)/sizeof(path_points_21[0]));
	storeIntoStationPaths(path_points_23, station_paths[2][3],sizeof(path_points_23)/sizeof(path_points_23[0]));
	storeIntoStationPaths(path_points_30, station_paths[3][0],sizeof(path_points_30)/sizeof(path_points_30[0]));
	storeIntoStationPaths(path_points_31, station_paths[3][1],sizeof(path_points_31)/sizeof(path_points_31[0]));
	storeIntoStationPaths(path_points_32, station_paths[3][2],sizeof(path_points_32)/sizeof(path_points_32[0]));
}


vector<geometry_msgs::Point> getPath(int stationPickUp, int stationDropOff)
{
	//some process to process it into normal ROS compatible format
	return station_paths[stationPickUp][stationDropOff];
}

}

namespace route_planner {

class RoutePlannerNode
{
public:
	RoutePlannerNode();
	~RoutePlannerNode();

private:
	ros::Publisher waypoint_pub_;
	ros::Publisher g_plan_pub_;
	ros::Publisher pointCloud_pub_;
	ros::Publisher poseStamped_pub_;

	ros::Timer timer_;
	tf::TransformListener tf_;
	void waypoint_pub_loop();
	bool getRobotGlobalPose(tf::Stamped<tf::Pose>& odom_pose) const;
	int distance_to_goal();
	std::vector<geometry_msgs::Point> targets_;
	int WaypointNo_;
	//assume that the vehicle will always stop at one of the station
	int currentStationID_;
	tf::Stamped<tf::Pose> global_pose;
};
};

namespace route_planner {


RoutePlannerNode::RoutePlannerNode()
{
	station_path::initialize();

	ros::NodeHandle n;
	waypoint_pub_ = n.advertise<geometry_msgs::PointStamped>("pnc_waypoint", 1);
	g_plan_pub_ = n.advertise<nav_msgs::Path>("pnc_globalplan", 1);
	pointCloud_pub_ = n.advertise<sensor_msgs::PointCloud>("pnc_waypointVis",1);
	poseStamped_pub_ = n.advertise<geometry_msgs::PoseStamped>("move_base_simple/goal",1);
	currentStationID_= 0;

	RoutePlanner rp("172.29.147.148", 8888);

	rp.sendStatus(0, VEHICLE_AVAILABLE, 0);

	int usrID, pickup, dropoff;

	while (1) {

		rp.getNewTask(usrID, pickup, dropoff);
		//the station starts with 1 as pickup dropoff point
		pickup--;dropoff--;
		cout<<usrID<<' '<<pickup<<' '<<dropoff<<endl;
		//new task obtain, start giving way points


		//go to the pickup point
		if(dropoff!=pickup)
		{
			if(currentStationID_!=pickup)
			{
				targets_ = station_path::getPath(currentStationID_, pickup);
				WaypointNo_=0;
				ROS_INFO("Going to pickup station %d from station %d Going into loop", pickup, currentStationID_);
				while (1){

					RoutePlannerNode::waypoint_pub_loop();
					//right now just assume every point takes one minute to travel
					int remaining_time_est = (targets_.size()-WaypointNo_);
					rp.sendStatus(0, VEHICLE_ON_CALL, RoutePlannerNode::distance_to_goal() );
					if(WaypointNo_==targets_.size()) break;
					cout<<RoutePlannerNode::distance_to_goal()<<endl;
					ros::Duration(0.1).sleep();
				}
				//arrive at pickup point
				cout<<"Arrive at pickup point"<<endl;

			}
			string input = "";
			cout << "Press enter key when you are ready!"<<endl;
			getline(cin, input);
			cout << "Let's go! "<<endl;
			targets_ = station_path::getPath(pickup, dropoff);
			WaypointNo_=0;
			ROS_INFO("Going to dropoff station %d from station %d Going into loop", dropoff, pickup);

			while (1){
				RoutePlannerNode::waypoint_pub_loop();
				//right now just assume every point takes one minute to travel
				int remaining_time_est = (targets_.size()-WaypointNo_);
				rp.sendStatus(0, VEHICLE_POB, RoutePlannerNode::distance_to_goal() );
				if(WaypointNo_==targets_.size()) break;
				cout<<RoutePlannerNode::distance_to_goal()<<endl;
				ros::Duration(0.1).sleep();
			}
			//arrive at dropoff point, update current ID
			ROS_INFO("Arrive at dropoff point");
			currentStationID_ = dropoff;
			cout << "Press enter key when you are outside the vehicle!"<<endl;
			getline(cin, input);
		}
		//i'm become available again
		rp.sendStatus(0, VEHICLE_AVAILABLE, 0);
		cout << "I'm Available, and might move to a pick up at any time"<<endl;
		ros::spinOnce();
	}
}

RoutePlannerNode::~RoutePlannerNode()
{

}

int RoutePlannerNode::distance_to_goal()
{
	double distance=0;

	for(int i= WaypointNo_; i<targets_.size()-1;i++)
	{
		distance += sqrt((targets_[i+1].x - targets_[i].x)*(targets_[i+1].x-targets_[i].x) +(targets_[i+1].y - targets_[i].y)*(targets_[i+1].y-targets_[i].y));
	}
	distance+= sqrt((global_pose.getOrigin().x()-targets_[WaypointNo_].x)*(global_pose.getOrigin().x()-targets_[WaypointNo_].x) + (global_pose.getOrigin().y()-targets_[WaypointNo_].y)*(global_pose.getOrigin().y()-targets_[WaypointNo_].y));

	return (int)distance;
}
void RoutePlannerNode::waypoint_pub_loop()
{
	//get global pose

	RoutePlannerNode::getRobotGlobalPose(global_pose);

	geometry_msgs::PointStamped map_point;
	geometry_msgs::PointStamped odom_point;
	map_point.header.frame_id = "/map";
	map_point.header.stamp = ros::Time();
	map_point.point = targets_[WaypointNo_];

	try {
		tf_.transformPoint("/odom", map_point, odom_point);
	}
	catch(tf::LookupException& ex) {
		ROS_ERROR("No Transform available Error: %s\n", ex.what());
	}
	catch(tf::ConnectivityException& ex) {
		ROS_ERROR("Connectivity Error: %s\n", ex.what());
	}
	catch(tf::ExtrapolationException& ex) {
		ROS_ERROR("Extrapolation Error: %s\n", ex.what());
	}
	//publish the first waypoint in odom frame then continue to send the points until the last one
	waypoint_pub_.publish(odom_point);
	sensor_msgs::PointCloud pc;
	pc.header.stamp = ros::Time::now();
	pc.header.frame_id = "/odom";
	pc.points.resize(1);
	pc.points[0].x = odom_point.point.x;
	pc.points[0].y = odom_point.point.y;
	pointCloud_pub_.publish(pc);
	//get how near it is to the goal point, if reaches the threshold, send the next point
	double mapx = map_point.point.x, mapy = map_point.point.y;
	double robotx = global_pose.getOrigin().x(), roboty = global_pose.getOrigin().y();
	double distance = sqrt((mapx-robotx)*(mapx-robotx)+(mapy-roboty)*(mapy-roboty));

	if(distance < 3.7)
	{
		WaypointNo_++;
		ROS_INFO("Status %d/%d", WaypointNo_, targets_.size());
	}

	geometry_msgs::PoseStamped ps;
	ps.header.stamp = ros::Time::now();
	ps.header.frame_id = "/map";

	ps.pose.position.x = map_point.point.x;
	ps.pose.position.y = map_point.point.y;
	ps.pose.orientation.w = 1.0;

	poseStamped_pub_.publish(ps);
}

bool RoutePlannerNode::getRobotGlobalPose(tf::Stamped<tf::Pose>& odom_pose) const
{
	odom_pose.setIdentity();
	tf::Stamped<tf::Pose> robot_pose;
	robot_pose.setIdentity();
	robot_pose.frame_id_ = "/base_link";
	robot_pose.stamp_ = ros::Time();
	ros::Time current_time = ros::Time::now(); // save time for checking tf delay later

	try {
		tf_.transformPose("/map", robot_pose, odom_pose);
	}
	catch(tf::LookupException& ex) {
		ROS_ERROR("No Transform available Error: %s\n", ex.what());
		return false;
	}
	catch(tf::ConnectivityException& ex) {
		ROS_ERROR("Connectivity Error: %s\n", ex.what());
		return false;
	}
	catch(tf::ExtrapolationException& ex) {
		ROS_ERROR("Extrapolation Error: %s\n", ex.what());
		return false;
	}
	// check odom_pose timeout
	if (current_time.toSec() - odom_pose.stamp_.toSec() > 0.1) {
		//ROS_WARN("PurePursuit transform timeout. Current time: %.4f, odom_pose stamp: %.4f, tolerance: %.4f",
			//	current_time.toSec(), odom_pose.stamp_.toSec(), 0.1);
		return false;
	}

	return true;
}
};

int main(int argc, char **argv)
{
	ros::init(argc, argv, "golfcar_route_planner");
	route_planner::RoutePlannerNode *rp = new route_planner::RoutePlannerNode();
	if(!rp) {
		ROS_ERROR("failed to start the process\n");
		return 1;
	}

	ros::spin();

	return 0;
}	
