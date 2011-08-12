#include <golfcar_route_planner/route_planner.h>

using namespace std;

namespace route_planner {


RoutePlannerNode::RoutePlannerNode()
{
	station_path::stationPath sp;
	MoveBaseClient ac("move_base", true);
	ac_ = &ac;
	while(!ac.waitForServer(ros::Duration(5.0))){
		ROS_INFO("Waiting for the move_base action server to come up");
	}
	ros::NodeHandle n;
	waypoint_pub_ = n.advertise<geometry_msgs::PointStamped>("pnc_waypoint", 1);
	g_plan_pub_ = n.advertise<nav_msgs::Path>("pnc_globalplan", 1);
	pointCloud_pub_ = n.advertise<sensor_msgs::PointCloud>("pnc_waypointVis",1);
	//poseStamped_pub_ = n.advertise<geometry_msgs::PoseStamped>("move_base_simple/goal",1);
	nextpose_pub_ = n.advertise<geometry_msgs::PoseStamped>("pnc_nextpose",1);
	gps_sub_ = n.subscribe("fix", 1, &RoutePlannerNode::gpsCallBack, this);
	currentStationID_= 0;
	gpsCount_=0;
	

	int usrID, pickup, dropoff;


	
	currentStationID_ = 0;
	string station = "";
	cout<<"Current station?"<<endl;
	getline(cin, station);

	currentStationID_ = atoi(station.c_str());
	cout<<"Got it, my current staion is "<<currentStationID_<<endl;

	ROS_INFO("READY!");
	RoutePlannerNode::clearscreen();
	while (ros::ok()) {
		string temp = "";
		cout<<"Drop off?"<<endl;
		getline(cin, temp);
	
		dropoff = atoi(temp.c_str());
		pickup = currentStationID_;

		//new task obtain, start giving way points


		//go to the pickup point
		if(dropoff!=pickup)
		{
			sp.getPath(pickup, dropoff, targets_);
			RoutePlannerNode::publishPathVis();

			string input = "";
			ROS_INFO("Welcome %d! You have requested pickup at station %d to %d", usrID,pickup,dropoff);
			//ROS_INFO("Press enter key when you are ready!");
			//getline(cin, input);
			ROS_INFO("Let's go!");


			RoutePlannerNode::publish_goal(pickup, dropoff);
			RoutePlannerNode::startLoop(VEHICLE_POB, pickup, dropoff);

			//arrive at dropoff point, update current ID
			ROS_INFO("Arrive at dropoff point");
			currentStationID_ = dropoff;
		}
		//i'm become available again, preparing to get next task

		ros::spinOnce();
	}
}

RoutePlannerNode::~RoutePlannerNode()
{

}

void RoutePlannerNode::gpsCallBack(const sensor_msgs::NavSatFixConstPtr& fix)
{
	if (fix->status.status == sensor_msgs::NavSatStatus::STATUS_NO_FIX) {
	    ROS_INFO("No fix.");
	    return;
	  }

	  if (fix->header.stamp == ros::Time(0)) {
	    return;
	  }

	  double lat= fix->latitude;
	  double lon = fix->longitude;
	  if(gpsCount_%5==0)
	  {

		  ROS_INFO("Sent gps location %lf %lf", lat, lon);
	  }

	  gpsCount_++;
}

void RoutePlannerNode::clearscreen() {
if (system( "clear" )) system( "cls" );
}

void RoutePlannerNode::publishPathVis()
{
	nav_msgs::Path p;
	p.header.stamp = ros::Time::now();
	p.header.frame_id = "/map";
	p.poses.resize(targets_.size());
	for(int i=0;i<targets_.size();i++)
	{
		p.poses[i].pose.position.x = targets_[i].x;
		p.poses[i].pose.position.y = targets_[i].y;
		p.poses[i].pose.orientation.w = 1.0;
	}
	g_plan_pub_.publish(p);
}

void RoutePlannerNode::startLoop(VehicleStatus vehstatus,int dropoff, int pickup)
{
	WaypointNo_=0;

	while (ros::ok()){
		int station_distance = RoutePlannerNode::distance_to_goal();

		if(WaypointNo_==targets_.size()) break;
		if(vehstatus == VEHICLE_ON_CALL) ROS_DEBUG("Going to pick up station %d from station %d, distance to go %d m", dropoff, pickup, station_distance);
		if(vehstatus == VEHICLE_POB) ROS_DEBUG("Going to drop off station %d from station %d, distance to go %d m", dropoff, pickup, station_distance);

		RoutePlannerNode::waypoint_pub();
		ros::spinOnce();
		ros::Duration(0.2).sleep();
	}
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

void RoutePlannerNode::publish_goal(double pickup, double dropoff)
{
	//use more expansive action server to properly trace the goal status
	geometry_msgs::PoseStamped ps;
	ps.header.stamp = ros::Time::now();
	ps.header.frame_id = "/map";

	ps.pose.position.x = pickup;
	ps.pose.position.y = dropoff;
	ps.pose.orientation.w = 1.0;

	move_base_msgs::MoveBaseGoal goal;
	goal.target_pose = ps;
	ac_->sendGoal(goal);

}

void RoutePlannerNode::waypoint_pub()
{
	//get global pose

	RoutePlannerNode::getRobotGlobalPose(global_pose);

	double map_yaw=0;
	geometry_msgs::PoseStamped map_pose;
	map_pose.pose.position = targets_[WaypointNo_];
	if(WaypointNo_<targets_.size()-1)
		map_yaw = atan2(targets_[WaypointNo_+1].y- targets_[WaypointNo_].y, targets_[WaypointNo_+1].x- targets_[WaypointNo_].x);
	else
		map_yaw = atan2(targets_[WaypointNo_].y- targets_[WaypointNo_-1].y, targets_[WaypointNo_].x- targets_[WaypointNo_-1].x);

	map_pose.pose.orientation = tf::createQuaternionMsgFromYaw(map_yaw);

	//transform from pose to point, planner expect point z as yaw
	geometry_msgs::PointStamped odom_point;
	RoutePlannerNode::transformMapToOdom(map_pose, odom_point);

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
	double mapx = map_pose.pose.position.x, mapy = map_pose.pose.position.y;
	double robotx = global_pose.getOrigin().x(), roboty = global_pose.getOrigin().y();
	double distance = sqrt((mapx-robotx)*(mapx-robotx)+(mapy-roboty)*(mapy-roboty));

	if(distance < 4)	WaypointNo_++;


}

void RoutePlannerNode::transformMapToOdom(geometry_msgs::PoseStamped &map_pose, geometry_msgs::PointStamped &odom_point)
{
	map_pose.header.frame_id = "/map";
	map_pose.header.stamp = ros::Time();
	geometry_msgs::PoseStamped odom_pose;

	try {
		tf_.transformPose("/odom", map_pose, odom_pose);
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

	odom_point.header = odom_pose.header;
	odom_point.point = odom_pose.pose.position;
	odom_point.point.z = tf::getYaw(odom_pose.pose.orientation);

	nextpose_pub_.publish(odom_pose);
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


