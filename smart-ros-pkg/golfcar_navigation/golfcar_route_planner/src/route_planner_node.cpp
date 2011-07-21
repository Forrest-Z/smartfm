#include <route_planner.h>

using namespace std;

namespace route_planner {


RoutePlannerNode::RoutePlannerNode()
{
	station_path::stationPath sp;

	ros::NodeHandle n;
	waypoint_pub_ = n.advertise<geometry_msgs::PointStamped>("pnc_waypoint", 1);
	g_plan_pub_ = n.advertise<nav_msgs::Path>("pnc_globalplan", 1);
	pointCloud_pub_ = n.advertise<sensor_msgs::PointCloud>("pnc_waypointVis",1);
	poseStamped_pub_ = n.advertise<geometry_msgs::PoseStamped>("move_base_simple/goal",1);
	currentStationID_= 0;

	RoutePlanner rp("localhost", 8888);

	rp.sendStatus(0, VEHICLE_AVAILABLE, 0);

	int usrID, pickup, dropoff;

	while (1) {

		rp.getNewTask(usrID, pickup, dropoff);
		//the station starts with 1 as pickup dropoff point
		pickup--;dropoff--;
		cout<<"Welcome "<<usrID<<"!"<<" You have requested pickup at station " <<' '<<pickup<<" to "<<dropoff<<endl;
		//new task obtain, start giving way points


		//go to the pickup point
		if(dropoff!=pickup)
		{
			if(currentStationID_!=pickup)
			{
				sp.getPath(currentStationID_, pickup, targets_ );
				WaypointNo_=0;

				while (1){
					RoutePlannerNode::waypoint_pub_loop();
					int station_distance = RoutePlannerNode::distance_to_goal();
					rp.sendStatus(0, VEHICLE_ON_CALL, station_distance);

					if(WaypointNo_==targets_.size()) break;
					ROS_INFO("Going to pickup station %d from station %d, distance to go %d m", pickup, currentStationID_, station_distance);
					ros::Duration(0.1).sleep();
				}
				//arrive at pickup point
				cout<<"Arrive at pickup point"<<endl;

			}
			string input = "";
			cout<<"Welcome "<<usrID<<"!"<<" You have requested pickup at station " <<' '<<pickup<<" to "<<dropoff<<endl;
			cout << "Press enter key when you are ready!"<<endl;
			getline(cin, input);
			cout << "Let's go! "<<endl;
			sp.getPath(pickup, dropoff, targets_);
			WaypointNo_=0;


			while (1){
				RoutePlannerNode::waypoint_pub_loop();
				int station_distance = RoutePlannerNode::distance_to_goal();
				rp.sendStatus(0, VEHICLE_POB, station_distance );

				if(WaypointNo_==targets_.size()) break;
				ROS_INFO("Going to dropoff station %d from station %d, distance to go %d m", dropoff, pickup, station_distance);
				ros::Duration(0.1).sleep();
			}
			//arrive at dropoff point, update current ID
			ROS_INFO("Arrive at dropoff point");
			currentStationID_ = dropoff;
			cout << "Press enter key when you are outside the vehicle!"<<endl;
			getline(cin, input);
		}
		//i'm become available again, preparing to get next task
		rp.sendStatus(0, VEHICLE_AVAILABLE, 0);
		cout << "I'm Available, and might move to a pick up point at any time"<<endl;
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

	if(distance < 3.7)	WaypointNo_++;

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


