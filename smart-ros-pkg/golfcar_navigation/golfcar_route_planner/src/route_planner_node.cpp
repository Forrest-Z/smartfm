#include <math.h>

#include <string>
#include <cmath>
#include <iostream>

using namespace std;

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <sensor_msgs/PointCloud.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

#include <golfcar_route_planner/station_path.h>


enum VehicleStatus
{
    VEHICLE_NOT_AVAILABLE,
    VEHICLE_ON_CALL,
    VEHICLE_POB,
    VEHICLE_BUSY, // either not available, on call, or pob
    VEHICLE_AVAILABLE
};



typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;


class RoutePlannerNode
{
public:
    RoutePlannerNode(bool standalone);
    void spin();

private:
    ros::NodeHandle n;
    ros::Publisher waypoint_pub_;
    ros::Publisher g_plan_pub_;
    ros::Publisher pointCloud_pub_;
    ros::Publisher poseStamped_pub_;
    ros::Publisher nextpose_pub_;
    ros::Subscriber gps_sub_;

    ros::Timer timer_;
    tf::TransformListener tf_;

    StationPaths sp_;
    StationPath targets_;

    tf::Stamped<tf::Pose> global_pose_;

    MoveBaseClient ac_;

    bool standalone_;
    unsigned waypointNo_;

    //assume that the vehicle will always stop at one of the station
    int currentStationID_;

    string usrID_;
    int dropoff_, pickup_;

    void pubWaypoint();
    void pubPathVis();
    void clearScreen();
    void publishGoal(int start, int end);
    bool getRobotGlobalPose();
    void transformMapToOdom(geometry_msgs::PoseStamped *map_pose, geometry_msgs::PointStamped *odom_point);
    double distanceToGoal();
    void waypointLoop(VehicleStatus vehstatus, int start, int end);
    void promptForCurrentStation();
    void promptForMission();
};


RoutePlannerNode::RoutePlannerNode(bool standalone)
    : ac_("move_base", true), standalone_(standalone)
{
    while( ! ac_.waitForServer(ros::Duration(5.0)) )
        ROS_INFO("Waiting for the move_base action server to come up");

    waypoint_pub_ = n.advertise<geometry_msgs::PointStamped>("pnc_waypoint", 1);
    g_plan_pub_ = n.advertise<nav_msgs::Path>("pnc_globalplan", 1);
    pointCloud_pub_ = n.advertise<sensor_msgs::PointCloud>("pnc_waypointVis",1);
    //poseStamped_pub_ = n.advertise<geometry_msgs::PoseStamped>("move_base_simple/goal",1);
    nextpose_pub_ = n.advertise<geometry_msgs::PoseStamped>("pnc_nextpose",1);
}


void RoutePlannerNode::promptForCurrentStation()
{
    string station = "";
    cout <<"Current station?" <<endl;
    getline(cin, station);

    currentStationID_ = atoi(station.c_str());
    cout <<"Got it, my current station is " <<currentStationID_ <<endl;

    ROS_INFO("READY!");
    clearScreen();
}


void RoutePlannerNode::promptForMission()
{
    string temp = "";

    cout <<"Pick up?" <<endl;
    getline(cin, temp);
    pickup_ = atoi(temp.c_str());

    cout <<"Drop off?" <<endl;
    getline(cin, temp);
    dropoff_ = atoi(temp.c_str());
}


void RoutePlannerNode::spin()
{
    promptForCurrentStation();

    while ( ros::ok() )
    {
        int pickup_ = currentStationID_;
        if( standalone_ ) {
            promptForMission();
        }
        else {
            ; //get dropoff and pickup from dbserver
        }

        if(currentStationID_ != pickup_)
        {
            ROS_INFO("On call... pickup at station %d from %d", pickup_, currentStationID_);
            targets_ = sp_.getPath(sp_.knownStations()(currentStationID_), sp_.knownStations()(pickup_));
            pubPathVis();
            publishGoal(currentStationID_, pickup_);
            waypointLoop(VEHICLE_ON_CALL, currentStationID_, pickup_);

            //arrive at pickup point
            ROS_INFO("Arrived at pickup point");
            currentStationID_ = pickup_;
        }

        targets_ = sp_.getPath(sp_.knownStations()(currentStationID_), sp_.knownStations()(dropoff_));
        pubPathVis();

        string input = "";
        ROS_INFO("Going to dropoff station %d", dropoff_);
        ROS_INFO("Press enter key when you are ready!");
        getline(cin, input);
        ROS_INFO("Let's go!");

        publishGoal(currentStationID_, dropoff_);
        waypointLoop(VEHICLE_POB, currentStationID_, dropoff_);

        //arrive at dropoff point, update current ID
        ROS_INFO("Arrive at dropoff point");
        currentStationID_ = dropoff_;
        ROS_INFO("Press enter key when you are outside the vehicle!");
        getline(cin, input);

        //I became available again, preparing to get next task
        if( ! standalone_ ) {
            ; // TODO: update status on dbserver
        }

        ROS_INFO("Waiting for next task. Prepared to move to a pick up point at any time");
        ros::spinOnce();
    }
}



void RoutePlannerNode::clearScreen() {
    // Try the "clear" command. If it does not exist, "system" returns non zero,
    // in which case we try with "cls".
    int ret;
    if( (ret = system("clear")) != 0 )
        ret = system("cls");
}

void RoutePlannerNode::pubPathVis()
{
    nav_msgs::Path p;
    p.header.stamp = ros::Time::now();
    p.header.frame_id = "/map";
    p.poses.resize(targets_.size());
    for( unsigned i=0; i<targets_.size(); i++ )
    {
        p.poses[i].pose.position.x = targets_[i].x;
        p.poses[i].pose.position.y = targets_[i].y;
        p.poses[i].pose.orientation.w = 1.0;
    }
    g_plan_pub_.publish(p);
}

void RoutePlannerNode::waypointLoop(VehicleStatus vehstatus, int start, int end)
{
    waypointNo_ = 0;

    while (ros::ok())
    {
        double d = distanceToGoal();

        if( waypointNo_ == targets_.size() )
            break;
        if( vehstatus == VEHICLE_ON_CALL )
            ROS_DEBUG("Going to pick up station %d from station %d, distance to go %.0f m", end, start, d);
        if( vehstatus == VEHICLE_POB )
            ROS_DEBUG("Going to drop off station %d from station %d, distance to go %.0f m", end, start, d);

        pubWaypoint();
        ros::spinOnce();
        ros::Duration(0.2).sleep();
    }
}

double RoutePlannerNode::distanceToGoal()
{
    double d = 0;

    for( unsigned i = waypointNo_+1; i<targets_.size(); i++ )
        d += station_path::distance(targets_[i-1], targets_[i]);

    geometry_msgs::Point p;
    p.x = global_pose_.getOrigin().x();
    p.y = global_pose_.getOrigin().y();
    d += station_path::distance(p, targets_[waypointNo_]);

    return d;
}


void RoutePlannerNode::publishGoal(int start, int end)
{
    //use more expansive action server to properly trace the goal status
    geometry_msgs::PoseStamped ps;
    ps.header.stamp = ros::Time::now();
    ps.header.frame_id = "/map";

    ps.pose.position.x = (double) start;
    ps.pose.position.y = (double) end;
    ps.pose.orientation.w = 1.0;

    move_base_msgs::MoveBaseGoal goal;
    goal.target_pose = ps;
    ac_.sendGoal(goal);

}

void RoutePlannerNode::pubWaypoint()
{
    //get global pose
    getRobotGlobalPose();

    double map_yaw=0;
    geometry_msgs::PoseStamped map_pose;
    map_pose.pose.position = targets_[waypointNo_];
    if( waypointNo_ < targets_.size()-1 ) {
        map_yaw = atan2(targets_[waypointNo_+1].y - targets_[waypointNo_].y, targets_[waypointNo_+1].x - targets_[waypointNo_].x);
    }
    else {
        assert(waypointNo_!=0);
        map_yaw = atan2(targets_[waypointNo_].y - targets_[waypointNo_-1].y, targets_[waypointNo_].x - targets_[waypointNo_-1].x);
    }

    map_pose.pose.orientation = tf::createQuaternionMsgFromYaw(map_yaw);

    //transform from pose to point, planner expect point z as yaw
    geometry_msgs::PointStamped odom_point;
    transformMapToOdom(&map_pose, &odom_point);
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
    double robotx = global_pose_.getOrigin().x(), roboty = global_pose_.getOrigin().y();
    double d = sqrt((mapx-robotx)*(mapx-robotx)+(mapy-roboty)*(mapy-roboty));
    if( d < 4 )
        waypointNo_++;
}

void RoutePlannerNode::transformMapToOdom(geometry_msgs::PoseStamped *map_pose, geometry_msgs::PointStamped *odom_point)
{
    map_pose->header.frame_id = "/map";
    map_pose->header.stamp = ros::Time();
    geometry_msgs::PoseStamped odom_pose;

    try {
        tf_.transformPose("/odom", *map_pose, odom_pose);
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

    odom_point->header = odom_pose.header;
    odom_point->point = odom_pose.pose.position;
    odom_point->point.z = tf::getYaw(odom_pose.pose.orientation);

    nextpose_pub_.publish(odom_pose);
}


bool RoutePlannerNode::getRobotGlobalPose()
{
    global_pose_.setIdentity();
    tf::Stamped<tf::Pose> robot_pose;
    robot_pose.setIdentity();
    robot_pose.frame_id_ = "/base_link";
    robot_pose.stamp_ = ros::Time();
    ros::Time current_time = ros::Time::now(); // save time for checking tf delay later

    try {
        tf_.transformPose("/map", robot_pose, global_pose_);
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



int main(int argc, char **argv)
{
    ros::init(argc, argv, "route_planner_node");
    ros::NodeHandle n;
    bool standalone;
    n.param<bool>("standalone", standalone, true);
    RoutePlannerNode rpn(standalone);
    rpn.spin();
    return 0;
}
