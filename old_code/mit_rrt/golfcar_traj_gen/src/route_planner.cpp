/*
 * route_planner.cpp
 *
 *  Created on: Aug 21, 2012
 *      Author: demian
 */

#include <math.h>

#include <string>
#include <cmath>
#include <iostream>

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <sensor_msgs/PointCloud.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <StationPath.h>
#include <fmutil/fm_math.h>

/// Drives the vehicle from A to B by feeding it a sequence of waypoints.
class RoutePlanner
{

public:
	RoutePlanner(const int start, const int end);

private:
    ros::NodeHandle n;
    ros::Publisher g_plan_pub_;
    ros::Publisher nextpose_pub_;

    tf::TransformListener tf_;
    tf::Stamped<tf::Pose> global_pose_;
    StationPaths sp_;

    StationPath path_;
    Station destination_;

    unsigned waypointNo_;

    bool goToDest();
    void initDest(const int start, const int end);

    void pubPathVis();
    bool getRobotGlobalPose();
    void transformMapToOdom(geometry_msgs::PoseStamped *map_pose,
                            geometry_msgs::PointStamped *odom_point);
    double distanceToGoal();
};

RoutePlanner::RoutePlanner(const int start, const int end)
{
    g_plan_pub_ = n.advertise<nav_msgs::Path>("pnc_globalplan", 1, true);
    nextpose_pub_ = n.advertise<geometry_msgs::PoseStamped>("pnc_nextpose",1);

    ros::Rate loop_rate(10);
    int count=0;
    initDest(start, end);
    bool initialized = false;
    while(ros::ok())
    {
    	if(initialized)
    		goToDest();
    	else
    	{
    		if(getRobotGlobalPose())
    		{
    			initDest(start, end);
    			//search for the path nearest to the car current pose
    			double min_dist = std::numeric_limits<double>::max();
    			for( unsigned i=0; i<path_.size(); i++ )
    			{
    				double dist = fmutil::distance(global_pose_.getOrigin().x(), global_pose_.getOrigin().y(), path_[i].x_, path_[i].y_);
    				if(dist<min_dist)
    				{
    					waypointNo_ = i;
    					min_dist = dist;
    				}
    			}
    			initialized = true;
    		}

    	}
    	ros::spinOnce();
    	loop_rate.sleep();
    }
}


void RoutePlanner::initDest(const int start, const int end)
{
    destination_ = sp_.knownStations()(end);
    path_ = sp_.getPath(sp_.knownStations()(start),sp_.knownStations()(end));
    waypointNo_ = 0;

    // publish the path (for visualization)
    nav_msgs::Path p;
    p.header.stamp = ros::Time::now();
    p.header.frame_id = "/map";
    p.poses.resize(path_.size());
    for( unsigned i=0; i<path_.size(); i++ )
    {
        p.poses[i].pose.position.x = path_[i].x_;
        p.poses[i].pose.position.y = path_[i].y_;
        p.poses[i].pose.orientation.w = 1.0;
    }
    g_plan_pub_.publish(p);

}


bool RoutePlanner::goToDest()
{
    getRobotGlobalPose();

    ROS_INFO_THROTTLE(3, "Going to %s. Distance=%.0f.", destination_.c_str(), distanceToGoal());

    geometry_msgs::PoseStamped map_pose;
    map_pose.pose.position.x = path_[waypointNo_].x_;
    map_pose.pose.position.y = path_[waypointNo_].y_;
    double map_yaw = 0;
    if( waypointNo_ < path_.size()-1 ) {
        map_yaw = atan2(path_[waypointNo_+1].y_ - path_[waypointNo_].y_, path_[waypointNo_+1].x_ - path_[waypointNo_].x_);
    }
    else {
        assert(waypointNo_!=0);
        map_yaw = atan2(path_[waypointNo_].y_ - path_[waypointNo_-1].y_, path_[waypointNo_].x_ - path_[waypointNo_-1].x_);
    }

    map_pose.pose.orientation = tf::createQuaternionMsgFromYaw(map_yaw);

    //transform from pose to point, planner expect point z as yaw
    //publish the first waypoint in map frame then continue to send the points until the last one
    map_pose.header.frame_id="/map";
    map_pose.header.stamp=ros::Time::now();
    nextpose_pub_.publish(map_pose);

    //get how near it is to the goal point, if reaches the threshold, send the next point
    double mapx = map_pose.pose.position.x, mapy = map_pose.pose.position.y;
    double robotx = global_pose_.getOrigin().x(), roboty = global_pose_.getOrigin().y();
    double d = sqrt(pow(mapx-robotx,2)+pow(mapy-roboty,2));

    if( waypointNo_ < path_.size()-1 && d < 4 )
        waypointNo_++;
    else if( waypointNo_ == path_.size()-1 )
        return true;

    return false;
}

double RoutePlanner::distanceToGoal()
{
    double d = 0;

    for( unsigned i = waypointNo_+1; i<path_.size(); i++ )
        d += PathPoint::distance(path_[i-1], path_[i]);

    PathPoint p(global_pose_.getOrigin().x(), global_pose_.getOrigin().y());
    d += PathPoint::distance(p, path_[waypointNo_]);

    return d;
}



bool RoutePlanner::getRobotGlobalPose()
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


int main(int argc, char** argcv)
{
	ros::init(argc, argcv, "route_planner");
	if(argc<3)
		std::cout<<"Usage: route_planner start end"<<std::endl;
	else
		RoutePlanner rp(atoi(argcv[1]), atoi(argcv[2]));
	return 0;

}
