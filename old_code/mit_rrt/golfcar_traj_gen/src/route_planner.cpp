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
#include <svg_path/StationPath.h>
#include <fmutil/fm_math.h>
#include <std_msgs/Bool.h>
#include <rrts/rrts_status.h>

using namespace std;

/// Drives the vehicle from A to B by feeding it a sequence of waypoints.
class RoutePlanner
{

public:
    RoutePlanner(const int start, const int end);

private:
    ros::NodeHandle n;
    ros::Publisher g_plan_pub_;
    ros::Publisher nextpose_pub_;
    ros::Publisher norminal_lane_pub_;

    ros::Subscriber goal_in_collision_sub_;
    tf::TransformListener tf_;
    tf::Stamped<tf::Pose> global_pose_;
    StationPaths sp_;

    StationPath path_;
    Station destination_;
    bool goal_collision_;
    bool root_in_goal_;
    bool robot_near_root_;
    bool switched_root_;
    bool initialized_;
    bool goal_infeasible_;
    unsigned waypointNo_;

    bool goToDest();
    void initDest(const int start, const int end);

    void pubPathVis();
    bool getRobotGlobalPose();
    void transformMapToOdom(geometry_msgs::PoseStamped *map_pose,
            geometry_msgs::PointStamped *odom_point);
    double distanceToGoal();
    void rrts_status(rrts::rrts_status rrts_status);
    int transform_map_to_local_map(const double stateIn[3], double &zlx, double &zly, double &yl);
};

void RoutePlanner::rrts_status(rrts::rrts_status rrts_status)
{
    if(initialized_)
    {
        goal_collision_ = rrts_status.goal_in_collision;
        goal_infeasible_ = rrts_status.goal_infeasible;
        root_in_goal_ = rrts_status.root_in_goal;
        robot_near_root_ = rrts_status.robot_near_root;
        switched_root_ = rrts_status.switched_root;
        goToDest();
    }
}
RoutePlanner::RoutePlanner(const int start, const int end)
{
    g_plan_pub_ = n.advertise<nav_msgs::Path>("pnc_globalplan", 1, true);
    nextpose_pub_ = n.advertise<geometry_msgs::PoseStamped>("pnc_nextpose",1);
    norminal_lane_pub_ = n.advertise<std_msgs::Bool>("norminal_lane", 1, true);
    goal_in_collision_sub_ = n.subscribe("rrts_status",1, &RoutePlanner::rrts_status, this);
    ros::Rate loop_rate(3);
    initDest(start, end);
    initialized_ = false;
    goal_collision_ = false;

    while(!getRobotGlobalPose())
    {
        ros::spinOnce();
        loop_rate.sleep();
        cout<<"Waiting for Robot pose"<<endl;
    }

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
    cout<<waypointNo_<<" waypointNo after getting closest path"<<endl;
    //then increment it to make sure the waypoint is in front
    double dist=0.0;
    do//( ; waypointNo_<path_.size(); waypointNo_++)
    {
        waypointNo_++;
        dist = fmutil::distance(global_pose_.getOrigin().x(), global_pose_.getOrigin().y(), path_[waypointNo_].x_, path_[waypointNo_].y_);
        cout<<"Increment waypoint "<<waypointNo_<<endl;
    }while(dist<10);
    //waypointNo_++;
    cout<<"Initialized waypoint "<<waypointNo_<<endl;
    goToDest();
    initialized_ = true;
    ros::spin();


}


void RoutePlanner::initDest(const int start, const int end)
{
    //Default to true which is towards McD
    bool norminal_lane = true;
    //All paths starting from 0 and 1 are moving away from McD
    if(start == 0 || start == 1) norminal_lane = false;
    //Except when moving from 0 to 1 (DCC to McD)
    if(start == 0 && end == 1) norminal_lane = true;
    //All paths starting from 2 and 3 are moving towards McD
    //Except when moving from 2 to 3 (EA to E3A)
    if(start == 2 && end == 3) norminal_lane = false;
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
    std_msgs::Bool temp; temp.data = norminal_lane;
    norminal_lane_pub_.publish(temp);
}

using namespace std;

bool RoutePlanner::goToDest()
{
    getRobotGlobalPose();

    ROS_INFO_THROTTLE(3, "Going to %s. Distance=%.0f.", destination_.c_str(), distanceToGoal());
    cout<<"goToDest() waypointNo="<<waypointNo_<<endl;
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



    if(goal_collision_ || goal_infeasible_)
    {
        double dist = fmutil::distance(global_pose_.getOrigin().x(), global_pose_.getOrigin().y(), path_[waypointNo_].x_, path_[waypointNo_].y_);
        if(dist < 15) 	
        {
            if(waypointNo_ < path_.size())
                waypointNo_++;	
            cout<<"Goal in collision/infeasible reported, increment waypoint"<<endl;
        }
    }


    if(robot_near_root_ && (switched_root_ || root_in_goal_))
    {
        if(waypointNo_>=path_.size())
        {
            cout<<waypointNo_<<" "<<path_.size()<<endl;
            cout<<"Reach last pose, exiting..."<<endl;
            exit(0);
        }
        waypointNo_++;
        cout<<"Nominal increment for next path"<<endl;
    }

    map_pose.header.frame_id="/map";
    map_pose.header.stamp=ros::Time::now();
    nextpose_pub_.publish(map_pose);

    //transform from pose to point, planner expect point z as yaw
    //publish the first waypoint in map frame then continue to send the points until the last one

    return true;
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


inline
int RoutePlanner::transform_map_to_local_map(const double stateIn[3], double &zlx, double &zly, double &yl)
{
    zlx = zly = yl = 0.0;
    // map frame z, yaw
    geometry_msgs::PoseStamped tmp;
    tf::poseStampedTFToMsg(global_pose_, tmp);
    double roll=0, pitch=0, yaw=0;
    tf::Quaternion q;
    tf::quaternionMsgToTF(tmp.pose.orientation, q);
    tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
    double map_origin[3] = {global_pose_.getOrigin().getX(), global_pose_.getOrigin().getY(), yaw};
    double zm[2] = {stateIn[0], stateIn[1]};
    double ym = stateIn[2];
    //cout<<"zm: "<< zm[0]<<" "<<zm[1]<<" "<<ym<<endl; 

    // base_link frame
    double cos_map_yaw = cos(map_origin[2]);
    double sin_map_yaw = sin(map_origin[2]);

    // rotate zm by yaw, subtract map_origin to get zlocal
    zlx = (zm[0]-map_origin[0])*cos_map_yaw + (zm[1]-map_origin[1])*sin_map_yaw;
    zly = -(zm[0]-map_origin[0])*sin_map_yaw + (zm[1]-map_origin[1])*cos_map_yaw;
    yl = ym - map_origin[2];
    while(yl > M_PI)
        yl -= 2.0*M_PI;
    while(yl < -M_PI)
        yl += 2.0*M_PI;

    return 0;
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
