#include "RoutePlannerVehicle.h"

///TODO: publish GPS and ETA

RoutePlannerVehicle::RoutePlannerVehicle(const StationPaths & sp)
    : RoutePlanner(sp), ac_("move_base", true)
{
    while( ! ac_.waitForServer(ros::Duration(5.0)) && ros::ok() )
        ROS_INFO("Waiting for the move_base action server to come up");

    waypoint_pub_ = n.advertise<geometry_msgs::PointStamped>("pnc_waypoint", 1);
    g_plan_pub_ = n.advertise<nav_msgs::Path>("pnc_globalplan", 1);
    pointCloud_pub_ = n.advertise<sensor_msgs::PointCloud>("pnc_waypointVis",1);
    nextpose_pub_ = n.advertise<geometry_msgs::PoseStamped>("pnc_nextpose",1);
}


void RoutePlannerVehicle::initDest(const Station & start, const Station & end)
{
    destination_ = end;
    path_ = sp_.getPath(start, end);
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

    // publish the path for the local controller
    // we encode it: x contains the id of the starting station, and y the id of the
    // destination station.
    move_base_msgs::MoveBaseGoal goal;
    goal.target_pose.header.stamp = ros::Time::now();
    goal.target_pose.header.frame_id = "/map";
    goal.target_pose.pose.position.x = (double) start.number();
    goal.target_pose.pose.position.y = (double) end.number();
    goal.target_pose.pose.orientation.w = 1.0;
    ac_.sendGoal(goal);
}


bool RoutePlannerVehicle::goToDest()
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
    double d = sqrt(pow(mapx-robotx,2)+pow(mapy-roboty,2));

    if( waypointNo_ < path_.size()-1 && d < 4 )
        waypointNo_++;
    else if( waypointNo_ == path_.size()-1 )
        return true;

    return false;
}

double RoutePlannerVehicle::distanceToGoal()
{
    double d = 0;

    for( unsigned i = waypointNo_+1; i<path_.size(); i++ )
        d += PathPoint::distance(path_[i-1], path_[i]);

    PathPoint p(global_pose_.getOrigin().x(), global_pose_.getOrigin().y());
    d += PathPoint::distance(p, path_[waypointNo_]);

    return d;
}

void RoutePlannerVehicle::transformMapToOdom(geometry_msgs::PoseStamped *map_pose,
                                             geometry_msgs::PointStamped *odom_point)
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


bool RoutePlannerVehicle::getRobotGlobalPose()
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
