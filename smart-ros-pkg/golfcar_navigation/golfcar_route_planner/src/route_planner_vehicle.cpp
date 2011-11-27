#include "route_planner_vehicle.h"


RoutePlannerVehicle::RoutePlannerVehicle() : ac_("move_base", true)
{
    n.param<bool>("standalone", standalone_, true);

    while( ! debugMode_ && ! ac_.waitForServer(ros::Duration(5.0)) && ros::ok() )
        ROS_INFO("Waiting for the move_base action server to come up");

    waypoint_pub_ = n.advertise<geometry_msgs::PointStamped>("pnc_waypoint", 1);
    g_plan_pub_ = n.advertise<nav_msgs::Path>("pnc_globalplan", 1);
    pointCloud_pub_ = n.advertise<sensor_msgs::PointCloud>("pnc_waypointVis",1);
    //poseStamped_pub_ = n.advertise<geometry_msgs::PoseStamped>("move_base_simple/goal",1);
    nextpose_pub_ = n.advertise<geometry_msgs::PoseStamped>("pnc_nextpose",1);

    //Prompt for current station
}


void RoutePlannerVehicle::run()
{
    //
    // TODO: Get the pickup and drop off stations
    //

    if( currentStationID_ != pickup_ )
    {
        ROS_INFO("On call... pickup at station %s from %s", pickup_.c_str(), currentStationID_.c_str());
        targets_ = sp_.getPath(currentStationID_, pickup_);

        pubPathVis();
        publishGoal(currentStationID_, pickup_);
        waypointLoop(VehicleStatus::GoingToPickupLocation, currentStationID_, pickup_);

        //arrive at pickup point
        ROS_INFO("Arrived at pickup point");
        currentStationID_ = pickup_;
    }

    // TODO: move to passenger comm
    string input = "";
    ROS_INFO("Going to dropoff station %s", dropoff_.c_str());
    ROS_INFO("Press enter key when you are ready!");
    getline(cin, input);
    ROS_INFO("Let's go!");

    targets_ = sp_.getPath(currentStationID_, dropoff_);

    pubPathVis();
    publishGoal(currentStationID_, dropoff_);
    waypointLoop(VehicleStatus::GoingToDropOffLocation, currentStationID_, dropoff_);

    // TODO: move to missionComm arrive at dropoff point, update current ID
    ROS_INFO("Arrive at dropoff point");
    ROS_INFO("Press enter key when you are outside the vehicle!");
    getline(cin, input);
    currentStationID_ = dropoff_;

    //I became available again, preparing to get next task
    // TODO: update mission status
    ROS_INFO("Waiting for next task. Prepared to move to a pick up point at any time");

    ros::spinOnce();
}


void RoutePlannerVehicle::pubPathVis()
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

void RoutePlannerVehicle::waypointLoop(VehicleStatus::Status status,
                                    const Station & start, const Station & end)
{
    waypointNo_ = 0;

    while( ros::ok() )
    {
        double d = distanceToGoal();

        if( waypointNo_ == targets_.size() )
            break;
        if( status == VehicleStatus::GoingToPickupLocation )
            ROS_DEBUG("Going to pick up station %s from station %s, distance to go %.0f m", end.c_str(), start.c_str(), d);
        if( status == VehicleStatus::GoingToDropOffLocation )
            ROS_DEBUG("Going to drop off station %s from station %s, distance to go %.0f m", end.c_str(), start.c_str(), d);

        pubWaypoint();
        ros::spinOnce();
        ros::Duration(0.2).sleep();
    }
}

double RoutePlannerVehicle::distanceToGoal()
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


void RoutePlannerVehicle::publishGoal(const Station & start, const Station & end)
{
    //use more expansive action server to properly trace the goal status
    geometry_msgs::PoseStamped ps;
    ps.header.stamp = ros::Time::now();
    ps.header.frame_id = "/map";

    ps.pose.position.x = (double) start.number();
    ps.pose.position.y = (double) end.number();
    ps.pose.orientation.w = 1.0;

    move_base_msgs::MoveBaseGoal goal;
    goal.target_pose = ps;
    ac_.sendGoal(goal);
}


void RoutePlannerVehicle::pubWaypoint()
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
