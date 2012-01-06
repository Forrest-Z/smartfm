#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>

#include "SimpleGoal.h"

using namespace std;


SimpleGoal::SimpleGoal(const StationPaths & sp) : RoutePlanner(sp)
{
    goal_pub_ = nh.advertise<geometry_msgs::PoseStamped>("move_base_simple/goal", 100);
    speed_status_sub_ = nh.subscribe("speed_status", 1, &SimpleGoal::speedStatusCallBack, this);
    has_reached_ = false;

    // a service to convert utm coordinates into latitude and longitude
    utmToLatLonSrvClient_ = nh.serviceClient<fmutil::UtmToLatLon>("utm_to_latlon", true);
}

void SimpleGoal::initDest()
{
    has_reached_ = false;

    geometry_msgs::PoseStamped goal;
    goal.header.stamp = ros::Time::now();
    goal.header.frame_id = "/map";
    goal.pose.position.x = (double) currentStation_.number();
    goal.pose.position.y = (double) destination_.number();
    goal.pose.orientation.z = 1.0;
    goal_pub_.publish(goal);

    ROS_INFO("sent goal: [%f %f]", goal.pose.position.x, goal.pose.position.y);
}

void SimpleGoal::speedStatusCallBack(const golfcar_ppc::speed_contribute &msg)
{
    has_reached_ = msg.goal;
    eta_ = msg.dist_goal / 2; //velocity is taken as constant 2m/s
}

bool SimpleGoal::goToDest()
{
    getLatLon();
    return has_reached_;
}

void SimpleGoal::getLatLon()
{
    try
    {
        // retrieve robot's position in map coordinates
        tf::Stamped<tf::Pose> global_pose = getRobotGlobalPose();

        // TODO: implement a mechanism to reconnect to the service should it get
        // disconnected.
        if( utmToLatLonSrvClient_ )
        {
            // translate into UTM coordinates
            fmutil::UtmToLatLon::Request req;
            // TODO: add the offset from map to GPS coordinates
            req.easting = global_pose.getOrigin().x();
            req.northing = global_pose.getOrigin().y();

            // Singapore UTM zone is 48N
            req.zone = "48N";

            // convert to latitude and longitude using the service
            fmutil::UtmToLatLon::Response res;
            if( ! utmToLatLonSrvClient_.call(req,res) )
                ROS_ERROR_THROTTLE(1, "Call to utm_to_latlon service failed");

            // these will be published by the mission comm
            latitude_ = res.latitude;
            longitude_ = res.longitude;
        }
        else
        {
            ROS_ERROR_THROTTLE(1, "Connection to utm_to_latlon service failed");
        }
    }
    catch (tf::TransformException & e)
    {
        ROS_ERROR_THROTTLE(1, "Exception occured while getting robot's global pose: %s", e.what());
    }
}

tf::Stamped<tf::Pose> SimpleGoal::getRobotGlobalPose() const
{
    tf::Stamped<tf::Pose> global_pose;
    global_pose.setIdentity();

    tf::Stamped<tf::Pose> robot_pose;
    robot_pose.setIdentity();
    robot_pose.frame_id_ = "/base_link";
    robot_pose.stamp_ = ros::Time();
    ros::Time current_time = ros::Time::now(); // save time for checking tf delay later

    tf_.transformPose("/map", robot_pose, global_pose);

    return global_pose;
}
