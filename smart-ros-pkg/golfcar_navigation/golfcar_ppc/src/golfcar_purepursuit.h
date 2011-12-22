#ifndef __GOLFCAR_PUREPURSUIT_H__
#define __GOLFCAR_PUREPURSUIT_H__

#include <string>
#include <cmath>

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PolygonStamped.h>

namespace golfcar_purepursuit
{

class PurePursuit
{

public:

    PurePursuit();

    bool steering_control(double& wheel_angle, double &dist_to_goal, double &dist_to_ints);
    geometry_msgs::Point current_point_, next_point_;

    ros::Publisher pp_vis_;

    nav_msgs::Path path_;
    geometry_msgs::Pose vehicle_base_;
    double Lfw_, lfw_;
    int path_n_;
    int nextPathThres_;
    int nextPathCount_;
    bool initialized_;
    double dist_to_final_point;

private:
    bool heading_lookahead(double &heading_la, double &dist_to_goal, double &dist_to_ints);
    bool circle_line_collision(geometry_msgs::Point& anchor_point, geometry_msgs::Point& intersect_point);
    double sqrt_distance(geometry_msgs::Point wp_a, geometry_msgs::Point wp_b);

    double car_length;

};

}

#endif
