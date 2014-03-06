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

using namespace std;

namespace golfcar_purepursuit
{

class PurePursuit
{
public:
    PurePursuit(string global_frameID, double min_look_ahead_dist, double forward_achor_pt_dist, double car_length);

    bool steering_control(double *wheel_angle, double *dist_to_goal);
    bool current_pos_to_point_dist(int end_point, double* path_dist);
    bool current_pos_to_point_dist_simple(int end_point, double* path_dist);
    nav_msgs::Path path_;
    geometry_msgs::Pose vehicle_base_;
    int path_n_;
    bool initialized_, lookahead_ini_;
    double dist_to_final_point;
    void updateCommandedSpeed(double speed);
private:
    geometry_msgs::Point current_point_, next_point_;

    ros::Publisher pp_vis_pub_;

    double Lfw_, lfw_, min_lookahead_;
    int nextPathThres_;
    int nextPathCount_;
    string global_frameID_;
    geometry_msgs::Point collided_pt_;
    double car_length_;

    bool heading_lookahead(double *heading_la, double *dist_to_goal);

    /// check whether there is an intersection between the segment formed by
    /// [current_point_, next_point_] and the circle centred on anchor_point
    /// with radius Lfw_
    bool circle_line_collision(geometry_msgs::Point anchor_point,
            geometry_msgs::Point *intersect_point);

};

}

#endif
