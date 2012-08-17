/*
 * speed_advisor_direct.h
 *
 *  Created on: Dec 28, 2011
 *      Author: golfcar
 */

#ifndef SPEED_ADVISOR_DIRECT_H_
#define SPEED_ADVISOR_DIRECT_H_

#include <string>
#include <cmath>


#include <ros/ros.h>

#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>

#include <std_msgs/Bool.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PolygonStamped.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>

#include <fmutil/fm_math.h>
#include <pnc_msgs/speed_contribute.h>
#include <pnc_msgs/move_status.h>

#include "intersection_handler.h"

using namespace std;


class SpeedAttribute
{
public:
    enum SpeedAttributeDescription
    {
        no_response, path_exist, norm_zone, slow_zone,
        emergency, max_brake, need_brake, e_zone, warn_brake,
        intersection, app_goal, goal
    };

    string description_;
    SpeedAttributeDescription element_;
    double target_speed_;
    double speed_inc_;
    double speed_dec_;
    double final_speed_;
    double speed_now_;

    /** Generates a SpeedAttribute with the given profile.
     *
     * If speed_now is different from target_speed, then modify it by
     * speed_inc or speed_dec. This generates a trapezoidal
     * speed profile.
     */
    static SpeedAttribute generate(const string & description,
            SpeedAttributeDescription element,
            double speed_now, double target_speed,
            double speed_inc, double speed_dec);
};

class SpeedSettings : public vector<SpeedAttribute>
{
public:
    /// Find the SpeedAttribute element with the lowest final_speed_
    SpeedAttribute & find_min_speed();
};



/** The speed advisor receives message from the move_base node
 * and performs necessary speed profile generation.
 *
 * Currently only trapezoidal profile is implemented, and the speed
 * regulation is largely separated into 2 zones: slow-down and stopping zone.
 */
class SpeedAdvisor
{
public:
    SpeedAdvisor();

    double max_speed_;
    double acc_;
    double max_dec_, norm_dec_, dec_ints_, dec_station_;
    double stop_ints_dist_;
    double frequency_;
    double tolerance_; //to track for last update from move_base package
    double e_zone_; //apply full brake if there exist an obstacles within this distance from base link
    double high_speed_, slow_zone_, slow_speed_, enterstation_speed_;
    double ppc_stop_dist_, stationspeed_dist_;

private:
    ros::NodeHandle nh_;
    tf::TransformListener tf_;
    ros::Publisher recommend_speed_pub_;
    ros::Publisher speed_contribute_pub_;
    ros::Publisher left_blinker_pub_, right_blinker_pub_;
    ros::Subscriber move_base_speed_;
    ros::Subscriber global_plan_;
    ros::Subscriber slowzone_sub_;
    ros::Timer timer_;

    IntersectionHandler int_h_;

    bool kinematics_acc_;
    int attribute_, zone_;
    ros::Time last_update_;
    double stopping_distance_, baselink_carfront_length_; //automatic calculate based on the maximum speed and normal deceleration
    double speed_now_;
    bool use_sim_time_;
    SpeedAttribute::SpeedAttributeDescription element_pre_, element_now_;
    int signal_type_; // 0: left_signal, 1: right_signal, 2: off_signals

    string base_link_, map_id_;
    
    geometry_msgs::Twist move_speed_;
    pnc_msgs::move_status move_status_;
    vector<geometry_msgs::Point> stoppingPoint_;
    geometry_msgs::PoseArray slowZone_;

    void ControlLoop(const ros::TimerEvent& event);
    bool getRobotGlobalPose(tf::Stamped<tf::Pose>& odom_pose) const;
    void moveSpeedCallback(pnc_msgs::move_status status);
    void slowZoneCallback(geometry_msgs::PoseArrayConstPtr slowzones);
};


#endif /* SPEED_ADVISOR_DIRECT_H_ */
