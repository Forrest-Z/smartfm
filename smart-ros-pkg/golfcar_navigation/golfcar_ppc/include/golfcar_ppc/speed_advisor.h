/*
 * speed_advisor_direct.h
 *
 *  Created on: Dec 28, 2011
 *      Author: golfcar
 */

#ifndef SPEED_ADVISOR_DIRECT_H_
#define SPEED_ADVISOR_DIRECT_H_

#include <math.h>

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
#include <std_msgs/Bool.h>
#include <golfcar_ppc/move_status.h>
#include <geometry_msgs/PoseArray.h>
#include <golfcar_ppc/speed_contribute.h>
#include <interactive_markers/interactive_marker_server.h>

using namespace std;
using namespace visualization_msgs;
class SpeedAttribute
{
public:
    enum speedDescription
    {
        no_response, movebase_dec, norm_zone, slow_zone,
        emergency, max_brake, need_brake, e_zone, warn_brake,
        intersection, app_goal, goal 
    };

    SpeedAttribute(double speed_now)
    {
        speed_now_ = speed_now;
    };
    void final_speed(string description, int element, double speed_inc, double speed_dec, double target_speed)
    {
        double speed;
        //delta addition
        if(speed_now_==target_speed) final_speed_ = speed_now_;
        if(speed_now_>target_speed)
        {
            speed=speed_now_+speed_dec;
            if(target_speed>speed) speed = target_speed;
        }
        else
        {
            speed=speed_now_+speed_inc;
            if(speed>target_speed) speed = target_speed;
        }

        final_speed_ =  speed;
        description_ = description;
        element_ = element;
        speed_inc_ = speed_inc;
        speed_dec_ = speed_dec;
        target_speed_ = target_speed;
    };
    string description_;
    int element_;
    double target_speed_;
    double speed_inc_;
    double speed_dec_;
    double final_speed_;
    double speed_now_;
};

class SpeedSettings : public vector<SpeedAttribute>
{
public:
    SpeedAttribute* find_min_speed()
    {
        unsigned int element_no=0;
        double min_speed = this->at(0).final_speed_;
        for(unsigned int i=1; i<this->size(); i++)
        {
            if(this->at(i).final_speed_<min_speed)
            {
                min_speed = this->at(i).final_speed_;
                element_no = i;
            }
        }
        return &this->at(element_no);
    };
};

class SpeedAdvisor
{
public:
    SpeedAdvisor();

    ros::NodeHandle n;
    tf::TransformListener tf_;
    ros::Publisher recommend_speed_;
    ros::Publisher speed_contribute_;
    ros::Subscriber move_base_speed_;
    ros::Subscriber global_plan_;
    ros::Subscriber slowzone_sub_;
    double max_speed_;
    double acc_;
    double max_dec_, norm_dec_, dec_ints_, dec_station_;
    double stop_ints_dist_;
    double frequency_;
    double tolerance_; //to track for last update from move_base package
    double e_zone_; //apply full brake if there exist an obstacles within this distance from base link
    double high_speed_,slow_zone_,slow_speed_,enterstation_speed_,ppc_stop_dist_,stationspeed_dist_;

private:
    bool junction_stop_,through_ints_;
    int attribute_, zone_;
    ros::Time last_update_;
    double stopping_distance_; //automatic calculate based on the maximum speed and normal deceleration
    double speed_now_;
    bool use_sim_time_;
    void moveSpeedCallback(golfcar_ppc::move_status status);
    void slowZoneCallback(geometry_msgs::PoseArrayConstPtr slowzones);
    double sqrtDistance(double x1, double y1, double x2, double y2);
    void ControlLoop(const ros::TimerEvent& event);
    geometry_msgs::Twist move_speed_;
    golfcar_ppc::move_status move_status_;
    vector<geometry_msgs::Point> stoppingPoint_;
    geometry_msgs::PoseArray slowZone_;
    bool getRobotGlobalPose(tf::Stamped<tf::Pose>& odom_pose) const;

    void add_button_marker(interactive_markers::InteractiveMarkerServer &server, geometry_msgs::Vector3 scale, std_msgs::ColorRGBA color, geometry_msgs::Pose pose, std::string name, std::string description);
    void processFeedback(const InteractiveMarkerFeedbackConstPtr &feedback );
    interactive_markers::InteractiveMarkerServer *marker_server_;
    geometry_msgs::Point int_point_;
};


#endif /* SPEED_ADVISOR_DIRECT_H_ */
