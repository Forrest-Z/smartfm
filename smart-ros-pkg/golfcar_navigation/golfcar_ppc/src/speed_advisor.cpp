#include <math.h>

#include <string>
#include <cmath>

using namespace std;

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

class SpeedAdvisor
{
public:
    SpeedAdvisor();

    ros::NodeHandle n;
    tf::TransformListener tf_;
    ros::Publisher recommend_speed_;
    ros::Subscriber move_base_speed_;
    ros::Subscriber global_plan_;
    ros::Subscriber junction_sub_;
    double max_speed_;
    double acc_;
    double max_dec_, norm_dec_, dec_ints_;
    double stop_ints_dist_;
    double frequency_;
    double tolerance_; //to track for last update from move_base package
    double e_zone_; //apply full brake if there exist an obstacles within this distance from base link
    double high_speed_,slow_zone_,slow_speed_,enterstation_speed_,stationspeed_dist_;
private:
    bool junction_stop_,through_ints_;
    ros::Time last_update_;
    double stopping_distance_; //automatic calculate based on the maximum speed and normal deceleration
    bool use_sim_time_;
    void moveSpeedCallback(golfcar_ppc::move_status status);
    void junctionCallback(std_msgs::BoolConstPtr junction);
    double sqrtDistance(double x1, double y1, double x2, double y2);
    void ControlLoop(const ros::TimerEvent& event);
    geometry_msgs::Twist move_speed_;
    golfcar_ppc::move_status move_status_;
    vector<geometry_msgs::Point> stoppingPoint_;
};


SpeedAdvisor::SpeedAdvisor()
{
    /* the speed_advisor receive message from move_base package and perform neccessary speed profile generation
     * Currently only trapezoidal profile is implemented, and the speed regulation largely seperated into 2 zone:
     * slow-down and stopping zone.
     *
     */

    recommend_speed_= n.advertise<geometry_msgs::Twist>("cmd_vel", 1);
    move_base_speed_=n.subscribe("/move_status",1, &SpeedAdvisor::moveSpeedCallback, this);
    junction_sub_=n.subscribe("nav_junction",1,&SpeedAdvisor::junctionCallback,this);
    ros::NodeHandle nh("~");
    nh.param("max_speed", high_speed_, 2.0);
    nh.param("acc", acc_, 0.5);
    nh.param("max_dec", max_dec_, 2.0);
    nh.param("norm_dec", norm_dec_, 0.5);

    nh.param("dec_ints", dec_ints_, 0.4);
    nh.param("stop_ints_dist", stop_ints_dist_,5.0); //the stopping distance from obstacles.
    //It was found that the distance given by move_status will reach as small as 4 meter
    //Hence, stopping distance should be at least larger than 4 for the stopping manoeuvre to work

    nh.param("frequency", frequency_,20.0);
    nh.param("tolerance", tolerance_, 0.5);
    nh.param("emergency_zone", e_zone_, 2.0);
    nh.param("slow_zone", slow_zone_, 10.0);
    nh.param("slow_speed", slow_speed_, 1.0);
    nh.param("enterstation_speed",enterstation_speed_, slow_speed_); //by default, enterstation speed is the same as slow speed
    nh.param("stationspeed_dist", stationspeed_dist_, 20.0);
    n.param("use_sim_time", use_sim_time_, false);
    ROS_DEBUG_STREAM("Simulated time is "<<use_sim_time_);
    junction_stop_ = false;
    through_ints_ = true;
    ros::Timer timer = n.createTimer(ros::Duration(1.0/frequency_),&SpeedAdvisor::ControlLoop,this);

    ros::spin();
}

void SpeedAdvisor::junctionCallback(std_msgs::BoolConstPtr junction)
{
    junction_stop_=junction->data;
    if(junction_stop_) ROS_INFO("Approaching Junction");
}
void SpeedAdvisor::ControlLoop(const ros::TimerEvent& event)
{

    //precompute the amount of change in speed
    double max_neg_speed = -max_dec_ / frequency_;
    double pos_speed = acc_ / frequency_;
    double norm_neg_speed = -norm_dec_ / frequency_;

    if((ros::Time::now()-last_update_).toSec()>tolerance_)
    {
        ROS_DEBUG("No response from move base package, stopping....");
        move_speed_.linear.x += max_neg_speed;
    }
    else
    {
        vector<double> speed_delta;
        if(move_status_.acc_dec)
        {
            /* In this speed design, situations that need to be taken care of are
             * 1: Stopping: must apply deceleration make sure no collision should happen
             * 2: Junction = reach goal: must apply deceleration slowly
             * 3: Slow speed: max speed limit at slow speed
             * 4: Normal: max speed limit at normal speed
             */

            //try a more general solution
            double obs_dist = move_status_.obstacles_dist;
            if(obs_dist<slow_zone_||move_status_.dist_to_goal<stationspeed_dist_)
            {
                max_speed_ = slow_speed_;
                if(obs_dist<e_zone_)
                {
                    speed_delta.push_back(max_neg_speed);
                }
                else
                {
                    double deceleration_required = move_speed_.linear.x*move_speed_.linear.x/(2*(obs_dist-e_zone_));
                    ROS_DEBUG_STREAM("Calculated deceleration: "<<deceleration_required<<"Obs dist: "<<obs_dist);
                    if(deceleration_required>=norm_dec_)
                    {
                        if(deceleration_required>max_dec_) deceleration_required=max_dec_;
                        speed_delta.push_back(-deceleration_required/frequency_);
                    }
                    else speed_delta.push_back(pos_speed);
                }
            }
            else
            {
                max_speed_ = high_speed_;
            }

            //to slow down at intersection, dist_to_ints = -1 if there is no intersection
            if(move_status_.dist_to_ints>0 && through_ints_)
            {
                double stopping_dist_ints = move_status_.dist_to_ints-stop_ints_dist_;
                double recommended_speed=0;
                if(stopping_dist_ints>0)
                {
                    recommended_speed= sqrt(2*dec_ints_*stopping_dist_ints);
                    if(recommended_speed<high_speed_) max_speed_ = recommended_speed;
                }
                else
                {
                    move_speed_.angular.z = move_status_.steer_angle;
                    move_speed_.linear.x = 0;
                    recommend_speed_.publish(move_speed_);
                    ros::spinOnce();
                    string temp = "";
                    cout<<"Clear to go?"<<endl;
                    getline(cin, temp);
                    cout<<"Continue"<<endl;
                    through_ints_ = false;
                }
                ROS_DEBUG_STREAM("Stopping dist: "<<stopping_dist_ints<<" recommended_speed: "<<recommended_speed);
            }
            //reset the through_ints by making sure that the previous stop has passed
            //this assume that the stopping distance between intersection is more than 2 meter a part
            if(!through_ints_ && move_status_.dist_to_ints> stop_ints_dist_+2) through_ints_ = true;
            //also reset the through_ints if there is no long an intersection
            if(move_status_.dist_to_ints == -1) through_ints_ = true;

        }
        else speed_delta.push_back(norm_neg_speed);

        if(move_status_.emergency) speed_delta.push_back(max_neg_speed);

        if(junction_stop_) speed_delta.push_back(norm_neg_speed);
        else speed_delta.push_back(pos_speed);

        double min_speed_delta = *(min_element(speed_delta.begin(), speed_delta.end()));

        ROS_DEBUG_STREAM("min_speed_delta: "<<min_speed_delta<<" Emergency "<<int(move_status_.emergency)<<" Junction: "<<junction_stop_);
        //racing with the max_speed_ specification by the if condition is not added
        if(move_speed_.linear.x<max_speed_) move_speed_.linear.x+=min_speed_delta;
    }
    if(move_speed_.linear.x>max_speed_)
    {
        //respect the acceleration and deceleration
        if(move_speed_.linear.x>max_speed_) move_speed_.linear.x+=norm_neg_speed;
        else move_speed_.linear.x=max_speed_;
        //if(max_speed_ == slow_speed_)
            ROS_DEBUG("Max speed reduced, speed now %lf, %lf", move_speed_.linear.x, norm_neg_speed);
    }
    else if(move_speed_.linear.x<0) move_speed_.linear.x=0;
    move_speed_.angular.z = move_status_.steer_angle;
    ROS_DEBUG_STREAM(move_speed_.linear.x<<" Distance to goal: "<<move_status_.dist_to_goal);
    geometry_msgs::Twist move_speed;
    move_speed = move_speed_;
    double speed_compensation = 1.0;
    //it was found that the although commanded to travel 2 m/s, it is actually travelling at 3.33x faster in simulation with stage, compensation is needed
    if(use_sim_time_) speed_compensation=0.3;
    move_speed.linear.x = move_speed_.linear.x * speed_compensation;
    recommend_speed_.publish(move_speed);
}

double SpeedAdvisor::sqrtDistance(double x1, double y1, double x2, double y2)
{
    return sqrt((x1-x2)*(x1-x2)+(y1-y2)*(y1-y2));
}

void SpeedAdvisor::moveSpeedCallback(golfcar_ppc::move_status status)
{
    last_update_ = ros::Time::now();
    move_status_ = status;
}



int main(int argc, char **argv)
{
    ros::init(argc, argv, "speed_advisor");
    SpeedAdvisor sa;
    ros::spin();
    return 0;
}
