#ifndef SPEED_ADVISOR_DIRECT_H_
#define SPEED_ADVISOR_DIRECT_H_

#include <string>
#include <cmath>

#include <boost/thread/mutex.hpp>

#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PolygonStamped.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>

#include <std_msgs/Bool.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <ros/ros.h>
#include <fmutil/fm_math.h>
#include <pnc_msgs/speed_contribute.h>
#include <pnc_msgs/move_status.h>
#include <rrts/rrts_status.h>

#include "IntersectionHandler.h"

using namespace std;

struct SpeedAttribute {
    
    enum Description {
        
        no_response, 
        robot_collision, 
        path_noexist, 
        norm_zone, 
        slow_zone, 
        bwd_driving,
        switching_gear, 
        emergency, 
        max_brake, 
        need_brake, 
        e_zone, 
        warn_brake,
        intersection, 
        app_goal, 
        goal, 
        manual_mode
                
    };
    
    Description     description_;
    
    string          description_str_;
    
    double          target_speed_, final_speed_;
    
};

class SpeedSettings {
    
public:
    
    double max_neg_speed_; ///< deceleration parameter: -max_dec_ / frequency_
    
    double pos_speed_; ///< acceleration parameter: acc_ / frequency_
    
    double norm_neg_speed_; ///< deceleration parameter: -norm_dec_ / frequency_
    
protected:
    
    ros::NodeHandle nh_;
    
    ros::Subscriber automode_sub_, emergency_sub_, odo_sub_;
    
    boost::mutex                    mutex_;
    
    std::vector < SpeedAttribute >  attrs_;

    bool automode_, emergency_;
    
    /// This is the current velocity target
    double curr_vel_;

    /// the current velocity as given by odometry
    double odo_vel_;

    int switching_gear_;
    
public:
    
    // Constructors
    
    SpeedSettings();
    
    /// adds a SpeedAttribute
    void add( const string & description_str, SpeedAttribute::Description description, 
    
              double target_speed, double speed_inc, double speed_dec);

    /// adds a SpeedAttribute: speed_inc and speed_dec are taken as pos_speed_
    /// and norm_neg_speed_
    void add( const string & description_str, SpeedAttribute::Description description, 
    
              double target_speed) 
    
    { add( description_str, description, target_speed, pos_speed_, norm_neg_speed_); }
    
    /// adds a SpeedAttribute with target velocity 0, speed_inc=pos_speed_ and
    /// speed_dec=max_neg_speed_
    void add_brake( const string & description_str, SpeedAttribute::Description description) 
    
    { add( description_str, description, 0, pos_speed_, max_neg_speed_); }
    
    double curr_vel() const { return curr_vel_; }
    
    bool is_gear_switching() const { return (switching_gear_ > 0); }
    
    /// Finds the SpeedAttribute element with the lowest final_speed_,
    /// updates curr_vel_ and clears the SpeedAttribute vector.
    SpeedAttribute select_min_speed();
    
    void switch_gear() { switching_gear_ = 1; }
    
protected:
    
    // Callback methods
    
    void automode_callback( const std_msgs::Bool &);
    
    void emergency_callback( const std_msgs::Bool &);
    
    void odom_callback( const nav_msgs::Odometry &);
    
    // Other methods
    
    void button_state( bool automode, bool emergency);
    
};

/** The speed advisor receives message from the move_base node
 * and performs necessary speed profile generation.
 *
 * Currently only trapezoidal profile is implemented, and the speed
 * regulation is largely separated into 2 zones: slow-down and stopping zone.
 */

class SpeedAdvisor {
    
private:
    
    // Speed-related members
    
    double max_speed_;
    double min_speed_;
    double bwd_speed_;
    
    double high_speed_, slow_zone_, slow_speed_;
    
    bool bwd_drive_;
    
    // Acceleration-related members
    
    double acc_, max_dec_, norm_dec_, dec_ints_, dec_station_;
    
    bool kinematics_acc_;
    
    // Other members
    
    double baselink_carfront_length_;
    
    double frequency_;
    
    double tolerance_; // to track for last update from move_base package
    
    double e_zone_; // apply full brake if there exist an obstacles within this distance from base link
    
    double stop_ints_dist_;
    
    double ppc_stop_dist_, final_stop_dist_;
    
    int signal_type_; // 0: left_signal, 1: right_signal, 2: off_signals
    
    int attribute_, zone_;
    
    bool use_sim_time_;
    
    IntersectionHandler int_h_;
    
    SpeedAttribute::Description element_pre_, element_now_;
    
    SpeedSettings speed_settings_;
    
    // ROS-related members
    
    string base_link_, map_id_;
    
    ros::NodeHandle nh_;
    
    ros::Subscriber global_plan_;
    
    ros::Subscriber move_base_speed_;
    
    ros::Subscriber rrts_sub_;
    
    ros::Subscriber slowzone_sub_;
    
    ros::Publisher left_blinker_pub_, right_blinker_pub_;
    
    ros::Publisher bwd_drive_pub_;
    
    ros::Publisher recommend_speed_pub_;
    
    ros::Publisher speed_contribute_pub_;
    
    ros::Time last_update_;
    
    ros::Timer timer_;
    
    tf::TransformListener tf_;
    
    pnc_msgs::move_status move_status_;
    
    rrts::rrts_status rrts_status_;
    
    geometry_msgs::PoseArray slowZone_;
    
public:
    
    SpeedAdvisor();
    
private:
    
    // Constructors
    
    void ControlLoop( const ros::TimerEvent& event);
    
    // Callback functions
    
    bool getRobotGlobalPose( tf::Stamped<tf::Pose>& odom_pose) const;
    
    void moveSpeedCallback( pnc_msgs::move_status status);
    
    void rrts_callback(const rrts::rrts_status &);
    
    void slowZoneCallback( geometry_msgs::PoseArrayConstPtr slowzones);
    
};


#endif /* SPEED_ADVISOR_DIRECT_H_ */
