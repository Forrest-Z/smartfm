#ifndef MVRRTS_V2_HPP
#define	MVRRTS_V2_HPP

// Header files: Standard C++
#include <ctime>
#include <cstdlib>
#include <iostream>

// Header files: ROS
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <message_filters/subscriber.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <std_msgs/Int16MultiArray.h>
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/Bool.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>

// Header files: ROS custom-made [SMART-FM]
#include <pnc_msgs/local_map.h>
#include <rrts/rrts_status.h>

// Header files: Custom [Luis]
#include "DubinsCar_MVRRTs.hpp"

class Golf_Cart_MVRRTs {
    
private:
    
    // --------------------------------------------------------------------------------------------
    // PRIVATE MEMBERS - RELATED TO LOCALIZATION
    // --------------------------------------------------------------------------------------------
    
    Map_Global map_global;
    
    Map_Local map_local;
    
    tf::TransformListener tf_listener_base_link_to_map;
    
    // --------------------------------------------------------------------------------------------
    // PRIVATE MEMBERS - RELATED TO THE MVRRT* ALGORITHM
    // --------------------------------------------------------------------------------------------
    
    DubinsCar golf_cart;
    
    format_t robot_state [3];
    
    DubRegion_Cylindrical goal_region;
    
    DubinsCar_MVRRTs * mvrrts;
    
    rrts::rrts_status mvrrts_status_msg;
    
    trajectory_t commited_traj;
    
    // --------------------------------------------------------------------------------------------
    // PRIVATE MEMBERS - RELATED TO ROS FUNCTIONALITIES
    // --------------------------------------------------------------------------------------------
    
    ros::NodeHandle node;
    
    ros::Publisher pub_committed_controls;
    
    ros::Publisher pub_committed_traj;
    
    ros::Publisher pub_committed_traj_RVIZ;
    
    ros::Publisher pub_mvrrts_nodes;
    
    ros::Publisher pub_mvrrts_tree;
    
    ros::Publisher pub_planner_status;
    
    ros::Subscriber sub_goal;
    
    ros::Subscriber sub_map_global;
    
    ros::Subscriber sub_map_local;
    
    ros::Timer timer_pub_committed_traj;
    
    ros::Timer timer_pub_mvrrts;
    
    ros::Time time_last_replan;
    
    // --------------------------------------------------------------------------------------------
    // PRIVATE MEMBERS - SWITCHES
    // --------------------------------------------------------------------------------------------
    
    volatile bool locked_committed_traj;
    
    volatile bool locked_tree;
    
    // --------------------------------------------------------------------------------------------
    // PRIVATE MEMBERS - FOR DEBUGGING PURPOSES
    // --------------------------------------------------------------------------------------------
    
    ros::Timer DEBUG_TIMER;
    
public:
    
    Golf_Cart_MVRRTs( const format_t turn_radius, 
            
                      const format_t delta_x, 
            
                      const Auto_AB& auto_AB);
    
    ~Golf_Cart_MVRRTs();
    
private:
    
    void callback_for_committed_traj( const ros::TimerEvent& event);
    
    void callback_for_goal( const geometry_msgs::PoseStamped::ConstPtr msg);
    
    void callback_for_map_global( const nav_msgs::OccupancyGrid& msg);
    
    void callback_for_map_local( const pnc_msgs::local_map& msg);
    
    void callback_for_mvrrts( const ros::TimerEvent& event);
    
    bool get_robot_state();
    
    void publish_committed_trajectory();
    
    void publish_tree();
    
    bool search_for_plan();
    
    void DEBUG_CALLBACK( const ros::TimerEvent& event);
    
};

#endif	/* MVRRTS_V2_HPP */
