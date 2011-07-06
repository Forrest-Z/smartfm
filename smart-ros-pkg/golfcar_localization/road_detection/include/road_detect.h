#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/LaserScan.h>
#include <laser_geometry/laser_geometry.h>
#include <sensor_msgs/PointCloud.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/PointStamped.h>
#include <vector>

// Thread suppport
#include <boost/thread.hpp>
#include <boost/shared_ptr.hpp>

//add message_filter
#include "tf/message_filter.h"
#include "message_filters/subscriber.h"

namespace road_detection{

	class road_detect{

	   public:	   
	   road_detect();	   
	   ~road_detect();
	   
	   private:

           //laser geometry
	   laser_geometry::LaserProjection projector_;
	   sensor_msgs::PointCloud laser_cloud_;	
    	ros::Publisher laser_pub_;        
 	   message_filters::Subscriber<sensor_msgs::LaserScan> laser_sub_;
      tf::TransformListener tf_; 
      tf::MessageFilter<sensor_msgs::LaserScan> * tf_filter_;
	   void scanCallback (const sensor_msgs::LaserScan::ConstPtr& scan_in);
	   
	   //********************Main Part 1: process single scan*******************
	   void ProcessSingleScan();     
           
      //1st sub function: detect all discontinuities in (angle, distance) coordinate. 
      void CurbEtraction();
	   //float GuassianDifferentialFilter[7];
	   
      std::vector<unsigned int> curb_candidate_PIDs_;     //PID: point ID. 
	   sensor_msgs::PointCloud curb_candidates_;
	   ros::Publisher curb_pub_;  

      //For debugging purposes (1), to get a clear idea of data procession,
		//show in rviz "show_laser_cloud_", "show_filter_response_", "show_boundary_candidates_";
	   //x:(0~180); y: sick_distance; according to CMU paper;
	   sensor_msgs::PointCloud show_laser_cloud_;    
	   ros::Publisher show_laser_pub_;
      sensor_msgs::PointCloud show_filter_response_;
	   ros::Publisher show_response_pub_;  
      sensor_msgs::PointCloud show_curb_candidates_;
	   ros::Publisher show_curb_pub_;

	   //2nd sub function
	   void RoadSelection();
	   sensor_msgs::PointCloud road_boundary_;     //only two points.
      ros::Publisher boundary_pub_; 
	   unsigned int begin_end_PID_[2];

	   //3rd sub function
      void CurbSideLine();
	   sensor_msgs::PointCloud left_curb_line_;
   	sensor_msgs::PointCloud right_curb_line_;
	   ros::Publisher left_curbline_pub_;
      ros::Publisher right_curbline_pub_;
      tf::TransformListener sick_to_baselink_;
	  
		void tfSicktoBaselink(geometry_msgs::Point32& point_para,geometry_msgs::PointStamped& stampedpoint_para);

		//******see curb_track.cpp*********
	  	sensor_msgs::PointCloud cloud_baselink_;
     	ros::Publisher cloud_baselink_pub_;

 };
};

	
