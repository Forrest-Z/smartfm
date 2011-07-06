#ifndef __TRACK_ESTIMATION_NODE__
#define __TRACK_ESTIMATION_NODE__

#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
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

#include "track_estimation.h"
#include "rolling_window.h"

namespace estimation{
	
	typedef boost::shared_ptr<nav_msgs::Odometry const> OdomConstPtr;

	class TrackEstimationNode{

	   public:	   
	   TrackEstimationNode();	   
	   ~TrackEstimationNode();

		private:
		//to get scan readings in "base_link" frame;
		message_filters::Subscriber<sensor_msgs::PointCloud> cloud_sub_;
		tf::TransformListener tf_;
		tf::MessageFilter<sensor_msgs::PointCloud> * tf_filter_;
	   ros::Publisher cloud_baselink_pub_;
		sensor_msgs::PointCloud cloud_baselink_;

		void cloudCallback (const sensor_msgs::PointCloud::ConstPtr& cloud_in);

		void RcurbCallback(const sensor_msgs::PointCloud::ConstPtr&  Rcurb);
      void LcurbCallback(const sensor_msgs::PointCloud::ConstPtr&  Lcurb);

		void odomCallback(const OdomConstPtr& odom);

		void RcurbProcess();
      void LcurbProcess();
		
		TrackEstimation 								R_filter_, L_filter_;
		rolling_window									R_window_, L_window_;
		ros::Subscriber 								Rcurb_sub_, Lcurb_sub_, odom_sub_;
		sensor_msgs::PointCloud 					Rcurb_line_, Lcurb_line_;

		tf::Transformer 								transformer_;

		tf::StampedTransform 						Rodom_meas_, Rodom_meas_old_, Rodom_meas_2old_, Rodom_meas_3old_;
		tf::StampedTransform 						Lodom_meas_, Lodom_meas_old_, Lodom_meas_2old_, Lodom_meas_3old_;

		road_detection::vec_point 					Rcurb_observation_, Lcurb_observation_;

		float 											Rtx_, Rty_, Rdelt_phi_;
      float 											Ltx_, Lty_, Ldelt_phi_;
      float  											Rassociation_gate_, Lassociation_gate_;
		bool                                   Rmeas_update_, Lmeas_update_;		
		bool 												Rmeas_exist_, Lmeas_exist_;
		bool 												Rreinitial_, Lreinitial_; 
		bool                                   Rtrack_output_cred_, Ltrack_output_cred_;
		
		//"right_point_" and "left_point_" are output of EKF tracker;
		//"right_Ptcred_"and "left_Ptcred_" are credible but conservative output of EKF tracker that we can use for localization;
		sensor_msgs::PointCloud  				   right_point_, left_point_, right_Ptcred_, left_Ptcred_;
		ros::Publisher 								right_point_pub_, left_point_pub_, right_Ptcred_pub_, left_Ptcred_pub_;
		
		//complete raw information;
		sensor_msgs::PointCloud						cred_full_right_curb_, cred_full_left_curb_;
		ros::Publisher									cred_full_right_pub_, cred_full_left_pub_;
	};
};
#endif //  
