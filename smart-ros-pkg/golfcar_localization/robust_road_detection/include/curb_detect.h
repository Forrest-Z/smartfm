/* NUS_Golf_Cart 2011-11-30
 * Description: "curb_detect" is 2nd version of original "road_detect";
 * adopting a more robust, neat, and systematic way to detect curb features, road_surface (by-product), and so on.
 */

#ifndef __CURB_DETECT__
#define __CURB_DETECT__

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
#include <boost/thread.hpp>
#include <boost/shared_ptr.hpp>
#include "tf/message_filter.h"
#include "message_filters/subscriber.h"


namespace road_detection{

typedef boost::shared_ptr<nav_msgs::Odometry const> OdomConstPtr;

struct pair_ids
{unsigned int front_id;
 unsigned int back_id;
};

class curb_detect{

public:
	curb_detect();
	~curb_detect();

private:

    ros::NodeHandle nh_;	
    ros::NodeHandle private_nh_;

    std::string base_frame_id_;
    std::string odom_frame_id_;
    int     disFirst_multi_;
	double  disConti_thresh_;
	int     rdPtNum_thresh_;
	double  rdWidth_thresh_;
	double  slope_thresh_;
    double  mergeAngle_thresh1_;
    double  mergeAngle_thresh2_;
    int     mergeNum_thresh2_;
	double  curbLow_thresh_;
	double  curbHigh_thresh_;
	double  curbAngle_thresh_;
    double  yLength_thresh_;
    double  crossing_thresh_;
    double  curbFilter_reinit_thresh_;
    double  curbFilter_assoc_thresh_;
    double  publish_dis_thresh_;
    double  publish_angle_thresh_;
    float   looking_forward_distance_;

	laser_geometry::LaserProjection                         projector_;
	message_filters::Subscriber<sensor_msgs::LaserScan>     laser_sub_;
	tf::TransformListener tf_;
	tf::MessageFilter<sensor_msgs::LaserScan>               *tf_filter_;

    //Main Part 0: subscribe to laser topic and do transformation
    void scanCallback (const sensor_msgs::LaserScan::ConstPtr& scan_in);
    sensor_msgs::LaserScan			                        laser_scan_;
	sensor_msgs::PointCloud                                 laser_cloud_laser_;
	sensor_msgs::PointCloud                                 laser_cloud_base_;
    sensor_msgs::PointCloud                                 laser_cloud_odom_;
	void scanProcessing();

    //1st subfunction: detect all discontinuous points, noted as DP;
	void DP_Extraction();
    //first-order discontinuous points DP1;
    std::vector<unsigned int> DP1_IDs_;
	sensor_msgs::PointCloud DP1_pcl_;
	ros::Publisher DP1_pub_;
    //second-order discontinuous points DP2;
	std::vector<unsigned int> DP2_IDs_;
	sensor_msgs::PointCloud DP2_pcl_;
	ros::Publisher DP2_pub_;
	//to visulize data processing of DP2; x Range, y Angle, noted as RA; "filter response" is noted as FR
	sensor_msgs::PointCloud raw_RA_pcl_;
	ros::Publisher raw_RA_pub_;
	sensor_msgs::PointCloud FR_RA_pcl_;
	ros::Publisher FR_RA_pub_;
	sensor_msgs::PointCloud DP_RA_pcl_;
	ros::Publisher DP_RA_pub_;

	//2nd subfunction: select two road_bounday points from DP_IDs_;
	void Road_Selection();
    bool Road_Cand_Verify(pair_ids &road_cand);
    pair_ids road_surface_;
    //road being extracted is one easily ignored prerequisite;
    bool road_extracted_;
    sensor_msgs::PointCloud road_boundary_pcl_;     //only two points.
	ros::Publisher road_boundary_pub_;

    //3rd subfunction: select two curb lines from DP_IDs_;
	void Curb_Searching();
    pair_ids rightCurb_cand_;
    pair_ids leftCurb_cand_;  
    bool Merge_Road_Curb(pair_ids &road_line,  pair_ids &curb_line);
    //using three pairs of flags to deal with various situations, in addition to flag "road_extracted_";
    bool right_break_;    
    bool left_break_;   
    bool right_crossing_;   
    bool left_crossing_; 
    bool right_curb_;
    bool left_curb_;   
   
    void Curb_Verification();
    bool Curb_Cand_Verify(pair_ids &road_surface, pair_ids &curb_cand);
    
    int  DP1_Find (pair_ids &curb_cand);

	sensor_msgs::PointCloud                     left_curbline_pcl_;
	sensor_msgs::PointCloud                     right_curbline_pcl_;
	ros::Publisher                              left_curbline_pub_;
	ros::Publisher                              right_curbline_pub_;
    sensor_msgs::PointCloud                     left_crossing_pcl_;
	sensor_msgs::PointCloud                     right_crossing_pcl_;
	ros::Publisher                              left_crossing_pub_;
	ros::Publisher                              right_crossing_pub_;

    bool Curb_Noise_Filtering( tf::StampedTransform &odom_meas_curb_old, std::vector <geometry_msgs::Point32> & curb_line, geometry_msgs::Point32 &meas_pt, unsigned int serial_num);
    bool RCurb_Noise_Filtering(geometry_msgs::Point32 &meas_pt, unsigned int serial_num);    
    bool LCurb_Noise_Filtering(geometry_msgs::Point32 &meas_pt, unsigned int serial_num);
    std::vector <geometry_msgs::Point32>        right_curb_line_;
    std::vector <geometry_msgs::Point32>        left_curb_line_;
    float Pt_Line_Dis(std::vector <geometry_msgs::Point32> &line, geometry_msgs::Point32 &pt);
    bool Eliminate_noise_other_sensors(unsigned int serial);
    
    void scanSource(const sensor_msgs::LaserScan::ConstPtr& scan_in);
    ros::Subscriber                             scan_sub_;
    sensor_msgs::PointCloud                     otherSource_pcl_;
    double                                      srcUse_thresh_;
    double                                      srcFilter_thresh_;
    double                                      filterHeight_thresh_;
    

    //publish at some distance interval;
    void publish_control();
    sensor_msgs::PointCloud                     hybrid_pcl_;
    ros::Publisher                              hybrid_pub_;
    ros::Subscriber                             odom_sub_;
    tf::Transformer 							transformer_;
    tf::StampedTransform 						odom_meas_, odom_meas_old_, odom_meas_RCurb_old_, odom_meas_LCurb_old_;
    bool                                        pub_init_;
    bool                                        publish_flag_;
    void odomCallback(const OdomConstPtr& odom);
};
};

#endif 
