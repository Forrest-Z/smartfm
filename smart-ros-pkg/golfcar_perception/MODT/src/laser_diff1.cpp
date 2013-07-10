#include <ros/ros.h>
#include <diagnostic_updater/publisher.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

#include <tf/message_filter.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

#include <laser_geometry/laser_geometry.h>

#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PolygonStamped.h>

#include <opencv/cv.h>
#include <opencv/highgui.h>

#include <boost/thread.hpp>
#include <boost/shared_ptr.hpp>

#include <vector>
#include <cmath>

using namespace std;
using namespace ros;
using namespace tf;
using namespace cv;

typedef boost::shared_ptr<nav_msgs::Odometry const> OdomConstPtr;

class laser_diff
{

public:
	laser_diff();

private:

	ros::NodeHandle nh_;
	ros::NodeHandle private_nh_;

	std::string base_frame_id_;
	std::string odom_frame_id_;

	laser_geometry::LaserProjection                         projector_;
	tf::TransformListener tf_;

	message_filters::Subscriber<sensor_msgs::LaserScan>     *laser_sub0_;
	message_filters::Subscriber<sensor_msgs::LaserScan>     *laser_sub1_;
	message_filters::Subscriber<sensor_msgs::LaserScan>     *laser_sub2_;
	message_filters::Subscriber<sensor_msgs::LaserScan>     *laser_sub3_;
	message_filters::TimeSynchronizer<sensor_msgs::LaserScan, sensor_msgs::LaserScan, sensor_msgs::LaserScan, sensor_msgs::LaserScan> *sync_;
	void scanProcess (const sensor_msgs::LaserScan::ConstPtr& scan_in0, const sensor_msgs::LaserScan::ConstPtr& scan_in1,
					  const sensor_msgs::LaserScan::ConstPtr& scan_in2, const sensor_msgs::LaserScan::ConstPtr& scan_in3);
	void extract_moving_points ();
	bool pointInPolygon(geometry_msgs::Point32 p, vector<geometry_msgs::Point32> poly);

	tf::MessageFilter<sensor_msgs::LaserScan>				*tf_filter_;
	void scanCallback (const sensor_msgs::LaserScan::ConstPtr& scan_in);

	void DP_segment (sensor_msgs::PointCloud scan_pointcloud, geometry_msgs::PoseStamped laser_origin);

	ros::Publisher 											vertical_laser_pub_;

	size_t 													interval_;
	double 													shrink_range_;

	vector<sensor_msgs::LaserScan>                         	scan_vector_;
	vector<sensor_msgs::PointCloud>                        	raw_cloud_vector_;
	vector<sensor_msgs::PointCloud>                        	shrink_cloud_vector_;

	vector<geometry_msgs::PoseStamped>						laser_pose_vector_;

	sensor_msgs::LaserScan                                  laser_scan_;
	sensor_msgs::PointCloud                                 current_laser_cloud_;
	geometry_msgs::PoseStamped								current_laser_pose_;

	ros::Publisher                              			moving_cloud_pub_;
	ros::Publisher                              			process_laser_pub_;

	ros::Publisher                              			polygon_t0_pub_;
	ros::Publisher                              			polygon_t2_pub_;

	ros::Publisher                              			approx_polygon_pub_;
};

laser_diff::laser_diff()
: private_nh_("~")
{
	private_nh_.param("base_frame_id",      base_frame_id_,     std::string("base_link"));
	private_nh_.param("odom_frame_id",      odom_frame_id_,     std::string("odom"));

	int inverval_tmp;
	private_nh_.param("interval",		    inverval_tmp,       	10);
	interval_ = (size_t) inverval_tmp;

	//corresponding to minimum/maximum speed;
	private_nh_.param("shrink_range",		shrink_range_,      0.2);

	moving_cloud_pub_   =   nh_.advertise<sensor_msgs::PointCloud>("moving_cloud", 2);
	process_laser_pub_  =   nh_.advertise<sensor_msgs::PointCloud>("laser_under_process", 2);

	polygon_t0_pub_		=   nh_.advertise<geometry_msgs::PolygonStamped>("polygon_t0", 2);
	polygon_t2_pub_		=   nh_.advertise<geometry_msgs::PolygonStamped>("polygon_t2", 2);

	approx_polygon_pub_		=   nh_.advertise<geometry_msgs::PolygonStamped>("approx_polygon", 2);

	vertical_laser_pub_  =   nh_.advertise<sensor_msgs::LaserScan>("vertical_laser0", 2);

	laser_sub0_ = new message_filters::Subscriber<sensor_msgs::LaserScan> (nh_, "/sickldmrs/scan0", 100);
	laser_sub1_ = new message_filters::Subscriber<sensor_msgs::LaserScan> (nh_, "/sickldmrs/scan1", 100);
	laser_sub2_ = new message_filters::Subscriber<sensor_msgs::LaserScan> (nh_, "/sickldmrs/scan2", 100);
	laser_sub3_ = new message_filters::Subscriber<sensor_msgs::LaserScan> (nh_, "/sickldmrs/scan3", 100);
	sync_	= new message_filters::TimeSynchronizer<sensor_msgs::LaserScan, sensor_msgs::LaserScan, sensor_msgs::LaserScan, sensor_msgs::LaserScan>(*laser_sub0_, *laser_sub1_, *laser_sub2_, *laser_sub3_, 10);
	sync_->registerCallback(boost::bind(&laser_diff::scanProcess, this, _1, _2, _3, _4));

	tf_filter_ = new tf::MessageFilter<sensor_msgs::LaserScan>(*laser_sub1_, tf_, odom_frame_id_, 10);
	tf_filter_->registerCallback(boost::bind(&laser_diff::scanCallback, this, _1));
	tf_filter_->setTolerance(ros::Duration(0.05));
}


void laser_diff::scanProcess (const sensor_msgs::LaserScan::ConstPtr& scan_in0, const sensor_msgs::LaserScan::ConstPtr& scan_in1,  const sensor_msgs::LaserScan::ConstPtr& scan_in2,  const sensor_msgs::LaserScan::ConstPtr& scan_in3)
{
	//1st step: extract vertical laser parts;
	sensor_msgs::LaserScan vertical_laser;
	vertical_laser = *scan_in1;
	float thetha = 0.8*M_PI/180.0;
	for(size_t i=0; i<vertical_laser.ranges.size(); i++)
	{
		float scan_range0 = scan_in0->ranges[i];
		float scan_range1 = scan_in1->ranges[i];
		float scan_range2 = scan_in2->ranges[i];
		float scan_range3 = scan_in3->ranges[i];

		float longer_range, shorter_range;
		if(scan_range0 > scan_in0->range_min)
		{
			longer_range = scan_range0 > scan_range1 ? scan_range0:scan_range1;
			shorter_range = scan_range0 <= scan_range1 ? scan_range0:scan_range1;
		}
		else if(scan_range2 > scan_in0->range_min)
		{
			longer_range = scan_range2 > scan_range3 ? scan_range2:scan_range3;
			shorter_range = scan_range2 <= scan_range3 ? scan_range2:scan_range3;
		}

		if(shorter_range<scan_in0->range_min)vertical_laser.ranges[i] = 0.0;
		else
		{
			float angle = atan2(longer_range*thetha, longer_range-shorter_range)+thetha/2.0;
			if(angle<M_PI/18.0) vertical_laser.ranges[i] = 0.0;
			else
			{
				if(scan_range2 > scan_in0->range_min) vertical_laser.ranges[i] = scan_in2->ranges[i];
			}
		}
	}
	vertical_laser_pub_.publish(vertical_laser);



	try{projector_.transformLaserScanToPointCloud(odom_frame_id_, *scan_in1, current_laser_cloud_, tf_);}
	catch (tf::TransformException& e){ROS_DEBUG("Wrong!!!!!!!!!!!!!"); std::cout << e.what();return;}

	//laser_scan_ = *scan_in1;
	laser_scan_ = vertical_laser;

	sensor_msgs::LaserScan shrink_scan = laser_scan_;
	sensor_msgs::PointCloud current_shrink_pointcloud;
	for(size_t i=0; i<shrink_scan.ranges.size();i++)
	{
		shrink_scan.ranges[i] = (shrink_scan.ranges[i]-shrink_range_)>0.0 ? (shrink_scan.ranges[i]-shrink_range_) : 0.0;
	}

	try{projector_.transformLaserScanToPointCloud(odom_frame_id_, shrink_scan, current_shrink_pointcloud, tf_);}
	catch (tf::TransformException& e){ROS_DEBUG("Wrong!!!!!!!!!!!!!"); std::cout << e.what();return;}

	geometry_msgs::PoseStamped ident;
	ident.header = laser_scan_.header;
	ident.pose.position.x=0;
	ident.pose.position.y=0;
	ident.pose.position.z=0;
	ident.pose.orientation.x=1;
	ident.pose.orientation.y=0;
	ident.pose.orientation.z=0;
	ident.pose.orientation.w=0;
	try
	{
		this->tf_.transformPose(odom_frame_id_, ident, current_laser_pose_);
	}
	catch(tf::TransformException e)
	{
		ROS_WARN("Failed to compute odom pose, skipping scan (%s)", e.what());
		return;
	}

	DP_segment(current_laser_cloud_, current_laser_pose_);

	scan_vector_.push_back(laser_scan_);
	laser_pose_vector_.push_back(current_laser_pose_);
	raw_cloud_vector_.push_back(current_laser_cloud_);
	shrink_cloud_vector_.push_back(current_shrink_pointcloud);
	//ROS_INFO("%ld, %ld",raw_cloud_vector_.size(), laser_pose_vector_.size());

	assert(scan_vector_.size() == laser_pose_vector_.size());
	assert(raw_cloud_vector_.size() == laser_pose_vector_.size());
	assert(shrink_cloud_vector_.size() == laser_pose_vector_.size());

	if(scan_vector_.size()==interval_*2+1)
	{
		extract_moving_points();
		laser_pose_vector_.erase(laser_pose_vector_.begin());
		scan_vector_.erase(scan_vector_.begin());
		raw_cloud_vector_.erase(raw_cloud_vector_.begin());
		shrink_cloud_vector_.erase(shrink_cloud_vector_.begin());
	}
}

void laser_diff::DP_segment (sensor_msgs::PointCloud scan_pointcloud, geometry_msgs::PoseStamped laser_origin)
{
	vector<Point2f> raw_cloud;
	for(size_t i=0; i<scan_pointcloud.points.size(); i++)
	{
		Point2f point_tmp;
		point_tmp.x = scan_pointcloud.points[i].x;
		point_tmp.y = scan_pointcloud.points[i].y;
		raw_cloud.push_back(point_tmp);
	}
	vector<Point2f> approxy_cloud;
	approxPolyDP( Mat(raw_cloud), approxy_cloud, 0.3, false );

	geometry_msgs::PolygonStamped	approxy_polygon;
	approxy_polygon.header = scan_pointcloud.header;
	geometry_msgs::Point32 point_tmp;
	for(size_t i=0; i<approxy_cloud.size(); i++)
	{

		point_tmp.x = approxy_cloud[i].x;
		point_tmp.y = approxy_cloud[i].y;
		point_tmp.z = laser_origin.pose.position.z;
		approxy_polygon.polygon.points.push_back(point_tmp);
	}
	point_tmp.x = laser_origin.pose.position.x;
	point_tmp.y = laser_origin.pose.position.y;
	point_tmp.z = laser_origin.pose.position.z;
	approxy_polygon.polygon.points.push_back(point_tmp);

	approx_polygon_pub_.publish(approxy_polygon);
}

void laser_diff::extract_moving_points ()
{
	ROS_INFO("extracting moving points!");
	sensor_msgs::PointCloud  	cloud_t1 		= 	raw_cloud_vector_[interval_];

	sensor_msgs::PointCloud  shrink_cloud_t0 	= 	shrink_cloud_vector_.front();
	sensor_msgs::PointCloud  shrink_cloud_t2 	= 	shrink_cloud_vector_.back();
	geometry_msgs::PoseStamped 	laser_pose_t0	=	laser_pose_vector_.front();
	geometry_msgs::PoseStamped 	laser_pose_t2	=	laser_pose_vector_.back();

	geometry_msgs::Point32 	 origin_t0;
	origin_t0.x = laser_pose_t0.pose.position.x;
	origin_t0.y = laser_pose_t0.pose.position.y;
	origin_t0.z = laser_pose_t0.pose.position.z;
	geometry_msgs::PolygonStamped	poly_shrink_t0;
	poly_shrink_t0.header 	= current_laser_cloud_.header;
	poly_shrink_t0.polygon.points	= 	shrink_cloud_t0.points;
	poly_shrink_t0.polygon.points.push_back(origin_t0);

	geometry_msgs::Point32 	 origin_t2;
	origin_t2.x = laser_pose_t2.pose.position.x;
	origin_t2.y = laser_pose_t2.pose.position.y;
	origin_t2.z = laser_pose_t2.pose.position.z;
	geometry_msgs::PolygonStamped	poly_shrink_t2;
	poly_shrink_t2.header 	= current_laser_cloud_.header;
	poly_shrink_t2.polygon.points	= 	shrink_cloud_t2.points;
	poly_shrink_t2.polygon.points.push_back(origin_t2);

	sensor_msgs::PointCloud moving_cloud_t1;
	moving_cloud_t1.header = current_laser_cloud_.header;

	for(size_t i=0; i<cloud_t1.points.size(); i++)
	{
		geometry_msgs::Point32 point_tmp = cloud_t1.points[i];
		bool moving_towards = pointInPolygon(point_tmp, poly_shrink_t0.polygon.points);
		bool moving_outwards = pointInPolygon(point_tmp, poly_shrink_t2.polygon.points);
		moving_outwards = false;
		if(moving_towards||moving_outwards)
		{
			moving_cloud_t1.points.push_back(point_tmp);
		}
	}

	cloud_t1.header = current_laser_cloud_.header;
	moving_cloud_pub_.publish(moving_cloud_t1);
	process_laser_pub_.publish(cloud_t1);
	polygon_t0_pub_.publish(poly_shrink_t0);
	polygon_t2_pub_.publish(poly_shrink_t2);
}

//http://alienryderflex.com/polygon/
inline bool laser_diff::pointInPolygon(geometry_msgs::Point32 p, vector<geometry_msgs::Point32> poly)
{
	int polySides = poly.size();
	int      i, j=polySides-1 ;
	bool  oddNodes = false;

	for (i=0; i<polySides; i++) {
		if (((poly[i].y< p.y && poly[j].y>=p.y)
		      ||   (poly[j].y< p.y && poly[i].y>=p.y))
				&&  (poly[i].x<=p.x || poly[j].x<=p.x)) {
			          if(poly[i].x+(p.y-poly[i].y)/(poly[j].y-poly[i].y)*(poly[j].x-poly[i].x)<p.x)
						{oddNodes=!oddNodes;} 			}
		j=i; }

	return oddNodes;
}

//this function actually does nothing; just help to synchronize the tf and topics;
void laser_diff::scanCallback (const sensor_msgs::LaserScan::ConstPtr& scan_in){}


int main(int argc, char** argv)
{
	ros::init(argc, argv, "laser_diff_node");
	laser_diff laser_diff_node;
	ros::spin();
	return (0);
}
