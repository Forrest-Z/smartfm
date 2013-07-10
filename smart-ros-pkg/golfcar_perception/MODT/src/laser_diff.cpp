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
//using namespace tf;
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
	void vertical_laser_pair(const sensor_msgs::LaserScan::ConstPtr& scan_in_a, const sensor_msgs::LaserScan::ConstPtr& scan_in_b, sensor_msgs::LaserScan & vertical_scan);
	bool pointInPolygon(geometry_msgs::Point32 p, vector<geometry_msgs::Point32> poly);
	void DP_segment (sensor_msgs::PointCloud scan_pointcloud, geometry_msgs::PoseStamped laser_origin);
	void extract_moving_points();
	void spacePt2ImgP(geometry_msgs::Point32 & spacePt, Point & imgPt);
	void ImgPt2spacePt(Point & imgPt, geometry_msgs::Point32 & spacePt);

	void process_moving_image(Mat &img_moving, Mat &img_t);

	tf::MessageFilter<sensor_msgs::LaserScan>				*tf_filter_;
	void scanCallback (const sensor_msgs::LaserScan::ConstPtr& scan_in);

	ros::Publisher 											vertical_laser_pub_;
	ros::Publisher                              			approx_polygon_pub_;
	ros::Publisher 											accumulate_laser_pub_;

	size_t 													interval_;
	vector<sensor_msgs::PointCloud>                        	cloud_vector_;
	vector<geometry_msgs::PoseStamped>						laser_pose_vector_;
	geometry_msgs::PoseStamped								laser_pose_current_;

	double													img_side_length_, img_resolution_;
	Mat														accumulated_image_;
	Point													LIDAR_pixel_coord_;
};

laser_diff::laser_diff()
: private_nh_("~")
{
	private_nh_.param("base_frame_id",      base_frame_id_,     std::string("base_link"));
	private_nh_.param("odom_frame_id",      odom_frame_id_,     std::string("odom"));

	int inverval_tmp;
	private_nh_.param("interval",		    inverval_tmp,       	50);
	interval_ = (size_t) inverval_tmp;

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


	private_nh_.param("img_side_length",      img_side_length_,     50.0);
	private_nh_.param("img_resolution",       img_resolution_,     0.2);
	accumulated_image_		= Mat((int)(2*img_side_length_/img_resolution_), (int)(2*img_side_length_/img_resolution_), CV_8UC1);
	LIDAR_pixel_coord_.x 	= (int)(img_side_length_/img_resolution_)-1;
	LIDAR_pixel_coord_.y 	= (int)(img_side_length_/img_resolution_)-1;
}


void laser_diff::scanProcess (const sensor_msgs::LaserScan::ConstPtr& scan_in0, const sensor_msgs::LaserScan::ConstPtr& scan_in1,  const sensor_msgs::LaserScan::ConstPtr& scan_in2,  const sensor_msgs::LaserScan::ConstPtr& scan_in3)
{
	sensor_msgs::LaserScan verti_laser1, verti_laser2;
	vertical_laser_pair(scan_in1, scan_in0, verti_laser1);
	vertical_laser_pair(scan_in2, scan_in3, verti_laser2);
	sensor_msgs::PointCloud verti_cloud1, verti_cloud2, verti_cloud;

	try{projector_.transformLaserScanToPointCloud(odom_frame_id_, verti_laser1, verti_cloud1, tf_);}
	catch (tf::TransformException& e){ROS_DEBUG("Wrong!!!!!!!!!!!!!"); std::cout << e.what();return;}
	try{projector_.transformLaserScanToPointCloud(odom_frame_id_, verti_laser2, verti_cloud2, tf_);}
	catch (tf::TransformException& e){ROS_DEBUG("Wrong!!!!!!!!!!!!!"); std::cout << e.what();return;}

	verti_cloud = verti_cloud1;
	for(size_t i=0; i<verti_cloud2.points.size(); i++)verti_cloud.points.push_back(verti_cloud2.points[i]);
	cloud_vector_.push_back(verti_cloud);

	geometry_msgs::PoseStamped ident;
	ident.header = scan_in0->header;
	ident.pose.position.x=0;
	ident.pose.position.y=0;
	ident.pose.position.z=0;
	ident.pose.orientation.x=1;
	ident.pose.orientation.y=0;
	ident.pose.orientation.z=0;
	ident.pose.orientation.w=0;
	try
	{
		this->tf_.transformPose(odom_frame_id_, ident, laser_pose_current_);
	}
	catch(tf::TransformException e)
	{
		ROS_WARN("Failed to compute odom pose, skipping scan (%s)", e.what());
		return;
	}
	laser_pose_vector_.push_back(ident);

	DP_segment(verti_cloud, laser_pose_current_);

	assert(cloud_vector_.size() == laser_pose_vector_.size());

	if(cloud_vector_.size()==interval_*2)
	{
		extract_moving_points();
		cloud_vector_.erase(cloud_vector_.begin());
		laser_pose_vector_.erase(laser_pose_vector_.begin());
	}
}

void laser_diff::extract_moving_points()
{
	accumulated_image_ = Scalar(0);
	Mat current_image = accumulated_image_.clone();
	Mat image_t = accumulated_image_.clone();
	for(size_t i=0; i<interval_; i++)
	{
		for(size_t j=0; j<cloud_vector_[i].points.size(); j++)
		{
			geometry_msgs::Point32 spacePt_tmp = cloud_vector_[i].points[j];
			Point imgpt_tmp;
			spacePt2ImgP(spacePt_tmp, imgpt_tmp);
			if(imgpt_tmp.y>= 0 && imgpt_tmp.y <accumulated_image_.rows && imgpt_tmp.x>= 0 && imgpt_tmp.x <accumulated_image_.cols)
			{
				if(accumulated_image_.at<uchar>(imgpt_tmp.y,imgpt_tmp.x) <255) accumulated_image_.at<uchar>(imgpt_tmp.y,imgpt_tmp.x) = accumulated_image_.at<uchar>(imgpt_tmp.y,imgpt_tmp.x)+1;

			}
		}
	}

	for(size_t i=interval_; i<2*interval_; i++)
	{
		for(size_t j=0; j<cloud_vector_[i].points.size(); j++)
		{
			geometry_msgs::Point32 spacePt_tmp = cloud_vector_[i].points[j];
			Point imgpt_tmp;
			spacePt2ImgP(spacePt_tmp, imgpt_tmp);
			if(imgpt_tmp.y>= 0 && imgpt_tmp.y <accumulated_image_.rows && imgpt_tmp.x>= 0 && imgpt_tmp.x <accumulated_image_.cols)
			{
				if(current_image.at<uchar>(imgpt_tmp.y,imgpt_tmp.x) <255) current_image.at<uchar>(imgpt_tmp.y,imgpt_tmp.x) = current_image.at<uchar>(imgpt_tmp.y,imgpt_tmp.x)+1;
				if(i==2*interval_-1)
				{
					if(image_t.at<uchar>(imgpt_tmp.y,imgpt_tmp.x) <255) image_t.at<uchar>(imgpt_tmp.y,imgpt_tmp.x) = image_t.at<uchar>(imgpt_tmp.y,imgpt_tmp.x)+1;
				}
			}
		}
	}


	threshold( current_image, current_image, 0, 255, 0 );
	threshold( image_t, image_t, 0, 255, 0 );
	imshow("current_image", current_image);
	imshow("image_t", image_t);
	waitKey(1);

	threshold(accumulated_image_, accumulated_image_, 0, 255, 0);


	int dilation_size = 1;
	Mat element = getStructuringElement( MORPH_RECT, Size( 2*dilation_size + 1, 2*dilation_size+1 ), Point( dilation_size, dilation_size ) );
	dilate(accumulated_image_, accumulated_image_, element);
	threshold(accumulated_image_, accumulated_image_, 0, 255, 1);
	imshow("accumulated_image_", accumulated_image_);
	waitKey(1);

	/*
	 * try opencv API for scan matching, it helps, but performance not robust;
	 * 1) it turns out not robust, the estimation is not consistent but jumps a lot;
	 * 2) it will try to match moving points when other surrounding features not enough, since no odometry information used;
	 */

	/*
	ROS_INFO("1");
	Mat affine_tranform = estimateRigidTransform(accumulated_image_, current_image, false);
	warpAffine(accumulated_image_, accumulated_image_, affine_tranform, accumulated_image_.size());
	cout << affine_tranform << endl;
	ROS_INFO("2");
	*/

	bitwise_and(accumulated_image_, current_image, current_image);
	morphologyEx(current_image, current_image, MORPH_CLOSE, element);

	imshow("moving_objects", current_image);
	waitKey(1);

	process_moving_image(current_image, image_t);

}

void laser_diff::process_moving_image(Mat &img_moving, Mat &img_t)
{
	Mat moving_tmp = img_moving.clone();
	vector<vector<Point> > contours;
	vector<Vec4i> hierarchy;
	findContours( moving_tmp, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE, Point(0,0) );

	vector<Moments> mu(contours.size() );
	for( int i = 0; i < contours.size(); i++ ) {mu[i] = moments( contours[i], false );}
	vector<double> area( contours.size() );
	for( int i = 0; i < contours.size(); i++ ) {area[i] = contourArea(contours[i]);}
	vector<double> length( contours.size() );
	for( int i = 0; i < contours.size(); i++ ) {length[i] = arcLength(contours[i], true);}

	Mat moving_tmp2 = img_moving.clone();
	int dilation_size = 2;
	Mat element = getStructuringElement( MORPH_RECT, Size( 2*dilation_size + 1, 2*dilation_size+1 ), Point( dilation_size, dilation_size ) );
	morphologyEx(moving_tmp2, moving_tmp2, CV_MOP_OPEN, element);
	imshow("erode", moving_tmp2);
	waitKey(1);
}



void laser_diff::spacePt2ImgP(geometry_msgs::Point32 & spacePt, Point & imgPt)
{
	imgPt.x = int((spacePt.x - laser_pose_current_.pose.position.x)/img_resolution_) + LIDAR_pixel_coord_.x;
	imgPt.y =  accumulated_image_.rows -( int((spacePt.y - laser_pose_current_.pose.position.y)/img_resolution_) + LIDAR_pixel_coord_.y);
}
void laser_diff::ImgPt2spacePt(Point & imgPt, geometry_msgs::Point32 & spacePt)
{
	spacePt.x = (imgPt.x-LIDAR_pixel_coord_.x)*img_resolution_+laser_pose_current_.pose.position.x;
	spacePt.y = (accumulated_image_.rows - imgPt.y - LIDAR_pixel_coord_.y)*img_resolution_+laser_pose_current_.pose.position.y;
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

void laser_diff::vertical_laser_pair(const sensor_msgs::LaserScan::ConstPtr& scan_in_a, const sensor_msgs::LaserScan::ConstPtr& scan_in_b, sensor_msgs::LaserScan & vertical_scan)
{
	vertical_scan = *scan_in_a;
	float thetha = 0.8*M_PI/180.0;
	for(size_t i=0; i<vertical_scan.ranges.size(); i++)
	{
		float scan_range0 = scan_in_a->ranges[i];
		float scan_range1 = scan_in_b->ranges[i];
		float longer_range, shorter_range;
		longer_range = scan_range0 > scan_range1 ? scan_range0:scan_range1;
		shorter_range = scan_range0 <= scan_range1 ? scan_range0:scan_range1;

		if(shorter_range<scan_in_a->range_min)vertical_scan.ranges[i] = 0.0;
		else
		{
			float angle = atan2(longer_range*thetha, longer_range-shorter_range)+thetha/2.0;
			if(angle<M_PI/4.0) vertical_scan.ranges[i] = 0.0;
		}
	}
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
