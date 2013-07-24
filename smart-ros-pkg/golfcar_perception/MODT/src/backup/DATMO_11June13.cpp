#include <ros/ros.h>
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

#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>

#include <opencv/cv.h>
#include <opencv/highgui.h>

#include <boost/thread.hpp>
#include <boost/shared_ptr.hpp>

#include <vector>
#include <cmath>

using namespace std;
using namespace ros;
using namespace cv;
//using namespace tf;

typedef boost::shared_ptr<nav_msgs::Odometry const> OdomConstPtr;

class DATMO
{

public:
	DATMO();

private:

	ros::NodeHandle nh_;
	ros::NodeHandle private_nh_;

	std::string base_frame_id_;
	std::string odom_frame_id_;
	std::string map_frame_id_;

	tf::TransformListener tf_;
	message_filters::Subscriber<sensor_msgs::LaserScan>     *verti_laser_sub_;
	laser_geometry::LaserProjection                         projector_;

	void scanCallback (const sensor_msgs::LaserScan::ConstPtr& verti_scan_in);

	bool pointInPolygon(geometry_msgs::Point32 p, vector<geometry_msgs::Point32> poly);
	void DP_segment (sensor_msgs::PointCloud scan_pointcloud, geometry_msgs::PoseStamped laser_origin);
	void extract_moving_points();
	void GridSearch4BestAlignment(Mat& origninal_image, Mat& target_image, Mat& affine_transform);

	void spacePt2ImgP(geometry_msgs::Point32 & spacePt, Point & imgPt);
	void ImgPt2spacePt(Point & imgPt, geometry_msgs::Point32 & spacePt);
	void process_moving_image(Mat &img_moving, Mat &img_t);

	tf::MessageFilter<sensor_msgs::LaserScan>				*tf_filter_;
	ros::Publisher                              			approx_polygon_pub_;

	size_t 													interval_;
	vector<sensor_msgs::PointCloud>                        	cloud_vector_;
	vector<geometry_msgs::PoseStamped>						laser_pose_vector_;
	geometry_msgs::PoseStamped								laser_pose_current_;

	double													img_side_length_, img_resolution_;
	Mat														accumulated_image_;
	Point													LIDAR_pixel_coord_;

	Mat														road_map_prior_;
	std::string												map_image_path_;
	geometry_msgs::Point32									map_origin;
	double													map_resolution_;
	void													initialize_roadmap();
	bool 													check_onRoad(geometry_msgs::Point32 &point);


};

DATMO::DATMO()
: private_nh_("~")
{
	private_nh_.param("base_frame_id",      base_frame_id_,     std::string("base_link"));
	private_nh_.param("odom_frame_id",      odom_frame_id_,     std::string("odom"));
	private_nh_.param("map_frame_id",       map_frame_id_,      std::string("map"));

	private_nh_.param("map_resolution",     map_resolution_,    0.05);
	private_nh_.param("map_image_path",     map_image_path_,    std::string("./launch/road_map.jpg"));
	initialize_roadmap();

	int inverval_tmp;
	private_nh_.param("interval",		    inverval_tmp,       	50);
	interval_ = (size_t) inverval_tmp;

	approx_polygon_pub_		=   nh_.advertise<geometry_msgs::PolygonStamped>("approx_polygon", 2);

	verti_laser_sub_ = new message_filters::Subscriber<sensor_msgs::LaserScan> (nh_, "/sickldmrs/verti_laser", 100);
	tf_filter_ = new tf::MessageFilter<sensor_msgs::LaserScan>(*verti_laser_sub_, tf_, odom_frame_id_, 10);
	tf_filter_->registerCallback(boost::bind(&DATMO::scanCallback, this, _1));
	tf_filter_->setTolerance(ros::Duration(0.05));

	private_nh_.param("img_side_length",      img_side_length_,     50.0);
	private_nh_.param("img_resolution",       img_resolution_,     0.2);
	accumulated_image_		= Mat((int)(2*img_side_length_/img_resolution_), (int)(2*img_side_length_/img_resolution_), CV_8UC1);
	LIDAR_pixel_coord_.x 	= (int)(img_side_length_/img_resolution_)-1;
	LIDAR_pixel_coord_.y 	= (int)(img_side_length_/img_resolution_)-1;
}

void DATMO::initialize_roadmap()
{
	map_origin.x = 0.0;
	map_origin.y = 0.0;
	road_map_prior_ = imread( map_image_path_.c_str(), CV_LOAD_IMAGE_GRAYSCALE );

	int dilation_size = 10;
	Mat element = getStructuringElement( MORPH_RECT, Size( 2*dilation_size + 1, 2*dilation_size+1 ), Point( dilation_size, dilation_size ) );
	erode(road_map_prior_, road_map_prior_, element);
}

inline bool DATMO::check_onRoad(geometry_msgs::Point32 &point)
{
	Point2i image_point;
	image_point.x =  (floor((point.x - map_origin.x) / map_resolution_ + 0.5));
	image_point.y =  (floor((point.y - map_origin.y) / map_resolution_ + 0.5));
	if(road_map_prior_.at<uchar>(road_map_prior_.rows-1-image_point.y, image_point.x)> 127) return true;
	else return false;
}

void DATMO::scanCallback (const sensor_msgs::LaserScan::ConstPtr& verti_scan_in)
{
	sensor_msgs::PointCloud verti_cloud;
	try{projector_.transformLaserScanToPointCloud(odom_frame_id_, *verti_scan_in, verti_cloud, tf_);}
	catch (tf::TransformException& e){ROS_DEBUG("Wrong!!!!!!!!!!!!!"); std::cout << e.what();return;}

	cloud_vector_.push_back(verti_cloud);

	geometry_msgs::PoseStamped ident;
	ident.header = verti_scan_in->header;
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
		ROS_WARN("Failed to compute map pose, skipping scan (%s)", e.what());
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

void DATMO::extract_moving_points()
{
	 /*PCL ICP slow and not working fine;
	 /*
	 pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in (new pcl::PointCloud<pcl::PointXYZ>);
	 pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out (new pcl::PointCloud<pcl::PointXYZ>);
	 cloud_in->height   = 1;
	 cloud_in->is_dense = false;
	 cloud_out->height   = 1;
	 cloud_out->is_dense = false;
	 for(size_t i=0; i<interval_; i++)
	 {
		 for(size_t j=0; j<cloud_vector_[i].points.size(); j++)
		 {
			 pcl::PointXYZ point_tmp;
			 geometry_msgs::Point32 spacePt_tmp = cloud_vector_[i].points[j];
			 point_tmp.x = spacePt_tmp.x;
			 point_tmp.y = spacePt_tmp.y;
			 point_tmp.x = 0.0;
			 cloud_in->push_back(point_tmp);
		 }
	 }
	 for(size_t i=interval_; i<2*interval_; i++)
	 {
		 for(size_t j=0; j<cloud_vector_[i].points.size(); j++)
		 {
			 pcl::PointXYZ point_tmp;
			 geometry_msgs::Point32 spacePt_tmp = cloud_vector_[i].points[j];
			 point_tmp.x = spacePt_tmp.x;
			 point_tmp.y = spacePt_tmp.y;
			 point_tmp.x = 0.0;
			 cloud_out->push_back(point_tmp);
		 }
	 }
	 pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
	 icp.setInputCloud(cloud_in);
	 icp.setInputTarget(cloud_out);
	 icp.setMaxCorrespondenceDistance (0.50);
	 // Set the maximum number of iterations (criterion 1)
	 icp.setMaximumIterations (50);
	 pcl::PointCloud<pcl::PointXYZ> Final;
	 icp.align(Final);
	 std::cout << "has converged:" << icp.hasConverged() << " score: " <<
	 icp.getFitnessScore() << std::endl;
	 std::cout << icp.getFinalTransformation() << std::endl;
	*/

	accumulated_image_ = Scalar(0);
	Mat current_acc_image = accumulated_image_.clone();
	Mat image_t = accumulated_image_.clone();
	for(size_t i=0; i<interval_; i++)
	{
		for(size_t j=0; j<cloud_vector_[i].points.size(); j++)
		{
			geometry_msgs::Point32 spacePt_tmp = cloud_vector_[i].points[j];
			//if(!check_onRoad(spacePt_tmp)) continue;
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
			//if(!check_onRoad(spacePt_tmp)) continue;
			Point imgpt_tmp;
			spacePt2ImgP(spacePt_tmp, imgpt_tmp);
			if(imgpt_tmp.y>= 0 && imgpt_tmp.y <accumulated_image_.rows && imgpt_tmp.x>= 0 && imgpt_tmp.x <accumulated_image_.cols)
			{
				if(current_acc_image.at<uchar>(imgpt_tmp.y,imgpt_tmp.x) <255) current_acc_image.at<uchar>(imgpt_tmp.y,imgpt_tmp.x) = current_acc_image.at<uchar>(imgpt_tmp.y,imgpt_tmp.x)+1;
				if(i==2*interval_-1)
				{
					if(image_t.at<uchar>(imgpt_tmp.y,imgpt_tmp.x) <255) image_t.at<uchar>(imgpt_tmp.y,imgpt_tmp.x) = image_t.at<uchar>(imgpt_tmp.y,imgpt_tmp.x)+1;
				}
			}
		}
	}

	/*
	 * try opencv API for scan matching, it helps, but performance not robust;
	 * 1) it turns out not robust, the estimation is not consistent but jumps a lot;
	 * 2) it will try to match moving points when other surrounding features not enough, since no odometry information used;
	 *
	 * the reason: "estimateRigidTransform" is realized using RANSAC with 3 points, but performance is not guaranteed;
	 */
	/*
	ROS_INFO("1");
	Mat affine_tranform = estimateRigidTransform(accumulated_image_, current_acc_image, false);
	warpAffine(accumulated_image_, accumulated_image_, affine_tranform, accumulated_image_.size());
	cout << affine_tranform << endl;
	ROS_INFO("2");
	*/
	//Grid Search for the alignment;
	//It turns out to be too slow;
	/*
	Mat affine_tranform;
	ROS_INFO("GridSearch");
	GridSearch4BestAlignment(accumulated_image_, current_acc_image, affine_tranform);
	ROS_INFO("GS finished");
	warpAffine(accumulated_image_, accumulated_image_, affine_tranform, accumulated_image_.size());
	*/


	threshold( image_t, image_t, 0, 255, 0 );
	imshow("image_t", image_t);
	waitKey(1);

	int dilation_size = 1;
	Mat element = getStructuringElement( MORPH_RECT, Size( 2*dilation_size + 1, 2*dilation_size+1 ), Point( dilation_size, dilation_size ) );

	threshold(current_acc_image, current_acc_image, 0, 255, 0 );
	threshold(accumulated_image_, accumulated_image_, 0, 255, 0);
	Mat accumulate_image_dilated, current_acc_image_dilated;
	dilate(accumulated_image_, accumulate_image_dilated, element);
	dilate(current_acc_image, current_acc_image_dilated, element);

	Mat acc_dilated_inverted_tmp, current_accDilated_inverted_tmp;
	Mat new_appear, old_disappear, consistent_existing;
	threshold(accumulate_image_dilated, acc_dilated_inverted_tmp, 0, 255, 1);
	threshold(current_acc_image_dilated, current_accDilated_inverted_tmp, 0, 255, 1);

	bitwise_and(current_acc_image, acc_dilated_inverted_tmp, new_appear);
	bitwise_and(accumulate_image_dilated, current_accDilated_inverted_tmp, old_disappear);
	bitwise_and(accumulate_image_dilated, current_acc_image_dilated, consistent_existing);

	Mat three_chanels[3]={new_appear, consistent_existing, old_disappear};
	Mat merged_visualization;
	merge(three_chanels, 3, merged_visualization);
	imshow("accumulated_image", accumulated_image_);
	imshow("current_acc_image", current_acc_image);
	imshow("visualize-all", merged_visualization);
	waitKey(1);

}

void DATMO::GridSearch4BestAlignment(Mat& origninal_image, Mat& target_image, Mat& affine_transform)
{
	double angle_resolution = 0.5*M_PI;
	double angle_min		= -3.0*M_PI;
	double angle_max		= 3.0*M_PI;

	double x_resolution = 1.0;
	double x_min		= -1.0;
	double x_max		= 1.0;

	double y_resolution = 1.0;
	double y_min		= -1.0;
	double y_max		= 1.0;

	Scalar image_difference;
	image_difference.val[0]= 10000000.0;
	Mat best_affine_matrix;


	for(double angle_tmp = angle_min; angle_tmp <= angle_max; angle_tmp+=angle_resolution)
		for(double x_tmp = x_min; x_tmp <= x_max; x_tmp+=x_resolution)
			for(double y_tmp = y_min; y_tmp <= y_max; y_tmp+=y_resolution)
			{
				Mat affine_matrix_tmp(2,3,CV_64FC1);
				affine_matrix_tmp.at<double>(0,0) = cos(angle_tmp);
				affine_matrix_tmp.at<double>(0,1) = sin(angle_tmp);
				affine_matrix_tmp.at<double>(0,2) = x_tmp;
				affine_matrix_tmp.at<double>(1,0) = -sin(angle_tmp);
				affine_matrix_tmp.at<double>(1,1) = cos(angle_tmp);
				affine_matrix_tmp.at<double>(1,2) = y_tmp;

				Mat affined_target_tmp;
				warpAffine(origninal_image, affined_target_tmp, affine_matrix_tmp, origninal_image.size());
				absdiff(target_image, affined_target_tmp, affined_target_tmp);
				Scalar image_difference_tmp = sum(affined_target_tmp);
				if(image_difference_tmp.val[0]<image_difference.val[0])
				{
					image_difference.val[0] = image_difference_tmp.val[0];
					best_affine_matrix = affine_matrix_tmp;
				}
			}
	affine_transform = best_affine_matrix;
}

void DATMO::process_moving_image(Mat &img_moving, Mat &img_t)
{
	Mat moving_tmp = img_moving.clone();
	vector<vector<Point> > contours;
	vector<Vec4i> hierarchy;
	findContours( moving_tmp, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE, Point(0,0) );

	vector<Moments> mu(contours.size() );
	for( size_t i = 0; i < contours.size(); i++ ) {mu[i] = moments( contours[i], false );}
	vector<double> area( contours.size() );
	for( size_t i = 0; i < contours.size(); i++ ) {area[i] = contourArea(contours[i]);}
	vector<double> length( contours.size() );
	for( size_t i = 0; i < contours.size(); i++ ) {length[i] = arcLength(contours[i], true);}

	Mat moving_tmp2 = img_moving.clone();
	int dilation_size = 2;
	Mat element = getStructuringElement( MORPH_RECT, Size( 2*dilation_size + 1, 2*dilation_size+1 ), Point( dilation_size, dilation_size ) );
	morphologyEx(moving_tmp2, moving_tmp2, CV_MOP_OPEN, element);
	imshow("erode", moving_tmp2);
	waitKey(1);
}



void DATMO::spacePt2ImgP(geometry_msgs::Point32 & spacePt, Point & imgPt)
{
	imgPt.x = int((spacePt.x - laser_pose_current_.pose.position.x)/img_resolution_) + LIDAR_pixel_coord_.x;
	imgPt.y =  accumulated_image_.rows -( int((spacePt.y - laser_pose_current_.pose.position.y)/img_resolution_) + LIDAR_pixel_coord_.y);
}
void DATMO::ImgPt2spacePt(Point & imgPt, geometry_msgs::Point32 & spacePt)
{
	spacePt.x = (imgPt.x-LIDAR_pixel_coord_.x)*img_resolution_+laser_pose_current_.pose.position.x;
	spacePt.y = (accumulated_image_.rows - imgPt.y - LIDAR_pixel_coord_.y)*img_resolution_+laser_pose_current_.pose.position.y;
}


void DATMO::DP_segment (sensor_msgs::PointCloud scan_pointcloud, geometry_msgs::PoseStamped laser_origin)
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
	if(raw_cloud.size()<5) return;
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

//http://alienryderflex.com/polygon/
inline bool DATMO::pointInPolygon(geometry_msgs::Point32 p, vector<geometry_msgs::Point32> poly)
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


int main(int argc, char** argv)
{
	ros::init(argc, argv, "DATMO_node");
	DATMO DATMO_node;
	ros::spin();
	return (0);
}
