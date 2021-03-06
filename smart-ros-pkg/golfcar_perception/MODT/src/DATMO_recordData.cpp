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
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/PoseArray.h>

#include <iostream>
#include <opencv/cv.h>
#include <opencv/highgui.h>

#include <boost/thread.hpp>
#include <boost/shared_ptr.hpp>

#include <vector>
#include <cmath>

#include "DATMO_datatypes.h"

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
	std::string laser_frame_id_;
	std::string base_frame_id_;
	std::string odom_frame_id_;
	std::string map_frame_id_;

	tf::TransformListener tf_;
	message_filters::Subscriber<sensor_msgs::LaserScan>     *laser_sub_;
	laser_geometry::LaserProjection                         projector_;

	void scanCallback (const sensor_msgs::LaserScan::ConstPtr& verti_scan_in);
	void scan_approxy(const sensor_msgs::PointCloud& cloud_in, geometry_msgs::PoseStamped& laser_pose);

	bool pointInPolygon(geometry_msgs::Point32 p, vector<geometry_msgs::Point32> poly);
	void process_accumulated_points();
	void extract_moving_objects(Mat& accT, Mat& accTminusOne, Mat& new_appear, Mat& old_disappear, Mat& current_image);


	tf::MessageFilter<sensor_msgs::LaserScan>				*tf_filter_;
	size_t 													interval_;
	vector<sensor_msgs::PointCloud>                        	cloud_vector_;
	vector<sensor_msgs::LaserScan>                        	scan_vector_;
	sensor_msgs::PointCloud 								combined_pointcloud_;
	vector<geometry_msgs::PoseStamped>						laser_pose_vector_;

	geometry_msgs::PoseStamped								laser_pose_current_;
	ros::Publisher                              			vehicle_array_pub_;
	ros::Publisher                              			collected_cloud_pub_, accumulated_cloud_pub_;
	//vehicle local image;
	float													img_side_length_, img_resolution_;
	Mat														local_mask_;
	Point													LIDAR_pixel_coord_;
	void 													initialize_local_image();
	void spacePt2ImgP(geometry_msgs::Point32 & spacePt, Point2f & imgPt);
	void ImgPt2spacePt(Point2f & imgPt, geometry_msgs::Point32 & spacePt);
	bool LocalPixelValid(Point2f & imgPt);

	//roadmap global image;
	bool													use_prior_map_;
	Mat														road_map_prior_;
	std::string												map_image_path_;
	geometry_msgs::Point32									map_origin;
	double													map_resolution_;
	void													initialize_roadmap();
	bool 													check_onRoad(geometry_msgs::Point32 &point);

	ros::Publisher current_polygon_pub_;

	long int scan_serial_;
	void save_training_data(std::vector<DATMO_TrainingScan>& training_data_vector,std::vector<vector<Point> > & contour_candidate_vector);
};

DATMO::DATMO()
: private_nh_("~")
{
	private_nh_.param("laser_frame_id",     laser_frame_id_,    std::string("front_bottom_lidar"));
	private_nh_.param("base_frame_id",      base_frame_id_,     std::string("base_link"));
	private_nh_.param("odom_frame_id",      odom_frame_id_,     std::string("odom"));
	private_nh_.param("map_frame_id",       map_frame_id_,      std::string("map"));

	private_nh_.param("use_prior_map",       use_prior_map_,    false);
	private_nh_.param("map_resolution",     map_resolution_,    0.05);
	private_nh_.param("map_image_path",     map_image_path_,    std::string("./launch/road_map.jpg"));
	initialize_roadmap();

	int inverval_tmp;
	private_nh_.param("interval",		    inverval_tmp,       	10);
	interval_ = (size_t) inverval_tmp;

	vehicle_array_pub_		=   nh_.advertise<geometry_msgs::PoseArray>("vehicle_array", 2);
	collected_cloud_pub_		=   nh_.advertise<sensor_msgs::PointCloud>("collected_cloud", 2);
	accumulated_cloud_pub_		=   nh_.advertise<sensor_msgs::PointCloud>("accumulated_cloud", 2);

	laser_sub_ = new message_filters::Subscriber<sensor_msgs::LaserScan> (nh_, "/front_bottom_scan", 100);
	tf_filter_ = new tf::MessageFilter<sensor_msgs::LaserScan>(*laser_sub_, tf_, odom_frame_id_, 100);
	tf_filter_->registerCallback(boost::bind(&DATMO::scanCallback, this, _1));
	tf_filter_->setTolerance(ros::Duration(0.5));

	current_polygon_pub_ = nh_.advertise<geometry_msgs::PolygonStamped>("/approxy_polygon", 10);

	double img_side_length_tmp, img_resolution_tmp;
	private_nh_.param("img_side_length",      img_side_length_tmp,     50.0);
	private_nh_.param("img_resolution",       img_resolution_tmp,      0.2);
	img_side_length_ = (float)img_side_length_tmp;
	img_resolution_ = (float)img_resolution_tmp;

	initialize_local_image();
	scan_serial_ = 0;
}

void DATMO::initialize_roadmap()
{
	map_origin.x = 0.0;
	map_origin.y = 0.0;
	road_map_prior_ = imread( map_image_path_.c_str(), CV_LOAD_IMAGE_GRAYSCALE );
	int dilation_size = 1;

	//vehicle should have some distance to the road boundary;
	Mat element = getStructuringElement( MORPH_RECT, Size( 2*dilation_size + 1, 2*dilation_size+1 ), Point( dilation_size, dilation_size ) );
	erode(road_map_prior_, road_map_prior_, element);
}
inline bool DATMO::check_onRoad(geometry_msgs::Point32 &point)
{
	Point2i image_point;
	image_point.x =  (floor((point.x - map_origin.x) / map_resolution_ + 0.5));
	image_point.y =  (floor((point.y - map_origin.y) / map_resolution_ + 0.5));

	if(image_point.x < road_map_prior_.cols && image_point.x >=0 && image_point.y < road_map_prior_.rows && image_point.y >=0)
	{
		if(road_map_prior_.at<uchar>(road_map_prior_.rows-1-image_point.y, image_point.x)> 127) return true;
		else return false;
	}
	else return false;
}

void DATMO::initialize_local_image()
{
	local_mask_				= Mat((int)(img_side_length_/img_resolution_*2), (int)(img_side_length_/img_resolution_*2), CV_8UC1);
	local_mask_				= Scalar(0);
	LIDAR_pixel_coord_.x 	= (int)(img_side_length_/img_resolution_)-1;
	LIDAR_pixel_coord_.y 	= (int)(img_side_length_/img_resolution_)-1;

	vector<geometry_msgs::Point32> img_roi;
	geometry_msgs::Point32 p0, p1, p2, p3, p4;
	p0.x = 0.0;
	p0.y = 0.0;
	p1.x = 0.0;
	p1.y = float(local_mask_.rows-1);
	p2.x = float(LIDAR_pixel_coord_.x);
	p2.y = float(LIDAR_pixel_coord_.y);
	p3.x = float(local_mask_.cols-1);
	p3.y = float(local_mask_.rows-1);
	p4.x = float(local_mask_.cols-1);
	p4.y = 0.0;

	img_roi.push_back(p0);
	img_roi.push_back(p1);
	img_roi.push_back(p2);
	img_roi.push_back(p3);
	img_roi.push_back(p4);

	for(int i=0; i<(int)local_mask_.cols; i++)
		for(int j=0; j<(int)local_mask_.rows; j++)
		{
			geometry_msgs::Point32 point_tmp;
			point_tmp.x = (float)i;
			point_tmp.y = (float)j;
			if(pointInPolygon(point_tmp, img_roi))local_mask_.at<uchar>(j,i)=255;
		}
}

void DATMO::spacePt2ImgP(geometry_msgs::Point32 & spacePt, Point2f & imgPt)
{
	imgPt.x = LIDAR_pixel_coord_.x - (spacePt.y/img_resolution_);
	imgPt.y = LIDAR_pixel_coord_.y - (spacePt.x/img_resolution_);
}
void DATMO::ImgPt2spacePt(Point2f & imgPt, geometry_msgs::Point32 & spacePt)
{
	spacePt.x = (LIDAR_pixel_coord_.y-imgPt.y)*img_resolution_;
	spacePt.y = (LIDAR_pixel_coord_.x-imgPt.x)*img_resolution_;
}

inline bool DATMO::LocalPixelValid(Point2f & imgPt)
{
	if((int)imgPt.x < local_mask_.cols && (int)imgPt.x >=0 && (int)imgPt.y < local_mask_.rows && (int)imgPt.y >=0) return true;
	else return false;
}

void DATMO::scanCallback (const sensor_msgs::LaserScan::ConstPtr& verti_scan_in)
{
	ROS_INFO("scan callback %u ", verti_scan_in->header.seq);
	sensor_msgs::PointCloud verti_cloud;
	try{projector_.transformLaserScanToPointCloud(odom_frame_id_, *verti_scan_in, verti_cloud, tf_);}
	catch (tf::TransformException& e){ROS_DEBUG("Wrong!!!!!!!!!!!!!"); std::cout << e.what();return;}

	//pay attention to use the intensity value;
	scan_vector_.push_back(*verti_scan_in);
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
	laser_pose_vector_.push_back(laser_pose_current_);

	scan_approxy(verti_cloud, laser_pose_current_);

	assert(cloud_vector_.size() == laser_pose_vector_.size());

	scan_serial_ = scan_vector_.back().header.seq;
	if(cloud_vector_.size()==interval_*2)
	{
		process_accumulated_points();
		cloud_vector_.erase(cloud_vector_.begin(), cloud_vector_.begin()+ 1);
		laser_pose_vector_.erase(laser_pose_vector_.begin(), laser_pose_vector_.begin()+1);
		scan_vector_.erase(scan_vector_.begin(), scan_vector_.begin()+ 1);
	}
	//scan_serial_++;
	ROS_INFO("scan callback finished");
}

void DATMO::scan_approxy(const sensor_msgs::PointCloud& cloud_in, geometry_msgs::PoseStamped& laser_pose)
{
	std::vector<Point2f> raw_input, approxy_output;
	for(size_t i=0; i<cloud_in.points.size(); i++)
	{
		raw_input.push_back(Point2f(cloud_in.points[i].x, cloud_in.points[i].y));
	}
	approxPolyDP(raw_input, approxy_output, 1.0, false);

	geometry_msgs::PolygonStamped approxy_polygon;
	approxy_polygon.header = cloud_in.header;
	geometry_msgs::Point32 origin_tmp;
	origin_tmp.x =(float)laser_pose.pose.position.x;
	origin_tmp.y =(float)laser_pose.pose.position.y;
	origin_tmp.z =0.0;
	approxy_polygon.polygon.points.push_back(origin_tmp);

	for(size_t i=0; i<approxy_output.size(); i++)
	{
		origin_tmp.x =approxy_output[i].x;
		origin_tmp.y =approxy_output[i].y;
		approxy_polygon.polygon.points.push_back(origin_tmp);
	}
	current_polygon_pub_.publish(approxy_polygon);
}

void DATMO::process_accumulated_points()
{
	sensor_msgs::PointCloud accumulated_TminusOne_pcl;
	sensor_msgs::PointCloud accumulated_T_pcl;
	sensor_msgs::PointCloud current_T_pcl;

	accumulated_TminusOne_pcl.header = cloud_vector_.back().header;
	accumulated_T_pcl.header = cloud_vector_.back().header;
	current_T_pcl.header = cloud_vector_.back().header;


	combined_pointcloud_.points.clear();
	combined_pointcloud_.header = cloud_vector_.back().header;

	for(size_t i=0; i<interval_; i++)
	{
		for(size_t j=0; j<cloud_vector_[i].points.size(); j++)
		{
			geometry_msgs::Point32 spacePt_tmp = cloud_vector_[i].points[j];
			accumulated_TminusOne_pcl.points.push_back(spacePt_tmp);
			combined_pointcloud_.points.push_back(spacePt_tmp);
		}
	}
	for(size_t i=interval_; i<2*interval_; i++)
	{
		for(size_t j=0; j<cloud_vector_[i].points.size(); j++)
		{
			geometry_msgs::Point32 spacePt_tmp = cloud_vector_[i].points[j];
			accumulated_T_pcl.points.push_back(spacePt_tmp);
			combined_pointcloud_.points.push_back(spacePt_tmp);
		}
	}

	for(size_t j=0; j<cloud_vector_.back().points.size(); j++)
	{
		geometry_msgs::Point32 spacePt_tmp = cloud_vector_.back().points[j];
		current_T_pcl.points.push_back(spacePt_tmp);
	}

	try
	{
		tf_.transformPointCloud(laser_frame_id_, accumulated_TminusOne_pcl, accumulated_TminusOne_pcl);
		tf_.transformPointCloud(laser_frame_id_, accumulated_T_pcl, accumulated_T_pcl);
		tf_.transformPointCloud(laser_frame_id_, current_T_pcl, current_T_pcl);

		tf_.transformPointCloud(laser_frame_id_, combined_pointcloud_, combined_pointcloud_);
	}
	catch(tf::TransformException e)
	{
		ROS_WARN("Failed to transform accumulated point to laser_frame_id_ (%s)", e.what());
		return;
	}

	Mat accumulated_TminusOne_img = local_mask_.clone();
	accumulated_TminusOne_img = Scalar(0);
	Mat accumulated_T_img = accumulated_TminusOne_img.clone();
	Mat current_T_image = accumulated_TminusOne_img.clone();

	for(size_t i=0; i<accumulated_TminusOne_pcl.points.size(); i++)
	{
		geometry_msgs::Point32 spacePt_tmp = accumulated_TminusOne_pcl.points[i];
		Point2f imgpt_tmp;
		spacePt2ImgP(spacePt_tmp, imgpt_tmp);
		if(LocalPixelValid(imgpt_tmp))
		{
			if(accumulated_TminusOne_img.at<uchar>((int)imgpt_tmp.y,(int)imgpt_tmp.x) <255) accumulated_TminusOne_img.at<uchar>((int)imgpt_tmp.y,(int)imgpt_tmp.x) = accumulated_TminusOne_img.at<uchar>((int)imgpt_tmp.y,(int)imgpt_tmp.x)+1;
		}
	}

	for(size_t i=0; i<accumulated_T_pcl.points.size(); i++)
	{
		geometry_msgs::Point32 spacePt_tmp = accumulated_T_pcl.points[i];
		Point2f imgpt_tmp;
		spacePt2ImgP(spacePt_tmp, imgpt_tmp);
		if(LocalPixelValid(imgpt_tmp))
		{
			if(accumulated_T_img.at<uchar>((int)imgpt_tmp.y,(int)imgpt_tmp.x) <255) accumulated_T_img.at<uchar>((int)imgpt_tmp.y,(int)imgpt_tmp.x) = accumulated_T_img.at<uchar>((int)imgpt_tmp.y,(int)imgpt_tmp.x)+1;
		}
	}

	for(size_t i=0; i<current_T_pcl.points.size(); i++)
	{
		geometry_msgs::Point32 spacePt_tmp = current_T_pcl.points[i];
		Point2f imgpt_tmp;
		spacePt2ImgP(spacePt_tmp, imgpt_tmp);
		if(LocalPixelValid(imgpt_tmp))
		{
			if(current_T_image.at<uchar>((int)imgpt_tmp.y,(int)imgpt_tmp.x) <255) current_T_image.at<uchar>((int)imgpt_tmp.y,(int)imgpt_tmp.x) = current_T_image.at<uchar>((int)imgpt_tmp.y,(int)imgpt_tmp.x)+1;
		}
	}

	int dilation_size = 1;
	Mat element = getStructuringElement( MORPH_RECT, Size( 2*dilation_size + 1, 2*dilation_size+1 ), Point( dilation_size, dilation_size ) );
	threshold(current_T_image, current_T_image, 0, 255, 0 );

	threshold(accumulated_T_img, accumulated_T_img, 0, 255, 0 );
	threshold(accumulated_TminusOne_img, accumulated_TminusOne_img, 0, 255, 0);
	Mat accumulated_TminusOne_img_dilated, accumulated_T_img_dilated;
	dilate(accumulated_TminusOne_img, accumulated_TminusOne_img_dilated, element);
	dilate(accumulated_T_img, accumulated_T_img_dilated, element);

	Mat acc_dilated_inverted_tmp, current_accDilated_inverted_tmp;
	Mat new_appear, old_disappear, consistent_existing;

	threshold(accumulated_TminusOne_img_dilated, acc_dilated_inverted_tmp, 0, 255, 1);
	threshold(accumulated_T_img_dilated, current_accDilated_inverted_tmp, 0, 255, 1);

	bitwise_and(accumulated_T_img, acc_dilated_inverted_tmp, new_appear, local_mask_);
	bitwise_and(accumulated_TminusOne_img, current_accDilated_inverted_tmp, old_disappear, local_mask_);

	bitwise_and(accumulated_TminusOne_img_dilated, accumulated_T_img_dilated, consistent_existing, local_mask_);

	Mat three_chanels[3]={new_appear, consistent_existing, old_disappear};
	Mat merged_visualization;
	merge(three_chanels, 3, merged_visualization);

	imshow("new_appear", new_appear);
	imshow("old_disappear", old_disappear);
	imshow("accumulated_TminusOne_img", accumulated_TminusOne_img);
	imshow("accumulated_T_img", accumulated_T_img);
	imshow("visualize_all", merged_visualization);
	waitKey(1);

	stringstream  visualization_string;
	visualization_string<<"/home/baoxing/data/raw_data/image_"<< scan_serial_<<".jpg";
	const string image_name = visualization_string.str();
	imwrite(image_name, merged_visualization);

	extract_moving_objects(accumulated_T_img, accumulated_TminusOne_img, new_appear, old_disappear, current_T_image);
}

//here just for vehicle; some process need to be further modified for pedestrians;
void DATMO::extract_moving_objects(Mat& accT, Mat& accTminusOne, Mat& new_appear, Mat& old_disappear, Mat& current_image)
{

	//-----------------1st step: pair-up disappear and appear patches, as moving object candidates;-----------------

	int open_size = 3; int close_size = 1; int close_size2 = 1;
	Mat open_element = getStructuringElement( MORPH_RECT, Size( 2*open_size + 1, 2*open_size+1 ), Point( open_size, open_size ) );
	Mat close_element = getStructuringElement( MORPH_RECT, Size( 2*close_size + 1, 2*close_size+1 ), Point( close_size, close_size ) );
	Mat close_element2 = getStructuringElement( MORPH_RECT, Size( 2*close_size2 + 1, 2*close_size2+1 ), Point( close_size2, close_size2 ) );

	Mat combined_img;
	bitwise_or(accT, accTminusOne, combined_img, local_mask_);
	Mat new_appear_tmp, old_disappear_tmp;

	morphologyEx(combined_img, combined_img, CV_MOP_CLOSE, close_element2);
	//morphologyEx(combined_img, combined_img, CV_MOP_DILATE, open_element);
	morphologyEx(new_appear, new_appear_tmp,CV_MOP_CLOSE, close_element);
	//morphologyEx(new_appear_tmp, new_appear_tmp,CV_MOP_OPEN, open_element);
	morphologyEx(old_disappear, old_disappear_tmp, CV_MOP_CLOSE, close_element);
	//morphologyEx(old_disappear_tmp, old_disappear_tmp, CV_MOP_OPEN, open_element);

	vector<vector<Point> > contours_combined, contours_appear, contours_disappear;
	vector<Vec4i> hierarchy_combined, hierarchy_appear, hierarchy_disappear;

	findContours( combined_img, contours_combined, hierarchy_combined, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE, Point(0,0) );
	findContours( new_appear_tmp, contours_appear, hierarchy_appear, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE, Point(0,0) );
	findContours( old_disappear_tmp, contours_disappear, hierarchy_disappear, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE, Point(0,0) );

	vector<RotatedRect> minAreaRect_combined(contours_combined.size()); for( size_t i = 0; i < contours_combined.size(); i++ ) { minAreaRect_combined[i] = minAreaRect( Mat(contours_combined[i]) );}
	vector<RotatedRect> minAreaRect_appear(contours_appear.size()); for( size_t i = 0; i < contours_appear.size(); i++ ) { minAreaRect_appear[i] = minAreaRect( Mat(contours_appear[i]) );}
	vector<RotatedRect> minAreaRect_disappear(contours_disappear.size());  for( size_t  i = 0; i < contours_disappear.size(); i++ ) { minAreaRect_disappear[i] = minAreaRect( Mat(contours_disappear[i]) );}
	vector<double> area_combined( contours_combined.size() ); for( size_t i = 0; i < contours_combined.size(); i++ ) {area_combined[i] = contourArea(contours_combined[i]);}
	vector<double> area_appear( contours_appear.size() ); for( size_t i = 0; i < contours_appear.size(); i++ ) {area_appear[i] = contourArea(contours_appear[i]);}
	vector<double> area_disappear( contours_disappear.size() ); for( size_t i = 0; i < contours_disappear.size(); i++ ) {area_disappear[i] = contourArea(contours_disappear[i]);}

	for( int i = 0; i< (int)contours_combined.size(); i++ ) drawContours( combined_img, contours_combined, i, Scalar(255), 1, 8, vector<Vec4i>(), 0, Point() );
	for( int i = 0; i< (int)contours_appear.size(); i++ ) drawContours( new_appear_tmp, contours_appear, i, Scalar(255), -1, 8, vector<Vec4i>(), 0, Point() );
	for( int i = 0; i< (int)contours_disappear.size(); i++ ) drawContours( old_disappear_tmp, contours_disappear, i, Scalar(255), -1, 8, vector<Vec4i>(), 0, Point() );

	Mat three_chanels[3]={new_appear_tmp, combined_img, old_disappear_tmp};
	Mat merged_visualization;
	merge(three_chanels, 3, merged_visualization);
	imshow("colorful_plots", merged_visualization);
	waitKey(1);

	vector<pair<int, int> > appear_disappear_pairs;
	for(size_t i = 0; i<minAreaRect_combined.size(); i++)
	{
		int appear_serial = -1;
		int disappear_serial = -1;

		vector<size_t> combined_appear_tmp, combined_disappear_tmp;
		for(size_t j = 0; j<minAreaRect_appear.size(); j++)
		{
			double check = pointPolygonTest(contours_combined[i], minAreaRect_appear[j].center, false);
			if(check>=0)combined_appear_tmp.push_back(j);
		}
		for(size_t j = 0; j<minAreaRect_disappear.size(); j++)
		{
			double check = pointPolygonTest(contours_combined[i], minAreaRect_disappear[j].center, false);
			if(check>=0)combined_disappear_tmp.push_back(j);
		}

		double biggest_area_appear = 0.0;
		double biggest_area_disappear = 0.0;
		for(size_t j = 0; j<combined_appear_tmp.size(); j++)
		{
			size_t contour_serial = combined_appear_tmp[j];
			if(area_appear[contour_serial]>biggest_area_appear)
			{
				appear_serial = (int)contour_serial;
				biggest_area_appear = area_appear[contour_serial];
			}
		}
		for(size_t j = 0; j<combined_disappear_tmp.size(); j++)
		{
			size_t contour_serial = combined_disappear_tmp[j];
			if(area_disappear[contour_serial]>biggest_area_disappear)
			{
				disappear_serial = (int)contour_serial;
				biggest_area_disappear = area_disappear[contour_serial];
			}
		}
		pair <int,int> appear_disappear_tmp = make_pair(appear_serial, disappear_serial);
		appear_disappear_pairs.push_back(appear_disappear_tmp);

	}

	//------------------2nd: collect training data-------------------------
	//construct the training data;
	if(scan_serial_>0 && scan_serial_ % interval_==0)
	{
		//here we only record those possible contours consisting of appear-disappear pairs;
		std::vector<vector<Point> > contour_candidate_vector;
		size_t candidate_contour_serial=0;

		//each scan will has its own training_data from this vector;
		DATMO_TrainingScan init_training_data;
		init_training_data.movingObjectClusters.clear();
		std::vector<DATMO_TrainingScan> training_data_vector(scan_vector_.size(), init_training_data);

		//for(size_t i=0; i<training_data_vector.size(); i++)
		//{
			//DATMO_TrainingScan & trainingscan_tmp = training_data_vector[i];
			//trainingscan_tmp.laser_scan = scan_vector_[i];
			//trainingscan_tmp.poseInOdom = laser_pose_vector_[i];
		//}

		sensor_msgs::PointCloud collected_visualize_pcl;
		collected_visualize_pcl.header = combined_pointcloud_.header;
		for(size_t i=0;  i<appear_disappear_pairs.size(); i++)
		{
			int appear_serial = appear_disappear_pairs[i].first;
			int disappear_serial = appear_disappear_pairs[i].second;
			if(appear_serial>=0 && disappear_serial>=0)
			{
				//ROS_INFO("appear_serial, disappear: %d, %d", appear_serial, disappear_serial);
				//record inside LIDAR points;
				size_t pointSerialInCloud = 0;
				for(size_t a=0; a<scan_vector_.size();a++)
				{
					std::vector<int> serial_in_cluster;
					//cout<<"scan:"<<scan_vector_[a].header.seq<<"\t";
					for(size_t b=0; b<scan_vector_[a].ranges.size();b++)
					{
						//once there was a stupid bug here, when using "if(scan_vector_[a].intensities[b]>0)";
						if(scan_vector_[a].ranges[b]>=scan_vector_[a].range_min && scan_vector_[a].ranges[b]<=scan_vector_[a].range_max)
						{
							geometry_msgs::Point32 spacePt_tmp = combined_pointcloud_.points[pointSerialInCloud];
							Point2f imgpt_tmp;
							spacePt2ImgP(spacePt_tmp, imgpt_tmp);
							double check = pointPolygonTest(contours_combined[i], imgpt_tmp, true);
							if(check >= -1.0)
							{
								serial_in_cluster.push_back((int)b);
								collected_visualize_pcl.points.push_back(spacePt_tmp);
								//cout<<"point"<<b<<":("<<spacePt_tmp.x<<","<<spacePt_tmp.y<<")\t";
							}
							pointSerialInCloud++;
						}
					}
					//cout<<endl;
					std::pair<std::vector<int>, int> movingObjectCluster = make_pair(serial_in_cluster, candidate_contour_serial);
					training_data_vector[a].movingObjectClusters.push_back(movingObjectCluster);
				}
				contour_candidate_vector.push_back(contours_combined[i]);
				candidate_contour_serial++;
			}
		}
		save_training_data(training_data_vector, contour_candidate_vector);
		accumulated_cloud_pub_.publish(combined_pointcloud_);
		collected_cloud_pub_.publish(collected_visualize_pcl);
	}
}

//save extracted data into files;
void DATMO::save_training_data(std::vector<DATMO_TrainingScan>& training_data_vector,std::vector<vector<Point> > & contour_candidate_vector)
{
	stringstream  record_string;
	record_string<<"/home/baoxing/data/raw_data/training_data"<< scan_serial_<<".yml";
	const string data_name = record_string.str();
	FileStorage fs(data_name.c_str(), FileStorage::WRITE);

	//1st: record raw information: scan+odom;
	fs << "scan" << "[";
	for(size_t i=0; i<scan_vector_.size(); i++)
	{
		fs<<"{:"<<"serial"<<(int)scan_vector_[i].header.seq
			   <<"time"<<scan_vector_[i].header.stamp.toSec()
			   <<"angle_min"<<scan_vector_[i].angle_min
			   <<"angle_max"<<scan_vector_[i].angle_increment
			   <<"time_increment"<<scan_vector_[i].time_increment
			   <<"scan_time"<<scan_vector_[i].scan_time
			   <<"range_min"<<scan_vector_[i].range_min
			   <<"range_max"<<scan_vector_[i].range_max
			   <<"ranges"<<"[:";
		for(size_t j=0; j<scan_vector_[i].ranges.size(); j++)
		{
			fs << scan_vector_[i].ranges[j];
		}
		fs << "]" << "intensities"<<"[:";

		for(size_t j=0; j<scan_vector_[i].intensities.size(); j++)
		{
			fs << scan_vector_[i].intensities[j];
		}
		fs << "]" << "}";
	}
	fs<<"]";

	fs << "odom" << "[";
	for (size_t i=0; i<laser_pose_vector_.size(); i++)
	{
		tf::Quaternion q(laser_pose_vector_[i].pose.orientation.x, laser_pose_vector_[i].pose.orientation.y, laser_pose_vector_[i].pose.orientation.z, laser_pose_vector_[i].pose.orientation.w);
		tf::Matrix3x3 m(q);
		double roll, pitch, yaw;
		m.getRPY(roll, pitch, yaw);
		fs<<"{:"<<"x"<<laser_pose_vector_[i].pose.position.x
			    <<"y"<<laser_pose_vector_[i].pose.position.y
			    <<"z"<<laser_pose_vector_[i].pose.position.z
			    <<"roll"<<roll
			    <<"pitch"<<pitch
			    <<"yaw"<<yaw<<"}";
	}
	fs <<"]";

	//2nd: record processed clustering information: contours + movingObjectClusters;
	fs << "contours" << "[";
	for(size_t i=0; i<contour_candidate_vector.size(); i++)
	{
		fs<<"[:";
		for(size_t j=0; j<contour_candidate_vector[i].size(); j++)
		{
			fs<<"{:"<<"x"<< contour_candidate_vector[i][j].x
					<<"y"<< contour_candidate_vector[i][j].y
			  <<"}";
		}
		fs << "]" ;
	}
	fs <<"]";

	//debug trick;
	/*
	for(size_t i=0; i<training_data_vector.size(); i++)
	{
		if(training_data_vector[i].movingObjectClusters.size()>0)
		{
			for(size_t j=0; j<training_data_vector[i].movingObjectClusters.size(); j++)
			{
				cout<<"size"<<training_data_vector[i].movingObjectClusters[j].first.size()<<"\n";
				for(size_t k=0; k<training_data_vector[i].movingObjectClusters[j].first.size();k++)
				{
					cout <<"point"<<(int)training_data_vector[i].movingObjectClusters[j].first[k]<<"\t";
				}
				cout<<"\n corresponding_contour_serial"<<(int)training_data_vector[i].movingObjectClusters[j].second<<endl;
			}
		}
	}
	*/

	fs<<"training_data_vector"<<"[";
	for(size_t i=0; i<training_data_vector.size(); i++)
	{
		//if(training_data_vector[i].movingObjectClusters.size()>0)
		{
			fs<<"{:";
				fs<<"movingObjectClusters"<<"[:";
					for(size_t j=0; j<training_data_vector[i].movingObjectClusters.size(); j++)
					{
						fs<<"{:";
							fs<<"serial_in_cluster";
								fs<<"[:";
								for(size_t k=0; k<training_data_vector[i].movingObjectClusters[j].first.size();k++)
								{
									fs<<(int)training_data_vector[i].movingObjectClusters[j].first[k];
								}
								fs<<"]";

							fs<<"corresponding_contour_serial"<<(int)training_data_vector[i].movingObjectClusters[j].second;
						fs<<"}";
					}
				fs<<"]";
			fs<<"}";
		}
	}
	fs<<"]";

	fs.release();
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
