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
#include "svm_classifier.h"

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
	void supervised_segmentation(const sensor_msgs::LaserScan::ConstPtr& scan_in, const sensor_msgs::PointCloud& cloud_in, geometry_msgs::PoseStamped& laser_pose);
	void segmentation_visulization();

	bool pointInPolygon(geometry_msgs::Point32 p, vector<geometry_msgs::Point32> poly);
	void process_accumulated_points();
	void extract_moving_objects(Mat& accT, Mat& accTminusOne, Mat& new_appear, Mat& old_disappear, Mat& current_image);


	tf::MessageFilter<sensor_msgs::LaserScan>				*tf_filter_;
	size_t 													interval_;
	vector<sensor_msgs::PointCloud>                        	cloud_vector_;
	vector<sensor_msgs::PointCloud>                        	baselink_cloud_vector_;
	vector<sensor_msgs::PointCloud>                        	lastest_baselink_cloud_vector_;

	vector<sensor_msgs::LaserScan>                        	scan_vector_;
	sensor_msgs::PointCloud 								combined_pointcloud_;
	vector<geometry_msgs::PoseStamped>						laser_pose_vector_;

	geometry_msgs::PoseStamped								laser_pose_current_;
	ros::Publisher                              			vehicle_array_pub_;
	ros::Publisher                              			collected_cloud_pub_;
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

	std::string												abstract_summary_path_;
	ros::Publisher											labelled_scan_pub_;
	void construct_derived_data(std::vector<object_cluster_segments> &object_clusters);
	void save_derived_data(std::vector<object_cluster_segments> &object_clusters);
	std::string												derived_data_path_;
	golfcar_ml::svm_classifier *DATMO_classifier_;
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
	private_nh_.param("interval",		    inverval_tmp,       	5);
	interval_ = (size_t) inverval_tmp;

	vehicle_array_pub_		=   nh_.advertise<geometry_msgs::PoseArray>("vehicle_array", 2);
	collected_cloud_pub_		=   nh_.advertise<sensor_msgs::PointCloud>("collected_cloud", 2);

	laser_sub_ = new message_filters::Subscriber<sensor_msgs::LaserScan> (nh_, "/front_bottom_scan", 100);
	tf_filter_ = new tf::MessageFilter<sensor_msgs::LaserScan>(*laser_sub_, tf_, odom_frame_id_, 100);
	tf_filter_->registerCallback(boost::bind(&DATMO::scanCallback, this, _1));
	tf_filter_->setTolerance(ros::Duration(0.1));

	current_polygon_pub_ = nh_.advertise<geometry_msgs::PolygonStamped>("/approxy_polygon", 10);

	double img_side_length_tmp, img_resolution_tmp;
	private_nh_.param("img_side_length",      img_side_length_tmp,     50.0);
	private_nh_.param("img_resolution",       img_resolution_tmp,      0.2);
	img_side_length_ = (float)img_side_length_tmp;
	img_resolution_ = (float)img_resolution_tmp;

	initialize_local_image();
	scan_serial_ = 0;


	private_nh_.param("abstract_summary_path",       abstract_summary_path_,      std::string("/home/baoxing/data/labelled_data/abstract_summary.yml"));
	labelled_scan_pub_		=   nh_.advertise<sensor_msgs::LaserScan>("labelled_scan", 2);

	private_nh_.param("derived_data_path",       derived_data_path_,      std::string("/home/baoxing/data/derived_data"));


    string DATMO_model_path, DATMO_scale_path;
	private_nh_.param("DATMO_model_path", DATMO_model_path, std::string("/home/baoxing/workspace/data_and_model/DATMO.model"));
	private_nh_.param("DATMO_scale_path", DATMO_scale_path, std::string("/home/baoxing/workspace/data_and_model/DATMO.range"));
    DATMO_classifier_ = new golfcar_ml::svm_classifier(DATMO_model_path, DATMO_scale_path);
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

	//make a local "baselink" copy to facilitate later feature extraction;
	sensor_msgs::PointCloud baselink_verti_cloud;
	try{projector_.transformLaserScanToPointCloud(laser_frame_id_, *verti_scan_in, baselink_verti_cloud, tf_);}
	catch (tf::TransformException& e){ROS_DEBUG("Wrong!!!!!!!!!!!!!"); std::cout << e.what();return;}
	sensor_msgs::PointCloud verti_cloud;
	//make a global "odom" copy for later moving object extraction;
	try{projector_.transformLaserScanToPointCloud(odom_frame_id_, *verti_scan_in, verti_cloud, tf_);}
	catch (tf::TransformException& e){ROS_DEBUG("Wrong!!!!!!!!!!!!!"); std::cout << e.what();return;}
	//make sure both copies have the same number;
	assert(baselink_verti_cloud.points.size()==verti_cloud.points.size());
	collected_cloud_pub_.publish(baselink_verti_cloud);

	supervised_segmentation(verti_scan_in, baselink_verti_cloud, laser_pose_current_);

	//pay attention to use the intensity value;
	scan_vector_.push_back(*verti_scan_in);
	cloud_vector_.push_back(verti_cloud);
	baselink_cloud_vector_.push_back(baselink_verti_cloud);

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
		baselink_cloud_vector_.erase(baselink_cloud_vector_.begin(), baselink_cloud_vector_.begin()+1);
	}
	//scan_serial_++;
	ROS_INFO("scan callback finished");
}

//segment the scan using heuristic prior knowledge;
//similar to curb_detect::DP_Extraction();
//1st: objects on road are usually convex in shape;
//2nd: there should be a jump between range values on different objects;
//3rd: intensity readings on different objects should also be different;
void DATMO::supervised_segmentation(const sensor_msgs::LaserScan::ConstPtr& scan_in, const sensor_msgs::PointCloud& cloud_in, geometry_msgs::PoseStamped& laser_pose)
{
	sensor_msgs::PointCloud FR_pcl;
	FR_pcl=cloud_in;

	//1st channel is index; 2nd is for intensity; 3rd is used to put the response of ;
	assert(FR_pcl.channels.size()==2);
	sensor_msgs::ChannelFloat32 derivative_channel;
	derivative_channel.name = "second_order_derivative";
	derivative_channel.values.resize(FR_pcl.points.size(), 0.0);

}

void DATMO::segmentation_visulization()
{

}

void DATMO::scan_approxy(const sensor_msgs::PointCloud& cloud_in, geometry_msgs::PoseStamped& laser_pose)
{
	std::vector<Point2f> raw_input, approxy_output;
	for(size_t i=0; i<cloud_in.points.size(); i++)
	{
		raw_input.push_back(Point2f(cloud_in.points[i].x, cloud_in.points[i].y));
	}
	approxPolyDP(raw_input, approxy_output, 0.5, false);

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

	lastest_baselink_cloud_vector_.clear();
	for(size_t i=0; i<cloud_vector_.size();i++)
	{
		sensor_msgs::PointCloud latest_baselink_cloud;
		latest_baselink_cloud.header = cloud_vector_.back().header;
		try
		{
			tf_.transformPointCloud(laser_frame_id_, cloud_vector_[i], latest_baselink_cloud);
		}
		catch(tf::TransformException e)
		{
			ROS_WARN("Failed to transform accumulated point to laser_frame_id_ (%s)", e.what());
			return;
		}
		lastest_baselink_cloud_vector_.push_back(latest_baselink_cloud);
		//ROS_INFO("lastest_baselink_cloud_vector_: %u, %u", i, lastest_baselink_cloud_vector_[i].points.size());
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



	//------------------2nd: extract candidates and do online classification-------------------------
	std::vector<object_cluster_segments> object_clusters;
	for(size_t i=0;  i<appear_disappear_pairs.size(); i++)
	{
		//find the object "scan_segment" in this cluster;
		int appear_serial = appear_disappear_pairs[i].first;
		int disappear_serial = appear_disappear_pairs[i].second;
		if(appear_serial>=0 && disappear_serial>=0)
		{
			ROS_INFO("appear_disappear_pair %u", i);
			object_cluster_segments object_cluster_tmp;
			assert(scan_vector_.size()>0);
			bool stop_segment_extraction = false;

			//from the latest batch (noted as 0) to calculate, till the last batches (2*interval_);
			//if non-object label (or non-labelled scan) happen in between, the cluster will stop, and wrap up the already extracted scan batches;

			//loop from latest scan to scan 0;
			for(int a= (int)scan_vector_.size()-1; a>=0; a--)
			{
				int pointSerialInCloud = 0;
				compressed_scan_segment scan_segment_tmp;
				//cout<<"a:"<<a;
				for(size_t b=0; b<scan_vector_[a].ranges.size();b++)
				{
					if(scan_vector_[a].ranges[b]>=scan_vector_[a].range_min && scan_vector_[a].ranges[b]<=scan_vector_[a].range_max)
					{
						geometry_msgs::Point32 spacePt_tmp = lastest_baselink_cloud_vector_[a].points[pointSerialInCloud];
						Point2f imgpt_tmp;
						spacePt2ImgP(spacePt_tmp, imgpt_tmp);
						double check = pointPolygonTest(contours_combined[i], imgpt_tmp, true);
						if(check >= -2.0)
						{
							geometry_msgs::Point32 baselinkPt = baselink_cloud_vector_[a].points[pointSerialInCloud];
							scan_segment_tmp.rawPoints.push_back(baselinkPt);
							scan_segment_tmp.rawIntensities.push_back(scan_vector_[a].intensities[b]);
						}
						pointSerialInCloud++;
					}
				}
				if(scan_segment_tmp.rawPoints.size()!=0)
				{
					object_cluster_tmp.scan_segment_batch.push_back(scan_segment_tmp);
					object_cluster_tmp.contour_serial = (int)i;
				}
			}
			if(object_cluster_tmp.scan_segment_batch.size()>0)object_clusters.push_back(object_cluster_tmp);
			else ROS_INFO("batch no enought?");
		}
	}

	//process the extracted raw data;
	//construct_derived_data(object_clusters);

	//save the derived training data;
	//save_derived_data(object_clusters);

}

void DATMO::construct_derived_data(std::vector<object_cluster_segments> &object_clusters)
{
	ROS_INFO("construct derived data %u", object_clusters.size());
	//calculate the LIDAR pose in latest LIDAR frame;
	vector<geometry_msgs::Pose> Pose_inLatest_vector;
	tf::Pose odom_to_latestLIDAR;
	tf::poseMsgToTF(laser_pose_vector_.back().pose, odom_to_latestLIDAR);

	assert(laser_pose_vector_.size() == 2*interval_);
	for(int i=(int)(laser_pose_vector_.size())-1; i >=0 ; i--)
	{
		//cout<<i<<":";
		tf::Pose odom_to_LIDAR_tmp;
		tf::poseMsgToTF(laser_pose_vector_[i].pose, odom_to_LIDAR_tmp);
		tf::Pose LIDAR_new_to_old = odom_to_latestLIDAR.inverse()*odom_to_LIDAR_tmp;
		geometry_msgs::Pose pose_inLatest;
		tf::poseTFToMsg(LIDAR_new_to_old, pose_inLatest);
		Pose_inLatest_vector.push_back(pose_inLatest);
		//cout<<"("<<pose_inLatest.position.x<<","<<pose_inLatest.position.y<<")"<<"\t";
	}
	//cout<<endl;

	for(size_t i=0; i<object_clusters.size(); i++)
	{
		object_cluster_segments &object_cluster_tmp =object_clusters[i];
		for(size_t j=0; j<object_cluster_tmp.scan_segment_batch.size(); j++)
		{
			//matching the odom information with the cluster scan information;
			//pay attention to the sequence;
			assert(j<Pose_inLatest_vector.size());
			object_cluster_tmp.pose_InLatestCoord_vector.push_back(Pose_inLatest_vector[j]);

			//compress the scan_segment;
			object_cluster_tmp.scan_segment_batch[j].compress_scan();
		}
	}
}

void DATMO::save_derived_data(std::vector<object_cluster_segments> &object_clusters)
{
	ROS_INFO("classify the derived data");
	for(size_t i=0; i<object_clusters.size(); i++)
	{
		object_cluster_segments &object_cluster_tmp =object_clusters[i];
		size_t j=object_cluster_tmp.scan_segment_batch.size();

		assert(object_cluster_tmp.pose_InLatestCoord_vector.size()==object_cluster_tmp.scan_segment_batch.size());

		if(j!=2*interval_)
		{
			object_cluster_tmp.object_type = 0;
			ROS_INFO("can this happen? not likely");
			continue;
		}

		int vector_length = 19*2*int(interval_);
		double DATMO_feature_vector[vector_length];
		int vector_serial=0;
		for(size_t k=0; k<j; k++)
		{
			double pose[6];
			geometry_msgs::Pose pose_tmp = object_cluster_tmp.pose_InLatestCoord_vector[k];
			pose[0]=pose_tmp.position.x;
			pose[1]=pose_tmp.position.y;
			pose[2]=pose_tmp.position.z;
			tf::Quaternion q(pose_tmp.orientation.x, pose_tmp.orientation.y, pose_tmp.orientation.z, pose_tmp.orientation.w);
			tf::Matrix3x3 m(q);
			m.getRPY(pose[3], pose[4], pose[5]);

			//first printf the odometry information;
			for(size_t a=0; a<6; a++){DATMO_feature_vector[vector_serial+a]=pose[a];}
			vector_serial = vector_serial+6;
			//then printf the compressed scan segment;

			for(size_t a=0; a<3; a++)
			{
				DATMO_feature_vector[vector_serial+0]=object_cluster_tmp.scan_segment_batch[k].KeyPoint[a].x;
				DATMO_feature_vector[vector_serial+1]=object_cluster_tmp.scan_segment_batch[k].KeyPoint[a].y;
				DATMO_feature_vector[vector_serial+2]=object_cluster_tmp.scan_segment_batch[k].intensities[a];
				vector_serial = vector_serial+3;
			}

			DATMO_feature_vector[vector_serial+0] =  object_cluster_tmp.scan_segment_batch[k].m;
			DATMO_feature_vector[vector_serial+1] =  object_cluster_tmp.scan_segment_batch[k].n;
			DATMO_feature_vector[vector_serial+2] =  object_cluster_tmp.scan_segment_batch[k].sigmaM;
			DATMO_feature_vector[vector_serial+3] =  object_cluster_tmp.scan_segment_batch[k].sigmaN;
			vector_serial = vector_serial+4;
		}
		//cout<<endl;
		//for(int k=0; k<vector_length; k++)
		//{
			//cout<<DATMO_feature_vector[k]<<"\t";
		//}
		//cout<<endl;

		object_cluster_tmp.object_type = DATMO_classifier_->classify_objects(DATMO_feature_vector, vector_length);
		if(object_cluster_tmp.object_type ==1){ROS_INFO("vehicle!!!");}
		else if(object_cluster_tmp.object_type ==2){ROS_INFO("motorbike!!!");}
		else if (object_cluster_tmp.object_type ==0){ROS_INFO("background noise");}
		else{ROS_INFO("other type of objects");}
	}
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
