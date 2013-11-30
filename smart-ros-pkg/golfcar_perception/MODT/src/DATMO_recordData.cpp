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
	message_filters::Subscriber<sensor_msgs::LaserScan>     *verti_laser_sub_;
	laser_geometry::LaserProjection                         projector_;

	void scanCallback (const sensor_msgs::LaserScan::ConstPtr& verti_scan_in);

	bool pointInPolygon(geometry_msgs::Point32 p, vector<geometry_msgs::Point32> poly);
	void process_accumulated_points();
	void extract_moving_objects(Mat& accT, Mat& accTminusOne, Mat& new_appear, Mat& old_disappear, Mat& current_image);

	tf::MessageFilter<sensor_msgs::LaserScan>				*tf_filter_;
	size_t 													interval_;
	vector<sensor_msgs::PointCloud>                        	cloud_vector_;
	vector<geometry_msgs::PoseStamped>						laser_pose_vector_;
	geometry_msgs::PoseStamped								laser_pose_current_;
	ros::Publisher                              			vehicle_array_pub_;
	ros::Publisher                              			moving_cloud_pub_;
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
};

DATMO::DATMO()
: private_nh_("~")
{
	private_nh_.param("laser_frame_id",     laser_frame_id_,    std::string("ldmrs"));
	private_nh_.param("base_frame_id",      base_frame_id_,     std::string("base_link"));
	private_nh_.param("odom_frame_id",      odom_frame_id_,     std::string("odom"));
	private_nh_.param("map_frame_id",       map_frame_id_,      std::string("map"));

	private_nh_.param("use_prior_map",       use_prior_map_,    true);
	private_nh_.param("map_resolution",     map_resolution_,    0.05);
	private_nh_.param("map_image_path",     map_image_path_,    std::string("./launch/road_map.jpg"));
	initialize_roadmap();

	int inverval_tmp;
	private_nh_.param("interval",		    inverval_tmp,       	10);
	interval_ = (size_t) inverval_tmp;

	vehicle_array_pub_		=   nh_.advertise<geometry_msgs::PoseArray>("vehicle_array", 2);
	moving_cloud_pub_		=   nh_.advertise<sensor_msgs::PointCloud>("moving_cloud", 2);

	verti_laser_sub_ = new message_filters::Subscriber<sensor_msgs::LaserScan> (nh_, "/sickldmrs/verti_laser", 100);
	tf_filter_ = new tf::MessageFilter<sensor_msgs::LaserScan>(*verti_laser_sub_, tf_, odom_frame_id_, 10);
	tf_filter_->registerCallback(boost::bind(&DATMO::scanCallback, this, _1));
	tf_filter_->setTolerance(ros::Duration(0.05));

	double img_side_length_tmp, img_resolution_tmp;
	private_nh_.param("img_side_length",      img_side_length_tmp,     50.0);
	private_nh_.param("img_resolution",       img_resolution_tmp,      0.2);
	img_side_length_ = (float)img_side_length_tmp;
	img_resolution_ = (float)img_resolution_tmp;

	initialize_local_image();
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
	ROS_INFO("scan callback begin");
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

	assert(cloud_vector_.size() == laser_pose_vector_.size());

	if(cloud_vector_.size()==interval_*2)
	{
		process_accumulated_points();
		cloud_vector_.erase(cloud_vector_.begin(), cloud_vector_.begin()+ 1);
		laser_pose_vector_.erase(laser_pose_vector_.begin(), laser_pose_vector_.begin()+1);
	}
	ROS_INFO("scan callback finished");
}

void DATMO::process_accumulated_points()
{
	sensor_msgs::PointCloud accumulated_TminusOne_pcl;
	sensor_msgs::PointCloud accumulated_T_pcl;
	sensor_msgs::PointCloud current_T_pcl;
	accumulated_TminusOne_pcl.header = cloud_vector_.back().header;
	accumulated_T_pcl.header = cloud_vector_.back().header;
	current_T_pcl.header = cloud_vector_.back().header;
	for(size_t i=0; i<interval_; i++)
	{
		for(size_t j=0; j<cloud_vector_[i].points.size(); j++)
		{
			geometry_msgs::Point32 spacePt_tmp = cloud_vector_[i].points[j];
			accumulated_TminusOne_pcl.points.push_back(spacePt_tmp);
		}
	}
	for(size_t i=interval_; i<2*interval_; i++)
	{
		for(size_t j=0; j<cloud_vector_[i].points.size(); j++)
		{
			geometry_msgs::Point32 spacePt_tmp = cloud_vector_[i].points[j];
			accumulated_T_pcl.points.push_back(spacePt_tmp);
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

	extract_moving_objects(accumulated_T_img, accumulated_TminusOne_img, new_appear, old_disappear, current_T_image);
}

//here just for vehicle; some process need to be further modified for pedestrians;
void DATMO::extract_moving_objects(Mat& accT, Mat& accTminusOne, Mat& new_appear, Mat& old_disappear, Mat& current_image)
{

	//-----------------1st step: pair-up disappear and appear patches, as moving object candidates;-----------------

	int open_size = 3; int close_size = 1;
	Mat open_element = getStructuringElement( MORPH_RECT, Size( 2*open_size + 1, 2*open_size+1 ), Point( open_size, open_size ) );
	Mat close_element = getStructuringElement( MORPH_RECT, Size( 2*close_size + 1, 2*close_size+1 ), Point( close_size, close_size ) );

	Mat combined_img;
	bitwise_or(accT, accTminusOne, combined_img, local_mask_);
	Mat new_appear_tmp, old_disappear_tmp;

	morphologyEx(combined_img, combined_img, CV_MOP_CLOSE, close_element);
	//morphologyEx(combined_img, combined_img, CV_MOP_DILATE, open_element);
	morphologyEx(new_appear, new_appear_tmp,CV_MOP_CLOSE, close_element);
	//morphologyEx(new_appear_tmp, new_appear_tmp,CV_MOP_OPEN, open_element);
	morphologyEx(old_disappear, old_disappear_tmp, CV_MOP_CLOSE, close_element);
	//morphologyEx(old_disappear_tmp, old_disappear_tmp, CV_MOP_OPEN, open_element);

	vector<vector<Point> > contours_combined, contours_appear, contours_disappear;
	vector<Vec4i> hierarchy_combined, hierarchy_appear, hierarchy_disappear;

	findContours( combined_img, contours_combined, hierarchy_combined, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE, Point(0,0) );
	findContours( new_appear_tmp, contours_appear, hierarchy_appear, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE, Point(0,0) );
	findContours( old_disappear_tmp, contours_disappear, hierarchy_disappear, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE, Point(0,0) );

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


	//------------------2nd: perform classification on the appear-disappear pairs;------------------

	//simple function here: process the appear-disappear pairs based on their sizes and shapes;
	vector<pair<int, int> > vehicle_pairs;
	Mat visualize_img = Mat(local_mask_.rows, local_mask_.cols, CV_8UC3);
	visualize_img = Scalar(0);
	for(size_t i=0;  i<appear_disappear_pairs.size(); i++)
	{
		int appear_serial = appear_disappear_pairs[i].first;
		int disappear_serial = appear_disappear_pairs[i].second;
		if(appear_serial>=0 && disappear_serial>=0)
		{
			Mat pair_img = Mat(local_mask_.rows, local_mask_.cols, CV_8UC1);
			pair_img = Scalar(0);
			double area_appear_tmp, area_disappear_tmp;
			area_appear_tmp = area_appear[appear_serial];
			area_disappear_tmp = area_disappear[disappear_serial];

			ROS_INFO("area %3f, %3f", area_appear_tmp, area_disappear_tmp);
			//if(area_appear_tmp > 1.0 && area_disappear_tmp > 1.0)
			{
				drawContours( pair_img, contours_appear, appear_serial, Scalar(255), -1, 8, vector<Vec4i>(), 0, Point() );
				drawContours( pair_img, contours_disappear, disappear_serial, Scalar(255), -1, 8, vector<Vec4i>(), 0, Point() );
				line(pair_img, Point((int)minAreaRect_appear[appear_serial].center.x, (int)minAreaRect_appear[appear_serial].center.y),
						Point((int)minAreaRect_disappear[disappear_serial].center.x, (int)minAreaRect_disappear[disappear_serial].center.y), Scalar(255), 3);
				vector<vector<Point> > contours_pair;
				vector<Vec4i> pair_hierarchy;
				findContours( pair_img, contours_pair, pair_hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE, Point(0,0) );
				RotatedRect minAreaRect_pair;
				minAreaRect_pair = minAreaRect( Mat(contours_pair[0]));
				ROS_INFO("minAreaRect_pair %3f, %3f", minAreaRect_pair.size.width, minAreaRect_pair.size.height);

				Point2f rect_points[4]; minAreaRect_pair.points( rect_points );
				vector<Point2f> rect_boundary;
			    for( int j = 0; j < 4; j++ ){rect_boundary.push_back(rect_points[j]);}

			    //try to utilize current image information to do filtering;
			    vector<Point> current_points;
			    Point previous_point;
			    previous_point.x = -1; previous_point.y = -1;
				for(int p = 0; p < current_image.rows; p++)
				{
					for(int q = 0; q < current_image.cols; q++)
					{
						if(current_image.at<uchar>(p,q)> 127 && (pointPolygonTest(rect_boundary, Point2f(q, p), false)>=0))
						{
							if(previous_point.x >=0 && previous_point.y >=0)line(current_image, Point(q, p), previous_point, Scalar(255), 1);
							current_points.push_back(Point(q, p));
							previous_point = Point(q, p);
						}
						else current_image.at<uchar>(p,q) = 0;
					}
				}

				sensor_msgs::PointCloud current_points_space;
				current_points_space.header.stamp = cloud_vector_.back().header.stamp;
				current_points_space.header.frame_id = laser_frame_id_;
				for(size_t pt = 0; pt<current_points.size(); pt++)
				{
					geometry_msgs::Point32 pt_space_tmp;
					Point2f pt_img_tmp(current_points[pt].x, current_points[pt].y);

					ImgPt2spacePt(pt_img_tmp, pt_space_tmp);
					current_points_space.points.push_back(pt_space_tmp);
				}
				moving_cloud_pub_.publish(current_points_space);

				bool on_road_flag = true;
				if(use_prior_map_)
				{

					bool transform_to_map_succeed = true;

					try
					{
						tf_.transformPointCloud(map_frame_id_, current_points_space, current_points_space);
					}
					catch(tf::TransformException e)
					{
						ROS_WARN("Failed to compute map pose, skipping scan (%s)", e.what());
						transform_to_map_succeed = false;
					}

					if(transform_to_map_succeed)
					for(size_t pt = 0; pt<current_points_space.points.size(); pt++)
					{
						if(!check_onRoad(current_points_space.points[pt])){on_road_flag = false; break;}
					}

				}
				if(!on_road_flag) continue;

				vector<vector<Point> > current_contours;
				vector<Vec4i> current_hierarchy;
				findContours( current_image, current_contours, current_hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE, Point(0,0));
				vector<RotatedRect> current_rect(current_contours.size()); for( size_t j = 0; j < current_contours.size(); j++ ) { current_rect[j] = minAreaRect( Mat(current_contours[j]) );}
				float current_long_side = 0.0;
				if(current_rect.size()>0)
				{
					current_long_side = current_rect.front().size.width>current_rect.front().size.height? current_rect.front().size.width:current_rect.front().size.height;
					double acc_short_side = minAreaRect_pair.size.width<minAreaRect_pair.size.height? minAreaRect_pair.size.width: minAreaRect_pair.size.height;
					bool vehicle_flag = true;
					//if(acc_short_side<1.0) vehicle_flag = false;
					//else vehicle_flag = true;
					if(vehicle_flag)
					{
						drawContours( visualize_img, contours_appear, appear_serial, Scalar(255, 0, 0), -1, 8, vector<Vec4i>(), 0, Point() );
						drawContours( visualize_img, contours_disappear, disappear_serial, Scalar(0, 0, 255), -1, 8, vector<Vec4i>(), 0, Point() );
						for( int j = 0; j < 4; j++ ){line( visualize_img, rect_points[j], rect_points[(j+1)%4], Scalar(0, 255, 255), 1, 8 );}
						vehicle_pairs.push_back(appear_disappear_pairs[i]);
					}
				}

			}
		}
	}


	//------------------3rd: generate the vehicle pose based on available information------------------

	geometry_msgs::PoseArray vehicle_array;
	vehicle_array.header = cloud_vector_.back().header;
	vehicle_array.header.frame_id = laser_frame_id_;

	for(size_t i=0;  i<vehicle_pairs.size(); i++)
	{
		int appear_serial = vehicle_pairs[i].first;
		int disappear_serial = vehicle_pairs[i].second;

		Mat pair_img = Mat(local_mask_.rows, local_mask_.cols, CV_8UC1);
		pair_img = Scalar(0);
		drawContours( pair_img, contours_appear, appear_serial, Scalar(255), -1, 8, vector<Vec4i>(), 0, Point() );
		drawContours( pair_img, contours_disappear, disappear_serial, Scalar(255), -1, 8, vector<Vec4i>(), 0, Point() );
		line(pair_img, Point((int)minAreaRect_appear[appear_serial].center.x, (int)minAreaRect_appear[appear_serial].center.y),
				Point((int)minAreaRect_disappear[disappear_serial].center.x, (int)minAreaRect_disappear[disappear_serial].center.y), Scalar(255), 3);
		vector<vector<Point> > contours_pair;
		vector<Vec4i> pair_hierarchy;
		findContours( pair_img, contours_pair, pair_hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE, Point(0,0) );
		RotatedRect minAreaRect_pair;
		minAreaRect_pair = minAreaRect( Mat(contours_pair[0]));

		Point2f rect_points[4]; minAreaRect_pair.points( rect_points );
		vector<Point2f> rect_boundary;
		for( int j = 0; j < 4; j++ ){line( visualize_img, rect_points[j], rect_points[(j+1)%4], Scalar(0, 255, 255), 1, 8 );rect_boundary.push_back(rect_points[j]);}

		//try to utilize current image information to do filtering;
		vector<Point> current_points;
		Point previous_point;
		previous_point.x = -1; previous_point.y = -1;
		for(int p = 0; p < current_image.rows; p++)
		{
			for(int q = 0; q < current_image.cols; q++)
			{
				if(current_image.at<uchar>(p,q)> 127 && (pointPolygonTest(rect_boundary, Point2f(q, p), false)>=0))
				{
					if(previous_point.x >=0 && previous_point.y >=0)line(current_image, Point(q, p), previous_point, Scalar(255), 1);
					current_points.push_back(Point(q, p));
					previous_point = Point(q, p);
				}
				else current_image.at<uchar>(p,q) = 0;
			}
		}

		vector<vector<Point> > current_contours;
		vector<Vec4i> current_hierarchy;
		findContours( current_image, current_contours, current_hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE, Point(0,0));
		vector<RotatedRect> current_rect(current_contours.size()); for( size_t j = 0; j < current_contours.size(); j++ ) { current_rect[j] = minAreaRect( Mat(current_contours[j]) );}

		Point2f appear_center = minAreaRect_appear[appear_serial].center;
		Point2f disappear_center = minAreaRect_disappear[disappear_serial].center;
		float moving_direction = atan2f(appear_center.y-disappear_center.y, appear_center.x-disappear_center.x);

		//Step A: find the vehicle center in the perpendicular (perpendicular to moving) direction - front_center;
		Point2f candidate_conner_points[2];
		float appear_center_distances[4];
		for( int j=0; j<4; j++)
		{
			appear_center_distances[j] = sqrtf((rect_points[j].x-appear_center.x)*(rect_points[j].x-appear_center.x)+(rect_points[j].y-appear_center.y)*(rect_points[j].y-appear_center.y));
		}
		float shortest_distance = 1000000.0;
		float second_shortest_distance = 100000.0;
		int shortest_serial = 0;
		int second_shortest_serial =1;
		for( int j=0; j<4; j++)
		{
			if(appear_center_distances[j]<= shortest_distance)
			{
				second_shortest_serial =shortest_serial;
				second_shortest_distance = shortest_distance;
				shortest_serial = j;
				shortest_distance = appear_center_distances[j];
			}
			else if(appear_center_distances[j]<= second_shortest_distance)
			{
				second_shortest_serial =j;
				second_shortest_distance = appear_center_distances[j];
			}
		}
		candidate_conner_points[0] = rect_points[shortest_serial];
		candidate_conner_points[1] = rect_points[second_shortest_serial];

		Point2f ImgPt1(0.0, 0.0);
		Point2f ImgPt2(cos(moving_direction), sin(moving_direction));
		geometry_msgs::Point32 spacePt1, spacePt2;
		ImgPt2spacePt(ImgPt1, spacePt1);
		ImgPt2spacePt(ImgPt2, spacePt2);
		float moving_angle_space = atan2f(spacePt2.y-spacePt1.y, spacePt2.x-spacePt1.x);

		geometry_msgs::Point32 front_space_pt1, front_space_pt2;
		ImgPt2spacePt(candidate_conner_points[0], front_space_pt1);
		ImgPt2spacePt(candidate_conner_points[1], front_space_pt2);

		Point2f front_center;
		float front_width = sqrtf((front_space_pt2.x-front_space_pt1.x)*(front_space_pt2.x-front_space_pt1.x)+(front_space_pt2.y-front_space_pt1.y)*(front_space_pt2.y-front_space_pt1.y));
		float vehicle_model_width = 1.6;
		ROS_INFO("vehicle_model_width %3f", front_width);
		if(front_width>= vehicle_model_width)
		{
			front_center.x = (front_space_pt1.x + front_space_pt2.x)/2.0;
			front_center.y = (front_space_pt1.y + front_space_pt2.y)/2.0;
		}
		else
		{
			geometry_msgs::Point32 near_conner, far_conner;
			float distance1 = sqrtf(front_space_pt1.x*front_space_pt1.x+front_space_pt1.y*front_space_pt1.y);
			float distance2 = sqrtf(front_space_pt2.x*front_space_pt2.x+front_space_pt2.y*front_space_pt2.y);
			if(distance1<distance2){near_conner = front_space_pt1; far_conner = front_space_pt2;}
			else {near_conner = front_space_pt2; far_conner = front_space_pt1;}

			float front_side_angle = atan2f(far_conner.y-near_conner.y, far_conner.x-near_conner.x);
			far_conner.x = near_conner.x + vehicle_model_width*cos(front_side_angle);
			far_conner.y = near_conner.y + vehicle_model_width*sin(front_side_angle);

			front_center.x = (near_conner.x + far_conner.x)/2.0;
			front_center.y = (near_conner.y + far_conner.y)/2.0;
		}

		//Step B: find the vehicle center in the vehicle moving direction;

		Point nearest_movDirectPt, farest_movDirectPt;
		float nearest_movingDirectDis = 1000000.0;
		float farest_movDirectDis = 0.0;
		float perpend_direction = moving_direction + M_PI_2;
		for(size_t j=0; j<current_points.size(); j++)
		{
			//to calculate the moving direction distance to the origin;
			float a = sin(perpend_direction);
			float b = -cos(perpend_direction);
			float c = cos(perpend_direction)*(float)current_points[j].y-sin(perpend_direction)*(float)current_points[j].x;
			float origin_Dis2line = fabs(a*(float)LIDAR_pixel_coord_.x+b*(float)LIDAR_pixel_coord_.y+c);

			if(origin_Dis2line<nearest_movingDirectDis) 	{nearest_movDirectPt = current_points[j];		nearest_movingDirectDis = origin_Dis2line;}
			if(origin_Dis2line>farest_movDirectDis)			{farest_movDirectPt  = current_points[j];		farest_movDirectDis = origin_Dis2line; }
		}

		bool moving_direction_away = true;
		//when points appear as a line which is perpendicular to the moving direction; this case should be rare;
		if(nearest_movDirectPt.x==farest_movDirectPt.x && nearest_movDirectPt.y==farest_movDirectPt.y)
		{
			Point2f vector_origin_to_pt;
			vector_origin_to_pt.x = farest_movDirectPt.x - LIDAR_pixel_coord_.x;
			vector_origin_to_pt.y = farest_movDirectPt.y - LIDAR_pixel_coord_.y;
			float vector_inner_dot = vector_origin_to_pt.x*cos(moving_direction)+vector_origin_to_pt.y*sin(moving_direction);
			if(vector_inner_dot>=0)moving_direction_away = true;
			else moving_direction_away = false;
		}
		else
		{
			Point2f vector_near_to_far;
			vector_near_to_far.x = farest_movDirectPt.x - nearest_movDirectPt.x;
			vector_near_to_far.y = farest_movDirectPt.y - nearest_movDirectPt.y;
			float vector_inner_dot = vector_near_to_far.x*cos(moving_direction)+vector_near_to_far.y*sin(moving_direction);
			if(vector_inner_dot>=0)moving_direction_away = true;
			else moving_direction_away = false;
		}

		Point2f center_vehicle_MovDirectPt;
		float center_shift_direction = moving_direction_away? moving_direction : -moving_direction;
		float vehicle_length = 3.5;
		center_vehicle_MovDirectPt.x = nearest_movDirectPt.x + cos(center_shift_direction)*vehicle_length/img_resolution_/2.0;
		center_vehicle_MovDirectPt.y = nearest_movDirectPt.y + sin(center_shift_direction)*vehicle_length/img_resolution_/2.0;

		geometry_msgs::Point32 center_movDirection_pt;
		ImgPt2spacePt(center_vehicle_MovDirectPt, center_movDirection_pt);

		float projection_line_angle = moving_angle_space + M_PI_2;

		//Step C: to get the center in both direction;
		// to project the "front_center" to the line defined by point "center_movDirection_pt", with angle "projection_line_angle";

		Point2f projection_real_center;
		projection_real_center.x = cos(projection_line_angle)*sin(projection_line_angle)*(front_center.y - center_movDirection_pt.y)+sin(projection_line_angle)*sin(projection_line_angle)*center_movDirection_pt.x
																																	+cos(projection_line_angle)*cos(projection_line_angle)*front_center.x;
		if(projection_line_angle>M_PI_2-0.001 && projection_line_angle< M_PI_2+0.001)
		{
			projection_real_center.y = front_center.y;
		}
		else
		{
			projection_real_center.y = center_movDirection_pt.y + std::tan(projection_line_angle)*(projection_real_center.x - center_movDirection_pt.x);
		}

		geometry_msgs::Pose pose_tmp;
		pose_tmp.position.x = projection_real_center.x;
		pose_tmp.position.y = projection_real_center.y;
		pose_tmp.position.z = 0.0;
		pose_tmp.orientation = tf::createQuaternionMsgFromYaw(moving_angle_space);
		vehicle_array.poses.push_back(pose_tmp);
		drawContours( visualize_img, current_contours, 0, Scalar(255, 255, 0), -1, 8, vector<Vec4i>(), 0, Point() );

	}

	imshow("visualize_img", visualize_img);
	waitKey(1);
	vehicle_array_pub_.publish(vehicle_array);
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
