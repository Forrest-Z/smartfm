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
	ros::Publisher                              			approx_polygon_pub_;

	size_t 													interval_;
	vector<sensor_msgs::PointCloud>                        	cloud_vector_;
	vector<geometry_msgs::PoseStamped>						laser_pose_vector_;
	geometry_msgs::PoseStamped								laser_pose_current_;

	//vehicle local image;
	double													img_side_length_, img_resolution_;
	Mat														local_mask_;
	Point													LIDAR_pixel_coord_;
	void 													initialize_local_image();
	void spacePt2ImgP(geometry_msgs::Point32 & spacePt, Point & imgPt);
	void ImgPt2spacePt(Point & imgPt, geometry_msgs::Point32 & spacePt);
	bool LocalPixelValid(Point & imgPt);

	//roadmap global image;
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
	initialize_local_image();
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

void DATMO::initialize_local_image()
{
	local_mask_				= Mat((int)(img_side_length_/img_resolution_), (int)(img_side_length_/img_resolution_*2), CV_8UC1);
	local_mask_				= Scalar(0);


	LIDAR_pixel_coord_.x 	= (int)(img_side_length_/img_resolution_)-1;
	LIDAR_pixel_coord_.y 	= (int)(img_side_length_/img_resolution_)-1;

	vector<geometry_msgs::Point32> img_roi;
	geometry_msgs::Point32 p0, p1, p2;
	p0.x = (float)LIDAR_pixel_coord_.x;
	p0.y = (float)LIDAR_pixel_coord_.y;
	p1.x = 0.0;
	p1.y = 0.0;
	p2.x = float(local_mask_.cols-1);
	p2.y = 0.0;

	img_roi.push_back(p0);
	img_roi.push_back(p1);
	img_roi.push_back(p2);

	for(int i=0; i<(int)local_mask_.cols; i++)
		for(int j=0; j<(int)local_mask_.rows; j++)
		{
			geometry_msgs::Point32 point_tmp;
			point_tmp.x = (float)i;
			point_tmp.y = (float)j;
			if(pointInPolygon(point_tmp, img_roi))local_mask_.at<uchar>(j,i)=255;
		}
}

void DATMO::spacePt2ImgP(geometry_msgs::Point32 & spacePt, Point & imgPt)
{
	imgPt.x = LIDAR_pixel_coord_.x - int(spacePt.y/img_resolution_);
	imgPt.y = LIDAR_pixel_coord_.y - int(spacePt.x/img_resolution_);
}
void DATMO::ImgPt2spacePt(Point & imgPt, geometry_msgs::Point32 & spacePt)
{
	//spacePt.x = (imgPt.x-LIDAR_pixel_coord_.x)*img_resolution_;
	//spacePt.y = (LIDAR_pixel_coord_.y - imgPt.y)*img_resolution_;
}

inline bool DATMO::LocalPixelValid(Point & imgPt)
{
	if(imgPt.x < local_mask_.cols && imgPt.x >=0 && imgPt.y < local_mask_.rows && imgPt.y >=0) return true;
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
		cloud_vector_.erase(cloud_vector_.begin());
		laser_pose_vector_.erase(laser_pose_vector_.begin());
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

	tf_.transformPointCloud(laser_frame_id_, accumulated_TminusOne_pcl, accumulated_TminusOne_pcl);
	tf_.transformPointCloud(laser_frame_id_, accumulated_T_pcl, accumulated_T_pcl);
	tf_.transformPointCloud(laser_frame_id_, current_T_pcl, current_T_pcl);

	Mat accumulated_TminusOne_img = local_mask_.clone();
	accumulated_TminusOne_img = Scalar(0);
	Mat accumulated_T_img = accumulated_TminusOne_img.clone();
	Mat current_T_image = accumulated_TminusOne_img.clone();

	for(size_t i=0; i<accumulated_TminusOne_pcl.points.size(); i++)
	{
		geometry_msgs::Point32 spacePt_tmp = accumulated_TminusOne_pcl.points[i];
		Point imgpt_tmp;
		spacePt2ImgP(spacePt_tmp, imgpt_tmp);
		if(LocalPixelValid(imgpt_tmp))
		{
			if(accumulated_TminusOne_img.at<uchar>(imgpt_tmp.y,imgpt_tmp.x) <255) accumulated_TminusOne_img.at<uchar>(imgpt_tmp.y,imgpt_tmp.x) = accumulated_TminusOne_img.at<uchar>(imgpt_tmp.y,imgpt_tmp.x)+1;
		}
	}

	for(size_t i=0; i<accumulated_T_pcl.points.size(); i++)
	{
		geometry_msgs::Point32 spacePt_tmp = accumulated_T_pcl.points[i];
		Point imgpt_tmp;
		spacePt2ImgP(spacePt_tmp, imgpt_tmp);
		if(LocalPixelValid(imgpt_tmp))
		{
			if(accumulated_T_img.at<uchar>(imgpt_tmp.y,imgpt_tmp.x) <255) accumulated_T_img.at<uchar>(imgpt_tmp.y,imgpt_tmp.x) = accumulated_T_img.at<uchar>(imgpt_tmp.y,imgpt_tmp.x)+1;
		}
	}

	for(size_t i=0; i<current_T_pcl.points.size(); i++)
	{
		geometry_msgs::Point32 spacePt_tmp = current_T_pcl.points[i];
		Point imgpt_tmp;
		spacePt2ImgP(spacePt_tmp, imgpt_tmp);
		if(LocalPixelValid(imgpt_tmp))
		{
			if(current_T_image.at<uchar>(imgpt_tmp.y,imgpt_tmp.x) <255) current_T_image.at<uchar>(imgpt_tmp.y,imgpt_tmp.x) = current_T_image.at<uchar>(imgpt_tmp.y,imgpt_tmp.x)+1;
		}
	}

	int dilation_size = 1;
	Mat element = getStructuringElement( MORPH_RECT, Size( 2*dilation_size + 1, 2*dilation_size+1 ), Point( dilation_size, dilation_size ) );

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

void DATMO::extract_moving_objects(Mat& accT, Mat& accTminusOne, Mat& new_appear, Mat& old_disappear, Mat& current_image)
{
	//here just for vehicle; some process need to be further modified for pedestrians;

	int open_size = 1; int close_size = 2;
	Mat open_element = getStructuringElement( MORPH_RECT, Size( 2*open_size + 1, 2*open_size+1 ), Point( open_size, open_size ) );
	Mat close_element = getStructuringElement( MORPH_RECT, Size( 2*close_size + 1, 2*close_size+1 ), Point( close_size, close_size ) );

	Mat combined_img;
	bitwise_or(accT, accTminusOne, combined_img, local_mask_);
	Mat new_appear_tmp, old_disappear_tmp;

	morphologyEx(combined_img, combined_img, CV_MOP_CLOSE, close_element);
	morphologyEx(combined_img, combined_img, CV_MOP_DILATE, open_element);
	morphologyEx(new_appear, new_appear_tmp,CV_MOP_CLOSE, close_element);
	//morphologyEx(new_appear_tmp, new_appear_tmp,CV_MOP_OPEN, open_element);
	morphologyEx(old_disappear, old_disappear_tmp, CV_MOP_CLOSE, close_element);
	//morphologyEx(old_disappear_tmp, old_disappear_tmp, CV_MOP_OPEN, open_element);

	vector<vector<Point> > contours_combined, contours_appear, contours_disappear;
	vector<Vec4i> hierarchy_combined, hierarchy_appear, hierarchy_disappear;

	findContours( combined_img, contours_combined, hierarchy_combined, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE, Point(0,0) );
	findContours( new_appear_tmp, contours_appear, hierarchy_appear, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE, Point(0,0) );
	findContours( old_disappear_tmp, contours_disappear, hierarchy_disappear, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE, Point(0,0) );

	vector<RotatedRect> minAreaRect_combined(contours_combined.size());
	vector<RotatedRect> minAreaRect_appear(contours_appear.size());
	vector<RotatedRect> minAreaRect_disappear(contours_disappear.size());

	for( int i = 0; i< (int)contours_combined.size(); i++ ) drawContours( combined_img, contours_combined, i, Scalar(255), 1, 8, vector<Vec4i>(), 0, Point() );
	for( int i = 0; i< (int)contours_appear.size(); i++ ) drawContours( new_appear_tmp, contours_appear, i, Scalar(255), -1, 8, vector<Vec4i>(), 0, Point() );
	for( int i = 0; i< (int)contours_disappear.size(); i++ ) drawContours( old_disappear_tmp, contours_disappear, i, Scalar(255), -1, 8, vector<Vec4i>(), 0, Point() );

	Mat three_chanels[3]={new_appear_tmp, combined_img, old_disappear_tmp};
	Mat merged_visualization;
	merge(three_chanels, 3, merged_visualization);
	imshow("colorful_plots", merged_visualization);
	waitKey(1);

	/*
	vector<Moments> mu(contours.size() );
	for( size_t i = 0; i < contours.size(); i++ ) {mu[i] = moments( contours[i], false );}
	vector<double> area( contours.size() );
	for( size_t i = 0; i < contours.size(); i++ ) {area[i] = contourArea(contours[i]);}
	*/
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
