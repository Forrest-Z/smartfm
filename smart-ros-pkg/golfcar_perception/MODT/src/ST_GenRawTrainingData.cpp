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

#include <boost/thread.hpp>
#include <boost/shared_ptr.hpp>

#include <vector>
#include <cmath>

#include "pcl/point_cloud.h"
#include "pcl_ros/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl/ros/conversions.h"
#include "pcl/kdtree/kdtree_flann.h"
#include "segmentation/segment-graph.h"

#include <opencv/cv.h>
#include <opencv/highgui.h>

#include "DATMO_datatypes.h"
#include "svm_classifier.h"
#include "MODT/segment_pose_batches.h"

#include <fmutil/fm_stopwatch.h>
#include <opencv/cv.h>

using namespace std;
using namespace ros;
using namespace cv;

typedef boost::shared_ptr<nav_msgs::Odometry const> OdomConstPtr;
typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudRGB;

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
	void process_accumulated_data();
	void graph_segmentation();
	void perform_prefiltering_movingEvidence();

	void construct_raw_training_data();
	void save_raw_training_data();
	void save_visualization_image();


	tf::MessageFilter<sensor_msgs::LaserScan>				*tf_filter_;
	vector<sensor_msgs::PointCloud>                        	cloud_vector_;
	vector<sensor_msgs::PointCloud>                        	baselink_cloud_vector_;

	vector<sensor_msgs::LaserScan>                        	scan_vector_;
	vector<geometry_msgs::PoseStamped>						laser_pose_vector_;

	geometry_msgs::PoseStamped								laser_pose_current_;
	ros::Publisher                              			collected_cloud_pub_;

	PointCloudRGB 											combined_pcl_;
	ros::Publisher											segmented_pcl_pub_;
	ros::Publisher											filtered_pcl_pub_, erased_pcl_pub_;

	int 													downsample_interval_;
	int														scanNum_perVector_;
	size_t 													interval_;
	double													speed_threshold_;
	std::vector<std::pair<int, std::vector<int> > > 		object_cluster_IDs_;
	DATMO_RawTrainingData									raw_training_data_;


	double													img_side_length_, img_resolution_;
	cv::Mat													local_mask_;
	cv::Point												LIDAR_pixel_coord_;
	void initialize_local_image();
	void spacePt2ImgP(geometry_msgs::Point32 & spacePt, Point2f & imgPt);
	inline bool LocalPixelValid(Point2f & imgPt);
};

DATMO::DATMO()
: private_nh_("~")
{
	private_nh_.param("laser_frame_id",     laser_frame_id_,    std::string("front_bottom_lidar"));
	private_nh_.param("base_frame_id",      base_frame_id_,     std::string("base_link"));
	private_nh_.param("odom_frame_id",      odom_frame_id_,     std::string("odom"));
	private_nh_.param("map_frame_id",       map_frame_id_,      std::string("map"));

	private_nh_.param("speed_threshold",		speed_threshold_,      	3.0);

	//to reduce the size of feature vector;
	private_nh_.param("downsample_interval",	downsample_interval_,    1);
	private_nh_.param("scanNum_perVector",	scanNum_perVector_,    9);
	interval_ = (downsample_interval_)*(scanNum_perVector_-1) +1;

	laser_sub_ = new message_filters::Subscriber<sensor_msgs::LaserScan> (nh_, "/front_bottom_scan", 100);
	tf_filter_ = new tf::MessageFilter<sensor_msgs::LaserScan>(*laser_sub_, tf_, odom_frame_id_, 100);
	tf_filter_->registerCallback(boost::bind(&DATMO::scanCallback, this, _1));
	tf_filter_->setTolerance(ros::Duration(0.1));

	collected_cloud_pub_		=   nh_.advertise<sensor_msgs::PointCloud>("collected_cloud", 2);
	segmented_pcl_pub_			=   nh_.advertise<PointCloudRGB>("segmented_pcl", 2);
	filtered_pcl_pub_			=   nh_.advertise<PointCloudRGB>("filtered_pcl", 2);
	erased_pcl_pub_				=   nh_.advertise<PointCloudRGB>("erased_pcl", 2);

	private_nh_.param("img_side_length",      img_side_length_,     50.0);
	private_nh_.param("img_resolution",       img_resolution_,      0.2);

	initialize_local_image();
}

void DATMO::scanCallback (const sensor_msgs::LaserScan::ConstPtr& verti_scan_in)
{
	cout<<verti_scan_in->header.seq <<","<<verti_scan_in->header.frame_id;

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

	//pay attention to use the intensity value;
	scan_vector_.push_back(*verti_scan_in);
	cloud_vector_.push_back(verti_cloud);
	baselink_cloud_vector_.push_back(baselink_verti_cloud);

	geometry_msgs::PoseStamped ident;
	ident.header = verti_scan_in->header;
	ident.pose.position.x=0;
	ident.pose.position.y=0;
	ident.pose.position.z=0;
	ident.pose.orientation.x=0;
	ident.pose.orientation.y=0;
	ident.pose.orientation.z=0;
	ident.pose.orientation.w=1;
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

	assert(cloud_vector_.size() == laser_pose_vector_.size());

	if(cloud_vector_.size()== interval_)
	{
		process_accumulated_data();

		laser_pose_vector_.erase(laser_pose_vector_.begin(), laser_pose_vector_.begin()+1);
		cloud_vector_.erase(cloud_vector_.begin(), cloud_vector_.begin()+ 1);
		scan_vector_.erase(scan_vector_.begin(), scan_vector_.begin()+ 1);
		baselink_cloud_vector_.erase(baselink_cloud_vector_.begin(), baselink_cloud_vector_.begin()+1);
	}
	ROS_INFO("scan callback finished");
}

void DATMO::process_accumulated_data()
{
	cout<<endl;
	fmutil::Stopwatch sw;
	sw.start("graph segmentation");
	graph_segmentation();
	sw.end();

	sw.start("prefilter moving evidence");
	perform_prefiltering_movingEvidence();
	sw.end();

	construct_raw_training_data();
	save_visualization_image();
	save_raw_training_data();
}

void DATMO::graph_segmentation()
{
	fmutil::Stopwatch sw;
	sw.start("1st: edge vector construction");
	combined_pcl_.points.clear();
	combined_pcl_.header = cloud_vector_.back().header;
	srand (time(NULL));
	for(size_t i=0; i<interval_; i++)
	{
		for(size_t j=0; j<cloud_vector_[i].points.size(); j++)
		{
			pcl::PointXYZRGB pt_tmp;
			pt_tmp.x = cloud_vector_[i].points[j].x;
			pt_tmp.y = cloud_vector_[i].points[j].y;
			pt_tmp.z = cloud_vector_[i].points[j].z;
			pt_tmp.r = rand() % 256;
			pt_tmp.g = rand() % 256;
			pt_tmp.b = rand() % 256;
			combined_pcl_.points.push_back(pt_tmp);
		}
	}

	//1st:construct the data structure from collected pointcloud;
	//to learn the method from "segment_image.h"
	int vertix_num = (int)combined_pcl_.points.size();
	std::vector<edge> edge_vector;

	pcl::KdTreeFLANN<pcl::PointXYZRGB> kdtree;
	if(combined_pcl_.points.size()==0) return;
	kdtree.setInputCloud (combined_pcl_.makeShared());
	int K = 20;

	for(size_t i=0; i<combined_pcl_.points.size(); i++)
	{
		//choose K nearest points, and add the edges into "edge_vector";
		std::vector<int> pointIdxNKNSearch(K);
		std::vector<float> pointNKNSquaredDistance(K);
		pcl::PointXYZRGB searchPt_tmp = combined_pcl_.points[i];
		int num = kdtree.nearestKSearch (searchPt_tmp, K, pointIdxNKNSearch, pointNKNSquaredDistance);
		assert(num == (int)pointIdxNKNSearch.size() && num == (int)pointNKNSquaredDistance.size());
		for(size_t j=0; j<pointIdxNKNSearch.size(); j++)
		{
			edge edge_tmp;
			edge_tmp.a = i;
			edge_tmp.b = pointIdxNKNSearch[j];
			edge_tmp.w = pointNKNSquaredDistance[j];
			edge_vector.push_back(edge_tmp);
		}
	}

	int edge_num = (int)edge_vector.size();
	//edge edges[edge_num];
	//for(int i=0; i<edge_num; i++) edges[i] = edge_vector[i];
	float c = 3.0;
	sw.end();

	sw.start("2nd: graph segmentation");

	universe *u = segment_graph(vertix_num, edge_num, &edge_vector[0], c);
	object_cluster_IDs_.clear();
	for(size_t i=0; i<combined_pcl_.points.size(); i++)
	{
		int comp = u->find(i);
		combined_pcl_.points[i].r = combined_pcl_.points[comp].r;
		combined_pcl_.points[i].g = combined_pcl_.points[comp].g;
		combined_pcl_.points[i].b = combined_pcl_.points[comp].b;

		//store the cluster ids into certain data structure, for further classification;
		bool cluster_find = false;
		for(size_t j=0; j<object_cluster_IDs_.size(); j++)
		{
			if(comp == object_cluster_IDs_[j].first)
			{
				cluster_find = true;
				object_cluster_IDs_[j].second.push_back(i);
				break;
			}
		}
		if(!cluster_find)
		{
			//new_cluster: 	the first component is used to denote the representative in the disjoint-union forests,
			//				the seond vector is used to store point IDs in this cluster;
			std::pair<int, std::vector<int> > new_cluster;
			new_cluster.first = comp;
			new_cluster.second.push_back(i);
			object_cluster_IDs_.push_back(new_cluster);
		}
	}

	sw.end();

	segmented_pcl_pub_.publish(combined_pcl_);
}

void DATMO::perform_prefiltering_movingEvidence()
{
	cout<<"object_cluster_IDs number: "<<object_cluster_IDs_.size()<<endl;
	PointCloudRGB filtered_cloud;
	filtered_cloud.header = combined_pcl_.header;
	filtered_cloud.points.clear();

	PointCloudRGB erased_cloud;
	erased_cloud.header = combined_pcl_.header;
	erased_cloud.points.clear();

	for(size_t i=0; i<object_cluster_IDs_.size();)
	{
		PointCloudRGB cloud_tmp;
		for(size_t j=0; j<object_cluster_IDs_[i].second.size();j++)
		{
			int ID_tmp = object_cluster_IDs_[i].second[j];
			cloud_tmp.points.push_back(combined_pcl_.points[ID_tmp]);
		}

		bool cluster_tobe_filtered = false;

		//1st: to get the raw points in each scan segment_batch;
		object_cluster_segments cluster_tmp;
		assert(interval_ == cloud_vector_.size());
		cluster_tmp.scan_segment_batch.resize(interval_);
		for(size_t j=0; j<object_cluster_IDs_[i].second.size();j++)
		{
			int ID_tmp = object_cluster_IDs_[i].second[j];
			int base_IDs = 0;

			for(size_t k=0; k<cloud_vector_.size(); k++)
			{
				if(ID_tmp < (int)cloud_vector_[k].points.size()+base_IDs)
				{
					assert(ID_tmp >= base_IDs);
					int ID_in_singlecloud = ID_tmp - (int)base_IDs;
					//geometry_msgs::Point32 baselinkPt = baselink_cloud_vector_[k].points[ID_in_singlecloud];
					geometry_msgs::Point32 odomPt = cloud_vector_[k].points[ID_in_singlecloud];
					//cluster_tmp.scan_segment_batch[k].rawPoints.push_back(baselinkPt);
					cluster_tmp.scan_segment_batch[k].odomPoints.push_back(odomPt);

					break;
				}
				else
				{
					base_IDs = base_IDs + (int)cloud_vector_[k].points.size();
				}
			}
		}

		//2nd: check (a)geometric constraint and (b)moving evidence;
		PointCloud oldest_scanPoints, newest_scanPoints;
		for(size_t p=0;p<cluster_tmp.scan_segment_batch.front().odomPoints.size();p++)
		{
			pcl::PointXYZ pt_tmp;
			pt_tmp.x = cluster_tmp.scan_segment_batch.front().odomPoints[p].x;
			pt_tmp.y = cluster_tmp.scan_segment_batch.front().odomPoints[p].y;
			pt_tmp.z = cluster_tmp.scan_segment_batch.front().odomPoints[p].z;
			oldest_scanPoints.points.push_back(pt_tmp);
		}
		for(size_t p=0;p<cluster_tmp.scan_segment_batch.back().odomPoints.size();p++)
		{
			pcl::PointXYZ pt_tmp;
			pt_tmp.x = cluster_tmp.scan_segment_batch.back().odomPoints[p].x;
			pt_tmp.y = cluster_tmp.scan_segment_batch.back().odomPoints[p].y;
			pt_tmp.z = cluster_tmp.scan_segment_batch.back().odomPoints[p].z;
			newest_scanPoints.points.push_back(pt_tmp);
		}

		if(oldest_scanPoints.points.size()==0 || newest_scanPoints.points.size()==0)
		{
			cluster_tobe_filtered = true;
		}
		else
		{
			//a. geometric constraint;
			//a-1: for accumulated points;
			pcl::PointXYZRGB min_pt1, max_pt1;
			pcl::getMinMax3D (cloud_tmp, min_pt1, max_pt1);
			float x_dim1 = max_pt1.x - min_pt1.x;
			float y_dim1 = max_pt1.y - min_pt1.y;
			float max_dim1 = (float)sqrt(x_dim1*x_dim1+y_dim1*y_dim1);
			cluster_tobe_filtered = cluster_tobe_filtered || (max_dim1 < 2.0);

			//a-2: for latest scan points;
			pcl::PointXYZ min_pt2, max_pt2;
			pcl::getMinMax3D (newest_scanPoints, min_pt2, max_pt2);
			float x_dim2 = max_pt2.x - min_pt2.x;
			float y_dim2 = max_pt2.y - min_pt2.y;
			float max_dim2 = (float)sqrt(x_dim2*x_dim2+y_dim2*y_dim2);
			cluster_tobe_filtered = cluster_tobe_filtered || (max_dim2 > 15.0);

			//b: moving evidence;
			double moving_evidence_threshold = speed_threshold_ * (cloud_vector_.back().header.stamp.toSec()- cloud_vector_.front().header.stamp.toSec());

			pcl::KdTreeFLANN<pcl::PointXYZ> kdtree_oldest, kdtree_newest;
			kdtree_oldest.setInputCloud (oldest_scanPoints.makeShared());
			kdtree_newest.setInputCloud (newest_scanPoints.makeShared());

			int oldest_far =0;
			for(size_t p=0; p<oldest_scanPoints.points.size();p++)
			{
				std::vector<int> pointIdxNKNSearch(1);
				std::vector<float> pointNKNSquaredDistance(1);
				pcl::PointXYZ searchPt_tmp = oldest_scanPoints.points[p];
				int num = kdtree_newest.nearestKSearch (searchPt_tmp, 1, pointIdxNKNSearch, pointNKNSquaredDistance);
				assert(num==1);
				if(pointNKNSquaredDistance.front()>moving_evidence_threshold)oldest_far++;
			}

			/*
			newest_far = 0;
			for(size_t p=0; p<newest_scanPoints.points.size();p++)
			{
				std::vector<int> pointIdxNKNSearch(1);
				std::vector<float> pointNKNSquaredDistance(1);
				pcl::PointXYZ searchPt_tmp = newest_scanPoints.points[p];
				int num = kdtree_oldest.nearestKSearch (searchPt_tmp, 1, pointIdxNKNSearch, pointNKNSquaredDistance);
				assert(num==1);
				if(pointNKNSquaredDistance.front()>moving_evidence_threshold)newest_far++;
			}
			if(oldest_far>=2 && newest_far>=2)cluster_tobe_filtered = false;
			*/

			if(oldest_far<1)cluster_tobe_filtered = true;
		}

		if(cluster_tobe_filtered)
		{
			for(size_t p=0; p<cloud_tmp.points.size(); p++)
				erased_cloud.points.push_back(cloud_tmp.points[p]);
			object_cluster_IDs_.erase(object_cluster_IDs_.begin()+i);
		}
		else
		{
			for(size_t p=0; p<cloud_tmp.points.size(); p++)
				filtered_cloud.points.push_back(cloud_tmp.points[p]);
			i++;
		}

	}

	cout<<"after filtering, object_cluster_IDs number: "<<object_cluster_IDs_.size()<<endl;
	filtered_pcl_pub_.publish(filtered_cloud);
	erased_pcl_pub_.publish(erased_cloud);
}

void DATMO::construct_raw_training_data()
{
	raw_training_data_.scan_serials.clear();
	raw_training_data_.clusters_odom.clear();
	raw_training_data_.clusters_baselink.clear();
	raw_training_data_.scan_ClusterLabel_vector.clear();

	for(size_t i=0; i<scan_vector_.size(); i++)raw_training_data_.scan_serials.push_back((int)scan_vector_[i].header.seq);

	//here "-1" is for background noise, and object cluster labels begin with "0";
	std::vector<int> default_label(scan_vector_.back().ranges.size(), -1);
	raw_training_data_.scan_ClusterLabel_vector.resize(interval_, default_label);
	raw_training_data_.clusters_odom.resize(object_cluster_IDs_.size());
	raw_training_data_.clusters_baselink.resize(object_cluster_IDs_.size());

	for(size_t i=0; i<object_cluster_IDs_.size(); i++)
	{
		sensor_msgs::PointCloud cluster_points_inOdom;
		cluster_points_inOdom.header = cloud_vector_.back().header;
		//according to IDs in each cluster, find their corresponding points;
		for(size_t j=0; j<object_cluster_IDs_[i].second.size();j++)
		{
			int ID_tmp = object_cluster_IDs_[i].second[j];
			int base_IDs = 0;

			for(size_t k=0; k<cloud_vector_.size(); k++)
			{
				if(ID_tmp < (int)cloud_vector_[k].points.size()+base_IDs)
				{
					assert(ID_tmp >= base_IDs);
					int ID_in_singlecloud = ID_tmp - (int)base_IDs;

					//pay special attention: in case of using LIDARs with intensities, the first channel is "intensities", the second is "index";
					assert(cloud_vector_[k].channels[1].name == "index");
					int ID_in_singlescan = (int)cloud_vector_[k].channels[1].values[ID_in_singlecloud];

					geometry_msgs::Point32 odomPt = cloud_vector_[k].points[ID_in_singlecloud];
					cluster_points_inOdom.points.push_back(odomPt);
					raw_training_data_.scan_ClusterLabel_vector[k][ID_in_singlescan] = int(i);
					break;
				}
				else
				{
					base_IDs = base_IDs + (int)cloud_vector_[k].points.size();
				}
			}
		}
		raw_training_data_.clusters_odom[i] = cluster_points_inOdom;
	}
}

void DATMO::save_raw_training_data()
{
	stringstream  record_string;
	record_string<<"/home/baoxing/data/raw_data/data/training_data"<< scan_vector_.back().header.seq<<".yml";
	const string data_name = record_string.str();
	FileStorage fs(data_name.c_str(), FileStorage::WRITE);

	fs<<"scan_serials"<<"[:";
	for(size_t i=0; i<raw_training_data_.scan_serials.size(); i++)
	{
		fs<<(int)raw_training_data_.scan_serials[i];
	}
	fs<<"]";

	fs<<"scan_ClusterLabel_vector"<<"[";
	for(size_t i=0; i<raw_training_data_.scan_ClusterLabel_vector.size(); i++)
	{
		fs<<"[:";
		for(size_t j=0; j<raw_training_data_.scan_ClusterLabel_vector[i].size(); j++)
		{
			fs<<(int)raw_training_data_.scan_ClusterLabel_vector[i][j];
		}
		fs<<"]";
	}
	fs<<"]";

	fs<<"odom_points"<<"[";
	for(size_t i=0; i<raw_training_data_.clusters_odom.size(); i++)
	{
		fs<<"[:";
		for(size_t j=0; j<raw_training_data_.clusters_odom[i].points.size(); j++)
		{
			fs<<"{:"<<"x"<<raw_training_data_.clusters_odom[i].points[j].x<<"y"<<raw_training_data_.clusters_odom[i].points[j].y<<"}";
		}
		fs<<"]";
	}
	fs<<"]";

	fs<<"baselink_points"<<"[";
	for(size_t i=0; i<raw_training_data_.clusters_baselink.size(); i++)
	{
		fs<<"[:";
		for(size_t j=0; j<raw_training_data_.clusters_baselink[i].points.size(); j++)
		{
			fs<<"{:"<<"x"<<raw_training_data_.clusters_baselink[i].points[j].x<<"y"<<raw_training_data_.clusters_baselink[i].points[j].y<<"}";
		}
		fs<<"]";
	}
	fs<<"]";

	fs.release();
}

void DATMO::save_visualization_image()
{
	Mat cluster_image = local_mask_.clone();
	cluster_image = Scalar(0);

	for(size_t i=0; i<cloud_vector_.size(); i++)
	{
		sensor_msgs::PointCloud pointcloud_tmp;
		cloud_vector_[i].header = cloud_vector_.back().header;
		try{tf_.transformPointCloud(laser_frame_id_, cloud_vector_[i], pointcloud_tmp);}
		catch (tf::TransformException& e){ROS_DEBUG("Wrong!!!!!!!!!!!!!"); std::cout << e.what();return;}
		for(size_t j=0; j<pointcloud_tmp.points.size(); j++)
		{
			Point2f imgpt_tmp;
			spacePt2ImgP(pointcloud_tmp.points[j], imgpt_tmp);

			if(LocalPixelValid(imgpt_tmp))
			{
				cluster_image.at<uchar>((int)imgpt_tmp.y,(int)imgpt_tmp.x) = 100;
			}
		}
	}

	for(size_t i=0; i<raw_training_data_.clusters_odom.size(); i++)
	{
		sensor_msgs::PointCloud pointcloud_tmp;
		try{tf_.transformPointCloud(laser_frame_id_, raw_training_data_.clusters_odom[i], pointcloud_tmp);}
		catch (tf::TransformException& e){ROS_DEBUG("Wrong!!!!!!!!!!!!!"); std::cout << e.what();return;}
		for(size_t j=0; j<pointcloud_tmp.points.size(); j++)
		{
			Point2f imgpt_tmp;
			spacePt2ImgP(pointcloud_tmp.points[j], imgpt_tmp);

			if(LocalPixelValid(imgpt_tmp))
			{
				cluster_image.at<uchar>((int)imgpt_tmp.y,(int)imgpt_tmp.x) = 255;
			}
		}

		raw_training_data_.clusters_baselink[i] = pointcloud_tmp;
	}

	imshow("cluster_image", cluster_image);
	waitKey(3);

	stringstream  visualization_string;
	visualization_string<<"/home/baoxing/data/raw_data/vis_image/image_"<< scan_vector_.back().header.seq<<".jpg";
	const string image_name = visualization_string.str();
	imwrite(image_name, cluster_image);
}

void DATMO::initialize_local_image()
{
	local_mask_				= Mat((int)(img_side_length_/img_resolution_*2), (int)(img_side_length_/img_resolution_*2), CV_8UC1);
	local_mask_				= Scalar(0);
	LIDAR_pixel_coord_.x 	= (int)(img_side_length_/img_resolution_)-1;
	LIDAR_pixel_coord_.y 	= (int)(img_side_length_/img_resolution_)-1;

	vector<Point2f> img_roi;
	Point2f p0, p1, p2, p3, p4;
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
			Point2f point_tmp;
			point_tmp.x = (float)i;
			point_tmp.y = (float)j;
			if(cv::pointPolygonTest(img_roi, point_tmp, false)>0)local_mask_.at<uchar>(j,i)=255;
		}
}

void DATMO::spacePt2ImgP(geometry_msgs::Point32 & spacePt, Point2f & imgPt)
{
	imgPt.x = LIDAR_pixel_coord_.x - (spacePt.y/img_resolution_);
	imgPt.y = LIDAR_pixel_coord_.y - (spacePt.x/img_resolution_);
}

inline bool DATMO::LocalPixelValid(Point2f & imgPt)
{
	if((int)imgPt.x < local_mask_.cols && (int)imgPt.x >=0 && (int)imgPt.y < local_mask_.rows && (int)imgPt.y >=0) return true;
	else return false;
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "ST_GenRawTrainingData");
	DATMO DATMO_node;
	ros::spin();
	return (0);
}
