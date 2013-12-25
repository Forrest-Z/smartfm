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

using namespace std;
using namespace ros;

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

	void perform_prefiltering_simpleThresholding();
	void perform_prefiltering_movingEvidence();

	void construct_feature_vector();
	std::vector<double> get_vector(object_cluster_segments &object_cluster);

	void classify_clusters();

	void load_labeledScanMasks();
	void visualize_labelled_scan(const sensor_msgs::LaserScan::ConstPtr& scan_in);
	void save_training_data();

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
    ros::Publisher											vehicle_pcl_pub_;
    ros::Publisher											debug_pcl_pub_;
	std::vector<std::pair<int, std::vector<int> > > 		object_cluster_IDs_;
	std::vector<object_cluster_segments> 					object_feature_vectors_;
	golfcar_ml::svm_classifier 								*DATMO_classifier_;

	//to load labelled scan masks;
	std::string												abstract_summary_path_;
	int	Lstart_serial_, Lend_serial_;
	vector<vector<int> > labelled_masks_;
	ros::Publisher											labelled_scan_pub_;
	std::string												derived_data_path_;

	int														program_mode_;
	int 													feature_num_;
	int 													downsample_interval_;
	int														scanNum_perVector_;
	size_t 													interval_;
	double													speed_threshold_;
};

DATMO::DATMO()
: private_nh_("~")
{
	private_nh_.param("laser_frame_id",     laser_frame_id_,    std::string("front_bottom_lidar"));
	private_nh_.param("base_frame_id",      base_frame_id_,     std::string("base_link"));
	private_nh_.param("odom_frame_id",      odom_frame_id_,     std::string("odom"));
	private_nh_.param("map_frame_id",       map_frame_id_,      std::string("map"));


	//program_mode: "0" - save training data mode;
	//				"1" - online classification mode;
	private_nh_.param("program_mode",			program_mode_,     		1);
	private_nh_.param("feature_num",			feature_num_,       	13);
	private_nh_.param("speed_threshold",		speed_threshold_,      	3.0);

	//to reduce the size of feature vector;
	private_nh_.param("downsample_interval",	downsample_interval_,    4);
	private_nh_.param("scanNum_perVector",	scanNum_perVector_,    3);
	interval_ = (downsample_interval_)*(scanNum_perVector_-1) +1;

	laser_sub_ = new message_filters::Subscriber<sensor_msgs::LaserScan> (nh_, "/front_bottom_scan", 100);
	tf_filter_ = new tf::MessageFilter<sensor_msgs::LaserScan>(*laser_sub_, tf_, odom_frame_id_, 100);
	tf_filter_->registerCallback(boost::bind(&DATMO::scanCallback, this, _1));
	tf_filter_->setTolerance(ros::Duration(0.1));

	collected_cloud_pub_		=   nh_.advertise<sensor_msgs::PointCloud>("collected_cloud", 2);
	segmented_pcl_pub_			=   nh_.advertise<PointCloudRGB>("segmented_pcl", 2);
	filtered_pcl_pub_			=   nh_.advertise<PointCloudRGB>("filtered_pcl", 2);
	erased_pcl_pub_				=   nh_.advertise<PointCloudRGB>("erased_pcl", 2);
	vehicle_pcl_pub_			=   nh_.advertise<PointCloudRGB>("vehicle_pcl", 2);
	debug_pcl_pub_				=   nh_.advertise<sensor_msgs::PointCloud>("debug_cloud", 2);

    if(program_mode_==0)
    {
		private_nh_.param("abstract_summary_path",       abstract_summary_path_,      std::string("/home/baoxing/data/abstract_summary.yml"));
		labelled_scan_pub_		=   nh_.advertise<sensor_msgs::LaserScan>("labelled_scan", 2);
		private_nh_.param("derived_data_path",       derived_data_path_,      std::string("/home/baoxing/data/derived_data"));
		load_labeledScanMasks();
    }
    else if(program_mode_==1)
	{
	    string DATMO_model_path, DATMO_scale_path;
		private_nh_.param("DATMO_model_path", DATMO_model_path, std::string("/home/baoxing/workspace/data_and_model/MODT.model"));
		private_nh_.param("DATMO_scale_path", DATMO_scale_path, std::string("/home/baoxing/workspace/data_and_model/MODT.range"));
	    DATMO_classifier_ = new golfcar_ml::svm_classifier(DATMO_model_path, DATMO_scale_path);
	}
    else
    {
    	ROS_ERROR("currently program only has two modes: 0 or 1");
    	return;
    }
}

void DATMO::scanCallback (const sensor_msgs::LaserScan::ConstPtr& verti_scan_in)
{
	ROS_INFO("scan callback %u ", verti_scan_in->header.seq);
	if(program_mode_==0)visualize_labelled_scan(verti_scan_in);

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

	assert(cloud_vector_.size() == laser_pose_vector_.size());

	if(cloud_vector_.size()== interval_)
	{
		process_accumulated_data();
		if(program_mode_==0)
		{
			laser_pose_vector_.clear();
			cloud_vector_.clear();
			scan_vector_.clear();
			baselink_cloud_vector_.clear();
		}
		else
		{
			laser_pose_vector_.erase(laser_pose_vector_.begin(), laser_pose_vector_.begin()+1);
			cloud_vector_.erase(cloud_vector_.begin(), cloud_vector_.begin()+ 1);
			scan_vector_.erase(scan_vector_.begin(), scan_vector_.begin()+ 1);
			baselink_cloud_vector_.erase(baselink_cloud_vector_.begin(), baselink_cloud_vector_.begin()+1);
		}

	}
	ROS_INFO("scan callback finished");
}

void DATMO::process_accumulated_data()
{
	graph_segmentation();
	//perform_prefiltering_simpleThresholding();
	perform_prefiltering_movingEvidence();
	construct_feature_vector();

	if(program_mode_==0)save_training_data();
	else classify_clusters();
}

void DATMO::graph_segmentation()
{
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
	int K = 50;

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
	segmented_pcl_pub_.publish(combined_pcl_);
}

//do simple filtering for all the object clusters;
void DATMO::perform_prefiltering_simpleThresholding()
{
	cout<<"object_cluster_IDs number: "<<object_cluster_IDs_.size()<<endl;
	PointCloudRGB filtered_cloud;
	filtered_cloud.header = combined_pcl_.header;
	filtered_cloud.points.clear();

	for(size_t i=0; i<object_cluster_IDs_.size(); )
	{
		PointCloudRGB cloud_tmp;
		for(size_t j=0; j<object_cluster_IDs_[i].second.size();j++)
		{
			int ID_tmp = object_cluster_IDs_[i].second[j];
			cloud_tmp.points.push_back(combined_pcl_.points[ID_tmp]);
		}

		bool cluster_tobe_filtered;
		pcl::PointXYZRGB min_pt, max_pt;
		pcl::getMinMax3D (cloud_tmp, min_pt, max_pt);
		float x_dim = max_pt.x - min_pt.x;
		float y_dim = max_pt.y - min_pt.y;
		float max_dim = (float)sqrt(x_dim*x_dim+y_dim*y_dim);
		cluster_tobe_filtered = (max_dim < 2.0);

		if(cluster_tobe_filtered)
		{
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


//parse "object_cluster_IDs_", and construct feature vectors;
void DATMO::construct_feature_vector()
{
	sensor_msgs::PointCloud constructed_cloud;
	constructed_cloud.header = combined_pcl_.header;
	constructed_cloud.points.clear();

	object_feature_vectors_.clear();

	//cout<<"check ID in scans"<<endl;
	for(size_t i=0; i<object_cluster_IDs_.size(); i++)
	{
		//according to IDs in each cluster, find their corresponding points;
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

					//pay attention: in case of using LIDARs with intensities, the first channel is "intensities", the second is "index";
					assert(cloud_vector_[k].channels[1].name == "index");
					int ID_in_singlescan = (int)cloud_vector_[k].channels[1].values[ID_in_singlecloud];
					//cout<<ID_in_singlescan<<"\t";

					//pay attention here: to record the data in the baselink frame (the frame when the point is recorded);
					geometry_msgs::Point32 baselinkPt = baselink_cloud_vector_[k].points[ID_in_singlecloud];
					cluster_tmp.scan_segment_batch[k].serial_in_scan.push_back(ID_in_singlescan);
					cluster_tmp.scan_segment_batch[k].rawPoints.push_back(baselinkPt);
					geometry_msgs::Point32 odomPt = cloud_vector_[k].points[ID_in_singlecloud];
					cluster_tmp.scan_segment_batch[k].odomPoints.push_back(odomPt);
					cluster_tmp.scan_segment_batch[k].rawIntensities.push_back(scan_vector_[k].intensities[ID_in_singlescan]);

					constructed_cloud.points.push_back(odomPt);
					break;
				}
				else
				{
					base_IDs = base_IDs + (int)cloud_vector_[k].points.size();
				}
			}
		}
		object_feature_vectors_.push_back(cluster_tmp);
	}
	debug_pcl_pub_.publish(constructed_cloud);

	vector<geometry_msgs::Pose> Pose_inLatest_vector;
	tf::Pose odom_to_latestLIDAR;
	tf::poseMsgToTF(laser_pose_vector_.back().pose, odom_to_latestLIDAR);

	assert(laser_pose_vector_.size() == interval_);
	for(int i=0; i <(int)laser_pose_vector_.size() ; i++)
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

	for(size_t i=0; i<object_feature_vectors_.size(); i++)
	{
		object_cluster_segments &object_cluster_tmp =object_feature_vectors_[i];
		for(size_t j=0; j<object_cluster_tmp.scan_segment_batch.size(); j++)
		{
			//matching the odom information with the cluster scan information;
			//pay attention to the sequence;
			assert(j<Pose_inLatest_vector.size());
			object_cluster_tmp.pose_InLatestCoord_vector.push_back(Pose_inLatest_vector[j]);
			//compress the scan_segment (in another sense to extract the feature vector of the scan segment);
			object_cluster_tmp.scan_segment_batch[j].compress_scan();
		}
	}
}

void DATMO::save_training_data()
{
	if((int)scan_vector_.front().header.seq < Lstart_serial_ || (int)scan_vector_.back().header.seq > Lend_serial_ )
	{
		ROS_WARN("exceed range: cannot generate training data");
		return;
	}

	//try to generate the label "object_feature_vectors_" from labelled scan mask;
	std::vector<object_cluster_segments> object_feature_vectors_deputy_tmp;
	for(size_t i=0; i<object_feature_vectors_.size(); i++)
	{
		object_cluster_segments &object_cluster_tmp =object_feature_vectors_[i];
		bool partdata_notlabelled = false;	//Meaning some mask values are "4" - not properly labelled;
		for(size_t j=0; j<object_cluster_tmp.scan_segment_batch.size(); j++)
		{
			if(partdata_notlabelled)break;
			//cout<<"--------------"<<scan_vector_[j].header.seq<<"------------"<<endl;
			for(size_t k=0; k<object_cluster_tmp.scan_segment_batch[j].serial_in_scan.size(); k++)
			{
				int serial_in_scan_tmp = object_cluster_tmp.scan_segment_batch[j].serial_in_scan[k];
				int type_tmp  = labelled_masks_[((int)scan_vector_[j].header.seq-Lstart_serial_)][serial_in_scan_tmp];
				//cout<<"guess what?"<<serial_in_scan_tmp<<"-"<<type_tmp<<"\t";

				object_cluster_tmp.object_type = type_tmp;
				if(type_tmp == 4)
				{
					ROS_WARN("ffffff------kkkkkk");
					object_cluster_tmp.object_type = 4;
					partdata_notlabelled = true;
					break;
				}
			}
		}

		if(object_cluster_tmp.object_type != 4) object_feature_vectors_deputy_tmp.push_back(object_cluster_tmp);
		ROS_INFO("object labelled as %d", object_cluster_tmp.object_type);
	}
	object_feature_vectors_ = object_feature_vectors_deputy_tmp;


	//save derived training data;
	for(size_t i=0; i<object_feature_vectors_.size(); i++)
	{
		object_cluster_segments &object_cluster_tmp =object_feature_vectors_[i];
		size_t j=object_cluster_tmp.scan_segment_batch.size();
		assert(object_cluster_tmp.pose_InLatestCoord_vector.size()==object_cluster_tmp.scan_segment_batch.size());
		FILE *fp_write;
		stringstream file_path;
		file_path<<derived_data_path_<<"/"<< j << "_" <<downsample_interval_;
		fp_write = fopen(file_path.str().c_str(),"a");
		if(fp_write==NULL){ROS_ERROR("cannot write derived data file\n");return;}

		fprintf(fp_write, "%d\t", object_cluster_tmp.object_type);
		std::vector<double> feature_vector = get_vector(object_cluster_tmp);
		for(size_t k=0; k<feature_vector.size(); k++) fprintf(fp_write, "%lf\t", feature_vector[k]);
		fprintf(fp_write, "\n");
		fclose(fp_write);
	}
}

void DATMO::classify_clusters()
{
	ROS_INFO("classify the derived data");
	for(size_t i=0; i<object_feature_vectors_.size(); i++)
	{
		object_cluster_segments &object_cluster_tmp =object_feature_vectors_[i];
		size_t j=object_cluster_tmp.scan_segment_batch.size();

		assert(object_cluster_tmp.pose_InLatestCoord_vector.size()==object_cluster_tmp.scan_segment_batch.size());

		assert(j==interval_);

		int vector_length = feature_num_*int(scanNum_perVector_)-3;
		double DATMO_feature_vector[vector_length];

		std::vector<double> feature_vector = get_vector(object_cluster_tmp);

		/*
		cout<<"monitoring feature vector: ";
		for(int k=0; k<vector_length; k++)
		{
			DATMO_feature_vector[k]=feature_vector[k];
			cout<< k<<"-"<<DATMO_feature_vector[k]<<"\t";
		}
		cout<<endl;
		*/

		object_cluster_tmp.object_type = DATMO_classifier_->classify_objects(DATMO_feature_vector, vector_length);
		if(object_cluster_tmp.object_type ==1)
		{
			ROS_INFO("vehicle!!!");
			PointCloudRGB vehicle_cloud;
			vehicle_cloud.header = combined_pcl_.header;
			vehicle_cloud.points.clear();
			for(size_t j=0;j<object_cluster_tmp.scan_segment_batch.size(); j++)
			{
				for(size_t k=0;k<object_cluster_tmp.scan_segment_batch[j].odomPoints.size(); k++)
				{
					pcl::PointXYZRGB pt_tmp;
					pt_tmp.x = object_cluster_tmp.scan_segment_batch[j].odomPoints[k].x;
					pt_tmp.y = object_cluster_tmp.scan_segment_batch[j].odomPoints[k].y;
					pt_tmp.z = object_cluster_tmp.scan_segment_batch[j].odomPoints[k].z;
					pt_tmp.r = 255;
					pt_tmp.g = 0;
					pt_tmp.b = 0;
					vehicle_cloud.points.push_back(pt_tmp);
				}
			}
			vehicle_pcl_pub_.publish(vehicle_cloud);
		}
		else if(object_cluster_tmp.object_type ==2)
		{
			ROS_INFO("motorbike!!!");
		}
		else if (object_cluster_tmp.object_type ==0)
		{
			ROS_INFO("background noise");
		}
		else
		{
			ROS_INFO("other type of objects");
		}
	}
}

std::vector<double> DATMO::get_vector(object_cluster_segments &object_cluster)
{
	std::vector<double> feature_vector;
	size_t j=object_cluster.scan_segment_batch.size();
	assert(object_cluster.pose_InLatestCoord_vector.size()==object_cluster.scan_segment_batch.size());
	assert(j==interval_);

	//remember to reordered the sequence;
	//use "downsample_interval" to reduce the size of training data;
	for(int k=int(j)-1; k>=0; k=k-downsample_interval_)
	{
		double pose[6];
		geometry_msgs::Pose pose_tmp = object_cluster.pose_InLatestCoord_vector[k];
		pose[0]=pose_tmp.position.x;
		pose[1]=pose_tmp.position.y;
		pose[2]=pose_tmp.position.z;
		tf::Quaternion q(pose_tmp.orientation.x, pose_tmp.orientation.y, pose_tmp.orientation.z, pose_tmp.orientation.w);
		tf::Matrix3x3 m(q);
		m.getRPY(pose[3], pose[4], pose[5]);

		//first printf the odometry information: x, y, yaw;
		if(k!=int(j)-1)
		{
			feature_vector.push_back(pose[0]);
			feature_vector.push_back(pose[1]);
			feature_vector.push_back(pose[5]);
		}

		//then printf the compressed scan segment;
		for(size_t a=0; a<3; a++)
		{
			feature_vector.push_back((double)object_cluster.scan_segment_batch[k].KeyPoint[a].x);
			feature_vector.push_back((double)object_cluster.scan_segment_batch[k].KeyPoint[a].y);
		}

		feature_vector.push_back((double)object_cluster.scan_segment_batch[k].m);
		feature_vector.push_back((double)object_cluster.scan_segment_batch[k].n);
		feature_vector.push_back((double)object_cluster.scan_segment_batch[k].sigmaM);
		feature_vector.push_back((double)object_cluster.scan_segment_batch[k].sigmaN);
	}

	//very important: ignore the first 3 odom readings;
	int vector_length = feature_num_*int(scanNum_perVector_)-3;
	assert((int)feature_vector.size() == vector_length);
	return feature_vector;
}

void DATMO::load_labeledScanMasks()
{
	cv::FileStorage fs_read(abstract_summary_path_.c_str(), cv::FileStorage::READ);
	if(!fs_read.isOpened()){ROS_ERROR("cannot find data batch file"); return;}

	Lstart_serial_ = (int)fs_read["labelled_scan_startSerial"];
	Lend_serial_ = (int)fs_read["labelled_scan_endSerial"];

	cv::FileNode masks = fs_read["type_masks"];
	cv::FileNodeIterator mask_it = masks.begin(), mask_it_end = masks.end();
	int idx = 0;

	for( ; mask_it != mask_it_end; mask_it++, idx++)
	{
		vector<int> mask_tmp;
		(*mask_it) >> mask_tmp;
		labelled_masks_.push_back(mask_tmp);
	}

	assert(labelled_masks_.size() == (Lend_serial_ - Lstart_serial_+1));
	fs_read.release();
}

void DATMO::visualize_labelled_scan(const sensor_msgs::LaserScan::ConstPtr& scan_in)
{
	if((int)scan_in->header.seq > Lend_serial_ || (int)scan_in->header.seq < Lstart_serial_ )
	{
		ROS_WARN("scan %u not labelled", scan_in->header.seq);
		return;
	}
	else
	{
		assert(labelled_masks_[((int)scan_in->header.seq-Lstart_serial_)].size() == scan_in->ranges.size());
		sensor_msgs::LaserScan vis_scan = *scan_in;

		//cout<<"see the labelled scan_in:"<<scan_in->header.seq<<endl;
		for(size_t i=0; i<vis_scan.ranges.size(); i++)
		{
			if(labelled_masks_[((int)scan_in->header.seq-Lstart_serial_)][i]!=1) vis_scan.ranges[i]=0.0;
			else
			{
				//cout<<i<<"-"<<labelled_masks_[((int)scan_in->header.seq-Lstart_serial_)][i]<<"\t";
			}
		}
		//cout<<endl;
		labelled_scan_pub_.publish(vis_scan);
	}
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "DATMO_node");
	DATMO DATMO_node;
	ros::spin();
	return (0);
}
