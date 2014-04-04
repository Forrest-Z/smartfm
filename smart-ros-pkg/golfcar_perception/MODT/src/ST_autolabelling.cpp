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

#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/vfh.h>

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
	void process_accumulated_data(bool process_flag);
	void graph_segmentation();

	void perform_prefiltering_simpleThresholding();
	void perform_prefiltering_movingEvidence();

	void construct_feature_vector();
	void calc_ST_shapeFeatures(object_cluster_segments &object_cluster);

	int feature_extraction_type_;
	std::vector<double> (DATMO::*get_feature_vector_)(object_cluster_segments &object_cluster);
	std::vector<double> get_vector_V4(object_cluster_segments &object_cluster);
	std::vector<double> get_vector_3Dfeatures(object_cluster_segments &object_cluster);

	int calc_3dfeautres_method_;
	std::vector<double> (DATMO::*calc_3d_features_)(pcl::PointCloud<pcl::PointXYZ> &st_cloud);
	std::vector<double> ultrafast_shape_recognition(pcl::PointCloud<pcl::PointXYZ> &st_cloud);
	std::vector<double> moments2refpoint(pcl::PointCloud<pcl::PointXYZ> &st_cloud, pcl::PointXYZ &ref_point);
	std::vector<double> VFH(pcl::PointCloud<pcl::PointXYZ> &st_cloud);

	void classify_clusters();
	void calc_rough_pose();
	void calc_precise_pose();

	void load_labeledScanMasks();
	void visualize_labelled_scan(const sensor_msgs::LaserScan::ConstPtr& scan_in);
	void save_training_data();

	tf::MessageFilter<sensor_msgs::LaserScan>				*tf_filter_;
	vector<sensor_msgs::PointCloud>                        	cloud_vector_;
	vector<sensor_msgs::PointCloud>                        	baselink_cloud_vector_;
	vector<sensor_msgs::PointCloud>                        	latestLIDAR_cloud_vector_;

	vector<sensor_msgs::LaserScan>                        	scan_vector_;
	vector<geometry_msgs::PoseStamped>						laser_pose_vector_;

	geometry_msgs::PoseStamped								laser_pose_current_;
	ros::Publisher                              			collected_cloud_pub_;

	PointCloudRGB 											combined_pcl_;
	ros::Publisher											segmented_pcl_pub_;
	ros::Publisher											filtered_pcl_pub_, erased_pcl_pub_;
    ros::Publisher											vehicle_pcl_pub_, pedestrian_pcl_pub_;

    ros::Publisher											debug_pcl_pub_;
	std::vector<std::pair<int, std::vector<int> > > 		object_cluster_IDs_;
	std::vector<object_cluster_segments> 					object_feature_vectors_;

	//to load labelled scan masks;
	std::string												abstract_summary_path_;
	int	Lstart_serial_, Lend_serial_;
	vector<vector<int> > labelled_masks_;
	ros::Publisher											labelled_scan_pub_;
	std::string												derived_data_path_, autolabelled_data_path_;

	int														program_mode_;
	int 													feature_num_;
	int 													downsample_interval_;
	int														scanNum_perVector_;
	int														feature_vector_length_;

	size_t 													interval_;
	double													speed_threshold_;
	double 													gating_min_size_, gating_max_size_;

	golfcar_ml::svm_classifier 								*DATMO_classifier_;

	ros::Publisher											rough_pose_pub_;
	geometry_msgs::PoseArray								rough_poses_;

	ros::Publisher											segment_pose_batch_pub_;


	double													img_side_length_, img_resolution_;
	cv::Mat													local_mask_;
	cv::Point												LIDAR_pixel_coord_;
	void initialize_local_image();
	void spacePt2ImgP(geometry_msgs::Point32 & spacePt, cv::Point2f & imgPt);
	inline bool LocalPixelValid(cv::Point2f & imgPt);

	int														skip_scan_times_;
	bool 													process_scan_flag_;
	int														scan_count_;

	//this influence the quality (& speed) in the graph segmentation: in idea case, the bigger the better; but to speed up the process, this is set to some reasonable number; when setting a small number may lead to over-segmentation;
	int 													search_neighbourNum_;
	//this is the k parameter in the graph segmentation paper;
	double													graph_seg_k_;
	int 													control_trainingSample_number_;

	void load_obstacle_map();
	void autolabelling_data();
	int label_cluster_using_map(sensor_msgs::PointCloud &cluster_pcl);
	int check_onRoad(geometry_msgs::Point32 &point);

	cv::Mat													map_prior_, map_prior_unknown_;

	std::string												map_image_path_;
	double													map_resolution_;
	int 													dilation_size_, free_threshold_, unknown_threshold_;
	double													free_ratio_threshold_, unknown_ratio_threshold_;
	int 													tobe_labeled_label_;
	ros::Publisher                              			labeled_positive_objects_pub_;

	double													seg_time_coeff_;

	//choose whether to use "pose-variant" or "pose-invariant" features;
	bool													pose_variant_features_;


};

DATMO::DATMO()
: private_nh_("~")
{
	private_nh_.param("laser_frame_id",     laser_frame_id_,    std::string("front_bottom_lidar"));
	private_nh_.param("base_frame_id",      base_frame_id_,     std::string("base_link"));
	private_nh_.param("odom_frame_id",      odom_frame_id_,     std::string("odom"));
	private_nh_.param("map_frame_id",      	map_frame_id_,    	std::string("map"));
	private_nh_.param("gating_min_size",    gating_min_size_,   0.5);
	private_nh_.param("gating_max_size",    gating_max_size_,   15.0);

	//for vehicle detection;
	//private_nh_.param("graph_seg_k",    			graph_seg_k_,   		3.0);
	//private_nh_.param("search_neighbourNum",  	search_neighbourNum_,  	10);

	//for plaza pedestrian environment;
	private_nh_.param("graph_seg_k",    						graph_seg_k_,   		0.5);
	private_nh_.param("search_neighbourNum",  					search_neighbourNum_,  	50);
	private_nh_.param("control_trainingSample_number",  		control_trainingSample_number_,  	10);

	//program_mode: "0" - save training data mode;
	//				"1" - online classification mode;
	private_nh_.param("program_mode",			program_mode_,     			0);
	private_nh_.param("feature_num",			feature_num_,       		13);
	private_nh_.param("speed_threshold",		speed_threshold_,      		2.0);
	private_nh_.param("pose_variant_features",	pose_variant_features_,     true);

	//to reduce the size of feature vector;
	private_nh_.param("downsample_interval",	downsample_interval_,    1);
	private_nh_.param("scanNum_perVector",	scanNum_perVector_,    1);
	interval_ = (downsample_interval_)*(scanNum_perVector_-1) +1;

	//to process the scan at a lower rate;
	private_nh_.param("skip_scan_times",		skip_scan_times_,    2);
	scan_count_ = 0;

	//for version2;
	//private_nh_.param("feature_vector_length",	feature_vector_length_,    33);

	//V3: with ST_contour shape information added;
	//private_nh_.param("feature_vector_length",	feature_vector_length_,    40);

	//V4: with roll, pitch, and intensities[3] added;
	private_nh_.param("feature_vector_length",	feature_vector_length_,    25);

	private_nh_.param("img_side_length",	img_side_length_,    50.0);
	private_nh_.param("img_resolution",		img_resolution_,     0.2);
	private_nh_.param("seg_time_coeff",		seg_time_coeff_,     1.0);

	private_nh_.param("feature_extraction_type",		feature_extraction_type_,     1);

	if(feature_extraction_type_==1)
	{
		get_feature_vector_ = &DATMO::get_vector_V4;
	}
	else if(feature_extraction_type_ ==2)
	{
		get_feature_vector_ = &DATMO::get_vector_3Dfeatures;
	}
	else
	{
		ROS_ERROR("please select the correct feature type");
	}

	private_nh_.param("calc_3dfeautres_method",		calc_3dfeautres_method_,     1);
	if(calc_3dfeautres_method_==1)
	{
		calc_3d_features_ = &DATMO::ultrafast_shape_recognition;
	}
	else if(calc_3dfeautres_method_ ==2)
	{
		calc_3d_features_ = &DATMO::VFH;
	}
	else
	{
		ROS_ERROR("please select the method for 3d features calculation, if in use");
	}


	laser_sub_ = new message_filters::Subscriber<sensor_msgs::LaserScan> (nh_, "front_bottom_scan", 10);

	if(program_mode_ == 1) tf_filter_ = new tf::MessageFilter<sensor_msgs::LaserScan>(*laser_sub_, tf_, map_frame_id_, 10);
	else  tf_filter_ = new tf::MessageFilter<sensor_msgs::LaserScan>(*laser_sub_, tf_, odom_frame_id_, 10);

	tf_filter_->registerCallback(boost::bind(&DATMO::scanCallback, this, _1));
	tf_filter_->setTolerance(ros::Duration(0.1));

	collected_cloud_pub_		=   nh_.advertise<sensor_msgs::PointCloud>("collected_cloud", 2);
	segmented_pcl_pub_			=   nh_.advertise<PointCloudRGB>("segmented_pcl", 2);
	filtered_pcl_pub_			=   nh_.advertise<PointCloudRGB>("filtered_pcl", 2);
	erased_pcl_pub_				=   nh_.advertise<PointCloudRGB>("erased_pcl", 2);
	vehicle_pcl_pub_			=   nh_.advertise<PointCloudRGB>("vehicle_pcl", 2);
	pedestrian_pcl_pub_			=   nh_.advertise<PointCloudRGB>("pedestrian_pcl", 2);
	debug_pcl_pub_				=   nh_.advertise<sensor_msgs::PointCloud>("debug_cloud", 2);
	rough_pose_pub_				=   nh_.advertise<geometry_msgs::PoseArray>("rough_poses", 2);
	segment_pose_batch_pub_		=   nh_.advertise<MODT::segment_pose_batches>("segment_pose_batches", 2);

    if(program_mode_==0)
    {
		private_nh_.param("abstract_summary_path",       abstract_summary_path_,      std::string("/home/baoxing/data/abstract_summary.yml"));
		labelled_scan_pub_		=   nh_.advertise<sensor_msgs::LaserScan>("labelled_scan", 2);
		private_nh_.param("derived_data_path",       derived_data_path_,      std::string("/home/baoxing/data/derived_data"));
		load_labeledScanMasks();
    }
    else if(program_mode_ == 1)
    {
    	private_nh_.param("dilation_size",		dilation_size_,    0);
    	private_nh_.param("free_threshold",		free_threshold_,   220);
    	private_nh_.param("unknown_threshold",	unknown_threshold_,   100);
    	//local obstacle map;
    	private_nh_.param("map_image_path",     map_image_path_,      std::string("obstacle_layer.pgm"));

    	map_prior_ = cv::imread( map_image_path_.c_str(), CV_LOAD_IMAGE_GRAYSCALE );
    	map_prior_unknown_ = map_prior_.clone();

    	cv::Mat element = cv::getStructuringElement( cv::MORPH_RECT, cv::Size( 2*dilation_size_ + 1, 2*dilation_size_+1 ), cv::Point( dilation_size_, dilation_size_ ) );
    	cv::threshold(map_prior_, map_prior_, free_threshold_, 255, 0 );
    	cv::erode(map_prior_, map_prior_, element);
    	cv::imwrite("/home/baoxing/threshold_map.png", map_prior_);

    	cv::inRange(map_prior_unknown_, unknown_threshold_, free_threshold_, map_prior_unknown_);
    	cv::imwrite("/home/baoxing/unknown_area.png", map_prior_unknown_);

    	private_nh_.param("autolabelled_data_path",   	autolabelled_data_path_,     	std::string("/home/baoxing/data/auto_data"));
    	private_nh_.param("free_ratio_threshold",		free_ratio_threshold_,   		0.5);
    	private_nh_.param("unknown_ratio_threshold",	unknown_ratio_threshold_,   		0.4);
    	private_nh_.param("tobe_labeled_label",			tobe_labeled_label_,   			2);
    	labeled_positive_objects_pub_		=   nh_.advertise<sensor_msgs::PointCloud>("labeled_positive_objects", 2);
    	private_nh_.param("map_resolution",     map_resolution_,      0.1);

    }
    else if(program_mode_ == 2)
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

    initialize_local_image();
}

void DATMO::scanCallback (const sensor_msgs::LaserScan::ConstPtr& verti_scan_in)
{
	cout<<verti_scan_in->header.seq <<endl;
	if(program_mode_==0){ROS_INFO("use scan mask"); visualize_labelled_scan(verti_scan_in);}

	if(scan_count_%skip_scan_times_==0) process_scan_flag_ = true;
	else process_scan_flag_ = false;

	scan_count_++;

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

	latestLIDAR_cloud_vector_.clear();
	for(size_t i=0; i<cloud_vector_.size(); i++)
	{
		sensor_msgs::PointCloud cloud_latestLIDAR = cloud_vector_[i];
		cloud_latestLIDAR.header = cloud_vector_.back().header;
		try{tf_.transformPointCloud(laser_frame_id_, cloud_latestLIDAR, cloud_latestLIDAR);}
		catch (tf::TransformException& e){ROS_DEBUG("Wrong!!!!!!!!!!!!!"); std::cout << e.what();return;}
		latestLIDAR_cloud_vector_.push_back(cloud_latestLIDAR);
	}

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
		if(program_mode_==0 || program_mode_ ==1)
		{

			bool to_process_flag;
			//to control the same size of training sample, even when the time window is different;
			if(scan_count_%control_trainingSample_number_==0) to_process_flag = true;
			else to_process_flag = false;

			process_accumulated_data(to_process_flag);

			/*
			laser_pose_vector_.clear();
			cloud_vector_.clear();
			scan_vector_.clear();
			baselink_cloud_vector_.clear();
			*/

			laser_pose_vector_.erase(laser_pose_vector_.begin(), laser_pose_vector_.begin()+1);
			cloud_vector_.erase(cloud_vector_.begin(), cloud_vector_.begin()+ 1);
			scan_vector_.erase(scan_vector_.begin(), scan_vector_.begin()+ 1);
			baselink_cloud_vector_.erase(baselink_cloud_vector_.begin(), baselink_cloud_vector_.begin()+1);
		}
		else
		{
			process_accumulated_data(process_scan_flag_);
			laser_pose_vector_.erase(laser_pose_vector_.begin(), laser_pose_vector_.begin()+1);
			cloud_vector_.erase(cloud_vector_.begin(), cloud_vector_.begin()+ 1);
			scan_vector_.erase(scan_vector_.begin(), scan_vector_.begin()+ 1);
			baselink_cloud_vector_.erase(baselink_cloud_vector_.begin(), baselink_cloud_vector_.begin()+1);
		}
	}
	ROS_INFO("scan callback finished");
}

void DATMO::process_accumulated_data(bool process_flag)
{
	if(!process_flag)return;

	cout<<endl;
	fmutil::Stopwatch sw;
	sw.start("graph segmentation");
	graph_segmentation();
	sw.end();

	//perform_prefiltering_simpleThresholding();
	sw.start("prefilter moving evidence");
	perform_prefiltering_movingEvidence();
	sw.end();

	sw.start("construct feature vector");
	construct_feature_vector();
	sw.end();

	if(program_mode_==0)
	{
		save_training_data();
	}
	else if(program_mode_==1)
	{
		autolabelling_data();
	}
	else
	{
		sw.start("classify clusters");
		classify_clusters();
		sw.end();

		sw.start("calculate rough pose");
		calc_rough_pose();
		sw.end();

		sw.start("calculate accurate pose");
		calc_precise_pose();
		sw.end();
	}
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
			double time_delayed = cloud_vector_.back().header.stamp.toSec() - cloud_vector_[i].header.stamp.toSec();
			pcl::PointXYZRGB pt_tmp;
			pt_tmp.x = cloud_vector_[i].points[j].x;
			pt_tmp.y = cloud_vector_[i].points[j].y;
			//pt_tmp.z = cloud_vector_[i].points[j].z;
			pt_tmp.z = cloud_vector_[i].points[j].z + time_delayed*seg_time_coeff_;

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
	int K = search_neighbourNum_;

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
	float c = (float)graph_seg_k_;
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
			cluster_tobe_filtered = cluster_tobe_filtered || (max_dim1 < gating_min_size_);

			//a-2: for latest scan points;
			pcl::PointXYZ min_pt2, max_pt2;
			pcl::getMinMax3D (newest_scanPoints, min_pt2, max_pt2);
			float x_dim2 = max_pt2.x - min_pt2.x;
			float y_dim2 = max_pt2.y - min_pt2.y;
			float max_dim2 = (float)sqrt(x_dim2*x_dim2+y_dim2*y_dim2);
			cluster_tobe_filtered = cluster_tobe_filtered || (max_dim2 > gating_max_size_);

			//if(!cluster_tobe_filtered)ROS_INFO("size OK");

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
				if(pointNKNSquaredDistance.front()+0.0001>moving_evidence_threshold)oldest_far++;
			}

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


					//to insert the point in a proper position so that the points are recorded in a proper sequence.
					size_t proper_position = 0; proper_position = cluster_tmp.scan_segment_batch[k].serial_in_scan.size();

					for(size_t ss=0; ss<cluster_tmp.scan_segment_batch[k].serial_in_scan.size(); ss++)
					{
						if(ID_in_singlescan<cluster_tmp.scan_segment_batch[k].serial_in_scan[ss])
						{
							proper_position = ss-1;
							break;
						}
					}
					//cout<<ID_in_singlescan<<","<<proper_position<<"\t";

					//pay attention here: to record the data in the baselink frame (the frame when the point is recorded);
					geometry_msgs::Point32 baselinkPt = baselink_cloud_vector_[k].points[ID_in_singlecloud];
					geometry_msgs::Point32 odomPt = cloud_vector_[k].points[ID_in_singlecloud];
					geometry_msgs::Point32 latestLIDARpt = latestLIDAR_cloud_vector_[k].points[ID_in_singlecloud];

					/*
					cluster_tmp.scan_segment_batch[k].serial_in_scan.push_back(ID_in_singlescan);
					cluster_tmp.scan_segment_batch[k].rawPoints.push_back(baselinkPt);
					cluster_tmp.scan_segment_batch[k].odomPoints.push_back(odomPt);
					cluster_tmp.scan_segment_batch[k].rawIntensities.push_back(scan_vector_[k].intensities[ID_in_singlescan]);
					*/

					cluster_tmp.scan_segment_batch[k].serial_in_scan.insert(cluster_tmp.scan_segment_batch[k].serial_in_scan.begin()+proper_position, ID_in_singlescan);
					cluster_tmp.scan_segment_batch[k].rawPoints.insert(cluster_tmp.scan_segment_batch[k].rawPoints.begin()+proper_position, baselinkPt);
					cluster_tmp.scan_segment_batch[k].odomPoints.insert(cluster_tmp.scan_segment_batch[k].odomPoints.begin()+proper_position, odomPt);
					cluster_tmp.scan_segment_batch[k].lidarPoints.insert(cluster_tmp.scan_segment_batch[k].lidarPoints.begin()+proper_position, latestLIDARpt);

					cluster_tmp.scan_segment_batch[k].rawIntensities.insert(cluster_tmp.scan_segment_batch[k].rawIntensities.begin()+proper_position, scan_vector_[k].intensities[ID_in_singlescan]);

					constructed_cloud.points.push_back(odomPt);
					break;
				}
				else
				{
					base_IDs = base_IDs + (int)cloud_vector_[k].points.size();
				}
			}
		}

		for(size_t j=0; j<cluster_tmp.scan_segment_batch.size(); j++)
		{
			if(cluster_tmp.scan_segment_batch[j].rawPoints.size()==0)
			{
				cluster_tmp.scan_segment_batch[j].front_dist2background = 0.0;
				cluster_tmp.scan_segment_batch[j].back_dist2background  = 0.0;
				continue;
			}
			geometry_msgs::Point32 front_rawPt = cluster_tmp.scan_segment_batch[j].rawPoints.front();
			geometry_msgs::Point32 back_rawPt  = cluster_tmp.scan_segment_batch[j].rawPoints.back();
			double frontPt_distance = sqrt(front_rawPt.x*front_rawPt.x + front_rawPt.y*front_rawPt.y);
			double backPt_distance = sqrt(back_rawPt.x*back_rawPt.x + back_rawPt.y*back_rawPt.y);

			int front_scanSerial = cluster_tmp.scan_segment_batch[j].serial_in_scan.front();
			int back_scanSerial  = cluster_tmp.scan_segment_batch[j].serial_in_scan.back();

			//to deal with the FOV issue;
			if(front_scanSerial==0) cluster_tmp.scan_segment_batch[j].front_dist2background = 0.0;
			else
			{
				double front_backgroundDistance =scan_vector_[j].ranges[front_scanSerial-1];

				if(front_backgroundDistance>scan_vector_[j].range_max || front_backgroundDistance<scan_vector_[j].range_min) cluster_tmp.scan_segment_batch[j].front_dist2background = 0.0;
				else cluster_tmp.scan_segment_batch[j].front_dist2background = frontPt_distance - front_backgroundDistance;
			}

			if(back_scanSerial+1== (int)scan_vector_[j].ranges.size())cluster_tmp.scan_segment_batch[j].back_dist2background = 0.0;
			else
			{
				double back_backgroundDistance =scan_vector_[j].ranges[back_scanSerial+1];
				if(back_backgroundDistance>scan_vector_[j].range_max || back_backgroundDistance<scan_vector_[j].range_min) cluster_tmp.scan_segment_batch[j].back_dist2background = 0.0;
				else cluster_tmp.scan_segment_batch[j].back_dist2background = backPt_distance - back_backgroundDistance;
			}
		}

		calc_ST_shapeFeatures(cluster_tmp);

		//erase cluster who is generated without the latest lidar scan;
		if(cluster_tmp.scan_segment_batch.back().odomPoints.size()!=0) object_feature_vectors_.push_back(cluster_tmp);
	}
	debug_pcl_pub_.publish(constructed_cloud);

	vector<geometry_msgs::Pose> Pose_inLatest_vector, Pose_inOdom_vector;
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
		Pose_inOdom_vector.push_back(laser_pose_vector_[i].pose);
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
			object_cluster_tmp.pose_InOdom_vector.push_back(Pose_inOdom_vector[j]);
			//compress the scan_segment (in another sense to extract the feature vector of the scan segment);

			if(pose_variant_features_)object_cluster_tmp.scan_segment_batch[j].compress_scan();
			else object_cluster_tmp.scan_segment_batch[j].compress_scan_latestLIDAR();
		}
		object_cluster_tmp.GetCentroid_latestLIDAR();

		if(pose_variant_features_) object_cluster_tmp.GetCentroid_rawPoints();
		else object_cluster_tmp.GenPosInvariantCompressedSegment();
	}
}


void DATMO::calc_ST_shapeFeatures(object_cluster_segments &object_cluster)
{
	cv::Mat ST_shape_image = cv::Mat(local_mask_.rows, local_mask_.cols, CV_8UC1);
	cv::Mat ST_shape_image_show = cv::Mat(local_mask_.rows, local_mask_.cols, CV_8UC3);
	ST_shape_image = cv::Scalar(0);
	ST_shape_image_show =  cv::Scalar(0);

	std::vector<std::vector<cv::Point2f> > image_Pixels;
	for(size_t i=0; i<object_cluster.scan_segment_batch.size(); i++)
	{
		std::vector<cv::Point2f> single_scan_pixels;
		for(size_t j=0; j<object_cluster.scan_segment_batch[i].lidarPoints.size(); j++)
		{
			cv::Point2f pixel_tmp;
			spacePt2ImgP(object_cluster.scan_segment_batch[i].lidarPoints[j], pixel_tmp);
			single_scan_pixels.push_back(pixel_tmp);
		}
		image_Pixels.push_back(single_scan_pixels);
	}

	for(size_t i=0; i<image_Pixels.size(); i++)
	{
		for(size_t j=1; j<image_Pixels[i].size(); j++)
		{
			cv::line(ST_shape_image, image_Pixels[i][j-1], image_Pixels[i][j-1], cv::Scalar(255), 2);
		}
	}

	for(size_t i=1; i<image_Pixels.size(); i++)
	{
		if(image_Pixels[i-1].size()>0 && image_Pixels[i].size()>0)
		{
			cv::line(ST_shape_image, image_Pixels[i-1].front(), image_Pixels[i].front(), cv::Scalar(255, 255, 255), 2);
			cv::line(ST_shape_image, image_Pixels[i-1].back(), image_Pixels[i].back(), cv::Scalar(255, 255, 255), 2);
		}
	}

	vector<vector<cv::Point> > contours;
	vector<cv::Vec4i> hierarchy;
	cv::findContours( ST_shape_image, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0) );

	if(contours.size()>0)
	{
		object_cluster.ST_moments = cv::moments( contours.front(), true );
		cv::HuMoments(object_cluster.ST_moments, object_cluster.ST_Humoment);
	}
	else
	{
		for(size_t i=0; i<7; i++) object_cluster.ST_Humoment[i]=0.0;
	}

	//cv::drawContours( ST_shape_image_show, contours, 0, cv::Scalar(0, 255, 255), 2, 8, hierarchy, 0, cv::Point() );
	//cv::imshow("ST_contour_shape", ST_shape_image_show);
	//cv::waitKey(1);
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
		std::vector<double> feature_vector = (this->*get_feature_vector_)(object_cluster_tmp);
		for(size_t k=0; k<feature_vector.size(); k++) fprintf(fp_write, "%lf\t", feature_vector[k]);
		fprintf(fp_write, "\n");
		fclose(fp_write);
	}
}

void DATMO::classify_clusters()
{
	ROS_INFO("classify clusters");
	PointCloudRGB vehicle_cloud, pedestrian_cloud;
	vehicle_cloud.header = combined_pcl_.header;
	pedestrian_cloud.header = combined_pcl_.header;
	vehicle_cloud.points.clear();
	pedestrian_cloud.points.clear();

	for(size_t i=0; i<object_feature_vectors_.size(); i++)
	{
		object_cluster_segments &object_cluster_tmp =object_feature_vectors_[i];
		size_t j=object_cluster_tmp.scan_segment_batch.size();

		assert(object_cluster_tmp.pose_InLatestCoord_vector.size()==object_cluster_tmp.scan_segment_batch.size());

		assert(j==interval_);

		//int vector_length = feature_num_*int(scanNum_perVector_)-3;
		int vector_length = feature_vector_length_;
		double DATMO_feature_vector[vector_length];

		//std::vector<double> feature_vector = get_vector_V3(object_cluster_tmp);
		std::vector<double> feature_vector = (this->*get_feature_vector_)(object_cluster_tmp);

		for(int k=0; k<vector_length; k++)
		{
			DATMO_feature_vector[k]=feature_vector[k];
		}

		/*
		cout<<"monitoring feature vector: ";
		for(int k=0; k<vector_length; k++)
		{
			cout<< k<<"-"<<DATMO_feature_vector[k]<<"\t";
		}
		cout<<endl;
		*/

		object_cluster_tmp.object_type = DATMO_classifier_->classify_objects(DATMO_feature_vector, vector_length);
		if(object_cluster_tmp.object_type ==1)
		{
			ROS_DEBUG("vehicle !!!!!!");

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
		}
		else if(object_cluster_tmp.object_type ==2)
		{
			ROS_DEBUG("pedestrian!!!");
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
					pedestrian_cloud.points.push_back(pt_tmp);
				}
			}
		}
		else if (object_cluster_tmp.object_type ==0)
		{
			ROS_DEBUG("background noise");
		}
		else
		{
			ROS_DEBUG("other type of objects");
		}
	}
	vehicle_pcl_pub_.publish(vehicle_cloud);
	pedestrian_pcl_pub_.publish(pedestrian_cloud);
}

void DATMO::calc_rough_pose()
{
	ROS_DEBUG("calculate cluster pose");
	rough_poses_.header = cloud_vector_.back().header;
	rough_poses_.poses.clear();

	for(size_t i=0; i<object_feature_vectors_.size(); i++)
	{
		object_cluster_segments &object_cluster_tmp =object_feature_vectors_[i];

		if(object_cluster_tmp.object_type!=1)continue;
		//1st, to calculate cluster speed v, and heading direction thetha;

		//a: to calculate the centroids of clusters at different time stamps;
		std::vector<geometry_msgs::Point32> centroid_position;
		geometry_msgs::Point32 total_centroid_vis;
		total_centroid_vis.x = 0.0; total_centroid_vis.y = 0.0; total_centroid_vis.z = 0.0;

		size_t total_point_num=0;
		for(size_t j=0; j<object_cluster_tmp.scan_segment_batch.size(); j++)
		{
			geometry_msgs::Point32 centroid_tmp;
			centroid_tmp.x = 0.0;
			centroid_tmp.y = 0.0;
			centroid_tmp.z = 0.0;
			for(size_t k=0; k<object_cluster_tmp.scan_segment_batch[j].odomPoints.size(); k++)
			{
				centroid_tmp.x = centroid_tmp.x + object_cluster_tmp.scan_segment_batch[j].odomPoints[k].x;
				centroid_tmp.y = centroid_tmp.y + object_cluster_tmp.scan_segment_batch[j].odomPoints[k].y;
				total_centroid_vis.x = total_centroid_vis.x + object_cluster_tmp.scan_segment_batch[j].odomPoints[k].x;
				total_centroid_vis.y = total_centroid_vis.y + object_cluster_tmp.scan_segment_batch[j].odomPoints[k].y;
			}
			centroid_tmp.x = centroid_tmp.x/(float)object_cluster_tmp.scan_segment_batch[j].odomPoints.size();
			centroid_tmp.y = centroid_tmp.y/(float)object_cluster_tmp.scan_segment_batch[j].odomPoints.size();
			centroid_position.push_back(centroid_tmp);

			total_point_num = total_point_num + object_cluster_tmp.scan_segment_batch[j].odomPoints.size();
		}
		total_centroid_vis.x = total_centroid_vis.x/(float)total_point_num;
		total_centroid_vis.y = total_centroid_vis.y/(float)total_point_num;

		//b: visualize the direction as "rough_pose";
		float delt_x = centroid_position.back().x-centroid_position.front().x;
		float delt_y = centroid_position.back().y-centroid_position.front().y;
		float thetha = std::atan2(delt_y, delt_x);
		ROS_DEBUG("centroid_position front: (%f, %f), back:(%f, %f), delt: (%f, %f), thetha: %f", centroid_position.front().x, centroid_position.front().y, centroid_position.back().x, centroid_position.back ().y, delt_x, delt_y, thetha );
		tf::Pose pose_array;
		pose_array.setOrigin( tf::Vector3(total_centroid_vis.x, total_centroid_vis.y, 0.0) );
		pose_array.setRotation( tf::createQuaternionFromYaw(thetha) );
		geometry_msgs::Pose pose_array_msg;
		tf::poseTFToMsg(pose_array, pose_array_msg);
		rough_poses_.poses.push_back(pose_array_msg);
	}

	rough_pose_pub_.publish(rough_poses_);
}

//Here only publish the segmen+pose batches, and will calculate the precise pose in another code;
void DATMO::calc_precise_pose()
{
	MODT::segment_pose_batches extracted_batches;
	extracted_batches.header = cloud_vector_.back().header;
	for(size_t i=0; i<object_feature_vectors_.size(); i++)
	{
		object_cluster_segments &object_cluster_tmp =object_feature_vectors_[i];

		//if(object_cluster_tmp.object_type!=1)continue;

		MODT::segment_pose_batch batch_tmp;
		batch_tmp.object_label = object_cluster_tmp.object_type;

		assert(object_cluster_tmp.scan_segment_batch.size() == object_cluster_tmp.pose_InLatestCoord_vector.size());
		assert(object_cluster_tmp.scan_segment_batch.size() == cloud_vector_.size());
		for(size_t j=0; j<object_cluster_tmp.scan_segment_batch.size(); j++)
		{
			batch_tmp.ego_poses.push_back(object_cluster_tmp.pose_InOdom_vector[j]);
			sensor_msgs::PointCloud cloud_tmp;
			cloud_tmp.header = cloud_vector_[j].header;
			for(size_t k=0;k<object_cluster_tmp.scan_segment_batch[j].odomPoints.size();k++)
			{
				cloud_tmp.points.push_back(object_cluster_tmp.scan_segment_batch[j].odomPoints[k]);
			}
			batch_tmp.segments.push_back(cloud_tmp);
		}
		extracted_batches.clusters.push_back(batch_tmp);
	}
	segment_pose_batch_pub_.publish(extracted_batches);
}

std::vector<double> DATMO::get_vector_V4(object_cluster_segments &object_cluster)
{
	std::vector<double> feature_vector;
	size_t j=object_cluster.scan_segment_batch.size();
	assert(object_cluster.pose_InLatestCoord_vector.size()==object_cluster.scan_segment_batch.size());
	assert(j==interval_);

	//the latest delt pose;
	double pose[6];
	geometry_msgs::Pose pose_tmp;
	if(object_cluster.pose_InLatestCoord_vector.size()>1)
		{
			pose_tmp = object_cluster.pose_InLatestCoord_vector[j-2];
			pose[0]=pose_tmp.position.x;
			pose[1]=pose_tmp.position.y;
			pose[2]=pose_tmp.position.z;
		}
	else {
		pose[0]= 0.0;
		pose[1]= 0.0;
		pose[2]= 0.0;
	}

	//tf::Quaternion q(pose_tmp.orientation.x, pose_tmp.orientation.y, pose_tmp.orientation.z, pose_tmp.orientation.w);
	//tf::Matrix3x3 m(q);
	//m.getRPY(pose[3], pose[4], pose[5]);
	feature_vector.push_back(pose[0]);


	//use latest "roll" and "pitch";
	pose_tmp = object_cluster.pose_InOdom_vector.back();
	tf::Quaternion q(pose_tmp.orientation.x, pose_tmp.orientation.y, pose_tmp.orientation.z, pose_tmp.orientation.w);
	tf::Matrix3x3 m(q);
	m.getRPY(pose[3], pose[4], pose[5]);
	feature_vector.push_back(pose[3]);
	feature_vector.push_back(pose[4]);

	feature_vector.push_back(object_cluster.scan_segment_batch.back().front_dist2background);
	feature_vector.push_back(object_cluster.scan_segment_batch.back().back_dist2background);

	//feature 6-8 about intensities;
	for(size_t i=0; i<3; i++) feature_vector.push_back(object_cluster.scan_segment_batch.back().intensities[i]);

	//feature 9-15 about Hu-moments;
	for(size_t i=0; i<7; i++)
	{
		feature_vector.push_back(object_cluster.ST_Humoment[i]);
	}
	//feature 16;
	feature_vector.push_back(object_cluster.ST_moments.m00);

	/*
	feature_vector.push_back(object_cluster.ST_moments.m01);
	feature_vector.push_back(object_cluster.ST_moments.m10);
	feature_vector.push_back(object_cluster.ST_moments.m20);
	feature_vector.push_back(object_cluster.ST_moments.m11);
	feature_vector.push_back(object_cluster.ST_moments.m02);
	feature_vector.push_back(object_cluster.ST_moments.m30);
	feature_vector.push_back(object_cluster.ST_moments.m21);
	feature_vector.push_back(object_cluster.ST_moments.m03);
	feature_vector.push_back(object_cluster.ST_moments.m12);
	 */

	//remember to reordered the sequence;
	//use "downsample_interval" to reduce the size of training data;
	for(int k=int(j)-1; k>=0; k=k-downsample_interval_)
	{
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

		//the last pair of positions is always 0;
		/*
		if(pose_variant_features_)
		{
			feature_vector.push_back((double)object_cluster.rawpoints_centroids[k].x);
			feature_vector.push_back((double)object_cluster.rawpoints_centroids[k].y);
		}
		*/

		if(!pose_variant_features_)
		{
			if(k!= 0)
			{
				feature_vector.push_back((double)object_cluster.cluster_attached_centroids[k].x);
				feature_vector.push_back((double)object_cluster.cluster_attached_centroids[k].y);
			}
		}
	}

	assert((int)feature_vector.size() == feature_vector_length_);
	return feature_vector;
}

std::vector<double> DATMO::get_vector_3Dfeatures(object_cluster_segments &object_cluster)
{
	std::vector<double> feature_vector;

	//1st: contruct 3D data;
	pcl::PointCloud<pcl::PointXYZ> ST_cluster;
	for(size_t i=0; i<object_cluster.scan_segment_batch.size(); i++)
	{
		double time_delayed = cloud_vector_.back().header.stamp.toSec() - cloud_vector_[i].header.stamp.toSec();
		for(size_t j=0; j<object_cluster.scan_segment_batch[i].lidarPoints.size(); j++)
		{
			pcl::PointXYZ pt_tmp;
			pt_tmp.x = object_cluster.scan_segment_batch[i].lidarPoints[j].x;
			pt_tmp.y = object_cluster.scan_segment_batch[i].lidarPoints[j].y;
			pt_tmp.z = object_cluster.scan_segment_batch[i].lidarPoints[j].z + time_delayed*seg_time_coeff_;
			ST_cluster.push_back(pt_tmp);
		}
	}

	//2nd: extract 3D features;
	feature_vector = (this->*calc_3d_features_)(ST_cluster);

	assert((int)feature_vector.size() == feature_vector_length_);
	return feature_vector;
}

std::vector<double> DATMO::ultrafast_shape_recognition(pcl::PointCloud<pcl::PointXYZ> &st_cloud)
{
	std::vector<double> moments_sets;

	//find 4 keypoints;
	pcl::PointXYZ ctd, cst, fct, ftf;
	ctd.x = 0.0; ctd.y=0.0; ctd.z = 0.0;
	cst.x = 0.0; cst.y=0.0; cst.z = 0.0;
	fct.x = 0.0; fct.y=0.0; fct.z = 0.0;
	ftf.x = 0.0; ftf.y=0.0; ftf.z = 0.0;

	//calculate ctd;
	for(size_t i=0; i<st_cloud.size(); i++) {ctd.x +=st_cloud[i].x; ctd.y +=st_cloud[i].y; ctd.z +=st_cloud[i].z;}
	ctd.x = ctd.x/((float)st_cloud.size()+0.0000001);
	ctd.y = ctd.y/((float)st_cloud.size()+0.0000001);
	ctd.z = ctd.z/((float)st_cloud.size()+0.0000001);

	//calculate cst and fct;
	float cst2ctd = (float)DBL_MAX;
	float fct2ctd = 0.0;
	for(size_t i=0; i<st_cloud.size(); i++)
	{
		float distance_tmp = sqrtf((st_cloud[i].x-ctd.x)*(st_cloud[i].x-ctd.x)+(st_cloud[i].y-ctd.y)*(st_cloud[i].y-ctd.y)+(st_cloud[i].z-ctd.z)*(st_cloud[i].z-ctd.z));
		if(distance_tmp<cst2ctd){ cst2ctd = distance_tmp; cst = st_cloud[i];}
		if(distance_tmp>fct2ctd){ fct2ctd = distance_tmp; fct = st_cloud[i];}
	}

	//calculate ftf;
	float ftf2fct = 0.0;
	for(size_t i=0; i<st_cloud.size(); i++)
	{
		float distance_tmp = sqrtf((st_cloud[i].x-fct.x)*(st_cloud[i].x-fct.x)+(st_cloud[i].y-fct.y)*(st_cloud[i].y-fct.y)+(st_cloud[i].z-fct.z)*(st_cloud[i].z-fct.z));
		if(distance_tmp>ftf2fct){ ftf2fct = distance_tmp; ftf = st_cloud[i];}
	}

	//calculate the 3-Moments according to each point;
	std::vector<double> ctd_moments = moments2refpoint(st_cloud, ctd);
	std::vector<double> cst_moments = moments2refpoint(st_cloud, cst);
	std::vector<double> fct_moments = moments2refpoint(st_cloud, fct);
	std::vector<double> ftf_moments = moments2refpoint(st_cloud, ftf);
	for(size_t i=0; i<3; i++)moments_sets.push_back(ctd_moments[i]);
	for(size_t i=0; i<3; i++)moments_sets.push_back(cst_moments[i]);
	for(size_t i=0; i<3; i++)moments_sets.push_back(fct_moments[i]);
	for(size_t i=0; i<3; i++)moments_sets.push_back(ftf_moments[i]);

	return moments_sets;
}

//in this application only consider first 3 orders of moments
//http://en.wikipedia.org/wiki/Moment_(mathematics)
std::vector<double> DATMO::moments2refpoint(pcl::PointCloud<pcl::PointXYZ> &st_cloud, pcl::PointXYZ &ref_point)
{
	std::vector<double> moments;

	std::vector<float> distance_values;
	float moments1st = 0.0, moments2nd=0.0, moments3rd=0.0;

	for(size_t i=0; i<st_cloud.size(); i++)
	{
		float distance_tmp = sqrtf((st_cloud[i].x-ref_point.x)*(st_cloud[i].x-ref_point.x)+(st_cloud[i].y-ref_point.y)*(st_cloud[i].y-ref_point.y)+(st_cloud[i].z-ref_point.z)*(st_cloud[i].z-ref_point.z));
		distance_values.push_back(distance_tmp);
	}

	for(size_t i=0; i<distance_values.size(); i++)
	{
		moments1st+=distance_values[i];
	}
	moments1st = moments1st/((float)st_cloud.size()+0.0000001);

	for(size_t i=0; i<distance_values.size(); i++)
	{
		moments2nd += (distance_values[i]-moments1st)*(distance_values[i]-moments1st);
	}
	moments2nd = moments2nd/((float)st_cloud.size()*st_cloud.size()+0.0000001);

	for(size_t i=0; i<distance_values.size(); i++)
	{
		moments3rd += std::pow((distance_values[i]-moments1st), 3.0);
	}
	moments3rd = moments3rd/((float)st_cloud.size()*st_cloud.size()*st_cloud.size()+0.0000001);
	moments3rd = moments3rd/(std::pow(moments2nd,1.5)+0.0000001);

	moments.push_back(moments1st);
	moments.push_back(moments2nd);
	moments.push_back(moments3rd);

	return moments;
}

std::vector<double> DATMO::VFH(pcl::PointCloud<pcl::PointXYZ> &st_cloud)
{
	std::vector<double> feature_set;

	// Object for storing the normals.
	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
	// Object for storing the VFH descriptor.
	pcl::PointCloud<pcl::VFHSignature308>::Ptr descriptor(new pcl::PointCloud<pcl::VFHSignature308>);

	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normalEstimation;
	normalEstimation.setInputCloud(st_cloud.makeShared());
	normalEstimation.setRadiusSearch(0.3);
	pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZ>);
	normalEstimation.setSearchMethod(kdtree);
	normalEstimation.compute(*normals);

	pcl::VFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::VFHSignature308> vfh;
	vfh.setInputCloud(st_cloud.makeShared());
	vfh.setInputNormals(normals);
	vfh.setSearchMethod(kdtree);
	// Optionally, we can normalize the bins of the resulting histogram,
	vfh.setNormalizeBins(true);
	// Also, we can normalize the SDC with the maximum size found between
	// the centroid and any of the cluster's points.
	vfh.setNormalizeDistance(false);
	vfh.setViewPoint(0.0,0.0,0.0);
	vfh.compute(*descriptor);

	//descriptor->points.size () should be of size 1;
	for(size_t i=0; i<308; i++)
	{
		feature_set.push_back(descriptor->points.front().histogram[i]);
	}

	return feature_set;
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
	ROS_INFO("scan masks loaded");
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

void DATMO::initialize_local_image()
{
	local_mask_				= cv::Mat((int)(img_side_length_/img_resolution_*2), (int)(img_side_length_/img_resolution_*2), CV_8UC1);
	local_mask_				= cv::Scalar(0);
	LIDAR_pixel_coord_.x 	= (int)(img_side_length_/img_resolution_)-1;
	LIDAR_pixel_coord_.y 	= (int)(img_side_length_/img_resolution_)-1;

	vector<cv::Point2f> img_roi;
	cv::Point2f p0, p1, p2, p3, p4;
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
			cv::Point2f point_tmp;
			point_tmp.x = (float)i;
			point_tmp.y = (float)j;
			if(cv::pointPolygonTest(img_roi, point_tmp, false)>0)local_mask_.at<uchar>(j,i)=255;
		}
}

void DATMO::spacePt2ImgP(geometry_msgs::Point32 & spacePt, cv::Point2f & imgPt)
{
	imgPt.x = LIDAR_pixel_coord_.x - (spacePt.y/img_resolution_);
	imgPt.y = LIDAR_pixel_coord_.y - (spacePt.x/img_resolution_);
}

inline bool DATMO::LocalPixelValid(cv::Point2f & imgPt)
{
	if((int)imgPt.x < local_mask_.cols && (int)imgPt.x >=0 && (int)imgPt.y < local_mask_.rows && (int)imgPt.y >=0) return true;
	else return false;
}

//label the clusters by checking its position in the prior obstacle map;
void DATMO::autolabelling_data()
{
	//try to generate the label "object_feature_vectors_" from labelled scan mask;
	sensor_msgs::PointCloud visualize_pedestrian_cluster;
	visualize_pedestrian_cluster.header = combined_pcl_.header;
	visualize_pedestrian_cluster.header.frame_id = map_frame_id_;
	std::vector<object_cluster_segments> object_feature_vectors_deputy_tmp;
	for(size_t i=0; i<object_feature_vectors_.size(); i++)
	{
		object_cluster_segments &object_cluster_tmp =object_feature_vectors_[i];
		size_t j=object_cluster_tmp.scan_segment_batch.size();
		assert(object_cluster_tmp.pose_InLatestCoord_vector.size()==object_cluster_tmp.scan_segment_batch.size());
		assert(j==interval_);

		sensor_msgs::PointCloud global_pcl;
		global_pcl.header = combined_pcl_.header;
		global_pcl.header.frame_id = odom_frame_id_;
		global_pcl.header.stamp = ros::Time::now();

		for(size_t j=0;j<object_cluster_tmp.scan_segment_batch.size(); j++)
		{
			for(size_t k=0;k<object_cluster_tmp.scan_segment_batch[j].odomPoints.size(); k++)
			{
				geometry_msgs::Point32 pt_tmp;
				pt_tmp.x = object_cluster_tmp.scan_segment_batch[j].odomPoints[k].x;
				pt_tmp.y = object_cluster_tmp.scan_segment_batch[j].odomPoints[k].y;
				pt_tmp.z = object_cluster_tmp.scan_segment_batch[j].odomPoints[k].z;
				global_pcl.points.push_back(pt_tmp);
			}
		}

		try{tf_.transformPointCloud(map_frame_id_, global_pcl, global_pcl);}
		catch (tf::TransformException& e){ROS_WARN("cannot transform into map frame"); std::cout << e.what();return;}

		object_cluster_tmp.object_type = label_cluster_using_map(global_pcl);
		if(object_cluster_tmp.object_type ==-1) continue;

		object_feature_vectors_deputy_tmp.push_back(object_cluster_tmp);
		if(object_cluster_tmp.object_type == tobe_labeled_label_){for(size_t j=0; j<global_pcl.points.size(); j++) visualize_pedestrian_cluster.points.push_back(global_pcl.points[j]);}
	}
	object_feature_vectors_ = object_feature_vectors_deputy_tmp;
	labeled_positive_objects_pub_.publish(visualize_pedestrian_cluster);

	//save derived training data;
	for(size_t i=0; i<object_feature_vectors_.size(); i++)
	{
		object_cluster_segments &object_cluster_tmp =object_feature_vectors_[i];
		size_t j=object_cluster_tmp.scan_segment_batch.size();
		assert(object_cluster_tmp.pose_InLatestCoord_vector.size()==object_cluster_tmp.scan_segment_batch.size());
		FILE *fp_write;
		stringstream file_path;
		file_path<<autolabelled_data_path_<<"/autolabel_"<< j << "_" <<downsample_interval_;
		fp_write = fopen(file_path.str().c_str(),"a");
		if(fp_write==NULL){ROS_ERROR("cannot write auto data file\n");return;}

		fprintf(fp_write, "%d\t", object_cluster_tmp.object_type);
		//std::vector<double> feature_vector = get_vector_V3(object_cluster_tmp);
		std::vector<double> feature_vector = (this->*get_feature_vector_)(object_cluster_tmp);
		for(size_t k=0; k<feature_vector.size(); k++) fprintf(fp_write, "%lf\t", feature_vector[k]);
		fprintf(fp_write, "\n");
		fclose(fp_write);
	}
}

int DATMO::label_cluster_using_map(sensor_msgs::PointCloud &cluster_pcl)
{
	int onroad_number=0, offroad_number=0, unknown_number=0;
	for(size_t i=0; i<cluster_pcl.points.size(); i++)
	{
		if(check_onRoad( cluster_pcl.points[i]) == -1) unknown_number++;
		else if(check_onRoad( cluster_pcl.points[i]) == 1) onroad_number++;
		else offroad_number++;
	}
	ROS_INFO("onroad_number, offroad_number, unknown_number, %d, %d, %d", onroad_number, offroad_number, unknown_number);
	if(unknown_number > unknown_ratio_threshold_*(onroad_number+offroad_number+unknown_number )) return -1;
	else if(onroad_number > free_ratio_threshold_*(onroad_number+offroad_number+unknown_number )) return tobe_labeled_label_;
	else return 0;
}

inline int DATMO::check_onRoad(geometry_msgs::Point32 &point)
{
	cv::Point2i image_point;
	image_point.x =  (floor((point.x) / map_resolution_ + 0.5));
	image_point.y =  (floor((point.y) / map_resolution_ + 0.5));

	int status_flag = -1;
	if(image_point.x < map_prior_.cols && image_point.x >=0 && image_point.y < map_prior_.rows && image_point.y >=0)
	{
		if(map_prior_unknown_.at<uchar>(map_prior_unknown_.rows-1-image_point.y, image_point.x)==255) {status_flag = -1; return status_flag;}
		else if(map_prior_.at<uchar>(map_prior_.rows-1-image_point.y, image_point.x)==255){status_flag = 1; return status_flag;}
		else {status_flag = 0; return status_flag;}
	}
	else return status_flag;
}


int main(int argc, char** argv)
{
	ros::init(argc, argv, "DATMO_node");
	DATMO DATMO_node;
	ros::spin();
	return (0);
}
