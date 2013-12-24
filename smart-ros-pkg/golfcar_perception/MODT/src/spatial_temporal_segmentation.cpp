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
	void construct_feature_vector();

	void classify_clusters();

	void load_labeledScanMasks();
	void visualize_labelled_scan(const sensor_msgs::LaserScan::ConstPtr& scan_in);
	void save_training_data();

	tf::MessageFilter<sensor_msgs::LaserScan>				*tf_filter_;
	size_t 													interval_;
	vector<sensor_msgs::PointCloud>                        	cloud_vector_;
	vector<sensor_msgs::PointCloud>                        	baselink_cloud_vector_;

	vector<sensor_msgs::LaserScan>                        	scan_vector_;
	vector<geometry_msgs::PoseStamped>						laser_pose_vector_;

	geometry_msgs::PoseStamped								laser_pose_current_;
	ros::Publisher                              			collected_cloud_pub_;

	PointCloudRGB 											combined_pcl_;
	ros::Publisher											segmented_pcl_pub_;

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
	private_nh_.param("program_mode",		program_mode_,      1);

	int inverval_tmp;
	private_nh_.param("interval",		    inverval_tmp,       	4);
	interval_ = (size_t) inverval_tmp;

	laser_sub_ = new message_filters::Subscriber<sensor_msgs::LaserScan> (nh_, "/front_bottom_scan", 100);
	tf_filter_ = new tf::MessageFilter<sensor_msgs::LaserScan>(*laser_sub_, tf_, odom_frame_id_, 100);
	tf_filter_->registerCallback(boost::bind(&DATMO::scanCallback, this, _1));
	tf_filter_->setTolerance(ros::Duration(0.1));

	collected_cloud_pub_		=   nh_.advertise<sensor_msgs::PointCloud>("collected_cloud", 2);
	segmented_pcl_pub_			=   nh_.advertise<PointCloudRGB>("segmented_pcl", 2);

    if(program_mode_==0)
    {
		private_nh_.param("abstract_summary_path",       abstract_summary_path_,      std::string("/home/baoxing/data/labelled_data/abstract_summary.yml"));
		labelled_scan_pub_		=   nh_.advertise<sensor_msgs::LaserScan>("labelled_scan", 2);
		private_nh_.param("derived_data_path",       derived_data_path_,      std::string("/home/baoxing/data/derived_data"));
		load_labeledScanMasks();
    }
    else if(program_mode_==1)
	{
	    string DATMO_model_path, DATMO_scale_path;
		private_nh_.param("DATMO_model_path", DATMO_model_path, std::string("/home/baoxing/workspace/data_and_model/DATMO.model"));
		private_nh_.param("DATMO_scale_path", DATMO_scale_path, std::string("/home/baoxing/workspace/data_and_model/DATMO.range"));
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

	if(cloud_vector_.size()==interval_)
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
	graph_segmentation();
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

//parse "object_cluster_IDs_", and construct feature vectors;
void DATMO::construct_feature_vector()
{
	object_feature_vectors_.clear();

	cout<<"object_cluster_IDs number: "<<object_cluster_IDs_.size()<<endl;
	for(size_t i=0; i<object_cluster_IDs_.size(); i++)
	{
		cout<<object_cluster_IDs_[i].second.size()<<"\t";
	}

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
					int ID_in_singlescan = (int)cloud_vector_[k].channels.front().values[ID_in_singlecloud];

					//pay attention here: to record the data in the baselink frame (the frame when the point is recorded);
					geometry_msgs::Point32 baselinkPt = baselink_cloud_vector_[k].points[ID_in_singlecloud];
					cluster_tmp.scan_segment_batch[k].serial_in_scan.push_back(ID_in_singlescan);
					cluster_tmp.scan_segment_batch[k].rawPoints.push_back(baselinkPt);
					cluster_tmp.scan_segment_batch[k].rawIntensities.push_back(scan_vector_[k].intensities[ID_in_singlescan]);
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
			for(size_t k=0; k<object_cluster_tmp.scan_segment_batch[j].serial_in_scan.size(); k++)
			{
				int serial_in_scan_tmp = object_cluster_tmp.scan_segment_batch[j].serial_in_scan[k];
				int type_tmp  = labelled_masks_[((int)scan_vector_[j].header.seq-Lstart_serial_)][serial_in_scan_tmp];
				object_cluster_tmp.object_type = type_tmp;
				if(type_tmp == 4)
				{
					object_cluster_tmp.object_type = 4;
					partdata_notlabelled = true;
					break;
				}
			}
		}
		if(object_cluster_tmp.object_type != 4) object_feature_vectors_deputy_tmp.push_back(object_cluster_tmp);
	}
	object_feature_vectors_ = object_feature_vectors_deputy_tmp;


	//save derived training data;
	for(size_t i=0; i<object_feature_vectors_.size(); i++)
	{
		object_cluster_segments &object_cluster_tmp =object_feature_vectors_[i];
		for(size_t j=0; j<object_cluster_tmp.scan_segment_batch.size(); j++)
		{
			assert(object_cluster_tmp.pose_InLatestCoord_vector.size()==object_cluster_tmp.scan_segment_batch.size());

			FILE *fp_write;
			stringstream file_path;
			file_path<<derived_data_path_<<"/"<<(j+1);
			fp_write = fopen(file_path.str().c_str(),"a");
			if(fp_write==NULL){ROS_ERROR("cannot write derived data file\n");return;}

			fprintf(fp_write, "%d\t", object_cluster_tmp.object_type);

			//pay special attention to the sequence here "f", "k";
			for(int f=0, k=object_cluster_tmp.scan_segment_batch.size() -1; f<=(int)j; f++, k = object_cluster_tmp.scan_segment_batch.size() -1 - f )
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
				for(size_t a=0; a<6; a++)fprintf(fp_write, "%lf\t", pose[a]);

				//then printf the compressed scan segment;
				for(size_t a=0; a<3; a++)
				{
					fprintf(fp_write, "%f\t%f\t%f\t",object_cluster_tmp.scan_segment_batch[k].KeyPoint[a].x,object_cluster_tmp.scan_segment_batch[k].KeyPoint[a].y, object_cluster_tmp.scan_segment_batch[k].intensities[a]);
				}
				fprintf(fp_write, "%d\t%d\t%f\t%f\t", object_cluster_tmp.scan_segment_batch[k].m,object_cluster_tmp.scan_segment_batch[k].n,object_cluster_tmp.scan_segment_batch[k].sigmaM,object_cluster_tmp.scan_segment_batch[k].sigmaN);
			}
			fprintf(fp_write, "\n");
			fclose(fp_write);
		}
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

		int vector_length = 19*int(interval_);
		double DATMO_feature_vector[vector_length];
		int vector_serial=0;

		//remember to reordered the sequence;
		for(int k=int(j-1); k>=0; k--)
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

		cout<<endl;
		for(int k=0; k<vector_length; k++)
		{
			cout<<DATMO_feature_vector[k]<<"\t";
		}
		cout<<endl;

		object_cluster_tmp.object_type = DATMO_classifier_->classify_objects(DATMO_feature_vector, vector_length);
		if(object_cluster_tmp.object_type ==1){ROS_INFO("vehicle!!!");}
		else if(object_cluster_tmp.object_type ==2){ROS_INFO("motorbike!!!");}
		else if (object_cluster_tmp.object_type ==0){ROS_INFO("background noise");}
		else{ROS_INFO("other type of objects");}
	}
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
		for(size_t i=0; i<vis_scan.ranges.size(); i++)
		{
			if(labelled_masks_[((int)scan_in->header.seq-Lstart_serial_)][i]!=1) vis_scan.ranges[i]=0.0;
		}
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
