/*
 * observationCache.cpp
 * Collects all the pointcloud
 *  Created on: Jul 13, 2012
 *      Author: demian
 */

#include <ros/ros.h>

#include <slam_backend/NodeAdded.h>
#include <slam_backend/AddLoopClosure.h>
#include <slam_backend/Graph.h>

#include <sensor_msgs/PointCloud2.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/time_synchronizer.h>

#include <pcl_ros/point_cloud.h>
#include <pcl/ros/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/registration.h>
#include <pcl/filters/voxel_grid.h>

#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

#include "CorrelativeScanMatcher.h"

using namespace std;

struct ObservationData
{
	pcl::PointCloud<pcl::PointXYZ> data;
	geometry_msgs::Pose2D pose;
	int seq;
};

typedef map<int, ObservationData> ObservationWithID;
typedef pair<int, ObservationData> ObservationWithIDPair;
class ObservationCache
{
public:
	ObservationCache();

private:
	void syncCallback(const sensor_msgs::PointCloud2ConstPtr scan, const slam_backend::NodeAddedConstPtr graph);
	void graphCallback(const slam_backend::Graph &graph);
	geometry_msgs::Pose2D getRelativePose2D(geometry_msgs::Pose2D parent_pose, geometry_msgs::Pose2D child_pose);
	void transformPointCloud2D(pcl::PointCloud<pcl::PointXYZ> &pcl, geometry_msgs::Pose2D trans);
	void publishObservations(pcl::PointCloud<pcl::PointXYZ> &pcl_all);
	message_filters::Subscriber<sensor_msgs::PointCloud2> scan_sub_;
	message_filters::Subscriber<slam_backend::NodeAdded> nodeadded_sub_;
	ros::Publisher pc_pub_, matching1_pub_, matching2_pub_, bef_matching1_pub_, bef_matching2_pub_;

	ros::Subscriber graph_sub_;
	int node_seq_;
	ObservationWithID obss_, closeloop_obss_;
	size_t last_graph_size_;
	CorrelativeScanMatcher csm_;
};

ObservationCache::ObservationCache() : last_graph_size_(0)
{
	ros::NodeHandle nh;
	scan_sub_.subscribe(nh, "pc_out", 20);
	nodeadded_sub_.subscribe(nh, "node_added", 20);
	graph_sub_ = nh.subscribe("graph", 20, &ObservationCache::graphCallback, this);
	pc_pub_ = nh.advertise<sensor_msgs::PointCloud2>("graph_pointcloud", 10);
	matching1_pub_ = nh.advertise<sensor_msgs::PointCloud2>("graph_matching1", 10);
	matching2_pub_ = nh.advertise<sensor_msgs::PointCloud2>("graph_matching2", 10);
	bef_matching1_pub_ = nh.advertise<sensor_msgs::PointCloud2>("graph_befmatching1", 10);
	bef_matching2_pub_ = nh.advertise<sensor_msgs::PointCloud2>("graph_befmatching2", 10);

	//exactTime sync is used, this is possible when odometry is in sync with sensor, for example
	//when scan matcher is used. If this is not possible, approximateTime sync can be used
	//however, interpolation might be needed for an accurate transformation between frames
	typedef message_filters::sync_policies::ExactTime<sensor_msgs::PointCloud2, slam_backend::NodeAdded> syncPolicy;
	message_filters::Synchronizer<syncPolicy> sync(syncPolicy(100), scan_sub_, nodeadded_sub_);
	sync.registerCallback(boost::bind(&ObservationCache::syncCallback,this, _1, _2));
	ros::spin();
}

geometry_msgs::Pose2D ObservationCache::getRelativePose2D(geometry_msgs::Pose2D parent_pose, geometry_msgs::Pose2D child_pose)
{
	tf::Transform new_transform, previous_transform, relative_transform;
	tf::Vector3 origin(parent_pose.x, parent_pose.y, 0.0);
	new_transform.setOrigin(origin);
	new_transform.setRotation(tf::createQuaternionFromYaw(parent_pose.theta));
	tf::Vector3 pre_origin(child_pose.x, child_pose.y, 0);
	previous_transform.setOrigin(pre_origin);
	previous_transform.setRotation(tf::createQuaternionFromYaw(child_pose.theta));
	relative_transform.mult(new_transform.inverse(), previous_transform);
	tf::Quaternion orientation = relative_transform.getRotation();
	btQuaternion btq(orientation.x(), orientation.y(), orientation.z(), orientation.w());
	btScalar pitch, roll, yaw;
	btMatrix3x3(btq).getEulerYPR(yaw, pitch, roll);

	geometry_msgs::Pose2D relative_pose;
	relative_pose.x = relative_transform.getOrigin().x();
	relative_pose.y = relative_transform.getOrigin().y();
	relative_pose.theta = yaw;
	return relative_pose;
}

void ObservationCache::transformPointCloud2D(pcl::PointCloud<pcl::PointXYZ> &pcl, geometry_msgs::Pose2D trans)
{
	double ct = cos(trans.theta), st = sin(trans.theta);
	for(size_t i=0; i<pcl.points.size(); i++)
	{
		pcl::PointXYZ p = pcl.points[i];
		pcl::PointXYZ p_t;
		p_t.x = p.x*ct - p.y*st + trans.x;
		p_t.y = p.x*st + p.y*ct + trans.y;
		pcl.points[i] = p_t;
	}
}

void ObservationCache::publishObservations(pcl::PointCloud<pcl::PointXYZ> &pcl_all)
{
	pcl::PointCloud<pcl::PointXYZ> pcl_all_reduced;
	for(size_t i=0; i<pcl_all.points.size(); i++)
	{
		pcl_all.points[i].z = 0.0;
	}
	//downsample since now it is 2D
	cout<<"Before voxel filter: "<<pcl_all.points.size()<<endl;
	pcl::VoxelGrid<pcl::PointXYZ> sor;
	sor.setInputCloud(pcl_all.makeShared());
	sor.setLeafSize (0.2, 0.2, 0.2);
	sor.filter (pcl_all_reduced);
	cout<<"After voxel filter: "<<pcl_all_reduced.points.size()<<endl;
	sensor_msgs::PointCloud2 all_observations;
	pcl::toROSMsg(pcl_all_reduced, all_observations);
	all_observations.header.stamp = ros::Time::now();
	all_observations.header.frame_id = "/map";
	pc_pub_.publish(all_observations);
}

void ObservationCache::graphCallback(const slam_backend::Graph &graph)
{
	fmutil::Stopwatch sw("Reconstruct pointclouds");
	assert(graph.pose.size() == graph.id.size());
	pcl::PointCloud<pcl::PointXYZ> pcl_all;

	//get possible pairs to evaluate for possible loop closure
	cout<<"New total nodes added: "<<graph.id.size()-last_graph_size_<<endl;

	//update all stored observation with the latest position from the graph
	for(size_t i=0; i<graph.id.size(); i++)
	{
		ObservationWithID::iterator it = obss_.find(graph.id[i]);
		if(it == obss_.end()) ROS_WARN("ID %d not found in observation cache", graph.id[i]);
		else it->second.pose = graph.pose[i];
	}
	//only select one from the new added node
	ObservationWithID best_loop_obs;
	geometry_msgs::Pose2D best_loop_pose;
	bool first_passed = false;
	//evaluate new nodes for possible close loop
	for(size_t i=last_graph_size_; i<graph.id.size(); i++)
	{
		ObservationWithID::iterator it = obss_.find(graph.id[i]);
		//cout<<i<<": "<<graph.id[i]<<" ";
		if(it == obss_.end()) ROS_WARN("ID %d not found in observation cache", graph.id[i]);
		else
		{
			//get relative pose between the current node and the nodes before
			//select the nearest one with largest difference in yaw
			//sort of weight sampling


			for(size_t j=0; j<i; j++)
			{
				ObservationWithID::iterator previous_node = obss_.find(graph.id[j]);
				if(previous_node!=obss_.end())
				{
					geometry_msgs::Pose2D rel_pose = getRelativePose2D(graph.pose[j], graph.pose[i]);
					double dist_diff = sqrt(rel_pose.x*rel_pose.x + rel_pose.y*rel_pose.y);
					double yaw_diff = fabs(rel_pose.theta) ;
					bool dist_rule_1 = dist_diff < 10;
					bool dist_rule_2 = dist_diff > 2;
					bool yaw_rule = yaw_diff > (45.0/180*M_PI);
					if(yaw_rule && dist_rule_1)// && dist_rule_2)
					{
						//cout<<"Passes all rules"<<endl;
						if(!first_passed)
						{
							best_loop_obs.insert(ObservationWithIDPair(previous_node->first, previous_node->second));
							best_loop_obs.insert(ObservationWithIDPair(it->first, it->second));
							best_loop_pose = rel_pose;
							//cout<<"First pose: "<<best_loop_pose.x<<" "<<best_loop_pose.y<<" "<<best_loop_pose.theta<<endl;
							first_passed = true;
						}
						else
						{
							double dist_pre = sqrt(best_loop_pose.x*best_loop_pose.x + best_loop_pose.y*best_loop_pose.y);
							double yaw_pre = fabs(best_loop_pose.theta) ;
							//cout<<dist_pre<<" "<<yaw_pre<<" "<<dist_diff<<" "<<dist_pre<<endl;
							if(dist_diff < dist_pre && yaw_diff > yaw_pre)
							{
								best_loop_obs.clear();
								best_loop_obs.insert(ObservationWithIDPair(previous_node->first, previous_node->second));
								best_loop_obs.insert(ObservationWithIDPair(it->first, it->second));
								best_loop_pose = rel_pose;
							}
						}
					}
				}
			}

			/*
			geometry_msgs::Pose2D pose_ = graph.pose[j];
			sensor_msgs::PointCloud2 pc = it->second.data;
			pcl::PointCloud<pcl::PointXYZ> pcl;
			//conversion from pc to pcl took 139ms at the end of the bag
			//may need to speed up, too much data to process
			pcl::fromROSMsg(pc, pcl);
			//transformPointCloud2D(pcl, pose);
			//pcl_all += pcl;
			 * */
		}
	}
	if(best_loop_obs.size()>0)
	{
		cout<<"Posibble loop closure: ";
		assert(best_loop_obs.size()==2);
		vector<pcl::PointCloud<pcl::PointXYZ> > matching;
		int prior_nodeID = best_loop_obs.begin()->first;
		for(ObservationWithID::iterator i= best_loop_obs.begin(); i != best_loop_obs.end(); i++)
		{
			cout<<i->first<<" "<<i->second.data.points.size()<<" ";
			matching.push_back(i->second.data);
		}
		std_msgs::Header header; header.frame_id = "/map";header.stamp = ros::Time::now();
		pcl::PointCloud<pcl::PointXYZ> pcl_prior, pcl_match;
		pcl_prior.header = header;
		pcl_match.header= header;

		cout<<"Before match: "<<best_loop_pose.x<<","<<best_loop_pose.y<<","<<best_loop_pose.theta/M_PI*180<<endl;

		//need more prior for consistent matching, which means relying more on the odometry
		ObservationWithID::iterator prior_node = obss_.find(prior_nodeID);
		assert(prior_node!=obss_.end());
		for(int match_id = prior_nodeID-3; match_id < prior_nodeID+7; match_id++)
		{
			if(match_id == prior_nodeID) continue;
			ObservationWithID::iterator node = obss_.find(match_id);

			if(node!=obss_.end())
			{
				geometry_msgs::Pose2D rel_pose = getRelativePose2D(prior_node->second.pose, node->second.pose);
				pcl::PointCloud<pcl::PointXYZ> prior = node->second.data;
				transformPointCloud2D(prior, rel_pose);
				pcl_prior+=prior;
			}
		}

		pcl_prior += matching[0];
		pcl_prior = PointCloudHelper::compressTo2D(pcl_prior);
		pcl_match = PointCloudHelper::compressTo2D(matching[1]);
		//rotate according to current best pose
		transformPointCloud2D(pcl_match, best_loop_pose);
		sensor_msgs::PointCloud2 pc2_temp;
		pcl::toROSMsg(pcl_prior, pc2_temp);
		bef_matching1_pub_.publish(pc2_temp);
		pcl::toROSMsg(pcl_match, pc2_temp);
		pc2_temp.header = header;
		bef_matching2_pub_.publish(pc2_temp);
		csm_.updatePriorMap(pcl_prior);
		simple_pose after_match_pose;
		after_match_pose = csm_.findBestMatchMultiRes(pcl_match);
		cout<<"After match: "<<after_match_pose.x<<","<<after_match_pose.y<<","<<after_match_pose.t/M_PI*180<<endl;
		best_loop_pose.x = after_match_pose.x;
		best_loop_pose.y = after_match_pose.y;
		best_loop_pose.theta = after_match_pose.t;
		transformPointCloud2D(pcl_match, best_loop_pose);
		pcl::toROSMsg(pcl_prior, pc2_temp);
		matching1_pub_.publish(pc2_temp);
		pcl::toROSMsg(pcl_match, pc2_temp);
		pc2_temp.header = header;
		matching2_pub_.publish(pc2_temp);
	}
	cout<<endl;
	last_graph_size_ = graph.id.size();
	/*
	for(size_t i=0; i<graph.pose.size(); i++)
	{
		ObservationWithID::iterator it = obss_.find(graph.id[i]);
		if(it == obss_.end())
		{
			ROS_WARN("ID %d not found in observation cache", graph.id[i]);
		}
		else
		{
			geometry_msgs::Pose2D pose = graph.pose[i];
			sensor_msgs::PointCloud2 pc = it->second.data;
			pcl::PointCloud<pcl::PointXYZ> pcl;
			//conversion from pc to pcl took 139ms at the end of the bag
			//may need to speed up, too much data to process
			pcl::fromROSMsg(pc, pcl);
			//transformPointCloud2D(pcl, pose);
			//pcl_all += pcl;
		}
	}
	//publishObservations(pcl_all);
*/





	sw.end(true);
}

void ObservationCache::syncCallback(const sensor_msgs::PointCloud2ConstPtr scan, const slam_backend::NodeAddedConstPtr node_added)
{
	//scan is assumed to be in the sensor's body frame
	ObservationData ob;
	ob.seq = node_added->header.seq;
	pcl::PointCloud<pcl::PointXYZ> pt;
	pcl::fromROSMsg(*scan, pt);
	ob.data = pt;
	obss_.insert(ObservationWithIDPair(node_added->node_id, ob));
}



int main(int argc, char** argcv)
{
	ros::init(argc, argcv, "ObservationCache");
	ObservationCache oc;
	return 0;
}

