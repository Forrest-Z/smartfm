/*
 * cloud_cluster.h
 *
 *  Created on: Jun 5, 2013
 *      Author: Shen Xiaotong
 */

#ifndef CLOUD_CLUSTER_H_
#define CLOUD_CLUSTER_H_

#include <pcl/point_cloud.h>
#include <pcl/pcl_base.h>
#include "cluster_data_types.h"

#include <deque>
#include <utility>
#include <vector>
#include <algorithm>
#include <cmath>

using namespace std;

// features for cloud cluster, new features can be added later
class cloud_cluster{
public:
	int cluster_id;
	int group_id;
	int num_points;
	float rect_size[3]; //dx dy dz
	float position[3];  // x y z
	float area;
	float height;
	float diag_length;
	bool merged_flag;
	bool processed_flag;

	float belief;  // 0 - 1, the belief of being a car

	pcl::PointXYZRGB min_pt;
	pcl::PointXYZRGB max_pt;

	//TODO: find more features for classification

	pcl::PointCloud<pcl::PointXYZRGB> pc;
	void update(void);


};

// define a class for checking the minimum distance between two boxes
class box
{

	pcl::PointXYZRGB min_pt;
	pcl::PointXYZRGB max_pt;
public:
	box(pcl::PointXYZRGB point_A, pcl::PointXYZRGB point_B):min_pt(point_A),max_pt(point_B)
	{

	}
	box(void)
	{

	}
	void init(pcl::PointXYZRGB point_A, pcl::PointXYZRGB point_B);

	// calc the minimum distance between two boxes with the same orientation.
	float distance(const box & another_box);
	bool collision_check(const box & another_box);


};

class cluster_group{

private:
	int cur_group_id;
	int last_group_id;

	double box_dist_thres;
	double cluster_dist_thres;
	unsigned int split_index_; // the start index of the new group clusters, usable after calling update merged cluster info

	const unsigned int max_number_points; // limit the number of points in a cluster
	const float max_length;				  // limit the length of of each dimension
	const float max_area;		          // limit the area of the rect
	const float min_height;               // the minimum height of the box
	const float max_height;				  // the maximum height of the box
	const float max_diag_length;          // the maximum diag length


	vector< pair<unsigned int,unsigned int> > merge_pairs; // indexes are stored here, last_index - cur_index
// erase the clusters in the older group
	void delete_old_group(void);
// after finishing inserting all the points, should update the cloud, simply add the points together according the group
	void update_cloud(void);
// copy the points from one cluster to another cluster and upate the merged flag
	void merge_op(void);
// update the features for each cluster after merge operation for the new group (only for merged ones)
	void update_merged_cluster_info(void);




public:
// in the queue, always keep old cluster before new cluster:  begin->end: old->new
	std::deque<cloud_cluster> clusters;



	pcl::PointCloud<pcl::PointXYZRGB> cur_vis_points;

	pcl::PointCloud<ClusterPoint> cur_group_points;
	pcl::PointCloud<ClusterPoint> last_group_points;

	cluster_group(double _box_dist_thres,double _cluster_dist_thres);


// get the current id for updating the cloud cluster's group id;
	int get_cur_id(void);

// just update the id information, no cluster will be added, old clusters will be deleted, invoke the delete old group function
	void add_new_group(void);

// insert a new cluster to the group
	void insert_cluster(cloud_cluster & cluster_in);


// after get two groups of clusters, merge the clusters and only need to analyze the new group
	void merge_cluster(void);



// for the current group, prepare the points in the cur_vis_points and show the result
	void prepare_merge_result(void);

// a big point cloud to show all the boxes
	void show_cluster_box(void);






};

#endif /* CLOUD_CLUSTER_H_ */
