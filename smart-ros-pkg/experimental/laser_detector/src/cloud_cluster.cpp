/*
 * cloud_cluster.cpp
 *
 *  Created on: Jun 5, 2013
 *      Author: Shen Xiaotng
 */

#include "cloud_cluster.h"
#include <pcl/common/impl/common.hpp>
#include <pcl/kdtree/impl/kdtree_flann.hpp>

#include <time.h>
#include <stdlib.h>
#include <assert.h>

void cloud_cluster::update(void)
{
	num_points = pc.size();
	pcl::getMinMax3D(pc,min_pt,max_pt);

	position[0] = (min_pt.x + max_pt.x)/2;
	position[1] = (min_pt.y + max_pt.y)/2;
	position[2] = (min_pt.z + max_pt.z)/2;

	rect_size[0] = fabs(min_pt.x - max_pt.x)/2;
	rect_size[1] = fabs(min_pt.y - max_pt.y)/2;
	rect_size[2] = fabs(min_pt.z - max_pt.z)/2;

	area = rect_size[0] * rect_size[1];
	height = rect_size[2];
	diag_length = sqrt(rect_size[0] * rect_size[0] + rect_size[1] * rect_size[1]);

}


// get the belief of being a car
void cloud_cluster::analyze(void)
{
	//fixme here just use some lowlevel threshold to classify the pointcloud, better methods may be more helpful
	if(num_points < 20)
	{
		belief = 0;
		return;
	}

	if(height < 0.25)
	{
		belief = 0;
		return;
	}

	if(*std::max_element(rect_size,rect_size+3) < 0.75)
	{
		belief = 0;
		return;
	}

	belief = 1;

}

void cluster_group::analyze(void)
{
	for(unsigned int i = split_index_ ; i < clusters.size() ; i++)
	{
		if(clusters[i].pc.points.size() < 20)
		{
			clusters[i].belief = 0;
			continue;
		}
		else
		{
			clusters[i].analyze();
		}
	}
}



// according to the SAT theorem, only need to check the three axes, since the orientations are the same
bool box::collision_check(const box & another_box)
{
	bool x_collision, y_collision, z_collision, collision;
	if((min_pt.x  >= another_box.min_pt.x && min_pt.x <= another_box.max_pt.x)
	|| (max_pt.x  >= another_box.min_pt.x && max_pt.x <= another_box.max_pt.x)
	|| (another_box.min_pt.x >= min_pt.x && another_box.min_pt.x <= max_pt.x)
	|| (another_box.max_pt.x >= min_pt.x && another_box.max_pt.x <= max_pt.x))
	{
		x_collision = true;
	}
	else
	{
		x_collision = false;
	}


	if((min_pt.y  >= another_box.min_pt.y && min_pt.y <= another_box.max_pt.y)
	|| (max_pt.y  >= another_box.min_pt.y && max_pt.y <= another_box.max_pt.y)
	|| (another_box.min_pt.y >= min_pt.y && another_box.min_pt.y <= max_pt.y)
	|| (another_box.max_pt.y >= min_pt.y && another_box.max_pt.y <= max_pt.y))
	{
		y_collision = true;
	}
	else
	{
		y_collision = false;
	}

	if((min_pt.z  >= another_box.min_pt.z && min_pt.z <= another_box.max_pt.z)
	|| (max_pt.z  >= another_box.min_pt.z && max_pt.z <= another_box.max_pt.z)
	|| (another_box.min_pt.z >= min_pt.z && another_box.min_pt.z <= max_pt.z)
	|| (another_box.max_pt.z >= min_pt.z && another_box.max_pt.z <= max_pt.z))
	{
		z_collision = true;
	}
	else
	{
		z_collision = false;
	}

	if(x_collision && y_collision && z_collision)
	{
		collision = true;
	}
	else
	{
		collision = false;
	}

	return collision;

}

float box::distance(const box & another_box)
{
	// if collision the distance is zero
	if(collision_check(another_box))
	{
		return 0;
	}

	//otherwise try to calc the minimum distance
	float x_dist, y_dist, z_dist, dist;
	if((min_pt.x  >= another_box.min_pt.x && min_pt.x <= another_box.max_pt.x)
	|| (max_pt.x  >= another_box.min_pt.x && max_pt.x <= another_box.max_pt.x)
	|| (another_box.min_pt.x >= min_pt.x && another_box.min_pt.x <= max_pt.x)
	|| (another_box.max_pt.x >= min_pt.x && another_box.max_pt.x <= max_pt.x))
	{
		x_dist = 0;
	}
	else
	{
		x_dist = std::min(fabs(min_pt.x - another_box.max_pt.x),fabs(max_pt.x - another_box.min_pt.x));
	}


	if((min_pt.y  >= another_box.min_pt.y && min_pt.y <= another_box.max_pt.y)
	|| (max_pt.y  >= another_box.min_pt.y && max_pt.y <= another_box.max_pt.y)
	|| (another_box.min_pt.y >= min_pt.y && another_box.min_pt.y <= max_pt.y)
	|| (another_box.max_pt.y >= min_pt.y && another_box.max_pt.y <= max_pt.y))
	{
		y_dist = 0;
	}
	else
	{
		y_dist = std::min(fabs(min_pt.y - another_box.max_pt.y),fabs(max_pt.y - another_box.min_pt.y));
	}

	if((min_pt.z  >= another_box.min_pt.z && min_pt.z <= another_box.max_pt.z)
	|| (max_pt.z  >= another_box.min_pt.z && max_pt.z <= another_box.max_pt.z)
	|| (another_box.min_pt.z >= min_pt.z && another_box.min_pt.z <= max_pt.z)
	|| (another_box.max_pt.z >= min_pt.z && another_box.max_pt.z <= max_pt.z))
	{
		z_dist = 0;
	}
	else
	{
		z_dist = std::min(fabs(min_pt.z - another_box.max_pt.z),fabs(max_pt.z - another_box.min_pt.z));
	}

	dist = sqrt(x_dist*x_dist + y_dist*y_dist + z_dist*z_dist);
	return dist;

}

void box::init(pcl::PointXYZRGB point_A, pcl::PointXYZRGB point_B)
{
	min_pt = point_A;
	max_pt = point_B;
}

int cluster_group::get_cur_id(void)
{
	return cur_group_id;
}

void cluster_group::add_new_group(void)
{
	// fixme : please consider the overflow later
	cur_group_id++;
	last_group_id = cur_group_id - 1;
	delete_old_group();
}

void cluster_group::delete_old_group(void)
{
	//any cluster whose id is smaller than last_group id should be deleted
	//find the cluster who is next to the last group and delete from beginning
	int sz_old = 0;
	for(unsigned int  i = 0; i  < clusters.size(); i++)
	{
		if(clusters[i].group_id < last_group_id)
		{
			sz_old++;
		}
	}

	if(sz_old > 0 )
	{
		clusters.erase(clusters.begin(),clusters.begin() + sz_old);
	}
}


void cluster_group::insert_cluster(cloud_cluster & cluster_in)
{
	// update the cluster information before insert
	cluster_in.update();
	// filtering according to the size, number of points ...
	bool to_be_inserted = true;



	// these filters are just for too big clusters, since we don't know whether the small clusters are part of the car
	if(cluster_in.num_points >= max_number_points )
	{
		to_be_inserted = false;
	}

	if(*std::max_element(cluster_in.rect_size, cluster_in.rect_size + 3) >= max_length)
	{
		to_be_inserted = false;
	}

	if(cluster_in.height >= max_height)
	{
		to_be_inserted = false;
	}

	if(cluster_in.area >= max_area)
	{
		to_be_inserted = false;
	}

	if(cluster_in.diag_length >= max_diag_length)
	{
		to_be_inserted = false;
	}


	if(to_be_inserted)
	{
		clusters.push_back(cluster_in);
	}
}

cluster_group::cluster_group(double _box_dist_thres, double _cluster_dist_thres):max_number_points(5000),max_length(4.0),
		max_area(7.0),max_height(3.0), max_diag_length(6.0), min_height(0.2)
//the max_diag_length may not work because 4*1.414 = 5.6
{
	box_dist_thres = _box_dist_thres;
	cluster_dist_thres = _cluster_dist_thres;
	last_group_id = 0;
	cur_group_id = 0;
	positive_index_ = 1;  // +=2 to make it still odd
	negative_index_ = 2; // +=2 to make it still even
	others_index_ = 0;
}


void cluster_group::label_clusters(ros::Publisher & cloud_pub)
{
	pcl::PCDWriter writer;
	std::stringstream ss;
	int label = -1; // label can only be one or two
	int success = 0;

	pcl::PointXYZRGB xyzRGBpt; // label box is in red
	xyzRGBpt.r = 255;
	xyzRGBpt.g = 0;
	xyzRGBpt.b = 0;

	const unsigned int resolution = 40;
	const float resolution_f = resolution;

	label_box_points.points.clear();
	label_box_points.height =1;


	for(unsigned int i = split_index_ ; i < clusters.size() ; i++)
	{
		// step 1. show the box to be labelled
		// for each cluster, use 12 * 40 points to show the box
		label_box_points.points.clear(); //clear old points
		if(clusters[i].belief < 0.5)
		{
			continue; //only label the cluster with high belief
		}

		label_box_points.header = clusters[i].pc.header;
		for(unsigned int k = 0 ; k < resolution; k++)
		{
			xyzRGBpt.x = (k/resolution_f) * clusters[i].min_pt.x + ((resolution_f - k) / resolution_f) * clusters[i].max_pt.x;
			xyzRGBpt.y = clusters[i].max_pt.y;
			xyzRGBpt.z = clusters[i].max_pt.z;
			label_box_points.push_back(xyzRGBpt);

			xyzRGBpt.y = clusters[i].min_pt.y;
			xyzRGBpt.z = clusters[i].max_pt.z;
			label_box_points.push_back(xyzRGBpt);

			xyzRGBpt.y = clusters[i].max_pt.y;
			xyzRGBpt.z = clusters[i].min_pt.z;
			label_box_points.push_back(xyzRGBpt);

			xyzRGBpt.y = clusters[i].min_pt.y;
			xyzRGBpt.z = clusters[i].min_pt.z;
			label_box_points.push_back(xyzRGBpt);


			xyzRGBpt.y = (k/resolution_f) * clusters[i].min_pt.y + ((resolution_f - k)/resolution_f) * clusters[i].max_pt.y;
			xyzRGBpt.x = clusters[i].max_pt.x;
			xyzRGBpt.z = clusters[i].max_pt.z;
			label_box_points.push_back(xyzRGBpt);


			xyzRGBpt.x = clusters[i].min_pt.x;
			xyzRGBpt.z = clusters[i].max_pt.z;
			label_box_points.push_back(xyzRGBpt);

			xyzRGBpt.x = clusters[i].max_pt.x;
			xyzRGBpt.z = clusters[i].min_pt.z;
			label_box_points.push_back(xyzRGBpt);

			xyzRGBpt.x = clusters[i].min_pt.x;
			xyzRGBpt.z = clusters[i].min_pt.z;
			label_box_points.push_back(xyzRGBpt);

			xyzRGBpt.z = (k/resolution_f)* clusters[i].min_pt.z + ((resolution_f - k)/resolution_f) * clusters[i].max_pt.z;
			xyzRGBpt.x = clusters[i].max_pt.x;
			xyzRGBpt.y = clusters[i].max_pt.y;
			label_box_points.push_back(xyzRGBpt);

			xyzRGBpt.x = clusters[i].min_pt.x;
			xyzRGBpt.y = clusters[i].max_pt.y;
			label_box_points.push_back(xyzRGBpt);

			xyzRGBpt.x = clusters[i].max_pt.x;
			xyzRGBpt.y = clusters[i].min_pt.y;
			label_box_points.push_back(xyzRGBpt);

			xyzRGBpt.x = clusters[i].min_pt.x;
			xyzRGBpt.y = clusters[i].min_pt.y;
			label_box_points.push_back(xyzRGBpt);

		}
		cloud_pub.publish(label_box_points);

		//step 2. wait for the input , 0, negative, 1 ,positive
		//step 3. write it to files
		cout<<"please label the cluster::::";


		while(1)  // quit only when get the correct label
		{
			label = -1; //make sure that it will not quit the loop unless the correct value is specified
			cin.clear();
			cin.ignore(10000,'\n'); // make sure  the input is correct
			if(!(cin>>label))
			{
				continue;
			}

			if(label == 0)
			{
				negative_index_ +=2;
				ss<<"/home/sxt/CAR_PCD/negative/negative_"<<negative_index_<<".pcd";
				success = writer.write<pcl::PointXYZRGB>(ss.str(),clusters[i].pc,false);
				cout<<"NEGATIVE sample is saved, size is "<<clusters[i].pc.size()<<" , success is "<<success<<endl;
				ss.str("");
				ss.clear();
				break;
			}
			else if(label == 1)
			{
				positive_index_ += 2;
				ss<<"/home/sxt/CAR_PCD/positive/positive_"<<positive_index_<<".pcd";
				success = writer.write<pcl::PointXYZRGB>(ss.str(),clusters[i].pc,false);
				cout<<"POSITIVE sample is saved, size is "<<clusters[i].pc.size()<<" , success is "<<success<<endl;
				ss.str("");
				ss.clear();
				break;
			}
			else if(label == 3)
			{
				others_index_ += 1;
				ss<<"/home/sxt/CAR_PCD/others/others_"<<others_index_<<".pcd";
				success = writer.write<pcl::PointXYZRGB>(ss.str(),clusters[i].pc,false);
				cout<<"OTHER sample is saved, size is "<<clusters[i].pc.size()<<" , success is "<<success<<endl;
				ss.str("");
				ss.clear();
				break;
			}
			else
			{
				cout<<"wrong Input! label is "<<label<<endl;
			}
		}
	}
	cout<<"label end!!!!!!!!!!!!!!!"<<endl;
}

// according the minimum distance between the two clusters, if the
// distance is smaller than a predefine distance, then merge them together
// merge : delete them in the old group, put all the points in the new group;
void cluster_group::merge_cluster(void)
{
	// prepare the cloud data here
	//update_cloud();  no need now, because we did not use the the whole cloud

	merge_pairs.clear();

	// merge algorithm here
	// N1 clusters * N2 clusters
	// first use the box to check the distance, if the rough distance is too big,
	// there is no need to double check the clusters distance

	// step 1, find the index that separates the old and current group
	int index = 0;
	for (unsigned int i = 0; i < clusters.size() ;i++)
	{
		if(clusters[i].group_id == last_group_id)
		{
			index++;
		}
		else
		{
			break;
		}
	}

	const unsigned int split_index = index;


	// step 2, two loops try to merge two groups of clusters
	unsigned int cur_index = split_index;
	unsigned int last_index = 0;
	box cur_box, last_box;
	for(last_index = 0;last_index < split_index; last_index++)
	{
		if(clusters[last_index].pc.size() >= max_number_points)
		{
			continue;   // if size of the cloud exceeds certain limit, skip
		}

		if(*std::max_element(clusters[last_index].rect_size, clusters[last_index].rect_size+3) >= max_length)
		{
			continue;   // if the cloud is already very big, skip
		}

		for(cur_index = split_index ;cur_index < clusters.size();cur_index++)
		{
			cur_box.init(clusters[cur_index].min_pt, clusters[cur_index].max_pt);
			last_box.init(clusters[last_index].min_pt,clusters[last_index].max_pt);


			if(cur_box.distance(last_box) > box_dist_thres)
			{
				continue;
			}
			else
			{

				// fixme the cluster_dist_thres value need to be tuned
				// means that the box distance is small enough to check the cluster distance
				// construct a kd tree, if the any two points can be connected through the radius ball search
				// then merge and continue
				pcl::PointCloud<pcl::PointXYZRGB> cloud_in;


				cloud_in = clusters[last_index].pc + clusters[cur_index].pc;
				const  unsigned int last_cluster_size = clusters[last_index].pc.size();
				const  unsigned int cur_cluster_size = clusters[cur_index].pc.size();



 				// the cluster id information is stored in the index, because
				// if the index is smaller than the clusters[last_index].size, then
				// it belongs to the last group

				pcl::KdTreeFLANN<pcl::PointXYZRGB> kd_tree;
				kd_tree.setInputCloud(cloud_in.makeShared());
				std::vector<int> pointIdx;
				std::vector<float> pointDistSquare;
				bool to_be_merged = false;

				// between the current clusters and last clusters, choose the small size one to reduce the computation time
				if(last_cluster_size < cur_cluster_size)
				{
					for(unsigned int ii  = 0; ii < last_cluster_size; ii++)
					{
						if(kd_tree.radiusSearch(clusters[last_index].pc.points[ii],cluster_dist_thres,pointIdx,pointDistSquare)>0)
						{
							if( ((unsigned int) ( *std::max_element(pointIdx.begin(),pointIdx.end()))) >= last_cluster_size )
							{
								to_be_merged = true;
								break;
							}
						}
					}
				}
				else
				{
					for(unsigned int ii  = 0; ii < cur_cluster_size; ii++)
					{
						if(kd_tree.radiusSearch(clusters[cur_index].pc.points[ii],cluster_dist_thres,pointIdx,pointDistSquare)>0)
						{
							if( ((unsigned int) ( *std::min_element(pointIdx.begin(),pointIdx.end()))) < last_cluster_size )
							{
								to_be_merged = true;
								break;
							}
						}
					}
				}




				// save the to be merged index here first
				// do all the merge operation outside of the loop


				//only use box check
				//bool to_be_merged = true;

				if(to_be_merged == true)
				{
					pair<unsigned int,unsigned int> tmp_pair (last_index, cur_index) ;
					merge_pairs.push_back(tmp_pair);

					// fixme assuming that a cluster can only be merged with another cluster, not multiple clusters
					//break;

				}
			}
		}
	}

	// merge operation here
	merge_op(merge_pairs);

	// update merged cluster information
	update_merged_cluster_info();

	// fixme , decide whether or not merge the clusters in a group, maybe no need since euclidean clustering have done this
	//merge_inner_cluster();

	//merge_op(merge_inner_pairs);

	// update the new cloud
	update_cloud();


}


void cluster_group::merge_inner_cluster(void)
{
	merge_inner_pairs.clear();
	// split_index_ can be used here since this function is called after update merged cluster info
	box cur_box, last_box;
	for(unsigned int i = split_index_; i< clusters.size(); i++)
	{
		for(unsigned int j = i + 1; j < clusters.size(); j++)
		{
			cur_box.init(clusters[j].min_pt, clusters[j].max_pt);
			last_box.init(clusters[i].min_pt,clusters[i].max_pt);

			if(cur_box.distance(last_box) > box_dist_thres)
			{
				continue;
			}
			else
			{

				// fixme , the cluster_dist_thres needs to be tuned

				pcl::PointCloud<pcl::PointXYZRGB> cloud_in;
				cloud_in = clusters[i].pc + clusters[j].pc;
				const unsigned int last_cluster_size = clusters[i].pc.points.size();
				const unsigned int cur_cluster_size = clusters[j].pc.points.size();

				pcl::KdTreeFLANN<pcl::PointXYZRGB> kd_tree;
				kd_tree.setInputCloud(cloud_in.makeShared());
				std::vector<int> pointIdx;
				std::vector<float> pointDistSquare;
				bool to_be_merged = false;

				// between the current clusters and last clusters, choose the small size one to reduce the computation time
				if(last_cluster_size < cur_cluster_size)
				{
					for(unsigned int ii  = 0; ii < last_cluster_size; ii++)
					{
						if(kd_tree.radiusSearch(clusters[i].pc.points[ii],cluster_dist_thres,pointIdx,pointDistSquare)>0)
						{
							if( ((unsigned int) ( *std::max_element(pointIdx.begin(),pointIdx.end()))) >= last_cluster_size )
							{
								to_be_merged = true;
								break;
							}
						}
					}
				}
				else
				{
					for(unsigned int ii  = 0; ii < cur_cluster_size; ii++)
					{
						if(kd_tree.radiusSearch(clusters[j].pc.points[ii],cluster_dist_thres,pointIdx,pointDistSquare)>0)
						{
							if( ((unsigned int) ( *std::min_element(pointIdx.begin(),pointIdx.end()))) < last_cluster_size )
							{
								to_be_merged = true;
								break;
							}
						}
					}
				}

				// only use box distance
				//bool to_be_merged = true;
				if(to_be_merged == true)
				{
					pair<unsigned int, unsigned int> tmp_pair (i,j);
					merge_inner_pairs.push_back(tmp_pair);

					// fixme assuming that a cluster can only be merged with another cluster, not multiple clusters
					//break;
				}
			}
		}
	}
}

void cluster_group::update_merged_cluster_info(void)
{
	// step 1, find the split index for the new group
	unsigned int split_index = 0; // the start index of the new group
	for(unsigned int i = 0; i < clusters.size(); i++)
	{
		if(clusters[i].group_id == last_group_id)
		{
			split_index++;
		}
		else
		{
			break;
		}
	}
	split_index_ = split_index;

	// step 2, update the merge result
	for(unsigned int j = split_index; j < clusters.size(); j++)
	{
		if(clusters[j].merged_flag == true)
		{
			clusters[j].update();
		}
	}
}

void cluster_group::prepare_merge_result(void)
{
	pcl::PointXYZRGB xyzRGBpt;
	cur_vis_points.points.clear();

	for(unsigned int ii = 0; ii < clusters.size() ; ii++)
	{
		if(ii < split_index_)
		{
			assert(clusters[ii].group_id == last_group_id);
		}
		else
		{
			assert(clusters[ii].group_id == cur_group_id);
		}
	}

	srand(time(NULL));

	for(unsigned int i = split_index_; i < clusters.size() ; i++)
	{
		assert(clusters[i].group_id == cur_group_id);
		cur_vis_points.header = clusters[i].pc.header;
		cur_vis_points.height = 1;
		size_t sz = cur_vis_points.points.size();
		cur_vis_points.points.resize(sz + clusters[i].pc.points.size());

		// set the color according to the merged flag, merged in red, others in other colors without any red
		if(clusters[i].merged_flag == true)
		{
			xyzRGBpt.r = 255;
			xyzRGBpt.g = 0;
			xyzRGBpt.b = 0;
		}
		else
		{
			xyzRGBpt.r = 0;
			xyzRGBpt.g = rand()%255;
			xyzRGBpt.b = rand()%255;
		}

		//copy points
		for(unsigned int j = 0; j < clusters[i].pc.points.size(); j++)
		{
			xyzRGBpt.x = clusters[i].pc.points[j].x;
			xyzRGBpt.y = clusters[i].pc.points[j].y;
			xyzRGBpt.z = clusters[i].pc.points[j].z;
			cur_vis_points.points[j + sz] = xyzRGBpt;
		}


	}
	// because we are not using push back, remember to update the width
	cur_vis_points.width = cur_vis_points.points.size();
	cur_vis_points.height = 1;

}


void cluster_group::show_cluster_box(void)
{
	cur_vis_points.height = 1;
	cur_vis_points.clear();
	pcl::PointXYZRGB xyzRGBpt;

	srand(time(NULL));
	cout<<"--------------------------"<<endl;
	for(unsigned int i = 0; i < clusters.size(); i++)
	{
		cur_vis_points.header = clusters[i].pc.header;
		if(clusters[i].group_id == last_group_id)
		{
		   // no red, all blue, random green, the cold color for the last group
		   xyzRGBpt.r = 0;
		   xyzRGBpt.g = rand()%255;
		   xyzRGBpt.b = 255;
		}
		else
		{
			// all red, no blue, random green, the hot color for the current group
			xyzRGBpt.b = 0;
			xyzRGBpt.g = rand()%255;
			xyzRGBpt.r = 255;
		}
		// debug, show the size of each cluster
		cout<<" the "<<i<< "-th cluster"<<" dx "<<clusters[i].rect_size[0] << " dy "<< clusters[i].rect_size[1] << " dz "<<clusters[i].rect_size[2]<<endl;

		const unsigned int resolution = 40;
		const float resolution_f = resolution;
			// for each cluster, use 12 * 10 points to show the box
			for(unsigned int k = 0 ; k < resolution; k++)
			{
				xyzRGBpt.x = (k/resolution_f) * clusters[i].min_pt.x + ((resolution_f - k) / resolution_f) * clusters[i].max_pt.x;
				xyzRGBpt.y = clusters[i].max_pt.y;
				xyzRGBpt.z = clusters[i].max_pt.z;
				cur_vis_points.push_back(xyzRGBpt);

				xyzRGBpt.y = clusters[i].min_pt.y;
				xyzRGBpt.z = clusters[i].max_pt.z;
				cur_vis_points.push_back(xyzRGBpt);

				xyzRGBpt.y = clusters[i].max_pt.y;
				xyzRGBpt.z = clusters[i].min_pt.z;
				cur_vis_points.push_back(xyzRGBpt);

				xyzRGBpt.y = clusters[i].min_pt.y;
				xyzRGBpt.z = clusters[i].min_pt.z;
				cur_vis_points.push_back(xyzRGBpt);


				xyzRGBpt.y = (k/resolution_f) * clusters[i].min_pt.y + ((resolution_f - k)/resolution_f) * clusters[i].max_pt.y;
				xyzRGBpt.x = clusters[i].max_pt.x;
				xyzRGBpt.z = clusters[i].max_pt.z;
				cur_vis_points.push_back(xyzRGBpt);


				xyzRGBpt.x = clusters[i].min_pt.x;
				xyzRGBpt.z = clusters[i].max_pt.z;
				cur_vis_points.push_back(xyzRGBpt);

				xyzRGBpt.x = clusters[i].max_pt.x;
				xyzRGBpt.z = clusters[i].min_pt.z;
				cur_vis_points.push_back(xyzRGBpt);

				xyzRGBpt.x = clusters[i].min_pt.x;
				xyzRGBpt.z = clusters[i].min_pt.z;
				cur_vis_points.push_back(xyzRGBpt);

				xyzRGBpt.z = (k/resolution_f)* clusters[i].min_pt.z + ((resolution_f - k)/resolution_f) * clusters[i].max_pt.z;
				xyzRGBpt.x = clusters[i].max_pt.x;
				xyzRGBpt.y = clusters[i].max_pt.y;
				cur_vis_points.push_back(xyzRGBpt);

				xyzRGBpt.x = clusters[i].min_pt.x;
				xyzRGBpt.y = clusters[i].max_pt.y;
				cur_vis_points.push_back(xyzRGBpt);

				xyzRGBpt.x = clusters[i].max_pt.x;
				xyzRGBpt.y = clusters[i].min_pt.y;
				cur_vis_points.push_back(xyzRGBpt);

				xyzRGBpt.x = clusters[i].min_pt.x;
				xyzRGBpt.y = clusters[i].min_pt.y;
				cur_vis_points.push_back(xyzRGBpt);

			}

	}

}


void cluster_group::show_car_box(void)
{
	cur_vis_points.height = 1;
	cur_vis_points.clear();
	pcl::PointXYZRGB xyzRGBpt;

	srand(time(NULL));
	cout<<"--------------------------"<<endl;
	for(unsigned int i = split_index_; i < clusters.size(); i++)
	{
		if(clusters[i].belief < 0.5)
		{
			continue; // only show the condidate cars, if the belief is too low, then discard it
		}

		cur_vis_points.header = clusters[i].pc.header;

	   // cars are in green color
	   xyzRGBpt.r = 0;
	   xyzRGBpt.g = 255;
	   xyzRGBpt.b = 0;

		// debug, show the size of each cluster
		//cout<<" the "<<i<< "-th cluster"<<" dx "<<clusters[i].rect_size[0] << " dy "<< clusters[i].rect_size[1] << " dz "<<clusters[i].rect_size[2]<<endl;

		const unsigned int resolution = 40;
		const float resolution_f = resolution;
			// for each cluster, use 12 * 10 points to show the box
			for(unsigned int k = 0 ; k < resolution; k++)
			{
				xyzRGBpt.x = (k/resolution_f) * clusters[i].min_pt.x + ((resolution_f - k) / resolution_f) * clusters[i].max_pt.x;
				xyzRGBpt.y = clusters[i].max_pt.y;
				xyzRGBpt.z = clusters[i].max_pt.z;
				cur_vis_points.push_back(xyzRGBpt);

				xyzRGBpt.y = clusters[i].min_pt.y;
				xyzRGBpt.z = clusters[i].max_pt.z;
				cur_vis_points.push_back(xyzRGBpt);

				xyzRGBpt.y = clusters[i].max_pt.y;
				xyzRGBpt.z = clusters[i].min_pt.z;
				cur_vis_points.push_back(xyzRGBpt);

				xyzRGBpt.y = clusters[i].min_pt.y;
				xyzRGBpt.z = clusters[i].min_pt.z;
				cur_vis_points.push_back(xyzRGBpt);


				xyzRGBpt.y = (k/resolution_f) * clusters[i].min_pt.y + ((resolution_f - k)/resolution_f) * clusters[i].max_pt.y;
				xyzRGBpt.x = clusters[i].max_pt.x;
				xyzRGBpt.z = clusters[i].max_pt.z;
				cur_vis_points.push_back(xyzRGBpt);


				xyzRGBpt.x = clusters[i].min_pt.x;
				xyzRGBpt.z = clusters[i].max_pt.z;
				cur_vis_points.push_back(xyzRGBpt);

				xyzRGBpt.x = clusters[i].max_pt.x;
				xyzRGBpt.z = clusters[i].min_pt.z;
				cur_vis_points.push_back(xyzRGBpt);

				xyzRGBpt.x = clusters[i].min_pt.x;
				xyzRGBpt.z = clusters[i].min_pt.z;
				cur_vis_points.push_back(xyzRGBpt);

				xyzRGBpt.z = (k/resolution_f)* clusters[i].min_pt.z + ((resolution_f - k)/resolution_f) * clusters[i].max_pt.z;
				xyzRGBpt.x = clusters[i].max_pt.x;
				xyzRGBpt.y = clusters[i].max_pt.y;
				cur_vis_points.push_back(xyzRGBpt);

				xyzRGBpt.x = clusters[i].min_pt.x;
				xyzRGBpt.y = clusters[i].max_pt.y;
				cur_vis_points.push_back(xyzRGBpt);

				xyzRGBpt.x = clusters[i].max_pt.x;
				xyzRGBpt.y = clusters[i].min_pt.y;
				cur_vis_points.push_back(xyzRGBpt);

				xyzRGBpt.x = clusters[i].min_pt.x;
				xyzRGBpt.y = clusters[i].min_pt.y;
				cur_vis_points.push_back(xyzRGBpt);

			}

	}

}


void cluster_group::merge_op(vector< pair<unsigned int,unsigned int> > &  _merge_pairs)
{
	// do the merge operation here
	// step 1 : copy the points from last group to the current group, also update the merged_flag for each cluster
	cout<< " there are -----!!!!!" << _merge_pairs.size() << " pairs to be merged!" << endl;
	cout<< " there are !!!!!------ " << clusters.size() << "clusters ----" <<endl;
	for(unsigned int i = 0; i < _merge_pairs.size(); i++)
	{
		unsigned int last_index = _merge_pairs[i].first;
		unsigned int cur_index = _merge_pairs[i].second;
		unsigned int cur_size = clusters[cur_index].pc.points.size();
		unsigned int last_size = clusters[last_index].pc.points.size();
		// copy the points here
		clusters[_merge_pairs[i].second].pc.points.resize(cur_size + last_size);
		for(unsigned int j = 0; j < last_size; j++)
		{
			clusters[cur_index].pc.points[cur_size + j] = clusters[last_index].pc.points[j];
		}
		// update the merged flag here
		clusters[cur_index].merged_flag = true;
		clusters[cur_index].pc.width = clusters[cur_index].pc.points.size();
	}
	// step 2 : delete the old clusters that, just delete the points of the point cloud
	for(unsigned int i = 0; i < _merge_pairs.size();i++)
	{
		clusters[_merge_pairs[i].first].pc.points.clear();
	}
}

//because the cloud only simply check the id, maybe after the merging
//operation, we need to update the cloud again
void cluster_group::update_cloud(void)
{
	ClusterPoint cluster_point;
	cur_group_points.points.clear();
	last_group_points.points.clear();
	cur_group_points.height = 1;
	last_group_points.height = 1;
	bool cur_group_header_flag  = false;
	bool last_group_header_flag  = false; // just use to update the header info

	for(unsigned int i = 0; i < clusters.size(); i++)
	{
		cluster_point.cluster_id = clusters[i].cluster_id;
		cluster_point.group_id = clusters[i].group_id;

		//update the headers, the time may be not correct
		if(cur_group_header_flag == false)
		{
			if(clusters[i].group_id == cur_group_id)
			{
				cur_group_points.header = clusters[i].pc.header;
			}
		}

		//update the headers, the time may be not correct
		if(last_group_header_flag == false)
		{
			if(clusters[i].group_id == last_group_id)
			{
				last_group_points.header = clusters[i].pc.header;
			}
		}


		// simply copy the points
		if(clusters[i].group_id == cur_group_id)
		{
			unsigned int cur_size= cur_group_points.points.size();
			unsigned int added_size = clusters[i].pc.size();
			cur_group_points.points.resize(cur_size  +  added_size);
			for(unsigned int j = 0; j < added_size; j++)
			{
				cluster_point.x = clusters[i].pc.points[j].x;
				cluster_point.y = clusters[i].pc.points[j].y;
				cluster_point.z = clusters[i].pc.points[j].z;
				cur_group_points.points[j + cur_size] = cluster_point;

				/*
				cur_group_points.points[j + cur_size].x = clusters[i].pc.points[j].x;
				cur_group_points.points[j + cur_size].y = clusters[i].pc.points[j].y;
				cur_group_points.points[j + cur_size].z = clusters[i].pc.points[j].z;
				*/

			}
		}
		else if(clusters[i].group_id == last_group_id)
		{
			unsigned int cur_size = last_group_points.size();
			unsigned int added_size = clusters[i].pc.size();
			last_group_points.points.resize(cur_size + added_size);
			for(unsigned int j = 0; j < added_size; j++)
			{
				cluster_point.x = clusters[i].pc.points[j].x;
				cluster_point.y = clusters[i].pc.points[j].y;
				cluster_point.z = clusters[i].pc.points[j].z;
				last_group_points.points[j + cur_size] = cluster_point;
				/*
				last_group_points.points[j + cur_size].x = clusters[i].pc.points[j].x;
				last_group_points.points[j + cur_size].y = clusters[i].pc.points[j].y;
				last_group_points.points[j + cur_size].z = clusters[i].pc.points[j].z;
				*/
			}
		}
		else
		{
			continue; // just in case
		}
	}

	cur_group_points.width = cur_group_points.points.size();
	last_group_points.width = last_group_points.points.size();
}
