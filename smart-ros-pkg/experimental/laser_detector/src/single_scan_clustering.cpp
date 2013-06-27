/*
 * single_scan_clustering.cpp
 *
 *  Created on: Jun 5, 2013
 *      Author: Shen Xiaotong
 */



#include "single_scan_clustering.h"


// update the scan from filtered point cloud, assume that the point cloud has index and distance channels
void golfcar_perception::laser_scan_cloud::update(const sensor_msgs::LaserScan raw_scan)
{
	laser_geometry::LaserProjection temp_laser_projection;
	temp_laser_projection.projectLaser(raw_scan,pc_,200,laser_geometry::channel_option::Index | laser_geometry::channel_option::Distance);
	scan_.angles.resize(pc_.points.size());
	scan_.dists.resize(pc_.points.size());
	for (size_t i = 0; i < pc_.points.size() ; i++)
	{
		scan_.angles[i] = raw_scan.angle_min + raw_scan.angle_increment * pc_.channels[0].values[i];
		scan_.dists[i] = pc_.channels[1].values[i];
	}
}

void golfcar_perception::laser_scan_cluster::compute_minmax(void)
{
	min_index_ = * std::min_element(index_.begin(),index_.end());
	max_index_ = * std::max_element(index_.begin(),index_.end());

	max_distance_ = * std::max_element(distance_.begin(),distance_.end());
	min_distance_ = * std::min_element(distance_.begin(),distance_.end());

}

template <class PointT>
inline double golfcar_perception::single_scan_clustering::compute_dist(PointT a, PointT b)
{
    return sqrt((a.x - b.x) * (a.x - b.x) + (a.y - b.y)*(a.y - b.y) + (a.z - b.z)*(a.z - b.z));
}

golfcar_perception::single_scan_clustering::single_scan_clustering(void):nh_("")
{
	laser_sub_= nh_.subscribe("/sickldmrs/scan3",5,&single_scan_clustering::scan_callback,this);
	clusters_pub_ = nh_.advertise<sensor_msgs::PointCloud>("clusters_pts",10);

	line_fitting_inlier_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("line_inliers",10);
	line_fitting_outlier_pub_ = nh_.advertise<sensor_msgs::PointCloud2 >("line_outliers",10);
	corners_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("line_corners",10);
}

void golfcar_perception::single_scan_clustering::sac_line_fitting(sensor_msgs::LaserScan::ConstPtr laser_in)
{
	const int num_lines = 8;
	// convert the laser scan to a pcl point cloud
	laser_geometry::LaserProjection laser_proj;
	sensor_msgs::PointCloud pc;
	laser_proj.projectLaser(*laser_in,pc,200,laser_geometry::channel_option::Index | laser_geometry::channel_option::Distance); // keep track of index to find corners
	sensor_msgs::PointCloud2 pc2;
	sensor_msgs::convertPointCloudToPointCloud2(pc,pc2);
	pcl::PointCloud<pcl::PointXYZ> laser_pcl;
	pcl::fromROSMsg(pc2,laser_pcl);
	//copy the index and distance and do the same operation as the points: x - > index ,y -> distance
	pcl::PointCloud<pcl::PointXYZ> index_dist_cloud;
	index_dist_cloud.points.resize(pc.points.size());
	for(unsigned int i = 0 ; i < pc.points.size() ; i++)
	{
		index_dist_cloud.points[i].x = pc.channels[0].values[i];
		index_dist_cloud.points[i].y = pc.channels[1].values[i];
	}

	//record the clustering result and find the corners
	std::vector<laser_scan_cluster> line_clusters;

	// line fitting

	pcl::PointCloud<pcl::PointXYZ> laser_outliers_pcl;
	pcl::PointCloud<pcl::PointXYZI> laser_inliers_pcl;
	pcl::SACSegmentation<pcl::PointXYZ> line_seg;
	line_seg.setOptimizeCoefficients(true);
	line_seg.setModelType(pcl::SACMODEL_LINE);
	line_seg.setMaxIterations(500);
	line_seg.setDistanceThreshold(0.2);

	pcl::ModelCoefficients::Ptr line_coef(new pcl::ModelCoefficients);
	pcl::PointIndices::Ptr line_inliers_ind (new pcl::PointIndices);

	pcl::ExtractIndices<pcl::PointXYZ> extract;
	pcl::ExtractIndices<pcl::PointXYZ> extract_info;

	laser_outliers_pcl = laser_pcl;
	laser_inliers_pcl.header = laser_pcl.header;

	pcl::PointCloud<pcl::PointXYZ> pcl_container;
	pcl::PointCloud<pcl::PointXYZ> index_dist_container;



	srand(time(NULL));
    int line_id = 0;
	for(int j = 0; j< num_lines; j++)
	{
		//make sure the operations are the same and there are equal numbers of points in the clouds
		assert(index_dist_cloud.points.size() == laser_outliers_pcl.points.size());
		//cout<<" 1 size is "<<index_dist_cloud.points.size()<<" "<< laser_outliers_pcl.points.size();

		line_seg.setInputCloud(laser_outliers_pcl.makeShared());
		line_seg.segment(*line_inliers_ind,*line_coef);
		extract.setNegative(false);
		extract_info.setNegative(false);
		extract.setInputCloud(laser_outliers_pcl.makeShared());
		extract_info.setInputCloud(index_dist_cloud.makeShared());
		extract.setIndices(line_inliers_ind);
		extract_info.setIndices(line_inliers_ind);
		line_id += 10;
		extract.filter(pcl_container);
		extract_info.filter(index_dist_container);

		pcl::PointXYZI point_xyzi;
		laser_scan_cluster line_cluster;
		line_cluster.laser_cloud_ = pcl_container;
		assert(index_dist_container.points.size() == pcl_container.points.size());
		//cout<<" 2 size is "<<index_dist_container.points.size()<<" "<< pcl_container.points.size();
		if(pcl_container.points.size() > 4 && pcl_container.points.size() < 20) // at least 8 points in the line, otherwise discard
		{
			//copy the points to the inliers and set different line id for visualization
			for(unsigned int k = 0; k < pcl_container.points.size(); k++)
			{
				point_xyzi.x = pcl_container.points[k].x;
				point_xyzi.y = pcl_container.points[k].y;
				point_xyzi.z = pcl_container.points[k].z;
				point_xyzi.intensity = line_id;
				laser_inliers_pcl.points.push_back(point_xyzi);

				//copy the index and distance here
				line_cluster.distance_.push_back(index_dist_container.points[k].y);
				line_cluster.index_.push_back(index_dist_container.points[k].x);
				//copy the coefficients
				cout<<line_coef->values.size()<<" coefficients" <<endl;
			}

			// make sure that line cluster contains some data, otherwise there is segmentation fault error
			line_cluster.compute_minmax();
			line_clusters.push_back(line_cluster);
		}

		//get the left outliers to do line fitting again
		extract.setNegative(true);
		extract.filter(laser_outliers_pcl);
		extract_info.setNegative(true);
		extract_info.filter(index_dist_cloud);
		//std::cout<<"line is 148"<<std::endl;

		if(laser_outliers_pcl.size() < 8)
		{
			break;
		}


	}
	//std::cout<<"line is 151"<<std::endl;
	sensor_msgs::PointCloud2 outliers_pc2, inliers_pc2;
	pcl::toROSMsg(laser_inliers_pcl,inliers_pc2);
	pcl::toROSMsg(laser_outliers_pcl,outliers_pc2);
	line_fitting_outlier_pub_.publish(outliers_pc2);
	line_fitting_inlier_pub_.publish(inliers_pc2);

	pcl::PointCloud<pcl::PointXYZI> corner_filtered_pcl;
	pcl::PointXYZI temp_point_xyzi;
	int corner_id = 0;
#if 1
	// find the corners in the line clusters and combine them to a new point cloud
	for(unsigned int i = 0; i < line_clusters.size();i++)
		for(unsigned int j= 0; j < line_clusters.size(); j++)
		{
			if(j == i) continue;
			if(compute_dist<geometry_msgs::Point32>(pc.points[line_clusters[i].min_index_] , pc.points[line_clusters[i].max_index_]) > 3.0)
			{
				continue;
			}
			if(compute_dist<geometry_msgs::Point32>(pc.points[line_clusters[j].min_index_] , pc.points[line_clusters[j].max_index_]) > 3.0)
			{
				continue;
			}

			if(max(compute_dist<geometry_msgs::Point32>(pc.points[line_clusters[i].min_index_] , pc.points[line_clusters[j].max_index_]),
					compute_dist<geometry_msgs::Point32>(pc.points[line_clusters[j].min_index_] , pc.points[line_clusters[i].max_index_])) > 4.2)
			{
				continue;
			}
			corner_id += 10;
			//compute_dist<geometry_msgs::Point32>(pc.points[line_clusters[i].min_index_] , pc.points[line_clusters[j].max_index_]);

			// proximity criterion
			if((fabs(line_clusters[i].min_index_ - line_clusters[j].max_index_) <= 6  &&
					compute_dist<geometry_msgs::Point32>(pc.points[line_clusters[i].min_index_] , pc.points[line_clusters[j].max_index_]) < 0.3)
				|| (fabs(line_clusters[j].min_index_ - line_clusters[i].max_index_) <= 6  &&
						compute_dist<geometry_msgs::Point32>(pc.points[line_clusters[j].min_index_] , pc.points[line_clusters[i].max_index_]) < 0.3))
					{
						for(unsigned int k = 0; k < line_clusters[i].laser_cloud_.points.size(); k++)
						{
							temp_point_xyzi.x = line_clusters[i].laser_cloud_.points[k].x;
							temp_point_xyzi.y = line_clusters[i].laser_cloud_.points[k].y;
							temp_point_xyzi.z = line_clusters[i].laser_cloud_.points[k].z;
							temp_point_xyzi.intensity = corner_id;
							corner_filtered_pcl.points.push_back(temp_point_xyzi);
						}

						for(unsigned int k = 0; k < line_clusters[j].laser_cloud_.points.size(); k++)
						{
							temp_point_xyzi.x = line_clusters[j].laser_cloud_.points[k].x;
							temp_point_xyzi.y = line_clusters[j].laser_cloud_.points[k].y;
							temp_point_xyzi.z = line_clusters[j].laser_cloud_.points[k].z;
							temp_point_xyzi.intensity = corner_id;
							corner_filtered_pcl.points.push_back(temp_point_xyzi);

						}

					}

		}
#endif
	corner_filtered_pcl.header = laser_in->header;
	sensor_msgs::PointCloud2 corner_filtered_pc2;
	pcl::toROSMsg(corner_filtered_pcl,corner_filtered_pc2);
	corners_pub_.publish(corner_filtered_pc2);



}

void golfcar_perception::single_scan_clustering::scan_callback(sensor_msgs::LaserScan::ConstPtr laser_in)
{
	// manually project all the laser points, no filter is performed
	/*
	  clusters_pc_.header = laser_in->header;
	  geometry_msgs::Point32 zero_point;
	  zero_point.x = zero_point.y = zero_point.z = 0;
	  clusters_pc_.points.resize(laser_in->ranges.size(),zero_point);
	  clusters_pc_.channels.resize(1);
	  clusters_pc_.channels[0].name = "clusters_id";
	  clusters_pc_.channels[0].values.resize(laser_in->ranges.size(),0);

      for(size_t i = 0; i < laser_in->ranges.size(); i++)
      {
           clusters_pc_.points[i].x = laser_in->ranges[i]*cos(laser_in->angle_min + laser_in->angle_increment * i);
           clusters_pc_.points[i].y = laser_in->ranges[i]*sin(laser_in->angle_min + laser_in->angle_increment * i);
           clusters_pc_.points[i].z = 0;
           clusters_pc_.channels[0].values[i] = i;
      }
      */

	 // use sansac to find lines in the laser scans
	 sac_line_fitting(laser_in);

	  // use laser projection to filter the scan
      laser_filtered_.update(*laser_in);

      //initialize the cluster point cloud
      clusters_pc_.header = laser_filtered_.pc_.header;
	  geometry_msgs::Point32 zero_point;
	  zero_point.x = zero_point.y = zero_point.z = 0;
	  clusters_pc_.points.resize(laser_filtered_.pc_.points.size(),zero_point);
	  clusters_pc_.channels.resize(1);
	  clusters_pc_.channels[0].name = "clusters_id";
	  clusters_pc_.channels[0].values.resize(laser_filtered_.pc_.points.size(),0);

	  //simple euclidean clustering
	  int cur_id = 0;
	  clusters_pc_.points = laser_filtered_.pc_.points;
	  clusters_pc_.channels[0].values[0] = cur_id; // initial id is zero, no need to do the clustering
	  double euc_dist_thres = 5.0;
	  double dist_change = 0;
	  double max_dist_change = 0;
	  srand (time(NULL));
	  std::vector<sensor_msgs::PointCloud> vector_pc;
	  sensor_msgs::PointCloud temp_pc;
	  temp_pc.points.push_back(laser_filtered_.pc_.points[0]);
 	  for(size_t i = 1; i < laser_filtered_.pc_.points.size(); i++)
	  {
		  max_dist_change = euc_dist_thres * (laser_filtered_.scan_.angles[i] - laser_filtered_.scan_.angles[i-1]) * laser_filtered_.scan_.dists[i-1];
		  dist_change = laser_filtered_.scan_.dists[i] - laser_filtered_.scan_.dists[i-1];
		  if(fabs(dist_change) < fabs(max_dist_change))
		  {
			  // belongs to the same cluster
			  clusters_pc_.channels[0].values[i] = cur_id;
			  clusters_pc_.points[i].z = cur_id * 0.1;
			  temp_pc.points.push_back(clusters_pc_.points[i]);

		  }
		  else
		  {
			  //belongs to different clusters
			  //ROS_INFO("new segment is found %d" , i);
			  cur_id = (cur_id + 1)%255;
			  clusters_pc_.channels[0].values[i] = cur_id;
			  clusters_pc_.points[i].z = cur_id * 0.1;

			  if(temp_pc.points.size() >= 5)
			  {
				  vector_pc.push_back(temp_pc);
			  }
			  temp_pc.points.clear();
			  temp_pc.points.push_back(clusters_pc_.points[i]);
		  }
		  cout<<"dist change is "<<dist_change<<"max dist change is "<<max_dist_change<<endl;
	  }

 	  sensor_msgs::PointCloud large_clusters_pc;
 	  large_clusters_pc.header = clusters_pc_.header;
 	  ROS_INFO("# of segment is %d" , vector_pc.size());
 	  for(size_t j = 0; j < vector_pc.size(); j++)
 	  {
 		  large_clusters_pc.points.insert(large_clusters_pc.points.end(),vector_pc[j].points.begin(),vector_pc[j].points.end());
 	  }

      clusters_pub_.publish(large_clusters_pc);
}

int main(int argc, char ** argv)
{
    ros::init(argc,argv,"single_scan_clustering");
    golfcar_perception::single_scan_clustering single_scan;
    ros::spin();
	return 0;
}
