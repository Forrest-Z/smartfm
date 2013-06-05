/*
 * single_scan_clustering.cpp
 *
 *  Created on: Jun 5, 2013
 *      Author: Shen Xiaotong
 */



#include "single_scan_clustering.h"
#include <stdio.h>
#include <stdlib.h>
#include <time.h>

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

golfcar_perception::single_scan_clustering::single_scan_clustering(void):nh_("")
{
	laser_sub_= nh_.subscribe("/sickldmrs/scan3",5,&single_scan_clustering::scan_callback,this);
	clusters_pub_ = nh_.advertise<sensor_msgs::PointCloud>("clusters_pts",10);
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
