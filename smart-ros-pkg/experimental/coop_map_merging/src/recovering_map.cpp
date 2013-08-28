//
// Created 31 July 2013, 
// by Sean Kim, Xiaotong Shen
//
#include <iostream>
#include <time.h>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>
#include <laser_geometry/laser_geometry.h>
#include <geometry_msgs/Point32.h>
#include "csv.h"

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>

void Transformation(sensor_msgs::PointCloud &pcl_scan, float r_theta, float t_x, float t_y, float t_z)
{
  for (int i = 0; i < pcl_scan.points.size(); ++i) {
    float x, y, z;

    x = pcl_scan.points[i].x; y = pcl_scan.points[i].y; z = pcl_scan.points[i].z;

    pcl_scan.points[i].x = x * cos(r_theta) - y * sin(r_theta) + t_x;
    pcl_scan.points[i].y = x * sin(r_theta) + y * cos(r_theta) + t_y;
    pcl_scan.points[i].z = z + t_z;
  }
}

void AddVirtualLeader(sensor_msgs::PointCloud &pcl_scan, float r_theta, float t_x, float t_y, float t_z)
{
  for(float i = -0.4; i < 0.4; i+=0.1) 
  {
    float x, y, z;
    geometry_msgs::Point32 pt;
    x = 0;
    y = i;// + t_y;
    z = 0;// + t_z;

    pt.x = x * cos(r_theta) - y * sin(r_theta) + t_x;
    pt.y = x * sin(r_theta) + y * cos(r_theta) + t_y;
    pt.z = z + t_z;

    pcl_scan.points.push_back(pt);
  }
}

int NumberofNeighborPoints(sensor_msgs::PointCloud &cloud_in, geometry_msgs::Point32 pt, double diameter)
//int NumberofNeighborPoints(sensor_msgs::PointCloud &cloud_in, pcl::PointXYZ pt, double diameter)
{
  int nNeighbors=-1;
  double dist;

  for(unsigned int i = 0; i < cloud_in.points.size(); i++) 
  {
    dist = (pt.x - cloud_in.points[i].x)*(pt.x - cloud_in.points[i].x) +
      (pt.y - cloud_in.points[i].y)*(pt.y - cloud_in.points[i].y) +
      (pt.z - cloud_in.points[i].z)*(pt.z - cloud_in.points[i].z);

    if(dist < (diameter*diameter) ) nNeighbors++;
  }
  return nNeighbors;
}

// Leader
// (l_x, l_y) : position of leader
// Extracting overlapping area
void CreateSourcePCL(sensor_msgs::PointCloud &sourcePCL, sensor_msgs::PointCloud &pcl_target, sensor_msgs::PointCloud &pcl_scan, float &max_x, float l_x, float l_y)
{
  cout<<pcl_scan.points.size()<<endl;
  for(unsigned int i = 0; i < pcl_scan.points.size(); i++) 
  {
    int is_rejected = true;
    int nn = NumberofNeighborPoints(pcl_target, pcl_scan.points[i], 1.0);
    cout<<i<<"("<<nn<<") ";
    if( nn > 2 )
    {
      //int j = 1 + (int)( (double)(nn) * rand() / ( RAND_MAX + 1.0 ) );

      //if( j == 1) 
      is_rejected = false;
    }
    else is_rejected = false;

    if( is_rejected == false ) 
    {
      if(pcl_scan.points[i].x < max_x )
      {
        if( (pcl_scan.points[i].y > 1.5) || (pcl_scan.points[i].y < -1.5) ) // remove scan point of detected leader;
        {
          geometry_msgs::Point32 pt;
          pt.x = pcl_scan.points[i].x;
          pt.y = pcl_scan.points[i].y;
          pt.z = pcl_scan.points[i].z;

          sourcePCL.points.push_back(pt);
        }
      }
    }
  }
  cout<<endl;
}

// Ego vehicle
void CreateTargetPCL(sensor_msgs::PointCloud &targetPCL, sensor_msgs::PointCloud &pcl_scan, float l_x, float &max_x)
{
  max_x = 0;

  for(unsigned int i = 0; i < pcl_scan.points.size(); i++) 
  {
    if( pcl_scan.points[i].x > l_x )
    {
        geometry_msgs::Point32 pt;
        pt.x = pcl_scan.points[i].x;
        pt.y = pcl_scan.points[i].y;
        pt.z = pcl_scan.points[i].z;

        targetPCL.points.push_back(pt);

        if(max_x < pt.x) max_x = pt.x;
    }
  }
}

int AlignmentTwoPointCloud(sensor_msgs::PointCloud &cloud_leader, sensor_msgs::PointCloud &cloud_ego, pcl::PointCloud<pcl::PointXYZ>& Final, Eigen::Matrix4f &tf) // Ego, Leader, Final 
{ 
  if( !cloud_ego.points.size() || !cloud_leader.points.size() )
  {
    cout << "No cloud point is given." << endl;
    return false;
  }

  cout << "The size of Ego map is " << cloud_ego.points.size() << "." << endl;
  cout << "The size of Leader map is " << cloud_leader.points.size() << "." << endl;

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in (new pcl::PointCloud<pcl::PointXYZ>); // original data
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_target (new pcl::PointCloud<pcl::PointXYZ>);// shifted data

  cloud_in->header = cloud_leader.header;
  cloud_target->header = cloud_ego.header;

  for(unsigned int i = 0; i < cloud_ego.points.size(); i++) 
  {
    pcl::PointXYZ pt;
    pt.x = cloud_ego.points[i].x;
    pt.y = cloud_ego.points[i].y;
    pt.z = cloud_ego.points[i].z;

    cloud_target->points.push_back(pt);
  }

  for(unsigned int i = 0; i < cloud_leader.points.size(); i++) 
  {
    pcl::PointXYZ pt;
    pt.x = cloud_leader.points[i].x;
    pt.y = cloud_leader.points[i].y;
    pt.z = cloud_leader.points[i].z;

    cloud_in->points.push_back(pt);
  }

  cout << "The number of sampled leader points is " << cloud_in->points.size() << endl;

  pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
  icp.setInputCloud(cloud_in);
  icp.setInputTarget(cloud_target);


//// Set the max correspondence distance to 5cm (0.5) (e.g., correspondences with higher distances will be ignored)
icp.setMaxCorrespondenceDistance (0.5);
// Set the maximum number of iterations (criterion 1)
icp.setMaximumIterations (3);
//// Set the transformation epsilon (criterion 2)
//icp.setTransformationEpsilon (1e-8);
//// Set the euclidean distance difference epsilon (criterion 3)
//icp.setEuclideanFitnessEpsilon (1);

//  pcl::PointCloud<pcl::PointXYZ> Final;
  icp.align(Final);
  std::cout << "has converged:" << icp.hasConverged() << " score: " << icp.getFitnessScore() << std::endl;

  tf = icp.getFinalTransformation ();
  std::cout << tf << std::endl;

  if(icp.hasConverged() == 1)
    return true;
}

void TrasformationMatrix(sensor_msgs::PointCloud &pcl_scan, sensor_msgs::PointCloud &pcl_out, Eigen::Matrix4f &tf) 
{
  for(unsigned int i = 0; i < pcl_scan.points.size(); i++) 
  {
    float x, y, z;
    geometry_msgs::Point32 pt;

    x = pcl_scan.points[i].x;
    y = pcl_scan.points[i].y;
    z = pcl_scan.points[i].z;

    pt.x = x * tf(0,0) + y * tf(0,1) + tf(0,3);
    pt.y = x * tf(1,0) + y * tf(1,1) + tf(1,3);
    pt.z = z + tf(2,3);

    pcl_out.points.push_back(pt);
  }
}

sensor_msgs::PointCloud sourcePCL_, targetPCL_;
void TranformFromPCLtoScanPCL(pcl::PointCloud<pcl::PointXYZ> &FinalPCL, sensor_msgs::PointCloud &AlignedPCL)
{
  for(unsigned int i = 0; i < FinalPCL.points.size(); i++) 
  {
    geometry_msgs::Point32 pt;
    pt.x = FinalPCL.points[i].x;
    pt.y = FinalPCL.points[i].y;
    pt.z = FinalPCL.points[i].z;

    AlignedPCL.points.push_back(pt);
  }
}

void MapMerging(sensor_msgs::PointCloud &AlignedPCL, sensor_msgs::PointCloud &pcl_scan0, sensor_msgs::PointCloud &pcl_scan1, float l_x, float l_y, float l_z, sensor_msgs::PointCloud &debugPCL, Eigen::Matrix4f &tf)
{
  clock_t begin, end;		// to measure execution time
  double time_spent;		// to measure execution time
  float max_x_target=0;
//  Eigen::Matrix4f tf;

  sensor_msgs::PointCloud sourcePCL;		// leader
  sensor_msgs::PointCloud targetPCL;		// ego
  targetPCL.header.frame_id = "/ldmrs0";
  sourcePCL.header.frame_id = "/ldmrs0";

  pcl::PointCloud<pcl::PointXYZ> FinalPCL;

  cout<<"pcl_scan1 size: "<<pcl_scan1.points.size()<<endl;
  Transformation(pcl_scan1, 0, l_x, l_y, l_z);

  CreateTargetPCL(targetPCL, pcl_scan0, l_x, max_x_target);	// ego
  CreateSourcePCL(sourcePCL, targetPCL, pcl_scan1, max_x_target, l_x, l_y);// leader

  AddVirtualLeader(sourcePCL, 0, l_x, l_y, l_z);

  
  sourcePCL_ = sourcePCL;
  targetPCL_ = targetPCL;
  begin = clock();

  // Algorithm start
  if( true == AlignmentTwoPointCloud(sourcePCL, targetPCL, FinalPCL, tf) )
  {
    TrasformationMatrix(pcl_scan1, AlignedPCL, tf);
    TranformFromPCLtoScanPCL(FinalPCL, debugPCL);

    // Merged with ego vehicle's map
    for(unsigned int i = 0; i < pcl_scan0.points.size(); i++) 
    {
      geometry_msgs::Point32 pt;
      pt.x = pcl_scan0.points[i].x;
      pt.y = pcl_scan0.points[i].y;
      pt.z = pcl_scan0.points[i].z;

      AlignedPCL.points.push_back(pt);
    }
  }
  // Algorithm end
  
  end = clock();

  time_spent = (double)1000.0 * (end - begin) / CLOCKS_PER_SEC;

  cout << "Execution time is " << time_spent << " msec" << endl;
}

int main(int argc, char **argv)
{
  //float l_x=4, l_y=0, l_z=0;	// estimated position of leader vehicle from the perspective of ego vehicle
  // 1(x,y,z)=(7,0,0) for scandata9, 
  // 1(x,y,z)=(6,0,0) for scandata8, 
  // 1(x,y,z)=(5,0,0) for scandata7,
  // 1(x,y,z)=(4.5,1,0) for scandata4
  // 1(x,y,z)=(5,1,0) for scandata3, 2(x,y,z)=(3.5,1,0) for scandata3, 
  // 1(x,y,z)=(5,1,0) for scandata2, 

  sensor_msgs::PointCloud AlignedPCL01;
  sensor_msgs::PointCloud AlignedPCL12;
  sensor_msgs::PointCloud debugPCL0;
  sensor_msgs::PointCloud debugPCL1;
  sensor_msgs::PointCloud pcl_scan2_t0;
  sensor_msgs::PointCloud pcl_scan2_t1;
  sensor_msgs::PointCloud pcl_scan1_t0;

  Eigen::Matrix4f tf01;
  Eigen::Matrix4f tf12;

  ros::init(argc, argv, "map_merging_test");

  ros::NodeHandle n;

  sensor_msgs::PointCloud pcl_scan0;
  sensor_msgs::PointCloud pcl_scan1;
  sensor_msgs::PointCloud pcl_scan2;

  CSVtoPointCloud("pointcloud0.csv", pcl_scan0);
  CSVtoPointCloud("pointcloud1.csv", pcl_scan1);
  CSVtoPointCloud("pointcloud2.csv", pcl_scan2);

  ros::Publisher pc_ori0_pub = n.advertise<sensor_msgs::PointCloud>("pc_ori_0", 10);
  ros::Publisher pc_ori1_pub = n.advertise<sensor_msgs::PointCloud>("pc_ori_1", 10);
  ros::Publisher pc_ori2_pub = n.advertise<sensor_msgs::PointCloud>("pc_ori_2", 10);
  
  cout<<"pcl_scan0 size: "<<pcl_scan0.points.size()<<endl;
  cout<<"pcl_scan1 size: "<<pcl_scan1.points.size()<<endl;
  //MapMerging(AlignedPCL12, pcl_scan1, pcl_scan2, 3.5, 0, 0, debugPCL0, tf12);
  MapMerging(AlignedPCL01, pcl_scan0, pcl_scan1, 5, 1, 0, debugPCL1, tf01);

  TrasformationMatrix(pcl_scan2, pcl_scan2_t0, tf12);
  TrasformationMatrix(pcl_scan2_t0, pcl_scan2_t1, tf01);

  Transformation(pcl_scan2_t1, 0, 5, 1, 0);

  TrasformationMatrix(pcl_scan1, pcl_scan1_t0, tf01);

  Transformation(pcl_scan1_t0, 0, 5, 1, 0);

  ros::Publisher pc_pub0 = n.advertise<sensor_msgs::PointCloud>("ego_scan_pcl", 10);
  ros::Publisher pc_pub1 = n.advertise<sensor_msgs::PointCloud>("transformed_1st_leader_pcl", 10);
  ros::Publisher pc_pub2 = n.advertise<sensor_msgs::PointCloud>("transformed_2nd_leader_pcl", 10);
  ros::Publisher pc_pub3 = n.advertise<sensor_msgs::PointCloud>("final_merged_pcl", 10);
  ros::Publisher sourcePCL_pub = n.advertise<sensor_msgs::PointCloud>("source_pcl", 10, true);
  ros::Publisher targetPCL_pub = n.advertise<sensor_msgs::PointCloud>("target_pcl", 10, true);
  ros::Rate loop_rate(10);

  pcl_scan0.header.frame_id = "/ldmrs0";
  pcl_scan1.header.frame_id = "/ldmrs0";
  pcl_scan2.header.frame_id = "/ldmrs0";

  AlignedPCL01.header.frame_id = "/ldmrs0";
  AlignedPCL12.header.frame_id = "/ldmrs0";
  debugPCL0.header.frame_id = "/ldmrs0";
  debugPCL1.header.frame_id = "/ldmrs0";

  pcl_scan2_t0.header.frame_id = "/ldmrs0";
  pcl_scan2_t1.header.frame_id = "/ldmrs0";
  pcl_scan1_t0.header.frame_id = "/ldmrs0";

  
  while(ros::ok())
  {
/*    pc_pub0.publish(pcl_scan1);
    //pc_pub1.publish(pcl_scan2);
    pc_pub1.publish(debugPCL);
    pc_pub2.publish(AlignedPCL12);*/

    pc_pub0.publish(pcl_scan0);
    //pc_pub1.publish(pcl_scan2);
//    pc_pub1.publish(AlignedPCL12);
//    pc_pub1.publish(debugPCL12);
    pc_ori0_pub.publish(pcl_scan0);
    pc_ori1_pub.publish(pcl_scan1);
    pc_ori2_pub.publish(pcl_scan2);
    pc_pub1.publish(pcl_scan2_t1);
    pc_pub2.publish(pcl_scan1_t0);
    pc_pub3.publish(AlignedPCL01);
    
    sourcePCL_pub.publish(sourcePCL_);
    targetPCL_pub.publish(targetPCL_);
    loop_rate.sleep();
  }

//  ros::spin();

  return 0;
}
