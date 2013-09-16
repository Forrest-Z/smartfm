//
// Created 31 July 2013, 
// by Sean Kim, Xiaotong Shen
//

#define  EIGEN_YES_I_KNOW_SPARSE_MODULE_IS_NOT_STABLE_YET 
#include <mrpt/slam.h>
#include <mrpt/gui.h>

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
#include <fmutil/fm_stopwatch.h>
#include "geometry_helper.h"

#include "dbgstream.h"
#include <cv.h>
#include "readfrontend.h"
#include "CorrelativeMatchGPU.h"
#include "pcl_downsample.h"

using namespace mrpt;
using namespace mrpt::utils;
using namespace mrpt::slam;

sensor_msgs::PointCloud sourcePCL_, targetPCL_;
std::ofstream outFile_;
enum MergingType{
  ICP, MICP, CSM, NONE
};

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
  //cout<<pcl_scan.points.size()<<endl;
  for(unsigned int i = 0; i < pcl_scan.points.size(); i++) 
  {
    int is_rejected = true;
    int nn = NumberofNeighborPoints(pcl_target, pcl_scan.points[i], 1.0);
    //cout<<i<<"("<<nn<<") ";
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
  //cout<<endl;
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

template <class T>
pcl::PointCloud<T> ROSPCLtoPCL(sensor_msgs::PointCloud input, bool filter=false){
  
pcl::PointCloud<T> input_pcl;
  for(size_t i=0; i<input.points.size(); i++){
    T p;
    p.x = input.points[i].x; p.y = input.points[i].y; p.z = input.points[i].z;
    if(filter && (fabs(p.x) > 30 || fabs(p.y) > 10)) continue;
    input_pcl.push_back(p);
  }
  return input_pcl;
}

template <class T>
sensor_msgs::PointCloud PCLtoROSPCL(pcl::PointCloud<T> input_pcl){
  sensor_msgs::PointCloud input;
  for(size_t i=0; i<input_pcl.points.size(); i++){
    geometry_msgs::Point32 p;
    p.x = input_pcl.points[i].x;p.y= input_pcl.points[i].y;p.z = input_pcl.points[i].z;
    input.points.push_back(p);
  }
  return input;
}

sensor_msgs::PointCloud grid_map_;
template<typename T>
inline boost::tuples::tuple<T,T,T> matrixToYawPitchRoll(const Eigen::Matrix<T,3,3>& r)
{
  Eigen::Matrix<T,3,1> euler = r.eulerAngles(2, 1, 0);
  return boost::tuples::make_tuple(euler(0,0), euler(1,0), euler(2,0));
}

bool AlignmentTwoPointCloudCSM(sensor_msgs::PointCloud cloud_leader, sensor_msgs::PointCloud cloud_ego, pcl::PointCloud<pcl::PointXYZ>& Final, Eigen::Matrix4f &tf)
{
  Final.points.clear();
  if( !cloud_ego.points.size() || !cloud_leader.points.size() )
  {
    cout << "No cloud point is given." << endl;
    return false;
  }
  pcl::PointCloud<pcl::PointNormal> cloud_leader_pcl, cloud_ego_pcl;
  //doesn't appear to be filtering...
  cloud_leader_pcl = ROSPCLtoPCL<pcl::PointNormal>(cloud_leader, true);
  cloud_ego_pcl = ROSPCLtoPCL<pcl::PointNormal>(cloud_ego, true);
  pcl::io::savePCDFileASCII(string("cloud_leader.pcd"), cloud_leader_pcl);
  pcl::io::savePCDFileASCII(string("cloud_ego.pcd"), cloud_ego_pcl);
  
  vector<Simple2D> res; res.resize(1);
  res[0].x = res[0].y = 0.1;
  res[0].ang = 1;
  //res[1].x = res[1].y = 0.1;
  //res[1].ang = 0.5;
  vector<Simple2D> ranges; ranges.resize(1);
  ranges[0].x = 1; ranges[0].y = 1; ranges[0].ang = 15;
  //ranges[1].x = 0.5; ranges[1].y = 0.5; ranges[1].ang = 2;
  vector<pcl::PointCloud<pcl::PointNormal> > input_clouds;
  input_clouds.push_back(cloud_ego_pcl);
  CorrelativeMatchGPU cmgpu(cloud_ego_pcl, input_clouds, res, ranges, 2.0, 0); 
  Simple2D best_match = cmgpu.getBestMatch(cloud_leader_pcl);
  Eigen::Vector3f bl_trans(-best_match.x, -best_match.y, 0.);
  double yaw_rotate = -best_match.ang / 180. * M_PI;
  Eigen::Quaternionf bl_rotation(cos(yaw_rotate / 2.), 0, 0,
  -sin(yaw_rotate / 2.));
  Eigen::Translation3f translation (bl_trans);
  Eigen::Affine3f t;
  t = translation * bl_rotation;
  t = t.inverse();
  double yaw, pitch, roll;
  boost::tie(yaw,pitch,roll) = matrixToYawPitchRoll(t.rotation());
  double x = t.translation()(0);
  double y = t.translation()(1);
  double rot = yaw/M_PI*180;
  
  outFile_<<"Score: "<<best_match.score *100<<endl;
  pcl::PointCloud<pcl::PointNormal> matching_cloud_tf;
  pcl::PointCloud<pcl::PointNormal> cloud_leader_pcl_ori = ROSPCLtoPCL<pcl::PointNormal>(cloud_leader);
  pcl::transformPointCloud<pcl::PointNormal>(
	    cloud_leader_pcl_ori, matching_cloud_tf, t);
  
  
  tf = t.matrix();
  
  for(size_t i=0; i<matching_cloud_tf.size(); i++){
      pcl::PointXYZ p;
      p.x = matching_cloud_tf[i].x;
      p.y = matching_cloud_tf[i].y;
      Final.points.push_back(p);
  }
  for(size_t i=0; i<cloud_ego.points.size(); i++){
    pcl::PointXYZ p;
    p.x = cloud_ego.points[i].x;
    p.y = cloud_ego.points[i].y;
    Final.points.push_back(p);
  }
  return true;
}
bool AlignmentTwoPointCloudMICP(sensor_msgs::  PointCloud &cloud_leader, sensor_msgs::PointCloud &cloud_ego, pcl::PointCloud<pcl::PointXYZ>& Final, Eigen::Matrix4f &tf)
{
  if( !cloud_ego.points.size() || !cloud_leader.points.size() )
  {
    cout << "No cloud point is given." << endl;
    return false;
  }

  outFile_ << "The size of Ego map is " << cloud_ego.points.size() << "." << endl;
  outFile_ << "The size of Leader map is " << cloud_leader.points.size() << "." << endl;
  CSimplePointsMap m_ego,m_leader;
  
  pcl::PointCloud<pcl::PointNormal> cloud_leader_pcl, cloud_ego_pcl;
  cloud_leader_pcl = ROSPCLtoPCL<pcl::PointNormal>(cloud_leader, true);
  cloud_ego_pcl = ROSPCLtoPCL<pcl::PointNormal>(cloud_ego, true);
  
    m_ego.setFromPCLPointCloud<pcl::PointCloud<pcl::PointNormal> >(cloud_ego_pcl);
    m_leader.setFromPCLPointCloud<pcl::PointCloud<pcl::PointNormal> >(cloud_leader_pcl);
  
  CICP ICP;
  CICP::TReturnInfo info;
  ICP.options.ICP_algorithm = icpLevenbergMarquardt;
  ICP.options.maxIterations			= 100;
  ICP.options.thresholdAng			= DEG2RAD(10.0f);
  ICP.options.thresholdDist			= 2.5f;
  ICP.options.ALFA				= 0.5f;
  ICP.options.smallestThresholdDist		= 0.05f;
  ICP.options.doRANSAC = false;
  CPose2D	initialPose(0.0f,0.0f,(float)DEG2RAD(0.0f));
  float runningTime;
  CPosePDFPtr pdf = ICP.Align(
		&m_ego,
		&m_leader,
		initialPose,
		&runningTime,
		(void*)&info);
  //printf("ICP run in %.02fms, %d iterations (%.02fms/iter), %.01f%% goodness\n -> ",
	//		runningTime*1000,
	//		info.nIterations,
	//		runningTime*1000.0f/info.nIterations,
	//		info.goodness*100 );
  outFile_ <<"Running time: "<< runningTime*1000<<" goodness: "<<info.goodness*100<<endl;
  outFile_ << "Mean of estimation: " << pdf->getMeanVal() << endl;
  vector_double mean_pose;
  pdf->getMeanVal().getAsVector(mean_pose);
  
  Eigen::Vector3f bl_trans(mean_pose[0], mean_pose[1], 0.);
  double yaw_rotate = -mean_pose[2];
  Eigen::Quaternionf bl_rotation(cos(yaw_rotate / 2.), 0, 0,
  -sin(yaw_rotate / 2.));
  Eigen::Translation3f translation (bl_trans);
  tf = (translation * bl_rotation).matrix();
  
  return true;
}

bool AlignmentTwoPointCloud(sensor_msgs::PointCloud &cloud_leader, sensor_msgs::PointCloud &cloud_ego, pcl::PointCloud<pcl::PointXYZ>& Final, Eigen::Matrix4f &tf) // Ego, Leader, Final 
{ 
  if( !cloud_ego.points.size() || !cloud_leader.points.size() )
  {
    cout << "No cloud point is given." << endl;
    return false;
  }

  outFile_ << "The size of Ego map is " << cloud_ego.points.size() << "." << endl;
  outFile_ << "The size of Leader map is " << cloud_leader.points.size() << "." << endl;

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
    //if((fabs(pt.x) > 30 || fabs(pt.y) > 10)) continue;
    cloud_target->points.push_back(pt);
  }

  for(unsigned int i = 0; i < cloud_leader.points.size(); i++) 
  {
    pcl::PointXYZ pt;
    pt.x = cloud_leader.points[i].x;
    pt.y = cloud_leader.points[i].y;
    pt.z = cloud_leader.points[i].z;
    //if((fabs(pt.x) > 30 || fabs(pt.y) > 10)) continue;
    cloud_in->points.push_back(pt);
  }

  outFile_ << "The number of sampled leader points is " << cloud_in->points.size() << endl;

  pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
  icp.setInputCloud(cloud_in);
  icp.setInputTarget(cloud_target);


//// Set the max correspondence distance to 5cm (0.5) (e.g., correspondences with higher distances will be ignored)
icp.setMaxCorrespondenceDistance (1.0);
// Set the maximum number of iterations (criterion 1)
icp.setMaximumIterations (3);
//// Set the transformation epsilon (criterion 2)
//icp.setTransformationEpsilon (1e-8);
//// Set the euclidean distance difference epsilon (criterion 3)
//icp.setEuclideanFitnessEpsilon (1);

//  pcl::PointCloud<pcl::PointXYZ> Final;
  icp.align(Final);
  outFile_ << "has converged:" << icp.hasConverged() << " score: " << icp.getFitnessScore() << std::endl;

  tf = icp.getFinalTransformation ();
  outFile_ << tf << std::endl;

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

void MapMerging(sensor_msgs::PointCloud &AlignedPCL, sensor_msgs::PointCloud &pcl_scan0, sensor_msgs::PointCloud &pcl_scan1, float l_x, float l_y, float l_z, 
		sensor_msgs::PointCloud &debugPCL, Eigen::Matrix4f &tf, int merging_type)
{
  
  float max_x_target=0;
//  Eigen::Matrix4f tf;

  sensor_msgs::PointCloud sourcePCL;		// leader
  sensor_msgs::PointCloud targetPCL;		// ego
  targetPCL.header.frame_id = "/ldmrs0";
  sourcePCL.header.frame_id = "/ldmrs0";

  pcl::PointCloud<pcl::PointXYZ> FinalPCL;

  Transformation(pcl_scan1, 0, l_x, l_y, l_z);

  //CreateTargetPCL(targetPCL, pcl_scan0, l_x, max_x_target);	// ego
  //CreateSourcePCL(sourcePCL, targetPCL, pcl_scan1, max_x_target, l_x, l_y);// leader
  targetPCL = pcl_scan0;
  sourcePCL = pcl_scan1;
  fmutil::Stopwatch virtualLeadSw("VL");
  AddVirtualLeader(sourcePCL, 0, l_x, l_y, l_z);
  virtualLeadSw.end();
  
  sourcePCL_ = sourcePCL;
  targetPCL_ = targetPCL;
  
  //AlignmentTwoPointCloudCSM(sourcePCL, targetPCL, FinalPCL, tf);
  
  fmutil::Stopwatch sw("Matching");

  // Algorithm start
  bool merging_success =false;
  switch(merging_type){
    case ICP: merging_success = AlignmentTwoPointCloud(sourcePCL, targetPCL, FinalPCL, tf);
    break;
    case CSM: merging_success = AlignmentTwoPointCloudCSM(sourcePCL, targetPCL, FinalPCL, tf);
    break;
    case MICP: merging_success = AlignmentTwoPointCloudMICP(sourcePCL, targetPCL, FinalPCL, tf);
    break;
    case NONE: merging_success = true;
    tf = Eigen::Matrix4f::Identity();
    break;
  }
  if( merging_success)
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
  sw.end();

  outFile_ << "Execution time is " << sw.total_/1000 << " msec" << endl;
}

void getXYRot(Eigen::Matrix4f t){
  double rot = atan2(t(1,0),t(0,0))/M_PI*180;
  double x = t(0,3);
  double y = t(1,3);
  outFile_<<x<<" "<<y<<" "<<rot<<endl;
}

void getScanAccuracy(sensor_msgs::PointCloud base, sensor_msgs::PointCloud compare){
  outFile_<<"Base pt size: "<<base.points.size()<<endl;
  outFile_<<"Compare pt size: "<<compare.points.size()<<endl;
  double average_dist=0;
  double mean_of_square=0;
  for(size_t i=0; i<base.points.size(); i++){
    double dist = 1e99;
    for(size_t j=0; j<compare.points.size(); j++){
      double x = base.points[i].x - compare.points[j].x;
      double y = base.points[i].y - compare.points[j].y;
      double temp = x*x + y*y;
      if(dist > temp) 
	dist = temp;
    }
    average_dist +=sqrt(dist);
    mean_of_square += dist*dist;
  }
  average_dist/=base.points.size();
  outFile_<<"Accuracy: "<<average_dist<<". ";
  outFile_<<"Sample dev: "<<((mean_of_square - average_dist*average_dist)/base.points.size())/base.points.size()<<endl;
}

int main(int argc, char **argv)
{
  string error_msg = "Unknown matching type specified, please use ICP, CSM or NONE for matching type\n";
  error_msg += "Ex: ./recovering_map ICP folder/";
  if(argc != 3) {cout<<error_msg<<endl; return 1;}
  string merging_type_str = string(argv[1]);
  string folder = string(argv[2]);
  int merging_type=-1;
  
  if(merging_type_str == "ICP") merging_type = ICP;
  else if(merging_type_str == "CSM") merging_type = CSM;
  else if(merging_type_str == "NONE") merging_type = NONE;
  else if(merging_type_str == "MICP") merging_type = MICP;
  
  if(merging_type == -1){
    cout<<error_msg<<endl;
    return 1;
  }
  
  string log_file = string(folder+"log_"+merging_type_str);
  outFile_.open(log_file.c_str());  
  outFile_<<"Matching type: "<<merging_type_str<<endl;
  
  
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
  
  string file1 = folder+string("pointcloud0.csv");
  string file2 = folder+string("pointcloud1.csv");
  string file3 = folder+string("pointcloud2.csv");
  CSVtoPointCloud(file1, pcl_scan0);
  CSVtoPointCloud(file2, pcl_scan1);
  CSVtoPointCloud(file3, pcl_scan2);

  ros::Publisher pc_ori0_pub = n.advertise<sensor_msgs::PointCloud>("pc_ori_0", 10);
  ros::Publisher pc_ori1_pub = n.advertise<sensor_msgs::PointCloud>("pc_ori_1", 10);
  ros::Publisher pc_ori2_pub = n.advertise<sensor_msgs::PointCloud>("pc_ori_2", 10);
  
  outFile_<<"pcl_scan0 size: "<<pcl_scan0.points.size()<<endl;
  outFile_<<"pcl_scan1 size: "<<pcl_scan1.points.size()<<endl;
  outFile_<<"pcl_scan2 size: "<<pcl_scan2.points.size()<<endl;
  MapMerging(AlignedPCL12, pcl_scan1, pcl_scan2, 3.5, 0, 0, debugPCL0, tf12, merging_type);
  getScanAccuracy(AlignedPCL12, pcl_scan1);
  MapMerging(AlignedPCL01, pcl_scan0, AlignedPCL12, 5, 1, 0, debugPCL1, tf01, merging_type);
  getScanAccuracy(AlignedPCL01, pcl_scan0);
  TrasformationMatrix(pcl_scan2, pcl_scan2_t0, tf12);
  getXYRot(tf12);
  TrasformationMatrix(pcl_scan2_t0, pcl_scan2_t1, tf01);
  getXYRot(tf01);
  Transformation(pcl_scan2_t1, 0, 5, 1, 0);

  TrasformationMatrix(pcl_scan1, pcl_scan1_t0, tf01);

  Transformation(pcl_scan1_t0, 0, 5, 1, 0);

  ros::Publisher pc_pub0 = n.advertise<sensor_msgs::PointCloud>("ego_scan_pcl", 10);
  ros::Publisher pc_pub1 = n.advertise<sensor_msgs::PointCloud>("transformed_1st_leader_pcl", 10);
  ros::Publisher pc_pub2 = n.advertise<sensor_msgs::PointCloud>("transformed_2nd_leader_pcl", 10);
  ros::Publisher pc_pub3 = n.advertise<sensor_msgs::PointCloud>("final_merged_pcl", 10);
  ros::Publisher sourcePCL_pub = n.advertise<sensor_msgs::PointCloud>("source_pcl", 10);
  ros::Publisher targetPCL_pub = n.advertise<sensor_msgs::PointCloud>("target_pcl", 10);
  //ros::Publisher csm_point_pub = n.advertise<sensor_msgs::PointCloud>("csm_points", 10);
  ros::Publisher grid_map_pub = n.advertise<sensor_msgs::PointCloud>("grid_map", 10);
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

  grid_map_.header.frame_id = "/ldmrs0";

  
  
  pcl::io::savePCDFileASCII (string(folder+"final_merged_map_"+merging_type_str+".pcd"), ROSPCLtoPCL<pcl::PointNormal>(AlignedPCL01));
  outFile_.close();
  ifstream infile;

  infile.open(log_file.c_str());
  string sLine;
  while (!infile.eof())
  {
    getline(infile, sLine);
    cout << sLine << endl;
  }
  infile.close();
  cout<<"Log file saved at "<<log_file<<endl;
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
    //csm_point_pub.publish(csm_output_cloud_);
    sourcePCL_pub.publish(sourcePCL_);
    targetPCL_pub.publish(targetPCL_);
    grid_map_pub.publish(grid_map_);
    loop_rate.sleep();
  }

//  ros::spin();

  return 0;
}
