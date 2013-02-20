#include <pcl/visualization/pcl_visualizer.h>
#include "NormalCorrelativeMatching.h"
#include "readfrontend.h"
#include "mysql_helper.h"

void addScoreVer(pcl::PointCloud<pcl::PointNormal> &matching_clouds_j, vector<pcl::PointCloud<pcl::PointNormal> > &input_clouds,
     ScoreData &sd, MySQLHelper &mysql)
{
  RasterMapImage rmi_ver(0.05, 0.2);
  pcl::PointCloud<pcl::PointNormal> matching_cloud = matching_clouds_j;
  double yaw_rotate = sd.t / 180. * M_PI;
  Eigen::Quaternionf bl_rotation(cos(yaw_rotate / 2.), 0, 0,
            -sin(yaw_rotate / 2.));
  Eigen::Vector3f bl_trans(sd.x, sd.y, 0.);
  pcl_utils::transformPointCloudWithNormals<pcl::PointNormal>(
            matching_cloud, matching_cloud, bl_trans, bl_rotation);
  vector<cv::Point2f> input_pts, input_normals;
  input_pts.resize(matching_cloud.points.size());
  input_normals.resize(matching_cloud.points.size());
  for (size_t k = 0; k < matching_cloud.points.size(); k++) {
      input_pts[k].x = matching_cloud.points[k].x;
      input_pts[k].y = matching_cloud.points[k].y;
      input_normals[k].x = matching_cloud.points[k].normal_x;
      input_normals[k].y = matching_cloud.points[k].normal_y;
  }
  rmi_ver.getInputPoints(input_pts, input_normals);  
  double ver_score = 0;

  for(size_t k=0; k<input_clouds.size(); k++)
  {
    pcl::PointCloud<pcl::PointNormal> input_cloud_single = input_clouds[k];
    input_cloud_single = pcl_downsample(input_cloud_single, 0.1, 0.1, 0.1);
    vector<cv::Point2f> search_pt;
    vector<double> normal_pt;
    search_pt.resize(input_cloud_single.points.size());
    normal_pt.resize(input_cloud_single.points.size());
    
    for (size_t l = 0; l < input_cloud_single.points.size(); l++) {
        search_pt[l].x = input_cloud_single.points[l].x;
        search_pt[l].y = input_cloud_single.points[l].y;
        normal_pt[l] = atan2(input_cloud_single.points[l].normal_y,
                input_cloud_single.points[l].normal_x);
    }
    ScoreDetails s_det;
    double best_score = rmi_ver.getScoreWithNormal(search_pt, normal_pt, s_det);
    if(ver_score < best_score)
      ver_score = best_score;
  }
  #pragma omp critical
  mysql.updateScoreVer(ver_score, sd);
}
int main(int argc, char** argv)
{
  double res_ = 0.05;
  int start_idx = 5;
  
  int skip_read = atoi(argv[2]);

//#pragma omp parallel for
  map<int, pcl::PointCloud<pcl::PointNormal> > matching_clouds;
  string frontend_file = argv[1];
  vector<geometry_msgs::Pose> poses;
  readFrontEndFile(argv[1], poses);
  int total_files = poses.size();
  int end_idx = total_files-7; //654-7;
  int startfile_idx= frontend_file.find_last_of("/")+1;
  string folder = frontend_file.substr(0, startfile_idx);
  frontend_file = frontend_file.substr(startfile_idx, frontend_file.size()-startfile_idx-4);
  MySQLHelper mysql("normal_scanmatch", frontend_file);
  mysql.createTable();
  for(int j=0; j<total_files/*654*/; j+=skip_read)
  {
    stringstream matching_file;
    matching_file <<folder<<setfill('0')<<setw(5)<<j<<".pcd";
    pcl::PointCloud<pcl::PointNormal> matching_cloud;
    pcl::io::loadPCDFile(matching_file.str(), matching_cloud);
    matching_cloud = pcl_downsample(matching_cloud, res_*2, res_*2, res_*2);
    matching_clouds[j] = matching_cloud;
    cout<<"Reading pcd file "<<matching_file.str()<<"     \xd"<<flush;
  }
  cout<<endl;
#pragma omp parallel for
  for(int i=start_idx; i<end_idx; i++)
  {
    if(i%skip_read != 0) continue;
    stringstream input_file;
    input_file <<folder<<setfill('0')<<setw(5)<<i<<".pcd";    
    pcl::PointCloud<pcl::PointNormal> input_cloud;
    pcl::io::loadPCDFile(input_file.str(), input_cloud);
    vector<pcl::PointCloud<pcl::PointNormal> > input_clouds = append_input_cloud(input_cloud, string(argv[1]), input_file.str());
    input_cloud = pcl_downsample(input_cloud, res_*2, res_*2, res_*2);
    NormalCorrelativeMatching ncm(input_cloud, res_, 0.2);
    for(int j=i; j<total_files; j++)
    {
      if(j%skip_read!=0) continue;
      ScoreData sd;
      sd.node_src = i; sd.node_dst = j;   
      bool retrieve_data=false;
      #pragma omp critical
      retrieve_data = mysql.getData(sd, false);
      if(retrieve_data)
      {
        cout<<sd.node_src<<" "<<sd.node_dst<<" "<<sd.x<<" "<<sd.y<<" "<<sd.t<<" "<<sd.time_taken<<"\xd"<<flush;
        if(sd.final_score == 0)
          addScoreVer(matching_clouds[j], input_clouds, sd, mysql);
      }
      else
      {
        fmutil::Stopwatch sw("match");  
        ncm.bruteForceSearch(matching_clouds[j],true);
        sw.end(false);
        sd.score = ncm.best_score_;
        sd.x = ncm.offset_x;
        sd.y = ncm.offset_y;
        sd.t = ncm.rotation;
        sd.time_taken = sw.total_/1000;
        #pragma omp critical
        {
          mysql.insertData(sd);
          assert(mysql.getData(sd, false));
        }
        addScoreVer(matching_clouds[j], input_clouds, sd, mysql);
      }
    }
    //cout<<endl;
  }
}
