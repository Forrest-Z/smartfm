#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/transforms.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/Pose.h>
#include "NormalCorrelativeMatching.h"
#include "geometry_helper.h"
#include "readfrontend.h"
#include "CorrelativeMatchGPU.h"


void addScoreVer(pcl::PointCloud<pcl::PointNormal> &matching_clouds_j, vector<pcl::PointCloud<pcl::PointNormal> > &input_clouds,
     ScoreData &sd, MySQLHelper &mysql, NormalCorrelativeMatching &ncm)
{
  double ver_score = ncm.veriScore(matching_clouds_j, input_clouds, sd);
  mysql.updateScoreVer(ver_score, sd);
}

template<typename T>
inline boost::tuples::tuple<T,T,T> matrixToYawPitchRoll(const Eigen::Matrix<T,3,3>& r)
{
  Eigen::Matrix<T,3,1> euler = r.eulerAngles(2, 1, 0);
  return boost::tuples::make_tuple(euler(0,0), euler(1,0), euler(2,0));
}

int main(int argc, char** argv)
{
  int start_idx = 5;
  double res0 = 0.5;
  double rot_res0 = 2;
  double res1 = 0.1;
  double rot_res1 = 1;
  double range_cov = 1;
      
  int visualize = 0;
  
  vector<Simple2D> res; res.resize(2);
  res[0].x = res[0].y = res0;
  res[0].ang = rot_res0;
  res[1].x = res[1].y = res1;
  res[1].ang = rot_res1;
  vector<Simple2D> ranges; ranges.resize(2);
  ranges[0].x = 15; ranges[0].y = 20; ranges[0].ang = 180;
  ranges[1].x = 2; ranges[1].y = 2; ranges[1].ang = 5;

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
  start_idx = 2562;
  //end_idx = 1220;
  for(int j=0; j<total_files; j+=skip_read)
  {
    stringstream matching_file;
    matching_file <<folder<<setfill('0')<<setw(5)<<j<<".pcd";
    pcl::PointCloud<pcl::PointNormal> matching_cloud;
    pcl::io::loadPCDFile(matching_file.str(), matching_cloud);    
    matching_clouds[j] = matching_cloud;
    cout<<"Reading pcd file "<<matching_file.str()<<"     \xd"<<flush;
  }
  cout<<endl;

  for(int i=start_idx; i<end_idx; i++)
  {//1404 1620 //1868 2028// 2348 2432 //136 412 //988 1220
    if(i%skip_read != 0) continue;
    stringstream input_file;
    input_file <<folder<<setfill('0')<<setw(5)<<i<<".pcd";    
    pcl::PointCloud<pcl::PointNormal> input_cloud;
    pcl::io::loadPCDFile(input_file.str(), input_cloud);
    vector<pcl::PointCloud<pcl::PointNormal> > input_clouds = append_input_cloud(input_cloud, string(argv[1]), input_file.str());
    CorrelativeMatchGPU cm(input_cloud, res, ranges, range_cov, visualize);
    //NormalCorrelativeMatching ncm(input_cloud, res_, 0.2);
    map<int, ScoreData> score_datas;

    score_datas = mysql.getDataFromSrc(i);
    for(int j=i; j<total_files; j++)
    {
      if(j%skip_read!=0) continue;
      ScoreData sd;
      sd.node_src = i; sd.node_dst = j; 
      bool retrieve_data=false;
      if(score_datas.find(sd.node_dst) != score_datas.end()) {
	retrieve_data = true;
	sd = score_datas[sd.node_dst];
      }
     
      //retrieve_data = mysql.getData(sd, false);
      if(retrieve_data)
      {
        //cout<<sd.node_src<<" "<<sd.node_dst<<" "<<sd.x<<" "<<sd.y<<" "<<sd.t<<" "<<sd.time_taken<<"\xd"<<flush;
        //if(sd.final_score == 0)
         // addScoreVer(matching_clouds[j], input_clouds, sd, mysql, ncm);
      }
      else
      {
        fmutil::Stopwatch sw("match");  
	Simple2D best_match = cm.getBestMatch(matching_clouds[j]);
        //ncm.bruteForceSearch(matching_clouds[j],true);
        sw.end(false);
	double yaw_rotate = -best_match.ang / 180. * M_PI;
	Eigen::Quaternionf bl_rotation(cos(yaw_rotate / 2.), 0, 0,
		  -sin(yaw_rotate / 2.));
	Eigen::Quaternionf bl_rot_unity(1,0,0,0);
	Eigen::Vector3f bl_trans(-best_match.x, -best_match.y, 0.);
	Eigen::Vector3f bl_tran_unity(0,0,0);
	Eigen::Translation3f translation (bl_trans);
	Eigen::Affine3f t;
	t = translation * bl_rotation;
	t = t.inverse();
	double yaw, pitch, roll;
	boost::tie(yaw,pitch,roll) = matrixToYawPitchRoll(t.rotation());
        sd.score = best_match.score;
        sd.x = t.translation()[0];
        sd.y = t.translation()[1];
        sd.t = yaw;
        sd.time_taken = sw.total_/1000;
        
        
	//#pragma omp critical
        //double ver_score = ncm.veriScore(matching_clouds[j], input_clouds, sd);
        //sd.score_ver = ver_score;
        //sd.final_score = sqrt(ver_score * ncm.best_score_);
        mysql.insertData(sd);
      }
    }
    cout<<i<<" data all done"<<"\xd"<<flush;
  }
}
