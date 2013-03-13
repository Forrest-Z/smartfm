#include <pcl/visualization/pcl_visualizer.h>
#include "NormalCorrelativeMatching.h"
#include "readfrontend.h"



void addScoreVer(pcl::PointCloud<pcl::PointNormal> &matching_clouds_j, vector<pcl::PointCloud<pcl::PointNormal> > &input_clouds,
     ScoreData &sd, MySQLHelper &mysql, NormalCorrelativeMatching &ncm)
{
  double ver_score = ncm.veriScore(matching_clouds_j, input_clouds, sd);
  #pragma omp critical
  mysql.updateScoreVer(ver_score, sd);
}

int get_num_threads()
{
return 8;
}

int main(int argc, char** argv)
{
 
  double res_ = 0.1;
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
  //start_idx = 225;
  //end_idx = 340;
  for(int j=0; j<total_files; j+=skip_read)
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
#pragma omp parallel for num_threads(get_num_threads())
  for(int i=start_idx; i<end_idx; i++)
  {//1404 1620 //1868 2028// 2348 2432 //136 412 //988 1220
    if(i%skip_read != 0) continue;
    stringstream input_file;
    input_file <<folder<<setfill('0')<<setw(5)<<i<<".pcd";    
    pcl::PointCloud<pcl::PointNormal> input_cloud;
    pcl::io::loadPCDFile(input_file.str(), input_cloud);
    vector<pcl::PointCloud<pcl::PointNormal> > input_clouds = append_input_cloud(input_cloud, string(argv[1]), input_file.str());
    input_cloud = pcl_downsample(input_cloud, res_*2, res_*2, res_*2);
    NormalCorrelativeMatching ncm(input_cloud, res_, 0.2);
    map<int, ScoreData> score_datas;
    #pragma omp critical
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
        cout<<sd.node_src<<" "<<sd.node_dst<<" "<<sd.x<<" "<<sd.y<<" "<<sd.t<<" "<<sd.time_taken<<"\xd"<<flush;
        if(sd.final_score == 0)
          addScoreVer(matching_clouds[j], input_clouds, sd, mysql, ncm);
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
        mysql.insertData(sd);
	#pragma omp critical
        assert(mysql.getData(sd, false));
        addScoreVer(matching_clouds[j], input_clouds, sd, mysql, ncm);
      }
    }
    //cout<<endl;
  }
}
