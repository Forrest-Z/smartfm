#include <fmutil/fm_stopwatch.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <csm_cuda/csm.h>
#include "geometry_helper.h"
#include "dbgstream.h"
#include "readfrontend.h"
boost::shared_ptr<pcl::visualization::PCLVisualizer> initVisualizer() {
    // --------------------------------------------
    // -----Open 3D viewer and add point cloud-----
    // --------------------------------------------
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(
            new pcl::visualization::PCLVisualizer("3D Viewer"));
    viewer->setBackgroundColor(0, 0, 0);
    viewer->addCoordinateSystem(1.0);
    viewer->initCameraParameters();

    return (viewer);
}

void addPointCloud(boost::shared_ptr<pcl::visualization::PCLVisualizer> &viewer,
        pcl::PointCloud<pcl::PointNormal> &cloud_in, string cloud_name,
        string normal_name, bool red_color) {
    cout << cloud_in.points.size() << " loaded." << endl;
    if (red_color){
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointNormal> 
        color(cloud_in.makeShared(), 255, 0, 0);
	 viewer->addPointCloud<pcl::PointNormal>(cloud_in.makeShared(), color,
            cloud_name);
    }
    else{
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointNormal> 
        color(cloud_in.makeShared(), 0, 255, 0);
	 viewer->addPointCloud<pcl::PointNormal>(cloud_in.makeShared(), color,
            cloud_name);
    }
    viewer->setPointCloudRenderingProperties(
            pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, cloud_name);
}
string folder_str_, front_end_str_;
int input_idx_, matching_idx_;
CsmGPU<pcl::PointNormal> *cm_;
bool matching_mode = false;
pcl::PointCloud<pcl::PointNormal> input_cloud_;

enum SimpleState{
  SELECT_SOURCE_PCD, SELECT_MATCHING_PCD, NORMAL
};
SimpleState state_;
vector<pcl::PointCloud<pcl::PointNormal> > input_clouds;
void keyboardEventOccurred(const pcl::visualization::KeyboardEvent &event,
        void* viewer_void) {
  fmutil::Stopwatch sw_whole_vis("Whole event");
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer =
            *static_cast<boost::shared_ptr<pcl::visualization::PCLVisualizer> *>(viewer_void);
  string keySym = event.getKeySym();    
  if (event.keyDown()) {
    stringstream input_cloud_str;
    pcl::PointCloud<pcl::PointNormal> input_cloud;
    pcl::PointCloud<pcl::PointNormal> matching_cloud;
    switch (state_){
      case SELECT_SOURCE_PCD:
	viewer->removePointCloud("input_cloud");
	viewer->removePointCloud("input_normal_cloud");
	if(keySym == "Left") input_idx_--;
	if(keySym == "Right") input_idx_++;
	
	input_cloud_str << folder_str_ << setfill('0')<<setw(5)<<input_idx_<<".pcd";
	pcl::io::loadPCDFile(input_cloud_str.str(), input_cloud);
	cout<<input_cloud.points.size()<<" loaded from "<<input_cloud_str.str()<<endl; 
	if(keySym == "Return"){
	  delete cm_;
	  input_clouds = append_input_cloud(input_cloud, front_end_str_, input_cloud_str.str());
	  cm_ = new CsmGPU<pcl::PointNormal>(0.1, cv::Point2d(50.0, 75.0), input_cloud, false);
	  state_ = SELECT_MATCHING_PCD;
	}
	input_cloud_ = input_cloud;
	addPointCloud(viewer, input_cloud_, "input_cloud", "input_normal_cloud", true);
	
	break;
      case SELECT_MATCHING_PCD:
	if(keySym == "Right") matching_idx_++;
	if(keySym == "Left") matching_idx_--;
	if(keySym == "m") matching_mode = !matching_mode;
	if(keySym == "Escape") state_ = SELECT_SOURCE_PCD;
	stringstream matching_file;
	fmutil::Stopwatch sw_loadFile("Load file");
	matching_file << folder_str_<<setfill('0')<<setw(5)<<matching_idx_<<".pcd";
	cout<<"Performing matching on "<<matching_file.str()<<endl;
	
	
	pcl::io::loadPCDFile(matching_file.str(), matching_cloud);
	sw_loadFile.end();
	if(matching_cloud.size() == 0) return;
	viewer->removePointCloud("matching_cloud");
	viewer->removePointCloud("matching_normal_cloud");
	cout<<matching_cloud.size()<<" matching points loaded. "<<endl;
	
	if(matching_mode) {
	  fmutil::Stopwatch sw("Matching time");
	  fmutil::Stopwatch sw2("Matching1");
	  poseResult result;
	  result.x = result.y = result.r = 0.0;
	  result = cm_->getBestMatch(2.0, 2.0, 5, 20, 20, 180, matching_cloud, result);
	  cout<<"Best score 1 found: "<< result.x<<", "<<result.y<<", "<<result.r<<": "<<result.score<<endl;
	  sw2.end(); fmutil::Stopwatch sw3("Matching2");
	  result = cm_->getBestMatch(0.5, 0.5, 2, 2, 2, 10, matching_cloud, result);
	  cout<<"Best score 2 found: "<< result.x<<", "<<result.y<<", "<<result.r<<": "<<result.score<<endl;
	  sw3.end();
	  fmutil::Stopwatch sw4("Matching3");
	  result = cm_->getBestMatch(0.1, 0.1, 0.5, 0.5, 0.5, 4.0, matching_cloud, result);
	  sw4.end();
	  sw.end();
	  cout<<"Best score 3 found: "<< result.x<<", "<<result.y<<", "<<result.r<<": "<<result.score<<endl;
	  pcl::PointCloud<pcl::PointNormal> cloud_out;
	  Eigen::Matrix4f transform;
	  double d = result.r/180*M_PI;
	  transform<<cos(d),-sin(d),0,result.x,
		      sin(d),cos(d),0,result.y,
		      0,0,1,0,
		      0,0,0,1;
	  pcl::transformPointCloudWithNormals (matching_cloud, cloud_out, transform);
	  addPointCloud(viewer, cloud_out, "matching_cloud",
		      "matching_normal_cloud", false);
	}
	else {
	  addPointCloud(viewer, matching_cloud, "matching_cloud",
		      "matching_normal_cloud", false);
	}
	break;
    }
  }
  sw_whole_vis.end();
}
#include <boost/thread/thread.hpp>
int main( int argc, char** argv ) {
    if(argc != 4){
      cout<<"Usage = csm frontEnd.txt input_idx matching_idx"<<endl; 
      return 1;
    }
    state_ = SELECT_SOURCE_PCD;
    
    
    //input_cloud = pcl_downsample(input_cloud, res*2, res*2, res*2);
    front_end_str_ = argv[1];
    size_t delimiter_pose = front_end_str_.find_first_of("/");
    if(delimiter_pose == string::npos) folder_str_ = "";
    else folder_str_ = front_end_str_.substr(0, delimiter_pose+1);
    input_idx_ = atoi(argv[2]);
    matching_idx_ = atoi(argv[3]);
    cout<<"folder="<<folder_str_<<endl; 
    cout<<"input_idx="<<input_idx_<<endl;
    cout<<"matching_idx="<<matching_idx_<<endl;
    
    pcl::PointCloud<pcl::PointNormal> input_cloud;
    stringstream input_cloud_str;
    input_cloud_str << folder_str_ << setfill('0')<<setw(5)<<input_idx_<<".pcd";
    pcl::io::loadPCDFile(input_cloud_str.str(), input_cloud);
    vector<pcl::PointCloud<pcl::PointNormal> > input_clouds = append_input_cloud(input_cloud, front_end_str_, input_cloud_str.str());
    cout<<input_cloud.points.size()<<" loaded. "<<endl; 
    input_cloud_ = input_cloud;
    cm_ = new CsmGPU<pcl::PointNormal>(0.1, cv::Point2d(50.0, 75.0), input_cloud, false);
     boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer =
            initVisualizer();
	    addPointCloud(viewer, input_cloud, "input_cloud",
                  "input_normal_cloud", true);
    viewer->registerKeyboardCallback(keyboardEventOccurred, (void*) &viewer);
    while (!viewer->wasStopped()) {
        viewer->spinOnce();
        //boost::this_thread::sleep(boost::posix_time::microseconds(100000));
    }
    return 0;
}