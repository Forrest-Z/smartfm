#include <fmutil/fm_stopwatch.h>
#include <csm_cuda/csm.h>

int main(int argc, char** argv){
  pcl::PointCloud<pcl::PointNormal> cloud, cloud2;
  pcl::io::loadPCDFile("/home/demian/smartfm/smart-ros-pkg/experimental/csm_cuda_node/examples_pcd/00135.pcd", cloud);
  pcl::io::loadPCDFile("/home/demian/smartfm/smart-ros-pkg/experimental/csm_cuda_node/examples_pcd/00556.pcd", cloud2);
  cout<<cloud.size()<<" point loaded."<<endl;
  
  //just to initialize 
  
  /*4.5.2 Runtime API
    4.5.2.1 Initialization
    There is no explicit initialization function for the runtime API; 
    it initializes the first time a runtime function is called. 
    One needs to keep this in mind when timing runtime function calls 
    and when interpreting the error code from the first call into the runtime.
    */
  if(true)
    CsmGPU<pcl::PointNormal> csmGPU_Init(0.1, cv::Point2d(50.0, 75.0), cloud, false);
  
  
  fmutil::Stopwatch sw("voronoi_construction");
  CsmGPU<pcl::PointNormal> csmGPU(0.1, cv::Point2d(50.0, 75.0), cloud, false);
  sw.end();
  fmutil::Stopwatch sw1("Overall");
  fmutil::Stopwatch sw2("Matching1");
  poseResult result;
  result.x = result.y = result.r = 0.0;
  cout<<cloud2.size()<<" matching size loaded."<<endl;
  result = csmGPU.getBestMatch(2.0, 2.0, 5, 20, 20, 180, cloud2, result);
  cout<<"Best score 1 found: "<< result.x<<", "<<result.y<<", "<<result.r<<": "<<result.score<<endl;
  sw2.end(); fmutil::Stopwatch sw3("Matching2");
  result = csmGPU.getBestMatch(0.5, 0.5, 2, 2, 2, 10, cloud2, result);
  cout<<"Best score 2 found: "<< result.x<<", "<<result.y<<", "<<result.r<<": "<<result.score<<endl;
  sw3.end();
  fmutil::Stopwatch sw4("Matching3");
  result = csmGPU.getBestMatch(0.1, 0.1, 0.5, 0.5, 0.5, 4.0, cloud2, result);
  sw4.end();
  cout<<"Best score 3 found: "<< result.x<<", "<<result.y<<", "<<result.r<<": "<<result.score<<endl;
  sw1.end();
  fmutil::Stopwatch swTfCloud("Transform cloud");
  pcl::PointCloud<pcl::PointNormal> cloud_out;
  Eigen::Matrix4f transform;
  double d = result.r/180*M_PI;
  transform<<cos(d),-sin(d),0,result.x,
	      sin(d),cos(d),0,result.y,
	      0,0,1,0,
	      0,0,0,1;
  //cout<<r<<"deg ="<<endl<<transform<<endl;
  stringstream ss;
  ss<<"result.pcd";
  pcl::transformPointCloudWithNormals (cloud2, cloud_out, transform);
  swTfCloud.end();
  pcl::io::savePCDFileASCII(ss.str(), cloud_out);
  return 0;
}
