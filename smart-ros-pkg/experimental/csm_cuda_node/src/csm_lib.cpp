#include <fmutil/fm_stopwatch.h>
#include <csm_cuda/csm.h>


int main(int argc, char** argv){
  pcl::PointCloud<pcl::PointNormal> cloud, cloud2;
  pcl::io::loadPCDFile(argv[1], cloud);
  pcl::io::loadPCDFile(argv[2], cloud2);
  cout<<cloud.size()<<" point loaded."<<endl;
  fmutil::Stopwatch sw("voronoi_construction");
  
  CsmGPU<pcl::PointNormal> csmGPU(0.1, cv::Point2d(50.0, 75.0), cloud, false);
  sw.end();
  fmutil::Stopwatch sw1("Overall");
  fmutil::Stopwatch sw2("Matching1");
  poseResult result;
  result.x = result.y = result.r = 0.0;
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
