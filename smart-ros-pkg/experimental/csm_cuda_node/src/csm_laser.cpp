#include <fmutil/fm_stopwatch.h>
#include <csm_cuda/csm.h>
#include "save_laser_scan.h"

int main(int argc, char** argv){
  sensor_msgs::LaserScan laser_in = readROSFile<sensor_msgs::LaserScan>(string(argv[1]));
  sensor_msgs::LaserScan laser_match = readROSFile<sensor_msgs::LaserScan>(string(argv[2]));
  
  cout<<laser_in.ranges.size()<<" point loaded."<<endl;
  
  //just to initialize 
  
  /*4.5.2 Runtime API
    4.5.2.1 Initialization
    There is no explicit initialization function for the runtime API; 
    it initializes the first time a runtime function is called. 
    One needs to keep this in mind when timing runtime function calls 
    and when interpreting the error code from the first call into the runtime.
    */
  
  CsmGPU<pcl::PointXYZ> csmGPU_Init(0.1, cv::Point2d(80.0, 80.0), laser_in, false);
  fmutil::Stopwatch sw_init("Init");
  CsmGPU<pcl::PointXYZ> csmGPU(0.1, cv::Point2d(80.0, 80.0), laser_in, false);
  sw_init.end();
  fmutil::Stopwatch sw("Matching time");
  fmutil::Stopwatch sw2("Matching1");
  poseResult result;
  result.x = result.y = result.r = 0.0;
  result = csmGPU.getBestMatch(2.0, 2.0, 5, 10, 10, 90, laser_match, result);
  cout<<"Best score 1 found: "<< result.x<<", "<<result.y<<", "<<result.r<<": "<<result.score<<endl;
  sw2.end(); fmutil::Stopwatch sw3("Matching2");
  result = csmGPU.getBestMatch(0.5, 0.5, 2, 2, 2, 10, laser_match, result);
  cout<<"Best score 2 found: "<< result.x<<", "<<result.y<<", "<<result.r<<": "<<result.score<<endl;
  sw3.end();
  fmutil::Stopwatch sw4("Matching3");
  result = csmGPU.getBestMatch(0.1, 0.1, 0.5, 0.5, 0.5, 4.0, laser_match, result);
  sw4.end();
  sw.end();
  cout<<"Best score 3 found: "<< result.x<<", "<<result.y<<", "<<result.r<<": "<<result.score<<endl;
  
  return 0;
}
