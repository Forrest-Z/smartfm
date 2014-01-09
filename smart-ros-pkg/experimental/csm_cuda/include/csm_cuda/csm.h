#include <opencv2/opencv.hpp>
#include <pcl/point_types.h>
#include <pcl/ros/conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl_ros/transforms.h>
#include <thrust/device_vector.h>

#define PRINT_DEBUG 0
using namespace std;
using namespace thrust;


struct poseResult{
  double x,y,r,score;
};


struct rowMajorData{
    int x, y, data_idx;
};
  
  
 
template <class T>
class CsmGPU{
  public:
    cv::Size voronoi_size_;
    double res_;
    device_vector<int> dev_voronoi_data_;
    device_vector<int> dev_px_, dev_py_;

    CsmGPU(double res, cv::Point2d template_size, 
	    pcl::PointCloud<T> &cloud, bool visualize);
    
    poseResult getBestMatch(double x_step, double y_step, double r_step,
		      double x_range, double y_range, double r_range,
		      pcl::PointCloud<T> &matching_pts,
		      poseResult offset
			  );
	
    poseResult getBestTranslation(double x_step, double y_step, 
			    double x_range, double y_range,
			    pcl::PointCloud<T> &matching_pts, int stream_idx);
  private:
    
  
  cv::Mat removeRepeatedPts(pcl::PointCloud<T> &cloud,
			  vector<int> &seeds_x, vector<int> &seeds_y);
  
  void visualize_data(host_vector<int> voronoi_data, string name);


  cv::Vec3b HSVtoRGB(float hue, float sat, float value);


};

#include "../../src/csm.cpp"