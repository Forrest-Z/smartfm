#include <opencv2/opencv.hpp>
#include <pcl/point_types.h>
#include <pcl/ros/conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl_ros/transforms.h>
#include <thrust/device_vector.h>
#include <laser_geometry/laser_geometry.h>
#include <csm_cuda/clipper.h>
#include <csm_cuda/cudaVarType.h>
#define PRINT_DEBUG 0
using namespace std;
using namespace thrust;


struct poseResult{
  double x,y,r,score;
};


struct rowMajorData{
  int x, y, data_idx;
  float normal_x, normal_y;
};

template <class T>
class CsmGPU{
  public:
    cv::Size voronoi_size_;
    double res_;
    device_vector<int> dev_voronoi_data_;
    device_vector<cudaPointNormal> dev_p_;
    device_vector<float> dev_free_space_;
    CsmGPU(double res, cv::Point2d template_size, 
	    pcl::PointCloud<T> &cloud, bool visualize);
    CsmGPU(double res, cv::Point2d template_size, 
	    sensor_msgs::LaserScan &cloud, bool visualize);
    
    void drawFreeSpace(sensor_msgs::PointCloud &cloud, cv::Size template_size, bool try_param);
    void getVoronoiTemplate(pcl::PointCloud<T>& data, bool visualize);
	
    poseResult getBestMatch(double x_step, double y_step, double r_step,
		      double x_range, double y_range, double r_range,
		      sensor_msgs::LaserScan &matching_scan,
		      poseResult offset
			  );
    poseResult getBestTranslation(double x_step, double y_step, 
			    double x_range, double y_range,
			    pcl::PointCloud<T> &matching_pts, int stream_idx);
      
  poseResult getBestMatch(double x_step, double y_step, double r_step,
		    double x_range, double y_range, double r_range,
		    pcl::PointCloud<T> &matching_pts,
		    poseResult offset,void(*transformFunc)(const pcl::PointCloud< T > &, pcl::PointCloud< T > &, const Eigen::Matrix4f & ));

  ~CsmGPU();  
  private:
    bool gotNormal_;
    cv::Mat removeRepeatedPts(pcl::PointCloud<T> &cloud,
			    vector<cudaPointNormal> &seeds);
    
    void visualize_data(host_vector<int> voronoi_data, string name);

    cv::Vec3b HSVtoRGB(float hue, float sat, float value);


};

#include "../../src/csm.cpp"