#include <pcl/point_types.h>
#include <pcl/ros/conversions.h>
#include <pcl/io/pcd_io.h>
#include <cv.h>
#include <highgui.h>
#include <thrust/host_vector.h>
#include "boost/tuple/tuple.hpp"
#include "cpu_bitmap_grayscale.h"
#include "cuda_runtime_api.h"
#include "handle.h"

using namespace std;
extern void rasterMap(float *data_x, float *data_y, float min_x, float min_y, int data_size, int DIMW, int DIMH, 
		    float res, float *normals, int OSR, float *point_map, int *nn_index);
extern CPUBitmap rotateMap(int *input, float *data_x, float *data_y, int data_size,
		    int img_width, int img_height,
		    float min_x, float min_y, float max_x, float max_y, float res, float rot_res, float range_cov);
extern MatchScore findBestMatch(int *data_x, int *data_y, int *input_data_count, int data_size,
		   int raw_data_size, unsigned char *template_map, int img_width, int img_height,
		   float res, float deg_res, vector<int> x_step, vector<int> y_step, vector<int> rot_step,
		   int ini_r
 			);
struct Simple2D{
  double x, y, ang;
  double score;
};

struct simple2D{
  int x;
  int y;
  int index;
};

bool data_sort(simple2D s1, simple2D s2){
  return s1.index < s2.index;
}

class CorrelativeMatchGPU{
public:
  //Get input points as pcl and perform indexing from the 
  //specified resolution and range
  vector<Simple2D> res_, ranges_;
  vector<cv::Mat> templates_;
  double range_cov_;
  bool visualize_;
  CorrelativeMatchGPU(pcl::PointCloud<pcl::PointNormal> &input_cloud,
    vector<Simple2D> &res, vector<Simple2D> &ranges, double range_cov, bool visualize) :
    res_(res), ranges_(ranges), range_cov_(range_cov), visualize_(visualize)
  {
    assert(res_.size() == ranges_.size());
    
    double min_x(1e99), min_y(1e99), max_x(1e-99), max_y(1e-99);
    double dist = getMinMax(input_cloud, min_x, min_y, max_x, max_y);
    
    
    float x[input_cloud.points.size()], y[input_cloud.points.size()];
    float normals[input_cloud.points.size()];
    for(size_t i=0; i<input_cloud.points.size(); i++) {
      x[i] = input_cloud.points[i].x;
      y[i] = input_cloud.points[i].y;
      normals[i] = atan2(input_cloud.points[i].normal_y, input_cloud.points[i].normal_x);
    }
    //generate the index for each resolution
    double max_dist= sqrt(log(255) / range_cov_);
    
    for(size_t i=0; i<res_.size(); i++){
      assert(res_[i].x == res_[i].y);
      int gausian_length = (int) (max_dist / res_[i].x + 1);
      int DIMW = round(sqrt(dist)/res_[i].x)*2 + 2*gausian_length;
      int DIMH = DIMW;
      double offset_x = -DIMW/2.f*res_[i].x;
      double offset_y = -DIMH/2.f*res_[i].y;	
      
      //cout<<"map h = "<<DIMH<<" map w = "<<DIMW<<endl;
      //cout<<min_x<<" "<<min_y<<" "<<max_x<<" "<<max_y<<endl;
      min_x -= res_[i].x*gausian_length;
      min_y -= res_[i].y*gausian_length;
      max_x += res_[i].x*gausian_length;
      max_y += res_[i].y*gausian_length;
       //cout<<"Entering GPU"<<endl;
      int nn_index[DIMW*DIMH];
      float point_map[DIMW*DIMH];
      rasterMap(x, y, offset_x, offset_y, input_cloud.points.size(), DIMW, DIMH, res_[i].x, 
		normals, 1, point_map, nn_index);
      
      //cv::imwrite("master_template.png", template_map);
      //exit(0);
      
      CPUBitmap final_map = rotateMap(nn_index, x, y, input_cloud.points.size(),
		      DIMW, DIMH, min_x, min_y, max_x, max_y, res_[i].x, res_[i].ang, range_cov);
      

      //determine the minimum size of the collage
      cv::Mat template_map(DIMH, DIMW, CV_32FC1, point_map);
      cv::Mat template_nn(cv::Size(DIMH, DIMW), CV_32SC1, nn_index);
      cv::Mat template_nn_norm, template_map_norm;
      cv::normalize(template_nn, template_nn_norm, 0, 255, cv::NORM_MINMAX, CV_8UC1);
      cv::normalize(template_map, template_map_norm, 0, 255, cv::NORM_MINMAX, CV_8UC1);
      //template_map.data = point_map.pixels;
      if(visualize){
	cv::imshow("template nn", template_nn_norm);
	cv::waitKey(0);
      }
      cv::Mat final_mat(DIMH*final_map.rot_size, DIMW, CV_8UC1);
      for(int j=0; j<final_map.rot_size; j++){
	//cout<<"Show img "<<j<<endl;
	cv::Mat temp(DIMH, DIMW, CV_8UC1);
	
	temp.data = final_map.pixels+(DIMH*DIMW*j);
	if(visualize){
	  cv::imshow("temp pic", temp);
	  cv::waitKey(0);
	}
	temp.copyTo(final_mat(cv::Rect(0, j*DIMH, temp.cols, temp.rows)));
	
      }
      templates_.push_back(final_mat);
    }
  }
  
  Simple2D getBestMatch(pcl::PointCloud<pcl::PointNormal> &matching_cloud){
    Simple2D result;
    int data_size = matching_cloud.points.size();
    
    Simple2D best_match;
    best_match.x = 0; best_match.y = 0; best_match.ang = 0;
    for(int idx=0; idx<res_.size(); idx++){
      cv::Mat template_map = templates_[idx];
      float res = res_[idx].x;
      float deg_res = res_[idx].ang;
      int rot_step = 360/deg_res;
      int offset_x = template_map.cols/2;
      int offset_y = template_map.rows/(rot_step*2);
      //cout<<"Gridding the clouds with offset "<<offset_x<<" "<<offset_y<<"template_map.rows "<<template_map.rows<<endl;
      float score = 0;
      vector<simple2D> data(data_size);
      //cout<<"Initialized data"<<endl;
      for(int i=0; i<data_size; i++){
	data[i].x = round(matching_cloud.points[i].x/res)+offset_x;
	data[i].y = round(matching_cloud.points[i].y/res)+offset_y;
	if(data[i].x < offset_x*2 && data[i].y < offset_y*2){
	  int ptr = data[i].x + data[i].y*template_map.cols;
	  score+=template_map.data[ptr]/255.f;
	  
	  data[i].index = ptr;
	}
	else
	  data[i].index = template_map.cols*template_map.rows;
      }
      sort(data.begin(), data.end(), data_sort);
      //cout<<"Sort data"<<endl;
      vector<simple2D> filtered_data;
      vector<int> filtered_data_count;
      int last_idx = data[0].index;
      int data_count = 0;
      int raw_data_x[data_size];
      int raw_data_y[data_size];
      for(int i=0; i<data_size; i++){
	raw_data_x[i] = data[i].x;
	raw_data_y[i] = data[i].y;
	data_count++;
	if(data[i].index != last_idx){
	  filtered_data.push_back(data[i-data_count+1]);
	  filtered_data_count.push_back(data_count);
	  last_idx = data[i].index;
	  data_count= 0;
	}
      }
      int raw_input_data_count[data_size];
      for(int i=0; i<data_size; i++) raw_input_data_count[i]=1;
      filtered_data.push_back(data[data.size()-data_count]);
      filtered_data_count.push_back(data_count);
      //cout<<"Filtered data size = "<<filtered_data.size()<<endl;
      int data_x[filtered_data.size()];
      int data_y[filtered_data.size()];
      int input_data_count[filtered_data.size()];
      assert(filtered_data_count.size() == filtered_data.size());
      int check_data_count = 0;
      for(int i=0; i<filtered_data.size(); i++){
	data_x[i] = filtered_data[i].x;
	data_y[i] = filtered_data[i].y;
	input_data_count[i] = filtered_data_count[i];
	check_data_count += input_data_count[i];
	//cout<<data_x[i]<<" "<<data_y[i]<<" "<<input_data_count[i]<<endl;
      }
      //cout<<"Check data count = "<<check_data_count<<endl;
      assert(check_data_count == matching_cloud.points.size());
      //cout<<"CV type = "<<template_map.type()<<endl;
      
	  
      //cout<<"Gridded "<<" cpu score check = "<<score/data_size<<endl;
      
      
      int x_step = ranges_[idx].x/res;
      int y_step = ranges_[idx].y/res;
      float r_step = ranges_[idx].ang;
      int ini_x = best_match.x/res;
      int ini_y = best_match.y/res;
      float ini_r = best_match.ang;
      //cout<<"Performing x_step: "<<x_step<<" y_step: "<<y_step<<" r_step: "<<r_step<<endl;
      vector<int> x_steps, y_steps, r_steps;
      for(int i=-x_step; i<=x_step; i++){
	x_steps.push_back(i+ini_x);
	//cout<<x_steps[x_steps.size()-1]*res<<" ";
      }
      //cout<<endl;
      for(int j=-y_step; j<=y_step; j++) {
	y_steps.push_back(j+ini_y);
	//cout<<y_steps[y_steps.size()-1]*res<<" ";
      }
      //cout<<endl;
      for(float k=-r_step; k<=r_step; k+=deg_res)	{
	float ini_r_temp = k+ini_r;
	if(ini_r_temp < 0) ini_r_temp += 360;
	else if(ini_r_temp >= 360) ini_r_temp -= 360;
	//cout<<(int)ini_r_temp/deg_res<<" ";
	r_steps.push_back(ini_r_temp/deg_res);
      }
      //cout<<endl;
      
      MatchScore ms = findBestMatch(data_x, data_y,input_data_count, filtered_data.size(), matching_cloud.points.size(),
		    template_map.data, template_map.cols, template_map.rows, 
		    res, deg_res, x_steps, y_steps, r_steps, ini_r);
      best_match.x = ms.x*res;
      best_match.y = ms.y*res;
      best_match.ang = ms.rot*deg_res;
      best_match.score = ms.score/(255*matching_cloud.points.size());
      
      
    }
    return best_match;
  }
  
  
  double getMinMax(pcl::PointCloud<pcl::PointNormal> &input_cloud, double &min_x, double &min_y, 
	    double &max_x, double &max_y){
    
    double dist=0;
    for(size_t i=0; i<input_cloud.points.size(); i++) {
      double x = input_cloud.points[i].x;
      double y = input_cloud.points[i].y;
      double temp_dist = x*x + y*y;
      if(dist < temp_dist) {
	dist = temp_dist;
	max_x = x;
	max_y = y;
      }
      if(min_x > x) min_x = x;
      if(min_y > y) min_y = y;
      if(max_x < x) max_x = x;
      if(max_y < y) max_y = y;
    }
    return dist;
  }
};
