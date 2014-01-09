#include <stdio.h>
#include "algorithm"
#include "vector"
#include "iostream"
#include "math.h"
#include <thrust/host_vector.h>
#include <thrust/device_vector.h>

#define PRINT_DEBUG 0
using namespace std;
using namespace thrust;
texture<int> tex_dev_voronoi_data_;
#define stream_number 4
cudaStream_t stream_array[stream_number];
struct poseResult{
  double x,y,r,score;
};

__global__ void jfaKernel(int step, int voronoi_width, int voronoi_height, int *px, int *py, int *voronoi_data){
  int i = threadIdx.x + blockIdx.x * blockDim.x;
  int j = threadIdx.y + blockIdx.y * blockDim.y;
  if(i>=0 && j>=0 && i<voronoi_width && j<voronoi_height){
    int k=step;
    int offset = i + j*voronoi_width;
    int id = voronoi_data[offset];
    int nearest_id_candidates[9];
    int total_candidate = 0;
    if(id>0){
      nearest_id_candidates[total_candidate++] = id;
    }
    for(int step_x = i-k; step_x < i+k+1; step_x += k){
      for(int step_y = j-k; step_y < j+k+1; step_y += k){
	if(step_x < 0 || step_y < 0 || step_x >= voronoi_width || step_y >= voronoi_height)
	  continue;
	int offset_temp = step_x + step_y * voronoi_width;
	int nearest_seed = voronoi_data[offset_temp];
	if(nearest_seed > 0) 
	  nearest_id_candidates[total_candidate++] = nearest_seed;
      }
    }
    if(total_candidate > 0){
      int nearest_x = px[nearest_id_candidates[0]-1];
      int nearest_y = py[nearest_id_candidates[0]-1];
      
      int dist_nearest_x = nearest_x-i;
      int dist_nearest_y = nearest_y-j;
      int dist_nearest = dist_nearest_x*dist_nearest_x + dist_nearest_y*dist_nearest_y;
      int nearest_id_current = nearest_id_candidates[0];
      for(size_t seed_i=1; seed_i<total_candidate; seed_i++){
	nearest_x = px[nearest_id_candidates[seed_i]-1];
	nearest_y = py[nearest_id_candidates[seed_i]-1];
	dist_nearest_x = nearest_x-i;
	dist_nearest_y = nearest_y-j;
	int dist_nearest_cur = dist_nearest_x*dist_nearest_x + dist_nearest_y*dist_nearest_y;
	if(dist_nearest_cur < dist_nearest){
	  nearest_id_current = nearest_id_candidates[seed_i];
	  dist_nearest = dist_nearest_cur;
	}
      }
      voronoi_data[offset] = nearest_id_current;
    }
  }
}
__global__ void translationKernel(float res, float step_size, int voronoi_width, int voronoi_height, 
				 int x_step, int y_step, int x_range, int match_size,
				 int y_range, int* match_x, int* match_y, 
				 int* px, int* py, float* scores){
  //check index
  //single thread dimension with xy grid for each offset
  //maximum thread is limited to 1024, hence need to call this function repeatedly
  //get the real offset in pixel
  //the range will be -range to range at step interval
  int offset_x = -x_range/2 + x_step * blockIdx.x;
  int offset_y = -y_range/2 + y_step * blockIdx.y;
  
  extern __shared__ float scores_per_block[];
  int match_idx = threadIdx.x;
  scores_per_block[match_idx] = 0.0;
  if(match_idx < match_size){
    //get pixel location of the matching xy
    int temp_x = match_x[match_idx];
    int temp_y = match_y[match_idx];
    int x = temp_x + offset_x;
    int y = temp_y + offset_y;
    if(x < voronoi_width && x >= 0 && y < voronoi_height && y >= 0){
      int grid_idx = x+y*voronoi_width;
      int nearest_seed = tex1Dfetch(tex_dev_voronoi_data_, grid_idx)-1;
      int nearest_x = px[nearest_seed];
      int nearest_y = py[nearest_seed];
      int dist_nearest_x = nearest_x - x;
      int dist_nearest_y = nearest_y - y;
      int dist_nearest_pixel = (int)(sqrtf((dist_nearest_x*dist_nearest_x + dist_nearest_y*dist_nearest_y)*res)/step_size);
      float dist_nearest = dist_nearest_pixel * step_size;
      scores_per_block[match_idx] = expf(-0.3*dist_nearest);
    }
  }
  
  __syncthreads();
  int nTotalThreads = blockDim.x;
  int thread2;
  while(nTotalThreads > 1){
    int halfPoint = (nTotalThreads >> 1);
    
    if(threadIdx.x < halfPoint){
      thread2 = threadIdx.x + halfPoint;
      

      if(thread2 < blockDim.x){
	scores_per_block[threadIdx.x] += scores_per_block[thread2];
      }
    }
    __syncthreads();
    
    nTotalThreads = halfPoint;
  }
  if(threadIdx.x == 0){
    int score_idx = blockIdx.x + blockIdx.y * gridDim.x; 
    scores[score_idx] += scores_per_block[0];
  }
}

//form a euclidean based voronoi and return a device pointer of the resultant img
host_vector<int> voronoi_jfa(int voronoi_width, int voronoi_height, 
		 vector<int> point_x, vector<int> point_y,
		 device_vector<int> &dev_px, device_vector<int> &dev_py,
		 device_vector<int> &dev_voronoi_data
		){
  if(PRINT_DEBUG)
    cout<<"Received pts "<<point_x.size()<<"x"<<point_y.size()<<endl;
  host_vector<int> voronoi_data(voronoi_width*voronoi_height);
  for(size_t i=0; i<point_x.size(); i++)
    voronoi_data[point_x[i] + point_y[i]*voronoi_width] = i+1;
  dev_px = device_vector<int>(point_x);
  dev_py = device_vector<int>(point_y);
  dev_voronoi_data = device_vector<int>(voronoi_data);
  int *dev_px_ptr = raw_pointer_cast(dev_px.data());
  int *dev_py_ptr = raw_pointer_cast(dev_py.data());
  int *dev_voronoi_data_ptr = raw_pointer_cast(dev_voronoi_data.data());
  dim3 grids(ceil(voronoi_width/16.f),ceil(voronoi_height/16.f));
  dim3 threads(16,16);
  for(int k=512; k>0; k/=2)
    jfaKernel<<<grids,threads>>>(k, voronoi_width, voronoi_height, dev_px_ptr, dev_py_ptr, dev_voronoi_data_ptr);
  voronoi_data = dev_voronoi_data;
  
  cudaBindTexture(NULL, tex_dev_voronoi_data_, dev_voronoi_data_ptr, dev_voronoi_data.size()*sizeof(int));
  for(int i=0; i<stream_number; i++) cudaStreamCreate(&stream_array[i]);
  return voronoi_data;
}

poseResult best_translation(float resolution, float step_size, int x_step, int y_step, int x_range, int y_range,
  vector<int> point_x, vector<int> point_y,
  int voronoi_width, int voronoi_height,
  device_vector<int> &dev_px, device_vector<int> &dev_py,
  device_vector<int> &dev_voronoi_data, int stream_idx
		    )
{
  //step size will determine the resoution used when calculating
  //the distance value
  //Essentially the real distance will be passed through a step function
  //where each height of the step depends on the stepping resolution.
  //This helps to over estimate the cost function to ensure no
  //local minima
  
  //range from -_range to _range
  x_range *= 2;
  y_range *= 2;
  int total_scores = x_range/x_step * y_range/y_step;
  if(PRINT_DEBUG)
  cout<<"Total scores to be generated "<<total_scores <<
  "(range:"<<x_range<<"x"<<y_range<<", step:"<<x_step<<"x"<<y_step<<")"<<endl;
  vector<float> host_scores(total_scores, 0.0);
  float *dev_scores_ptr;
  cudaMalloc(&dev_scores_ptr, total_scores*sizeof(float));
  cudaMemcpyAsync(dev_scores_ptr, &host_scores[0], total_scores*sizeof(float), cudaMemcpyHostToDevice, stream_array[stream_idx]);
 // host_vector<float> host_scores;
  vector<int> match_x, match_y;
  int *dev_px_ptr = raw_pointer_cast(dev_px.data());
  int *dev_py_ptr = raw_pointer_cast(dev_py.data());
  int *dev_match_x_ptr, *dev_match_y_ptr;
  cudaMalloc(&dev_match_x_ptr, 1024*sizeof(int));
  cudaMalloc(&dev_match_y_ptr, 1024*sizeof(int));
  //float *dev_scores_ptr = raw_pointer_cast(dev_scores.data());
  

  
  for(int i=0; i<point_x.size(); i++){
    int match_size;
    if(point_x[i]>=0 && point_y[i]>=0 && point_x[i]<voronoi_width && point_y[i]<voronoi_height){
      match_x.push_back(point_x[i]);
      match_y.push_back(point_y[i]);
    }
    else continue;
    match_size = match_x.size();
    if(match_size == 1024 || i == point_x.size()-1){
      cudaMemcpyAsync(dev_match_x_ptr, &match_x[stream_idx], match_size*sizeof(int), cudaMemcpyHostToDevice, stream_array[stream_idx]);
      cudaMemcpyAsync(dev_match_y_ptr, &match_y[stream_idx], match_size*sizeof(int), cudaMemcpyHostToDevice, stream_array[stream_idx]);
      dim3 grids(x_range/x_step,y_range/y_step);
      dim3 threads(1024);
      if(PRINT_DEBUG){
	cout<<"Grid size of "<<grids.x<<"x"<<grids.y<<" initiated."<<endl;
	cout<<"Thread size of "<<match_size<<" used."<<endl;
      }
      translationKernel<<<grids, threads, threads.x*sizeof(float), stream_array[stream_idx]>>>
				    (resolution, step_size,
				      voronoi_width, voronoi_height, 
				    x_step, y_step, x_range, match_size,
				    y_range, dev_match_x_ptr, dev_match_y_ptr, 
				    dev_px_ptr, dev_py_ptr, dev_scores_ptr);
      match_x.clear();
      match_y.clear();
    }
  }
  //cudaDeviceSynchronize();
  cudaMemcpyAsync(&host_scores[0], dev_scores_ptr, total_scores*sizeof(float), cudaMemcpyDeviceToHost, stream_array[stream_idx]);
  int max_x, max_y;
  float max_value=0;
  
  for(int i=0; i<x_range/x_step; i++){
    //if(i>5) continue;
    for(int j=0; j<y_range/y_step; j++)
    {
      float temp_score = host_scores[i+j*x_range/x_step];
      //cout<<temp_score<<" ";
      if(max_value < temp_score){
	max_value = temp_score;
	max_x = i;
	max_y = j;
      }
    }
    //cout<<endl;
  }
  if(PRINT_DEBUG){
    cout<<"Best match: "<<(max_x-x_range/x_step/2)*x_step*resolution<<","<<(max_y-y_range/y_step/2)*y_step*resolution<<"("<<max_value/(double)point_x.size()*100<<")"<<endl;
    cout<<"Best match: "<<max_x<<","<<max_y<<"("<<max_value<<")"<<endl;
  }
  poseResult pose_result;
  pose_result.x = (max_x-x_range/x_step/2)*x_step*resolution;
  pose_result.y = (max_y-y_range/y_step/2)*y_step*resolution;
  pose_result.score = max_value/(double)point_x.size()*100;
  return pose_result;
}
