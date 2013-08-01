#include "handle.h"
#include "cpu_bitmap_grayscale.h"
#include "fm_stopwatch.h"
#include "algorithm"
#include "vector"
#include "iostream"
#include "math.h"

#include <thrust/device_vector.h>
#include <thrust/sort.h>

#define PI 3.14159265359
using namespace std;

__global__ void initImg(unsigned char *ptr, int size, unsigned char init_value=0){
    int x = threadIdx.x + blockIdx.x * blockDim.x;
    int y = threadIdx.y + blockIdx.y * blockDim.y;
    int offset = x + y * blockDim.x * gridDim.x;
    if(offset < size) {
        ptr[offset] =init_value;
    }
}
__global__ void initImg(int *ptr, int size, int init_value=0){
    int x = threadIdx.x + blockIdx.x * blockDim.x;
    int y = threadIdx.y + blockIdx.y * blockDim.y;
    int offset = x + y * blockDim.x * gridDim.x;
    if(offset < size) {
        ptr[offset] =init_value;
    }
}
__global__ void initImg(float *ptr, int size, float init_value=0){
    int x = threadIdx.x + blockIdx.x * blockDim.x;
    int y = threadIdx.y + blockIdx.y * blockDim.y;
    int offset = x + y * blockDim.x * gridDim.x;
    if(offset < size) {
        ptr[offset] =init_value;
    }
}

__global__ void drawTemplatePixelBased(float *data_x, float *data_y, 
				       float min_x, float min_y, float res, 
				       float *ptr, int *ptr_index, 
				       int img_width, int img_height){
  int pt_x = threadIdx.x + blockIdx.x * blockDim.x;
  int pt_y = threadIdx.y + blockIdx.y * blockDim.y;
  
  if(pt_x < img_width && pt_y <img_height){
    float x = pt_x*res+min_x;
    float y = pt_y*res+min_y;
    
    int i=blockIdx.z;
    
    float raw_data_x = data_x[i] - x;
    float raw_data_y = data_y[i] - y;
    float dist = raw_data_x*raw_data_x+raw_data_y*raw_data_y;
    
    
    int offset = pt_x + pt_y * img_width;
    if(ptr[offset] > dist) {
      ptr[offset] = dist;
      ptr_index[offset] = i;
    }
  }
}
__global__ void matchTemplateDirect(
  				    /************template data***************/
				    int *data_idx, float *data_x_template, float *data_y_template,
				    float min_x, float min_y, float max_x, float max_y,
				    int img_width, int img_height,
				    /************matching data**************/
				    float *data_x, float *data_y, int data_size, 
				    /**********output scores data***********/
				    float *scores, float *score_x, float *score_y, int *score_rot,
				    int x_size, int y_size, 
				    /**********matching spec****************/
				    float *x_steps, float *y_steps, float res,
				    /***********cached data*****************/
				    float *sin_data, float *cos_data
				    
 				  ){
  int offset_x = threadIdx.x + blockIdx.x*blockDim.x;
  int offset_y = threadIdx.y + blockIdx.y*blockDim.y;
  int offset_rot = threadIdx.z + blockIdx.z*blockDim.z;
  if(offset_x<x_size && offset_y<y_size && offset_rot < 180){
    int score_ptr = offset_x+offset_y*x_size+offset_rot*y_size*x_size;
    float score = 0;
    float sin = sin_data[offset_rot];
    float cos = cos_data[offset_rot];
    float center_offset_x =x_steps[offset_x];
    float center_offset_y =y_steps[offset_y];
    float img_width_p2 = img_width/2.f;
    float img_height_p2 = img_height/2.f;
    for(int i=0; i<data_size; i++){
      float pt_x_f = data_x[i];
      float pt_y_f = data_y[i];
      float ref_pt_x_f = pt_x_f * cos + pt_y_f * sin;
      float ref_pt_y_f = pt_y_f * cos - pt_x_f * sin;
      ref_pt_x_f += center_offset_x;
      ref_pt_y_f += center_offset_y;
      if(ref_pt_x_f < max_x && ref_pt_x_f > min_x && ref_pt_y_f < max_y && ref_pt_y_f > min_y)
      { 
	int ref_pt_x = roundf((ref_pt_x_f/res+img_width_p2));
	int ref_pt_y = roundf((ref_pt_y_f/res+img_height_p2));
	int ref_offset = ref_pt_x + ref_pt_y * img_width;
	int input_idx = data_idx[ref_offset];
	float x = data_x_template[input_idx];
	float y = data_y_template[input_idx];
	float dist_x = x - ref_pt_x_f;
	float dist_y = y - ref_pt_y_f;
	float d = sqrtf(dist_x * dist_x + dist_y * dist_y);
	score += expf(-d / 1.0);
      }
    }
    scores[score_ptr] = score;
    score_x[score_ptr] = center_offset_x;
    score_y[score_ptr] = center_offset_y;
    score_rot[score_ptr] = offset_rot;
  }
}

void findBestMatchDirect(
			  /************template data***************/
			  int *data_idx, float *data_x_template, float *data_y_template,
			  float min_x, float min_y, float max_x, float max_y,
			  int img_width, int img_height, int data_template_size,
			  /************matching data**************/
			  float *data_x, float *data_y, int data_size, 
			  /**********matching spec****************/
			  float x_range, float y_range, float res
			  ){
  cudaEvent_t start, stop;
  HANDLE_ERROR(cudaEventCreate(&start));
  HANDLE_ERROR(cudaEventCreate(&stop));
  HANDLE_ERROR(cudaEventRecord(start, 0));
  int range_degree = 180;
  float cos_data[range_degree], sin_data[range_degree];
  for(int i=0; i<range_degree; i++){
    cos_data[i] = cos(PI/90*i);
    sin_data[i] = sin(PI/90*i);
  }
  //make sure all variables declared, now precalculating the range_x and range_y
  vector<float> x_steps, y_steps;
  for(float i=-x_range; i<=x_range; i+=res)	x_steps.push_back(i);
  for(float j=-y_range; j<=y_range; j+=res)	y_steps.push_back(j);
   
  int *dev_input;
  float *dev_data_x_template, *dev_data_y_template;
  float *dev_sin, *dev_cos, *dev_data_x, *dev_data_y, *dev_x_steps, *dev_y_steps;
  float *host_x_steps = &x_steps[0];
  float *host_y_steps = &y_steps[0];
  int size_of_scores = x_steps.size()*y_steps.size()*range_degree;
  cout<<"Size of scores = "<<size_of_scores<<endl;
  float *dev_scores;
  float *scores = new float[size_of_scores];
  float *dev_score_x, *dev_score_y;
  int *dev_score_rot;
  float *score_x = new float[size_of_scores];
  float *score_y = new float[size_of_scores];
  int *score_rot = new int[size_of_scores];
  
  HANDLE_ERROR(cudaMalloc((void**)&dev_scores, size_of_scores*sizeof(float)));
  HANDLE_ERROR(cudaMalloc((void**)&dev_score_x, size_of_scores*sizeof(float)));
  HANDLE_ERROR(cudaMalloc((void**)&dev_score_y, size_of_scores*sizeof(float)));
  HANDLE_ERROR(cudaMalloc((void**)&dev_score_rot, size_of_scores*sizeof(int)));
  HANDLE_ERROR(cudaMalloc((void**)&dev_input, img_width*img_height*sizeof(int)));
  HANDLE_ERROR(cudaMalloc((void**)&dev_data_x_template, data_template_size*sizeof(int)));
  HANDLE_ERROR(cudaMalloc((void**)&dev_data_y_template, data_template_size*sizeof(int)));
  HANDLE_ERROR(cudaMalloc((void**)&dev_data_x, data_size*sizeof(int)));
  HANDLE_ERROR(cudaMalloc((void**)&dev_data_y, data_size*sizeof(int)));
  HANDLE_ERROR(cudaMalloc((void**)&dev_cos, range_degree*sizeof(float)));
  HANDLE_ERROR(cudaMalloc((void**)&dev_sin, range_degree*sizeof(float)));
  HANDLE_ERROR(cudaMalloc((void**)&dev_x_steps, x_steps.size()*sizeof(float)));
  HANDLE_ERROR(cudaMalloc((void**)&dev_y_steps, y_steps.size()*sizeof(float)));
  
  HANDLE_ERROR(cudaMemcpy(dev_input, data_idx, img_width*img_height*sizeof(int), cudaMemcpyHostToDevice));
  HANDLE_ERROR(cudaMemcpy(dev_data_x_template, data_x_template, data_template_size*sizeof(float), cudaMemcpyHostToDevice));
  HANDLE_ERROR(cudaMemcpy(dev_data_y_template, data_y_template, data_template_size*sizeof(float), cudaMemcpyHostToDevice));
  HANDLE_ERROR(cudaMemcpy(dev_data_x, data_x, data_size*sizeof(float), cudaMemcpyHostToDevice));
  HANDLE_ERROR(cudaMemcpy(dev_data_y, data_y, data_size*sizeof(float), cudaMemcpyHostToDevice));
  HANDLE_ERROR(cudaMemcpy(dev_cos, cos_data, range_degree*sizeof(float), cudaMemcpyHostToDevice));
  HANDLE_ERROR(cudaMemcpy(dev_sin, sin_data, range_degree*sizeof(float), cudaMemcpyHostToDevice));
  HANDLE_ERROR(cudaMemcpy(dev_x_steps, host_x_steps, x_steps.size()*sizeof(float), cudaMemcpyHostToDevice));
  HANDLE_ERROR(cudaMemcpy(dev_y_steps, host_y_steps, y_steps.size()*sizeof(float), cudaMemcpyHostToDevice));
  
  int threadx = 8, thready = 8, threadrot = 4;
  dim3 threads(threadx,thready,threadrot);
  dim3 grids(ceil(x_steps.size()/(float)threadx), ceil(y_steps.size()/(float)thready), ceil(range_degree/(float)threadrot));
  std::cout<<"Running cuda with grid size of "<<grids.x<<","<<grids.y<<","<<grids.z<< " matching "<<data_size<<endl;
  
  matchTemplateDirect<<<grids, threads>>>(
  				    /************template data***************/
				    dev_input, dev_data_x_template, dev_data_y_template,
				    min_x, min_y, max_x, max_y,
				    img_width, img_height,
				    /************matching data**************/
				    dev_data_x, dev_data_y, data_size, 
				    /**********output scores data***********/
				    dev_scores, dev_score_x, dev_score_y, dev_score_rot,
				    x_steps.size(), y_steps.size(), 
				    /**********matching spec****************/
				    dev_x_steps, dev_y_steps, res,
				    /***********cached data*****************/
				    dev_sin, dev_cos
				    
 				  );
  HANDLE_ERROR(cudaMemcpy(scores, dev_scores, size_of_scores*sizeof(float), cudaMemcpyDeviceToHost));
  HANDLE_ERROR(cudaMemcpy(score_x, dev_score_x, size_of_scores*sizeof(float), cudaMemcpyDeviceToHost));
  HANDLE_ERROR(cudaMemcpy(score_y, dev_score_y, size_of_scores*sizeof(float), cudaMemcpyDeviceToHost));
  HANDLE_ERROR(cudaMemcpy(score_rot, dev_score_rot, size_of_scores*sizeof(int), cudaMemcpyDeviceToHost));
  MatchScoref ms;
  ms.score = 0;
  for(int i=0; i<size_of_scores; i++){
    if(ms.score < scores[i]){
      ms.score = scores[i];
      ms.x = score_x[i];
      ms.y = score_y[i];
      ms.rot = score_rot[i];
    }
  }
  HANDLE_ERROR(cudaEventRecord(stop,0));
  HANDLE_ERROR(cudaEventSynchronize(stop));
  float elapsedTime;
  HANDLE_ERROR(cudaEventElapsedTime(&elapsedTime,
                                        start, stop));
  printf( "Time to generate:  %3.1f ms\n", elapsedTime);
  printf("best match %.2f, %.2f, %d, %f\n", ms.x, ms.y, ms.rot, ms.score/(data_size));
  
  
  
}

__global__ void rotateTemplate(int *input, float *data_x, float *data_y, int data_size, 
			       int img_width, int img_height, float *sin_data, float *cos_data, 
			       unsigned char *dev_bitmap, float range_cov, 
			       float min_x, float min_y, float max_x, float max_y, float res
			      ){
  int pt_x = threadIdx.x + blockIdx.x * blockDim.x;
  int pt_y = threadIdx.y + blockIdx.y * blockDim.y;
  if(pt_x < img_width && pt_y < img_height){
    float pt_x_f = pt_x - img_width/2.;
    float pt_y_f = pt_y - img_height/2.;

    int rot_idx = blockIdx.z;
    float sin = sin_data[rot_idx];
    float cos = cos_data[rot_idx];
 
    float ref_pt_x_f = pt_x_f * cos + pt_y_f * sin;
    float ref_pt_y_f = pt_y_f * cos - pt_x_f * sin;
    max_x/=res;
    max_y/=res;
    min_x/=res;
    min_y/=res;

    if(ref_pt_x_f < max_x && ref_pt_x_f > min_x && ref_pt_y_f < max_y && ref_pt_y_f > min_y)
    { 
      int ref_pt_x = roundf((ref_pt_x_f+img_width/2.));
      int ref_pt_y = roundf((ref_pt_y_f+img_height/2.));
      int ref_offset = ref_pt_x + ref_pt_y * img_width;
      int input_idx = input[ref_offset];
      if(input_idx < data_size && input_idx > 0){
        float x = data_x[input_idx];
        float y = data_y[input_idx];
        float dist_x = x - ref_pt_x_f*res;
        float dist_y = y - ref_pt_y_f*res;
        int offset = pt_x + (pt_y * img_width) + (img_width*img_height*rot_idx);
        
        float d = sqrtf(dist_x * dist_x + dist_y * dist_y);
        int gaussian_value = (int) (255 * expf(-d / range_cov));
        dev_bitmap[offset] = gaussian_value;
      }
    }
  }
}



CPUBitmap rotateMap(int *input, float *data_x, float *data_y, int data_size,
		    int img_width, int img_height, vector<float> &cos_data_vec, vector<float> &sin_data_vec,
		    float min_x, float min_y, float max_x, float max_y,
		    float res, float rot_res, float range_cov, unsigned char* dev_bitmap){
  cudaEvent_t start, stop;
  HANDLE_ERROR(cudaEventCreate(&start));
  HANDLE_ERROR(cudaEventCreate(&stop));
  HANDLE_ERROR(cudaEventRecord(start, 0));
  
  
  int range_degree = cos_data_vec.size();
  //printf("Generating templates at %f deg with size of %d\n", rot_res, range_degree);
  float *cos_data = &cos_data_vec[0];
  float *sin_data = &sin_data_vec[0];
  int *dev_input;
  float *dev_sin, *dev_cos, *dev_data_x, *dev_data_y;
  
  
  CPUBitmap bitmap(img_width, img_height*range_degree);
  HANDLE_ERROR(cudaMalloc((void**)&dev_input, img_width*img_height*sizeof(int)));
  HANDLE_ERROR(cudaMalloc((void**)&dev_data_x, data_size*sizeof(float)));
  HANDLE_ERROR(cudaMalloc((void**)&dev_data_y, data_size*sizeof(float)));
  
  HANDLE_ERROR(cudaMalloc((void**)&dev_cos, range_degree*sizeof(float)));
  HANDLE_ERROR(cudaMalloc((void**)&dev_sin, range_degree*sizeof(float)));
  dim3 init_grid(ceil(img_width/16.f), ceil(img_height/16.f)*range_degree);
  initImg<<<init_grid, dim3(16,16)>>>(dev_bitmap, img_width*img_height*range_degree);
  HANDLE_ERROR(cudaMemcpy(dev_input, input, img_width*img_height*sizeof(int), cudaMemcpyHostToDevice));
  HANDLE_ERROR(cudaMemcpy(dev_data_x, data_x, data_size*sizeof(float), cudaMemcpyHostToDevice));
  HANDLE_ERROR(cudaMemcpy(dev_data_y, data_y, data_size*sizeof(float), cudaMemcpyHostToDevice));
  HANDLE_ERROR(cudaMemcpy(dev_cos, cos_data, range_degree*sizeof(float), cudaMemcpyHostToDevice));
  HANDLE_ERROR(cudaMemcpy(dev_sin, sin_data, range_degree*sizeof(float), cudaMemcpyHostToDevice));
  
  dim3 grids(ceil(img_width/16.f), ceil(img_height/16.f), range_degree);
  dim3 threads(16,16);
  rotateTemplate<<<grids, threads>>>(dev_input, dev_data_x, dev_data_y, data_size, 
			       img_width, img_height, dev_sin, dev_cos, dev_bitmap, range_cov,
			       min_x, min_y, max_x, max_y, res
		);
  
  HANDLE_ERROR(cudaMemcpy(bitmap.get_ptr(), dev_bitmap, img_width*img_height*range_degree, cudaMemcpyDeviceToHost));
  HANDLE_ERROR(cudaEventRecord(stop,0));
  HANDLE_ERROR(cudaEventSynchronize(stop));
  float elapsedTime;
  HANDLE_ERROR(cudaEventElapsedTime(&elapsedTime,
                                        start, stop));
  //printf( "Time to generate:  %3.1f ms\n", elapsedTime);
  HANDLE_ERROR( cudaFree( dev_input ) );
  HANDLE_ERROR( cudaFree( dev_data_x ) );
  HANDLE_ERROR( cudaFree( dev_data_y ) );
  //HANDLE_ERROR( cudaFree( dev_bitmap ) );
  HANDLE_ERROR( cudaFree( dev_cos ) );
  HANDLE_ERROR( cudaFree( dev_sin ) );
  bitmap.rot_size = range_degree;
  return bitmap;
}

void rasterMap(float *data_x, float *data_y, float min_x, float min_y, int data_size, int DIMW, int DIMH, 
		    float res, float *normals, int oversample_ratio, float *bitmap, int *nn_index){
  //capture the start time
  cudaEvent_t start, stop;
  HANDLE_ERROR(cudaEventCreate(&start));
  HANDLE_ERROR(cudaEventCreate(&stop));
  HANDLE_ERROR(cudaEventRecord(start, 0));
  
  //let's create a template consists of 2 degree turning for now
  int range_degree = 1;
  //declare device variable
  float *dev_data_x, *dev_data_y, *dev_cos, *dev_sin;
  float *dev_bitmap;
  int *dev_nn_index;
  
 
 
  //allocation of GPU memory
  HANDLE_ERROR(cudaMalloc((void**)&dev_data_x, data_size*sizeof(float)));
  HANDLE_ERROR(cudaMalloc((void**)&dev_data_y, data_size*sizeof(float)));
  HANDLE_ERROR(cudaMalloc((void**)&dev_cos, range_degree*sizeof(float)));
  HANDLE_ERROR(cudaMalloc((void**)&dev_sin, range_degree*sizeof(float)));
  HANDLE_ERROR(cudaMalloc((void**)&dev_bitmap, DIMW*DIMH*sizeof(float)));
  HANDLE_ERROR(cudaMalloc((void**)&dev_nn_index, DIMW*DIMH*sizeof(int)));
  
  dim3    grids(ceil(DIMW/16.f),ceil(DIMH*range_degree/16.f));
  dim3    threads(16,16);
  initImg<<<grids,threads>>>( dev_bitmap, DIMW*DIMH*range_degree, DIMW*DIMW+DIMH*DIMH);
  initImg<<<grids,threads>>>(dev_nn_index, DIMW*DIMH, data_size);
  //copy data to the allocated GPU memory
  HANDLE_ERROR(cudaMemcpy(dev_data_x, data_x, data_size*sizeof(float), cudaMemcpyHostToDevice));
  HANDLE_ERROR(cudaMemcpy(dev_data_y, data_y, data_size*sizeof(float), cudaMemcpyHostToDevice));
  float cos_data[range_degree], sin_data[range_degree];
  for(int i=0; i<range_degree; i++){
    cos_data[i] = cos(PI/90*i);
    sin_data[i] = sin(PI/90*i);
  }
  HANDLE_ERROR(cudaMemcpy(dev_cos, cos_data, range_degree*sizeof(float), cudaMemcpyHostToDevice));
  HANDLE_ERROR(cudaMemcpy(dev_sin, sin_data, range_degree*sizeof(float), cudaMemcpyHostToDevice));
    
  //start GPU proccess, let's get each thread to compute a different rotation based on blockDim.y
  dim3 threadsPerGrid(16, 16);
  dim3 numBlocks(ceil(DIMW/16.f), ceil(DIMH/16.f), data_size);
  drawTemplatePixelBased<<<numBlocks, threadsPerGrid>>>(dev_data_x, dev_data_y, min_x, min_y, res, 
							  dev_bitmap, dev_nn_index, DIMW, DIMH);
  
  HANDLE_ERROR(cudaMemcpy(bitmap, dev_bitmap, DIMW*DIMH*sizeof(float), cudaMemcpyDeviceToHost));
  HANDLE_ERROR(cudaMemcpy(nn_index, dev_nn_index, DIMW*DIMH*sizeof(int), cudaMemcpyDeviceToHost));
  HANDLE_ERROR(cudaEventRecord(stop,0));
  HANDLE_ERROR(cudaEventSynchronize(stop));
  float elapsedTime;
  HANDLE_ERROR(cudaEventElapsedTime(&elapsedTime,
                                        start, stop));
  //printf( "Time to generate:  %3.1f ms\n", elapsedTime);

 
  HANDLE_ERROR( cudaFree( dev_nn_index ) );
  HANDLE_ERROR( cudaFree( dev_data_x ) );
  HANDLE_ERROR( cudaFree( dev_data_y ) );
  HANDLE_ERROR( cudaFree( dev_bitmap ) );
  HANDLE_ERROR( cudaFree( dev_cos ) );
  HANDLE_ERROR( cudaFree( dev_sin ) );
  
}
texture<unsigned char> tex_dev_template_;  
__global__ void matchTemplate(unsigned char *match_template, int *data_x, int *data_y, int *input_data_count,
			      int data_size, float *scores, int *score_x, int *score_y,
			      int *score_rot, int img_width, int img_height,
			      int *x_steps, int *y_steps, int *r_steps,
			      int x_size, int y_size, int r_size,
			      int ini_r
 			    ){
  int offset_x = threadIdx.x + blockIdx.x*blockDim.x;
  int offset_y = threadIdx.y + blockIdx.y*blockDim.y;
  int offset_rot = threadIdx.z + blockIdx.z*blockDim.z;
  
  if(offset_x<x_size && offset_y<y_size && offset_rot < r_size){
    
    
    
    float score = 0;
    int score_ptr = offset_x+offset_y*x_size+offset_rot*y_size*x_size;
    
    //change the logic here to real value comparison.
    int center_offset_x =x_steps[offset_x];
    int center_offset_y =y_steps[offset_y];
    int center_offset_rot = r_steps[offset_rot];
    score_rot[score_ptr] = center_offset_rot;
    score_x[score_ptr] = center_offset_x;
    score_y[score_ptr] = center_offset_y;
    int offset_template = (img_width*img_height*center_offset_rot);
    for(int i=0; i<data_size; i++){
      int x = data_x[i]+center_offset_x;
      int y = data_y[i]+center_offset_y;
      if(x>=0 && y>=0 && x<img_width && y<img_height){
	int score_temp = match_template[x + y*img_width + offset_template];
	//int score_temp = tex1Dfetch(tex_dev_template_,x + y*img_width + offset_template);
	score += input_data_count[i]*score_temp;
      }
    }
    
    scores[score_ptr] = score;
  }
  return;
}
 
struct sortMatchScore{
  __host__ __device__
  bool operator()(MatchScore ms1, MatchScore ms2){
    return ms1.score > ms2.score;
  }
};

MatchScore findBestMatch(int *data_x, int *data_y, int *input_data_count, int data_size,
		   int raw_data_size, unsigned char *dev_template, int img_width, int img_height,
		   float res, float deg_res, vector<int> x_steps, vector<int> y_steps, vector<int> r_steps,
		   int ini_r
		  ){
  
  cudaEvent_t start, stop, start_match, stop_match;
  HANDLE_ERROR(cudaEventCreate(&start));
  HANDLE_ERROR(cudaEventCreate(&stop));
  HANDLE_ERROR(cudaEventRecord(start, 0));
  HANDLE_ERROR(cudaEventCreate(&start_match));
  HANDLE_ERROR(cudaEventCreate(&stop_match));
  
  int rot_step = 360.f/deg_res;
  
  
  
 
  int *dev_data_x, *dev_data_y, *dev_input_data_count;
  //thrust::device_vector<MatchScore> dev_score_vec(x_step*y_step*4*rot_step);
  
  //MatchScore *dev_scores = thrust::raw_pointer_cast(&dev_score_vec[0]);
  int size_of_scores = x_steps.size()*y_steps.size()*r_steps.size();
  //cout<<"Size of scores = "<<size_of_scores<<endl;
  float *dev_scores;
  float *scores = new float[size_of_scores];
  int *dev_score_x, *dev_score_y, *dev_score_rot;
  int *score_x = new int[size_of_scores];
  int *score_y = new int[size_of_scores];
  int *score_rot = new int[size_of_scores];
  int *x_steps_array = &x_steps[0], *dev_x_steps;
  int *y_steps_array = &y_steps[0], *dev_y_steps;
  int *r_steps_array = &r_steps[0], *dev_r_steps;
  HANDLE_ERROR(cudaMalloc((void**)&dev_scores, size_of_scores*sizeof(float)));
  HANDLE_ERROR(cudaMalloc((void**)&dev_score_x, size_of_scores*sizeof(int)));
  HANDLE_ERROR(cudaMalloc((void**)&dev_score_y, size_of_scores*sizeof(int)));
  HANDLE_ERROR(cudaMalloc((void**)&dev_score_rot, size_of_scores*sizeof(int)));
  HANDLE_ERROR(cudaMalloc((void**)&dev_data_x, data_size*sizeof(int)));
  HANDLE_ERROR(cudaMalloc((void**)&dev_data_y, data_size*sizeof(int)));
  HANDLE_ERROR(cudaMalloc((void**)&dev_x_steps, x_steps.size()*sizeof(int)));
  HANDLE_ERROR(cudaMalloc((void**)&dev_y_steps, y_steps.size()*sizeof(int)));
  HANDLE_ERROR(cudaMalloc((void**)&dev_r_steps, r_steps.size()*sizeof(int)));
  HANDLE_ERROR(cudaMalloc((void**)&dev_input_data_count, data_size*sizeof(int)));
  HANDLE_ERROR(cudaMemcpy(dev_data_x, data_x, data_size*sizeof(int), cudaMemcpyHostToDevice));
  HANDLE_ERROR(cudaMemcpy(dev_data_y, data_y, data_size*sizeof(int), cudaMemcpyHostToDevice));
  HANDLE_ERROR(cudaMemcpy(dev_input_data_count, input_data_count, data_size*sizeof(int), cudaMemcpyHostToDevice));
  HANDLE_ERROR(cudaMemcpy(dev_x_steps, x_steps_array, x_steps.size()*sizeof(int), cudaMemcpyHostToDevice));
  HANDLE_ERROR(cudaMemcpy(dev_y_steps, y_steps_array, y_steps.size()*sizeof(int), cudaMemcpyHostToDevice));
  HANDLE_ERROR(cudaMemcpy(dev_r_steps, r_steps_array, r_steps.size()*sizeof(int), cudaMemcpyHostToDevice));
 // HANDLE_ERROR(cudaBindTexture(NULL, tex_dev_template_, dev_template, img_width*img_height));
  int threadx = 8, thready = 8, threadrot = 4;
  dim3 threads(threadx,thready,threadrot);
  dim3 grids(ceil(x_steps.size()/(float)threadx), ceil(y_steps.size()/(float)thready), ceil(r_steps.size()/(float)threadrot));
  //std::cout<<"Running cuda with grid size of "<<grids.x<<","<<grids.y<<","<<grids.z<< " matching "<<data_size<<endl;
  HANDLE_ERROR(cudaEventRecord(start_match, 0));
  matchTemplate<<<grids, threads>>>(dev_template, dev_data_x, dev_data_y, dev_input_data_count,
				    data_size, dev_scores, dev_score_x, dev_score_y, 
				    dev_score_rot, img_width, img_height/rot_step,
				    dev_x_steps, dev_y_steps, dev_r_steps,
				    x_steps.size(), y_steps.size(), r_steps.size(), ini_r
				   );
  HANDLE_ERROR(cudaEventRecord(stop_match,0));
  HANDLE_ERROR(cudaEventSynchronize(stop_match));
  float elapsedTimeCore;
  HANDLE_ERROR(cudaEventElapsedTime(&elapsedTimeCore,
                                        start_match, stop_match));
  
  //printf( "Time for match_core:  %3.1f ms\n", elapsedTimeCore);
  //thrust::sort(dev_score_vec.begin(), dev_score_vec.end(), sortMatchScore());
  HANDLE_ERROR(cudaMemcpy(scores, dev_scores, size_of_scores*sizeof(float), cudaMemcpyDeviceToHost));
  HANDLE_ERROR(cudaMemcpy(score_x, dev_score_x, size_of_scores*sizeof(int), cudaMemcpyDeviceToHost));
  HANDLE_ERROR(cudaMemcpy(score_y, dev_score_y, size_of_scores*sizeof(int), cudaMemcpyDeviceToHost));
  HANDLE_ERROR(cudaMemcpy(score_rot, dev_score_rot, size_of_scores*sizeof(int), cudaMemcpyDeviceToHost));
  MatchScore ms;
  
  thrust::device_vector<float> d_scores(scores, scores+size_of_scores);
  thrust::device_vector<float>::iterator iter =
  thrust::max_element(d_scores.begin(), d_scores.end());

  unsigned int position = iter - d_scores.begin();
  float max_val = *iter;

  //std::cout << "The maximum value is " << max_val << " at position " << position << std::endl;
  ms.score = scores[position];
  ms.x = score_x[position];
  ms.y = score_y[position];
  ms.rot = score_rot[position];
  /*ms.score = 0;
  for(int i=0; i<size_of_scores; i++){
    if(ms.score < scores[i]){
      ms.score = scores[i];
      ms.x = score_x[i];
      ms.y = score_y[i];
      ms.rot = score_rot[i];
    }
  }*/
  
  HANDLE_ERROR(cudaEventRecord(stop,0));
  HANDLE_ERROR(cudaEventSynchronize(stop));
  float elapsedTime;
  HANDLE_ERROR(cudaEventElapsedTime(&elapsedTime,
                                        start, stop));
  
  //printf( "Time to match:  %3.1f ms\n", elapsedTime);
  //the following line caused thrust::system::system_error, invalid device pointer
//  cudaDeviceReset();
  
  free(scores);
  free(score_x);
  free(score_y);
  free(score_rot);
  HANDLE_ERROR(cudaFree(dev_scores));
  HANDLE_ERROR(cudaFree(dev_score_x));
  HANDLE_ERROR(cudaFree(dev_score_y));
  HANDLE_ERROR(cudaFree(dev_score_rot));
  HANDLE_ERROR(cudaFree(dev_data_x));
  HANDLE_ERROR(cudaFree(dev_data_y));
  HANDLE_ERROR(cudaFree(dev_x_steps));
  HANDLE_ERROR(cudaFree(dev_y_steps));
  HANDLE_ERROR(cudaFree(dev_r_steps));
  HANDLE_ERROR(cudaFree(dev_input_data_count));
  return ms;
}
