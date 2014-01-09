extern poseResult best_translation(float res, float step_size,
			int x_step, int y_step, int x_range, int y_range,
			vector<int> point_x, vector<int> point_y,
			int voronoi_width, int voronoi_height,
			device_vector<int> &dev_px, device_vector<int> &dev_py,
			device_vector<int> &dev_voronoi_data, int stream_idx);
extern host_vector<int> voronoi_jfa(int voronoi_width, int voronoi_height, 
		 vector<int> point_x, vector<int> point_y,
		 device_vector<int> &dev_px, device_vector<int> &dev_py,
		 device_vector<int> &dev_voronoi_data
		);
bool rowMajorDataSort(rowMajorData d1, rowMajorData d2){
   return d1.data_idx < d2.data_idx;
 }
template <class T>
CsmGPU<T>::CsmGPU(double res, cv::Point2d template_size, 
	  pcl::PointCloud<T> &cloud, bool visualize):
  res_(res)
  {  
    cv::Size voronoi_size(template_size.x/res_, template_size.y/res_);
    voronoi_size_ = voronoi_size;
    vector<cv::Vec3b> id_rgbs;
    map<int, cv::Vec3b> color_map;
    cv::Mat voronoi_temp(voronoi_size, CV_16UC1);
    fmutil::Stopwatch sw("voronoi construction");
   
    vector<int> seeds_x, seeds_y;
    cv::Mat voronoi_initial_data = removeRepeatedPts(cloud, seeds_x, seeds_y);
    host_vector<int> voronoi_data = voronoi_jfa(voronoi_size_.width, voronoi_size_.height,
				    seeds_x, seeds_y, dev_px_, dev_py_,
				    dev_voronoi_data_
					  );
    if(visualize)
      visualize_data(voronoi_data, string("voronoi"));
  }
  template <class T>
  poseResult CsmGPU<T>::getBestMatch(double x_step, double y_step, double r_step,
		    double x_range, double y_range, double r_range,
		    pcl::PointCloud<T> &matching_pts,
		    poseResult offset
 			){
    poseResult best_result;
    best_result.score = 0.0;
    int stream_count = 0;
    for(double r = -r_range+offset.r; r<= r_range+offset.r; r+=r_step){
      pcl::PointCloud<T> cloud_out;
      Eigen::Matrix4f transform;
      double d = r/180*M_PI;
      transform<<cos(d),-sin(d),0,offset.x,
		  sin(d),cos(d),0,offset.y,
		  0,0,1,0,
		  0,0,0,1;
      //cout<<r<<"deg ="<<endl<<transform<<endl;
      stringstream ss;
      ss<<"rotate_"<<r<<".pcd";
      pcl::transformPointCloud (matching_pts, cloud_out, transform);
      
      
      //cudaDeviceSynchronize();
      //pcl::io::savePCDFileASCII (ss.str(), cloud_out);
      int stream_idx = stream_count%4;
      poseResult result = getBestTranslation(x_step, y_step, x_range, y_range, cloud_out,stream_idx);
      stream_count++;
      result.r = r;
      result.x += offset.x;
      result.y += offset.y;
      if(PRINT_DEBUG)
      cout<<r<<" deg = "<< result.x<<", "<<result.y<<", "<<result.r<<": "<<result.score<<endl;
      if(best_result.score < result.score) best_result = result;
    }
    return best_result;
  }
  template <class T>
  poseResult CsmGPU<T>::getBestTranslation(double x_step, double y_step, 
			  double x_range, double y_range,
			  pcl::PointCloud<T> &matching_pts, int stream_idx){
    vector<int> px, py;
    removeRepeatedPts(matching_pts, px, py);
    double step_size = x_step;
    if(step_size > y_step) step_size = y_step;
    //step_size = res_;
    if(PRINT_DEBUG)
      cout<<"matching pts "<<px.size()<<"x"<<py.size()<<endl;
    return best_translation(res_, step_size, x_step/res_, y_step/res_, x_range/res_, y_range/res_,
		     px, py, 
		     voronoi_size_.width, voronoi_size_.height,
		     dev_px_, dev_py_, dev_voronoi_data_,stream_idx
		    );
  
  }
  
  template <class T>
 cv::Mat CsmGPU<T>::removeRepeatedPts(pcl::PointCloud<T> &cloud,
			vector<int> &seeds_x, vector<int> &seeds_y){
    uint16_t total_seed = 1;
    seeds_x.clear(); seeds_y.clear();
    vector<rowMajorData> seeds;
    cv::Mat voronoi_data = cv::Mat::zeros(voronoi_size_, CV_16UC1);
    for(size_t i=0; i<cloud.size(); i++){
      
      int x = cloud[i].x/res_ + voronoi_size_.width/2;
      int y = cloud[i].y/res_ + voronoi_size_.height/2;
      
      if(x>0 && y>0 && x<voronoi_size_.width && y < voronoi_size_.height){
	//cout<<x<<" "<<y<<";"<<flush;
	
	if(voronoi_data.at<uint16_t>(x,y) == 0){
	  voronoi_data.at<uint16_t>(x,y) = total_seed;
	  //cout<<voronoi_data.at<uint16_t>(x,y)<<" "<<flush;
	  rowMajorData data;
	  data.x = x;
	  data.y = y;
	  data.data_idx = x + y * voronoi_size_.width;
	  seeds.push_back(data);
	  total_seed++;
	}
      }
    }
    std::sort(seeds.begin(), seeds.end(), rowMajorDataSort);
    for(size_t i=0; i<seeds.size(); i++){
      seeds_x.push_back(seeds[i].x);
      seeds_y.push_back(seeds[i].y);
    }
    return voronoi_data;
 }
 
 template <class T>
 void CsmGPU<T>::visualize_data(host_vector<int> voronoi_data, string name){
  //visualize
  cv::Mat voronoi_img(voronoi_size_.width, voronoi_size_.height, CV_16UC1);
  for(int i=0; i<voronoi_size_.width; i++){
    for(int j=0; j<voronoi_size_.height; j++){
      int index = i+j*voronoi_size_.width;
      voronoi_img.at<uint16_t>(i, j) = voronoi_data[index];
    }
  }
 
  cv::Mat voronoi_vis = cv::Mat::zeros(voronoi_img.cols, voronoi_img.rows, CV_8UC3);
  static map<uint16_t, cv::Vec3b> color_map;
  for(int i=0; i<voronoi_img.cols; i++){
    for(int j=0; j<voronoi_img.rows; j++){
      uint16_t id = voronoi_img.at<uint16_t>(i,j);
      if(id == 0) continue;
      if(color_map.find(id) == color_map.end()){
	float id_color = ((double) rand() / (RAND_MAX));
	float id_sat = ((double) rand() / (RAND_MAX));
	//cout<<"new color initiated wit id "<<id<<endl;
	color_map[id] = HSVtoRGB(id_color, id_sat, 1.0);
      }
      voronoi_vis.at<cv::Vec3b>(i,j) = color_map[id];
    }
  }
  //resize(voronoi_vis, voronoi_vis, cv::Size(0,0), 2, 2, cv::INTER_NEAREST);
  cv::imshow(name.c_str(), voronoi_vis);
  cv::waitKey(0);
}

template <class T>
cv::Vec3b CsmGPU<T>::HSVtoRGB(float hue, float sat, float value){
  float fH, fS, fV;
  float fR, fG, fB;
  const float FLOAT_TO_BYTE = 255.0f;
  // Convert from 8-bit integers to floats
  fH = hue;
  fS = sat;
  fV = value;

  // Convert from HSV to RGB, using float ranges 0.0 to 1.0
  int iI;
  float fI, fF, p, q, t;


  // If Hue == 1.0, then wrap it around the circle to 0.0
  if (fH >= 1.0f)
	  fH = 0.0f;

  fH *= 6.0;			// sector 0 to 5
  fI = floor( fH );		// integer part of h (0,1,2,3,4,5 or 6)
  iI = (int) fH;			//		"		"		"		"
  fF = fH - fI;			// factorial part of h (0 to 1)

  p = fV * ( 1.0f - fS );
  q = fV * ( 1.0f - fS * fF );
  t = fV * ( 1.0f - fS * ( 1.0f - fF ) );

  switch( iI ) {
	  case 0:
		  fR = fV;
		  fG = t;
		  fB = p;
		  break;
	  case 1:
		  fR = q;
		  fG = fV;
		  fB = p;
		  break;
	  case 2:
		  fR = p;
		  fG = fV;
		  fB = t;
		  break;
	  case 3:
		  fR = p;
		  fG = q;
		  fB = fV;
		  break;
	  case 4:
		  fR = t;
		  fG = p;
		  fB = fV;
		  break;
	  default:		// case 5 (or 6):
		  fR = fV;
		  fG = p;
		  fB = q;
		  break;
  }


  // Convert from floats to 8-bit integers
  int bR = (int)(fR * FLOAT_TO_BYTE);
  int bG = (int)(fG * FLOAT_TO_BYTE);
  int bB = (int)(fB * FLOAT_TO_BYTE);

  // Clip the values to make sure it fits within the 8bits.
  if (bR > 255)
	  bR = 255;
  if (bR < 0)
	  bR = 0;
  if (bG > 255)
	  bG = 255;
  if (bG < 0)
	  bG = 0;
  if (bB > 255)
	  bB = 255;
  if (bB < 0)
	  bB = 0;

  // Set the RGB pixel components. NOTE that OpenCV stores RGB pixels in B,G,R order.
  cv::Vec3b pRGB;
  pRGB[0] = bB;		// B component
  pRGB[1] = bG;		// G component
  pRGB[2] = bR;		// R components
  
  return pRGB;
}
