extern poseResult best_translation(float res, float step_size,
			int x_step, int y_step, int x_range, int y_range,
			vector<cudaPointNormal> point,
			int voronoi_width, int voronoi_height,
			device_vector<cudaPointNormal> &dev_p,
			device_vector<int> &dev_voronoi_data, int stream_idx);
extern host_vector<int> voronoi_jfa(int voronoi_width, int voronoi_height, 
		 vector<cudaPointNormal> point,
		 device_vector<cudaPointNormal> &dev_p,
		 device_vector<int> &dev_voronoi_data, bool gotNormal
		);
extern void clearCSM();
extern void setDevFreeSpaceData(device_vector<float> &device_free_space);

bool rowMajorDataSort(rowMajorData d1, rowMajorData d2){
   return d1.data_idx < d2.data_idx;
 }
 //add calculation of normal value as part of the scores
template <class T>
void CsmGPU<T>::getVoronoiTemplate(pcl::PointCloud<T> &cloud, bool visualize){
  vector<cudaPointNormal> seeds;
  cv::Mat voronoi_initial_data = removeRepeatedPts(cloud, seeds);
  voronoi_jfa(voronoi_size_.width, voronoi_size_.height,
				  seeds, dev_p_,
				  dev_voronoi_data_, gotNormal_
					);
  if(visualize){
    host_vector<int> voronoi_data = dev_voronoi_data_;
    visualize_data(voronoi_data, string("voronoi"));
  }
}

template <class T>
CsmGPU<T>::~CsmGPU(){
  clearCSM();
}

template <class T>
CsmGPU<T>::CsmGPU(double res, cv::Point2d template_size, 
	  pcl::PointCloud<T> &cloud, bool visualize):
  res_(res)
  {
    cudaSetDevice(0);
    cudaDeviceSynchronize();
    cudaThreadSynchronize();
    
    size_t free_byte ;
    size_t total_byte ;

    cudaMemGetInfo( &free_byte, &total_byte ) ;
    cout<<"Cuda usage: remaining "<<free_byte/1024000.0<<"MiB out of "<<total_byte/1024000.0<<"MiB"<<endl;
	      
    cv::Size voronoi_size(template_size.x/res_, template_size.y/res_);
    voronoi_size_ = voronoi_size;
    vector<cv::Vec3b> id_rgbs;
    map<int, cv::Vec3b> color_map;
    cv::Mat voronoi_temp(voronoi_size, CV_16UC1);
    
    string pclFieldList = pcl::getFieldsList(cloud);
    gotNormal_ = true;
    if(pclFieldList.find("normal") == string::npos)
      gotNormal_ = false;
    getVoronoiTemplate(cloud, visualize);
  }
  
  void drawPathOnOpenCV(ClipperLib::Path &path, cv::Mat &freeSpaceTemplate, cv::Scalar color, 
			bool gaussian=false, double std_dev=0.0){
    cv::Point cv_poly[1][path.size()];
    for(size_t i=0; i<path.size(); i++){
      cv_poly[0][i].x = path[i].X;
      cv_poly[0][i].y = path[i].Y;
    }
    const cv::Point* ppt[1] = {cv_poly[0]};
    int npt[] = {path.size()};
    
    if(gaussian){
      cv::fillPoly(freeSpaceTemplate, ppt, npt, 1, color, 8);
      cv::GaussianBlur(freeSpaceTemplate, freeSpaceTemplate, cv::Size(), std_dev);
    }
    else
      cv::polylines(freeSpaceTemplate, ppt, npt, 1, true, color, 1, 4);
    
    
  }
  
  void drawPathOnOpenCV(ClipperLib::Paths &paths, cv::Mat &freeSpaceTemplate, cv::Scalar color, 
			bool gaussian = false, double std_dev=0.0){
    for(size_t i=0; i<paths.size(); i++)
      drawPathOnOpenCV(paths[i], freeSpaceTemplate, color, gaussian, std_dev);
  }
  
  template <class T>
  void CsmGPU<T>::drawFreeSpace(sensor_msgs::PointCloud &cloud, cv::Size template_size, bool try_param=true){
    ClipperLib::Path path;
    ClipperLib::IntPoint p;
    p.X = template_size.width/2;
    p.Y = template_size.height/2;
    path.push_back(p);
    for(size_t i=0; i<cloud.points.size(); i++){
      ClipperLib::IntPoint temp_p;
      temp_p.X = cloud.points[i].x/res_ + p.X;
      temp_p.Y = cloud.points[i].y/res_ + p.Y;
      path.push_back(temp_p);
    }
    ClipperLib::ClipperOffset clipperOffset;
    clipperOffset.AddPath(path, ClipperLib::jtMiter, ClipperLib::etClosedPolygon);
    cv::Mat freeSpaceTemplate = cv::Mat(template_size, CV_32F, cv::Scalar(1.0));
    int offset = -3;
    double std_dev = 2;
    if(try_param){
      cout<<"use left right arrow key to adjust polygon offset"<<endl;
      cout<<"and up down arrow key to adjust gaussian std dev"<<endl;
      int capture_key = 1113938;
      while(true){
	freeSpaceTemplate = cv::Mat(template_size, CV_32F, cv::Scalar(1.0));
	switch (capture_key){
	  case 1113939: // right arrow
	    offset++;
	    break;	  
	  case 1113937: // left arrow
	    offset--;
	    break;
	  case 1113938: // up arrow
	    std_dev+=0.5;
	    break;
	  case 1113940: // down arrow
	    std_dev-=0.5;
	    break;
	  default:
	    return;
	}
	cout<<offset<<" "<<std_dev<<endl;
	vector<ClipperLib::Path> paths;
	clipperOffset.Execute(paths, offset);
	drawPathOnOpenCV(paths, freeSpaceTemplate, cv::Scalar(0.0), true, std_dev);
	drawPathOnOpenCV(path, freeSpaceTemplate, cv::Scalar(0.0));
	cv::imshow("freeSpaceTemplate", freeSpaceTemplate);
	capture_key = cv::waitKey(0);
      }
    }
    else {
      vector<ClipperLib::Path> paths;
      clipperOffset.Execute(paths, offset);
      drawPathOnOpenCV(paths, freeSpaceTemplate, cv::Scalar(0.0), true, std_dev);
    }
    
    host_vector<float> free_space_data;
    for(int i=0; i<freeSpaceTemplate.rows; i++){
      for(int j=0; j<freeSpaceTemplate.cols; j++){
	float data = freeSpaceTemplate.at<float>(j, i);
	free_space_data.push_back(data);
      }
    }
    dev_free_space_ = free_space_data;
    setDevFreeSpaceData(dev_free_space_);
    //just to check
    /*
    cv::Mat mat_check(template_size, CV_32F);
    for(int i=0; i<template_size.height; i++){
      for(int j=0; j<template_size.width; j++){
	mat_check.at<float>(j,i) = free_space_data[j+i*template_size.width];
      }
    }
    cv::imshow("Verification",mat_check);
    cv::waitKey(0);*/
  }
  
  template <class T>
  CsmGPU<T>::CsmGPU(double res, cv::Point2d template_size, sensor_msgs::LaserScan &laser, bool visualize):res_(res){
    cv::Size voronoi_size(template_size.x/res_, template_size.y/res_);
    gotNormal_ = false;
    voronoi_size_ = voronoi_size;
    laser_geometry::LaserProjection projector;
    sensor_msgs::PointCloud cloud;
    projector.projectLaser(laser, cloud);
    bool try_param = false;
    drawFreeSpace(cloud, voronoi_size, try_param);
    pcl::PointCloud<T> pcl_pts;
    for(size_t i=0; i<cloud.points.size(); i++)
      pcl_pts.push_back(T(cloud.points[i].x, cloud.points[i].y, 0.0));
    getVoronoiTemplate(pcl_pts, false);
  }

  template <class T>
  poseResult CsmGPU<T>::getBestMatch(double x_step, double y_step, double r_step,
				     double x_range, double y_range, double r_range,
				     sensor_msgs::LaserScan &matching_scan,
				     poseResult offset){
    laser_geometry::LaserProjection projector;
    sensor_msgs::PointCloud cloud;
    projector.projectLaser(matching_scan, cloud);
    pcl::PointCloud<T> pcl_pts;
    for(size_t i=0; i<cloud.points.size(); i++)
      pcl_pts.push_back(T(cloud.points[i].x, cloud.points[i].y, 0.0));
    return getBestMatch(x_step, y_step, r_step, x_range, y_range, r_range, pcl_pts, offset, &pcl::transformPointCloud);
  }
  template <class T>
  poseResult CsmGPU<T>::getBestMatch(double x_step, double y_step, double r_step,
		    double x_range, double y_range, double r_range,
		    pcl::PointCloud<T> &matching_pts,
		    poseResult offset,void(*transformFunc)(const pcl::PointCloud< T > &, pcl::PointCloud< T > &, const Eigen::Matrix4f &)
		    = &pcl::transformPointCloudWithNormals
 			){
    poseResult best_result;
    best_result.score = 0.0;
    int stream_count = 0;
    vector<double> r_array;
    for(double r = -r_range+offset.r; r<= r_range+offset.r; r+=r_step) r_array.push_back(r);
    vector<poseResult> results(r_array.size());
//#pragma omp parallel for
    for(size_t i=0; i<r_array.size(); i++){
      pcl::PointCloud<T> cloud_out;
      Eigen::Matrix4f transform;
      double d = r_array[i]/180*M_PI;
      transform<<cos(d),-sin(d),0,offset.x,
		  sin(d),cos(d),0,offset.y,
		  0,0,1,0,
		  0,0,0,1;
      //cout<<r<<"deg ="<<endl<<transform<<endl;
      stringstream ss;
      ss<<"rotate_"<<r_array[i]<<".pcd";
      transformFunc(matching_pts, cloud_out, transform);
	
      //cudaDeviceSynchronize();
      //pcl::io::savePCDFileASCII (ss.str(), cloud_out);
      int stream_idx = stream_count%4;
      poseResult result = getBestTranslation(x_step, y_step, x_range, y_range, cloud_out,stream_idx);
      stream_count++;
      result.r = r_array[i];
      result.x += offset.x;
      result.y += offset.y;
      results.push_back(result);
    }
    
    for(size_t i=0; i<results.size(); i++){
      poseResult result = results[i];
      if(PRINT_DEBUG)
	cout<<result.r<<" deg = "<< result.x<<", "<<result.y<<", "<<result.r<<": "<<result.score<<endl;
      if(best_result.score < result.score) best_result = result;
    }
    return best_result;
  }
  template <class T>
  poseResult CsmGPU<T>::getBestTranslation(double x_step, double y_step, 
			  double x_range, double y_range,
			  pcl::PointCloud<T> &matching_pts, int stream_idx){
    vector<cudaPointNormal> p;
    removeRepeatedPts(matching_pts, p);
    double step_size = x_step;
    if(step_size > y_step) step_size = y_step;
    //step_size = res_;
    if(PRINT_DEBUG)
      cout<<"matching pts "<<p.size()<<endl;
    return best_translation(res_, step_size, x_step/res_, y_step/res_, x_range/res_, y_range/res_,
		     p, 
		     voronoi_size_.width, voronoi_size_.height,
		     dev_p_, dev_voronoi_data_,stream_idx
		    );
  
  }
  
  template <class T>
 cv::Mat CsmGPU<T>::removeRepeatedPts(pcl::PointCloud<T> &cloud,
			vector<cudaPointNormal> &cuda_seeds){
    uint16_t total_seed = 1;
    cuda_seeds.clear();
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
	  
#if PCLNORMAL
	  if(gotNormal_){
	    data.normal_x = cloud[i].normal_x;
	    data.normal_y = cloud[i].normal_y;
	  }
#endif
	  seeds.push_back(data);
	  total_seed++;
	}
      }
    }
    std::sort(seeds.begin(), seeds.end(), rowMajorDataSort);
    for(size_t i=0; i<seeds.size(); i++){
      cudaPointNormal cuda_seed;
      cuda_seed.x = seeds[i].x;
      cuda_seed.y = seeds[i].y;
      cuda_seed.normal_x = seeds[i].normal_x;
      cuda_seed.normal_y = seeds[i].normal_y;
      cuda_seeds.push_back(cuda_seed);
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
