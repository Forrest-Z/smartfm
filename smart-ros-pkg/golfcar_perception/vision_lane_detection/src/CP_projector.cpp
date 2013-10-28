/*
 * created on 30/08/2012
 * by Baoxing
 * future work: write it in a general way to support multiple cameras;
 */

#include "CP_projector.h"

namespace golfcar_vision{
  
CP_projector::CP_projector():
    private_nh_("~"),
    it_(nh_)
  {
      cam_sub_ = it_.subscribeCamera("/camera_front/image_raw", 1, &CP_projector::ImageCallBack, this);

      private_nh_.param("vehicle_ID", vehicle_ID_, std::string("robot_0"));

	  private_nh_.param("odom_frame", odom_frame_, std::string("/odom"));

	  private_nh_.param("vehicle0_dest_frame", 	vehicle0_dest_frame_, 	std::string("/camera_front_img"));

	  private_nh_.param("vehicle0_Hdest_frame", vehicle0_Hdest_frame_, 	std::string("/cameraH_front_img"));

	  private_nh_.param("vehicle0_H2dest_frame", vehicle0_H2dest_frame_, 	std::string("/cameraH2_front_img"));
	  private_nh_.param("vehicle0_H3dest_frame", vehicle0_H3dest_frame_, 	std::string("/cameraH3_front_img"));
	  private_nh_.param("vehicle0_H4dest_frame", vehicle0_H4dest_frame_, 	std::string("/cameraH4_front_img"));
	  private_nh_.param("vehicle0_H5dest_frame", vehicle0_H5dest_frame_, 	std::string("/cameraH5_front_img"));
	  private_nh_.param("vehicle0_H6dest_frame", vehicle0_H6dest_frame_, 	std::string("/cameraH6_front_img"));
	  private_nh_.param("vehicle0_H7dest_frame", vehicle0_H7dest_frame_, 	std::string("/cameraH7_front_img"));
	  private_nh_.param("vehicle0_H8dest_frame", vehicle0_H8dest_frame_, 	std::string("/cameraH8_front_img"));
	  private_nh_.param("vehicle0_H9dest_frame", vehicle0_H9dest_frame_, 	std::string("/cameraH9_front_img"));


	  private_nh_.param("vehicle1_dest_frame", 	vehicle1_dest_frame_,	std::string("/robot_1/camera_image"));
	  private_nh_.param("vehicle1_Hdest_frame", vehicle1_Hdest_frame_, 	std::string("/robot_1/cameraH_image"));

	  private_nh_.param("image_height", 		image_height_,		360);
	  private_nh_.param("image_width",			image_width_,	640);

	  cloud_scan_sub_.subscribe(nh_, "/pts_rgb", 10);
	  cloud_scan_filter_ = new tf::MessageFilter<PointCloudRGB>(cloud_scan_sub_, tf_, odom_frame_, 100);
	  cloud_scan_filter_->registerCallback(boost::bind(&CP_projector::pclCallback, this, _1));
	  cloud_scan_filter_->setTolerance(ros::Duration(0.05));

	  cloud_scan_sub1_.subscribe(nh_, "/robot_1/pts_rgb", 10);
	  cloud_scan_filter1_ = new tf::MessageFilter<PointCloudRGB>(cloud_scan_sub1_, tf_, odom_frame_, 100);
	  cloud_scan_filter1_->registerCallback(boost::bind(&CP_projector::pclCallback, this, _1));
	  cloud_scan_filter1_->setTolerance(ros::Duration(0.05));

	  cloud_scan_sub2_.subscribe(nh_, "/robot_2/pts_rgb", 10);
	  cloud_scan_filter2_ = new tf::MessageFilter<PointCloudRGB>(cloud_scan_sub2_, tf_, odom_frame_, 100);
	  cloud_scan_filter2_->registerCallback(boost::bind(&CP_projector::pclCallback, this, _1));
	  cloud_scan_filter2_->setTolerance(ros::Duration(0.05));

	  vehicle0_velo_sub_ = nh_.subscribe("/odom",10, &CP_projector::velocity0_callback, this);
	  vehicle1st_velo_sub_ = nh_.subscribe("/robot_1/vehicle_vel",10, &CP_projector::velocity1st_callback, this);
	  vehicle2nd_velo_sub_ = nh_.subscribe("/robot_2/vehicle_vel",10, &CP_projector::velocity2nd_callback, this);

	  camera_model_initialized_ = false;

	  CvSize image_size = cvSize(image_width_, image_height_);

	  project_image00 = cvCreateImage(image_size, 8,3);
	  project_image01 = cvCreateImage(image_size, 8,3);
	  project_image02 = cvCreateImage(image_size, 8,3);
	  project_image11 = cvCreateImage(image_size, 8,3);
	  project_image12 = cvCreateImage(image_size, 8,3);
	  project_image22 = cvCreateImage(image_size, 8,3);

	  /*
	  project_imageH00 = cvCreateImage(image_size, 8,3);
	  project_image2H00 = cvCreateImage(image_size, 8,3);
	  project_image3H00 = cvCreateImage(image_size, 8,3);
	  project_image4H00 = cvCreateImage(image_size, 8,3);
	  project_image5H00 = cvCreateImage(image_size, 8,3);
	  project_image6H00 = cvCreateImage(image_size, 8,3);
	  project_image7H00 = cvCreateImage(image_size, 8,3);
	  project_image8H00 = cvCreateImage(image_size, 8,3);
	  project_image9H00 = cvCreateImage(image_size, 8,3);
	   */

	  project_imageH01 = cvCreateImage(image_size, 8,3);
	  project_imageH02 = cvCreateImage(image_size, 8,3);
	  project_imageH11 = cvCreateImage(image_size, 8,3);
	  project_imageH12 = cvCreateImage(image_size, 8,3);
	  project_imageH22 = cvCreateImage(image_size, 8,3);

	  project_image2H01 = cvCreateImage(image_size, 8,3);
	  project_image2H02 = cvCreateImage(image_size, 8,3);
	  project_image3H01 = cvCreateImage(image_size, 8,3);
	  project_image3H02 = cvCreateImage(image_size, 8,3);
	  project_image4H01 = cvCreateImage(image_size, 8,3);
	  project_image4H02 = cvCreateImage(image_size, 8,3);
	  project_image5H01 = cvCreateImage(image_size, 8,3);
	  project_image5H02 = cvCreateImage(image_size, 8,3);
	  project_image6H01 = cvCreateImage(image_size, 8,3);
	  project_image6H02 = cvCreateImage(image_size, 8,3);
	  project_image7H01 = cvCreateImage(image_size, 8,3);
	  project_image7H02 = cvCreateImage(image_size, 8,3);
	  project_image8H01 = cvCreateImage(image_size, 8,3);
	  project_image8H02 = cvCreateImage(image_size, 8,3);
	  project_image9H01 = cvCreateImage(image_size, 8,3);
	  project_image9H02 = cvCreateImage(image_size, 8,3);

	  private_nh_.param("show_scale",    show_scale_,   		 1.0);

	  private_nh_.param("vehicle_length",    	vehicle_length_,   		 3.5);
	  private_nh_.param("vehicle_width",    	vehicle_width_,   		 2.0);
	  private_nh_.param("vehicle_height",    	vehicle_height_,   		 1.7);
	  vehicle_box = new vehicle_model(vehicle_length_, vehicle_width_, vehicle_height_);

	  vehicle0_color = CV_RGB(255,0,0);
	  vehicle1st_color = CV_RGB(0,255,0);
	  vehicle2nd_color = CV_RGB(0,0,255);


	  cvNamedWindow("robot0_enhanced");
	  cvNamedWindow("robot0_enhanced2");
	  cvNamedWindow("robot0_enhanced3");
	  cvNamedWindow("robot0_enhanced4");
	  cvNamedWindow("robot0_enhanced5");
	  cvNamedWindow("robot0_enhanced6");
	  cvNamedWindow("robot0_enhanced7");
	  cvNamedWindow("robot0_enhanced8");
	  cvNamedWindow("robot0_enhanced9");

	  cvNamedWindow("robot_0");
	  cvNamedWindow("robot0_all");
	  cvNamedWindow("robot_1");
	  cvNamedWindow("robot1_all");
	  cvNamedWindow("vehicle1_src");
	  cvNamedWindow("vehicle2_src");
	  cvNamedWindow("vehicle1_on_vehicle0");
	  cvNamedWindow("vehicle2_on_vehicle0");
  }

  void CP_projector::pclCallback(const PointCloudRGB::ConstPtr& pcl_in)
  {
	  ROS_INFO("-------------pcl callback-----------");
	  std::string pcl_frame_id = pcl_in->header.frame_id;

	  ROS_INFO("pcl frame_id %s", pcl_frame_id.c_str());

	  if(vehicle_ID_ == "robot_0")
	  {
		  ROS_INFO("robot_0");
		  if(pcl_frame_id == "base_link")
		  {
			  //not showing themselves;
			  /*
			  pcl_process(pcl_in, vehicle0_Hdest_frame_,  project_imageH00,  vehicle0_color);
			  pcl_process(pcl_in, vehicle0_H2dest_frame_, project_image2H00, vehicle0_color);
			  pcl_process(pcl_in, vehicle0_H3dest_frame_, project_image3H00, vehicle0_color);
			  pcl_process(pcl_in, vehicle0_H4dest_frame_, project_image4H00, vehicle0_color);
			  pcl_process(pcl_in, vehicle0_H5dest_frame_, project_image5H00, vehicle0_color);
			  pcl_process(pcl_in, vehicle0_H6dest_frame_, project_image6H00, vehicle0_color);
			  pcl_process(pcl_in, vehicle0_H7dest_frame_, project_image7H00, vehicle0_color);
			  pcl_process(pcl_in, vehicle0_H8dest_frame_, project_image8H00, vehicle0_color);
			  pcl_process(pcl_in, vehicle0_H9dest_frame_, project_image9H00, vehicle0_color);
			  */
		  }
		  else if(pcl_frame_id== "robot_1/base_link")
		  {
			  ROS_INFO("/robot_1/base_link");
			  pcl_process(pcl_in, vehicle0_dest_frame_, project_image01, vehicle1st_color);
			  pcl_process(pcl_in, vehicle0_Hdest_frame_, project_imageH01, vehicle1st_color);
			  pcl_process(pcl_in, vehicle0_H2dest_frame_, project_image2H01, vehicle1st_color);
			  pcl_process(pcl_in, vehicle0_H3dest_frame_, project_image3H01, vehicle1st_color);
			  pcl_process(pcl_in, vehicle0_H4dest_frame_, project_image4H01, vehicle1st_color);
			  pcl_process(pcl_in, vehicle0_H5dest_frame_, project_image5H01, vehicle1st_color);
			  pcl_process(pcl_in, vehicle0_H6dest_frame_, project_image6H01, vehicle1st_color);
			  pcl_process(pcl_in, vehicle0_H7dest_frame_, project_image7H01, vehicle1st_color);
			  pcl_process(pcl_in, vehicle0_H8dest_frame_, project_image8H01, vehicle1st_color);
			  pcl_process(pcl_in, vehicle0_H9dest_frame_, project_image9H01, vehicle1st_color);
		  }
		  else if(pcl_frame_id == "robot_2/base_link")
		  {
			  ROS_INFO("/robot_2/base_link");
			  pcl_process(pcl_in, vehicle0_dest_frame_, project_image02, vehicle2nd_color);
			  pcl_process(pcl_in, vehicle0_Hdest_frame_, project_imageH02, vehicle2nd_color);
			  pcl_process(pcl_in, vehicle0_H2dest_frame_, project_image2H02, vehicle2nd_color);
			  pcl_process(pcl_in, vehicle0_H3dest_frame_, project_image3H02, vehicle2nd_color);
			  pcl_process(pcl_in, vehicle0_H4dest_frame_, project_image4H02, vehicle2nd_color);
			  pcl_process(pcl_in, vehicle0_H5dest_frame_, project_image5H02, vehicle2nd_color);
			  pcl_process(pcl_in, vehicle0_H6dest_frame_, project_image6H02, vehicle2nd_color);
			  pcl_process(pcl_in, vehicle0_H7dest_frame_, project_image7H02, vehicle2nd_color);
			  pcl_process(pcl_in, vehicle0_H8dest_frame_, project_image8H02, vehicle2nd_color);
			  pcl_process(pcl_in, vehicle0_H9dest_frame_, project_image9H02, vehicle2nd_color);
		  }
	  }
	  else if(vehicle_ID_ == "robot_1")
	  {
		  ROS_INFO("robot_1");
		  if(pcl_in->header.frame_id == "base_link")
		  {

		  }
		  else if(pcl_in->header.frame_id == "robot_1/base_link")
		  {

		  }
		  else if(pcl_in->header.frame_id == "robot_2/base_link")
		  {
			  ROS_INFO("/robot_2/base_link");
			  pcl_process(pcl_in, vehicle1_dest_frame_, project_image12, vehicle2nd_color);
			  pcl_process(pcl_in, vehicle1_Hdest_frame_, project_imageH12, vehicle2nd_color);
		  }
	  }
	  else if(vehicle_ID_ == "robot_2")
	  {
		  ROS_INFO("do nothing for the most front vehicle");
	  }
  }

  void CP_projector::pcl_process(const PointCloudRGB::ConstPtr& pcl_in, string dest_camera_frame, IplImage *project_image, CvScalar color_plot)
  {
	  ROS_INFO("pcl_process");

	  if(!camera_model_initialized_)
	  {
		  ROS_WARN("camera_model not initialized yet!");
		  return;
	  }

	  ROS_INFO("camera_model_initialized");

	  pcl_ros::transformPointCloud(dest_camera_frame, *pcl_in, pcl_batch_, tf_);

	  ROS_INFO("pcl transformed");

	  cvZero(project_image);

	  for(size_t i=0; i<pcl_batch_.points.size(); i++)
	  {
		  pcl::PointXYZRGB point_tmp = pcl_batch_.points[i];
		  cv::Point3d pt_cv(point_tmp.x, point_tmp.y, point_tmp.z);
		  cv::Point2d uv;
		  cam_model_.project3dToPixel(pt_cv, uv);

		  CvPoint pixel;
		  pixel.x = int(floor(uv.x));
		  pixel.y = int(floor(uv.y));

		  if(pixel.x>=640||pixel.x<0||pixel.y<0||pixel.y>=360)
		  {
		  }
		  else
		  {
			  CvScalar s;
			  s.val[0] = point_tmp.b;
			  s.val[1] = point_tmp.g;
			  s.val[2] = point_tmp.r;
			  cvSet2D(project_image, pixel.y, pixel.x, s);
		  }
	  }

	  PointCloud vehicle_skeleton;
	  vehicle_skeleton.header = pcl_in->header;
	  vehicle_skeleton.clear();
	  vehicle_skeleton.height = 1;
	  for(size_t i=0; i<8; i++)vehicle_skeleton.push_back(vehicle_box->skeleton3D_[i]);
	  pcl_ros::transformPointCloud(dest_camera_frame, vehicle_skeleton, vehicle_skeleton, tf_);
	  for(size_t i=0; i<8; i++)
	  {
		  pcl::PointXYZ point_tmp = vehicle_skeleton.points[i];
		  cv::Point3d pt_cv(point_tmp.x, point_tmp.y, point_tmp.z);
		  cv::Point2d uv;
		  cam_model_.project3dToPixel(pt_cv, uv);
		  vehicle_box->skeletonIMG_[i] = cvPoint(uv.x, uv.y);
	  }
	  vehicle_box->DrawSkel(project_image, color_plot, 3);

	  std_msgs::Header predecessor = vehicle_skeleton.header;
	  std_msgs::Header own_vehicle = vehicle_skeleton.header;

	  predecessor.frame_id = pcl_in->header.frame_id;
	  if(dest_camera_frame==vehicle0_dest_frame_ || dest_camera_frame==vehicle0_Hdest_frame_) own_vehicle.frame_id = "base_link";
	  else if(dest_camera_frame==vehicle1_dest_frame_ || dest_camera_frame==vehicle1_Hdest_frame_) own_vehicle.frame_id = "robot_1/base_link";

	  stringstream  project_image_name;
	  if(vehicle_ID_ == "robot_0" &&predecessor.frame_id == "robot_1/base_link" )
	  {
		  if(dest_camera_frame==vehicle0_dest_frame_ ) project_image_name<<"vehicle1_on_vehicle0";
		  else if(dest_camera_frame==vehicle0_Hdest_frame_ ) project_image_name<<"Enhanced: vehicle1_on_vehicle0-"<<vehicle0_Hdest_frame_;

		  vehicle01_distance = distance_between_vehicles(predecessor, own_vehicle)>0.0 ? distance_between_vehicles(predecessor, own_vehicle) : vehicle01_distance;
		  vehicle_box->PutInfo(project_image, color_plot, vehicle1st_velocity, vehicle01_distance, cvPoint(0, -5));
	  }
	  else if(vehicle_ID_ == "robot_0" && predecessor.frame_id == "robot_2/base_link")
	  {
		  if(dest_camera_frame==vehicle0_dest_frame_ ) project_image_name<<"vehicle2_on_vehicle0";
		  else if(dest_camera_frame==vehicle0_Hdest_frame_ ) project_image_name<<"Enhanced: vehicle2_on_vehicle0-"<<vehicle0_Hdest_frame_;

		  vehicle02_distance = distance_between_vehicles(predecessor, own_vehicle)>0.0 ? distance_between_vehicles(predecessor, own_vehicle) : vehicle02_distance;
		  vehicle_box->PutInfo(project_image, color_plot, vehicle2nd_velocity, vehicle02_distance, cvPoint(0, -5));
	  }
	  else if(vehicle_ID_ == "robot_1" && predecessor.frame_id == "robot_2/base_link")
	  {
		  if(dest_camera_frame==vehicle1_dest_frame_ ) project_image_name<<"vehicle2_on_vehicle1";
		  else if(dest_camera_frame==vehicle1_Hdest_frame_ ) project_image_name<<"Enhanced: vehicle2_on_vehicle1-"<<vehicle1_Hdest_frame_;
		  vehicle12_distance = distance_between_vehicles(predecessor, own_vehicle)>0.0 ? distance_between_vehicles(predecessor, own_vehicle) : vehicle12_distance;
		  vehicle_box->PutInfo(project_image, color_plot, vehicle2nd_velocity, vehicle12_distance, cvPoint(0, -5));
	  }

      IplImage* white_background = cvCloneImage(project_image);
      cvSet(white_background, cvScalar(255,255,255));
      merge_images(white_background, project_image);
	  resize_show(white_background, show_scale_, project_image_name.str().c_str());

	  cvWaitKey(1);
	  cvReleaseImage(&white_background);

	  ROS_INFO("pcl_process finished");
  }

  void CP_projector::ImageCallBack( const sensor_msgs::ImageConstPtr& image_msg,
													const sensor_msgs::CameraInfoConstPtr& info_msg)
  {
        ROS_INFO("ImageCallBack");

        ros::Time meas_time = info_msg->header.stamp;

        IplImage* color_image;
        try {
            color_image = bridge_.imgMsgToCv(image_msg, "bgr8");
            }
        catch (sensor_msgs::CvBridgeException& ex) {
            ROS_ERROR("Failed to convert image");
            return;
            }
        cam_model_.fromCameraInfo(info_msg);
        camera_model_initialized_ = true;

        resize_show(color_image, show_scale_, vehicle_ID_.c_str());

        IplImage* merged_image = cvCloneImage(color_image);
        IplImage* merged_Himage = cvCloneImage(color_image);
        IplImage* white_background = cvCloneImage(color_image);
        cvZero(merged_Himage);
        cvSet(white_background, cvScalar(255,255,255));

        string image_name, image_Hname;
        if(vehicle_ID_ == "robot_0")
		{
        	merge_images(merged_image, project_image01);
        	merge_images(merged_image, project_image02);

        	//merge_images(merged_Himage, project_imageH00);
        	merge_images(merged_Himage, project_imageH01);
        	merge_images(merged_Himage, project_imageH02);

        	image_name = "robot0_all";
        	image_Hname = "robot0_enhanced";

        	IplImage* merged_H2image = cvCloneImage(color_image);
        	IplImage* merged_H3image = cvCloneImage(color_image);
        	IplImage* merged_H4image = cvCloneImage(color_image);
        	IplImage* merged_H5image = cvCloneImage(color_image);
        	IplImage* merged_H6image = cvCloneImage(color_image);
        	IplImage* merged_H7image = cvCloneImage(color_image);
        	IplImage* merged_H8image = cvCloneImage(color_image);
        	IplImage* merged_H9image = cvCloneImage(color_image);
        	cvZero(merged_H2image);
        	cvZero(merged_H3image);
        	cvZero(merged_H4image);
        	cvZero(merged_H5image);
        	cvZero(merged_H6image);
        	cvZero(merged_H7image);
        	cvZero(merged_H8image);
        	cvZero(merged_H9image);

        	//merge_images(merged_H2image, project_image2H00);
        	merge_images(merged_H2image, project_image2H01);
        	merge_images(merged_H2image, project_image2H02);

        	//merge_images(merged_H3image, project_image3H00);
        	merge_images(merged_H3image, project_image3H01);
        	merge_images(merged_H3image, project_image3H02);

        	//merge_images(merged_H4image, project_image4H00);
        	merge_images(merged_H4image, project_image4H01);
        	merge_images(merged_H4image, project_image4H02);

        	//merge_images(merged_H5image, project_image5H00);
        	merge_images(merged_H5image, project_image5H01);
        	merge_images(merged_H5image, project_image5H02);

        	//merge_images(merged_H6image, project_image6H00);
        	merge_images(merged_H6image, project_image6H01);
        	merge_images(merged_H6image, project_image6H02);

        	//merge_images(merged_H7image, project_image7H00);
        	merge_images(merged_H7image, project_image7H01);
        	merge_images(merged_H7image, project_image7H02);

        	//merge_images(merged_H8image, project_image8H00);
        	merge_images(merged_H8image, project_image8H01);
        	merge_images(merged_H8image, project_image8H02);

        	//merge_images(merged_H9image, project_image9H00);
        	merge_images(merged_H9image, project_image9H01);
        	merge_images(merged_H9image, project_image9H02);

        	string image_H2name = "robot0_enhanced2";
        	string image_H3name = "robot0_enhanced3";
        	string image_H4name = "robot0_enhanced4";
        	string image_H5name = "robot0_enhanced5";
        	string image_H6name = "robot0_enhanced6";
        	string image_H7name = "robot0_enhanced7";
        	string image_H8name = "robot0_enhanced8";
        	string image_H9name = "robot0_enhanced9";

        	cvSet(white_background, cvScalar(255,255,255));
        	merge_images(white_background, merged_H2image);
        	cvCopy(white_background, merged_H2image);
        	cvSet(white_background, cvScalar(255,255,255));
        	merge_images(white_background, merged_H3image);
        	cvCopy(white_background, merged_H3image);
        	cvSet(white_background, cvScalar(255,255,255));
        	merge_images(white_background, merged_H4image);
        	cvCopy(white_background, merged_H4image);
        	cvSet(white_background, cvScalar(255,255,255));
        	merge_images(white_background, merged_H5image);
        	cvCopy(white_background, merged_H5image);
        	cvSet(white_background, cvScalar(255,255,255));
        	merge_images(white_background, merged_H6image);
        	cvCopy(white_background, merged_H6image);
        	cvSet(white_background, cvScalar(255,255,255));
        	merge_images(white_background, merged_H7image);
        	cvCopy(white_background, merged_H7image);
        	cvSet(white_background, cvScalar(255,255,255));
        	merge_images(white_background, merged_H8image);
        	cvCopy(white_background, merged_H8image);
        	cvSet(white_background, cvScalar(255,255,255));
        	merge_images(white_background, merged_H9image);
        	cvCopy(white_background, merged_H9image);

    		resize_show(merged_H2image, show_scale_, image_H2name.c_str());
    		resize_show(merged_H3image, show_scale_, image_H3name.c_str());
    		resize_show(merged_H4image, show_scale_, image_H4name.c_str());
    		resize_show(merged_H5image, show_scale_, image_H5name.c_str());
    		resize_show(merged_H6image, show_scale_, image_H6name.c_str());
    		resize_show(merged_H7image, show_scale_, image_H7name.c_str());
    		resize_show(merged_H8image, show_scale_, image_H8name.c_str());
    		resize_show(merged_H9image, show_scale_, image_H9name.c_str());

    		cvReleaseImage(&merged_H2image);
    		cvReleaseImage(&merged_H3image);
    		cvReleaseImage(&merged_H4image);
    		cvReleaseImage(&merged_H5image);
    		cvReleaseImage(&merged_H6image);
    		cvReleaseImage(&merged_H7image);
    		cvReleaseImage(&merged_H8image);
    		cvReleaseImage(&merged_H9image);
		}
        else if(vehicle_ID_ == "robot_1")
        {
        	merge_images(merged_image, project_image12);
			merge_images(merged_Himage, project_imageH12);
        	image_name = "robot1_all";
        	image_Hname = "robot1_enhanced";
        }

		cvSet(white_background, cvScalar(255,255,255));
		merge_images(white_background, merged_Himage);
		cvCopy(white_background, merged_Himage);

		resize_show(merged_image, show_scale_, image_name.c_str());
		resize_show(merged_Himage, show_scale_, image_Hname.c_str());

		cvReleaseImage(&merged_image);
		cvReleaseImage(&merged_Himage);
		cvReleaseImage(&white_background);
  }

  void CP_projector::merge_images(IplImage *dst_img, IplImage *src_img)
  {
		int img_height 		= dst_img -> height;
		int img_width  		= dst_img -> width;
		for(int ih=0; ih < img_height; ih++)
		{
			for(int iw=0; iw < img_width; iw++)
			{
				CvPoint pixel;
				pixel.x = iw;
				pixel.y = ih;
				CvScalar s=cvGet2D(src_img, pixel.y, pixel.x);
				if(s.val[0]!=0 || s.val[1]!=0 || s.val[2]!=0 )
				{
					cvSet2D(dst_img, pixel.y, pixel.x, s);
				}
			}
		}
  }

  double CP_projector::distance_between_vehicles(std_msgs::Header vehicle1, std_msgs::Header vehicle2)
  {
	  tf::StampedTransform transform;
      ros::Time acquisition_time = vehicle1.stamp;
      ros::Duration timeout(5.0 / 30);

      try
      {
    	  tf_.waitForTransform(vehicle1.frame_id, vehicle2.frame_id, acquisition_time, timeout);
    	  tf_.lookupTransform(vehicle1.frame_id, vehicle2.frame_id, acquisition_time, transform);
      }
	  catch (tf::TransformException& ex)
	  {
		  ROS_WARN("[draw_frames] TF exception:\n%s", ex.what());
		  return 0.0;
	  }

      double vehicle_x = transform.getOrigin().x();
      double vehicle_y = transform.getOrigin().y();
      return sqrt(vehicle_x*vehicle_x+vehicle_y*vehicle_y);
  }

  void CP_projector::velocity0_callback(const nav_msgs::Odometry::ConstPtr& velo_in)
  {
	  vehicle0_velo_.header = velo_in->header;
	  vehicle0_velo_.twist = velo_in->twist.twist;
  }


  void CP_projector::velocity1st_callback(const geometry_msgs::TwistStamped::ConstPtr& velo_in)
  {
	  vehicle1st_velo_ = *velo_in;
	  double linear_x, linear_y;

	  linear_x = vehicle1st_velo_.twist.linear.x + vehicle0_velo_.twist.linear.x;
	  linear_y = vehicle1st_velo_.twist.linear.y + vehicle0_velo_.twist.linear.y;
	  vehicle1st_velocity = sqrt(linear_x*linear_x+linear_y*linear_y);
  }

  void CP_projector::velocity2nd_callback(const geometry_msgs::TwistStamped::ConstPtr& velo_in)
  {
	  vehicle2nd_velo_ = *velo_in;
	  double linear_x, linear_y;

	  linear_x = vehicle2nd_velo_.twist.linear.x + vehicle1st_velo_.twist.linear.x + vehicle0_velo_.twist.linear.x;
	  linear_y = vehicle2nd_velo_.twist.linear.y + vehicle1st_velo_.twist.linear.y + vehicle0_velo_.twist.linear.y;
	  vehicle2nd_velocity = sqrt(linear_x*linear_x+linear_y*linear_y);
  }

   CP_projector::~CP_projector()
  {
	   cvReleaseImage(&project_image00);
	   cvReleaseImage(&project_image01);
	   cvReleaseImage(&project_image02);
	   cvReleaseImage(&project_image11);
	   cvReleaseImage(&project_image12);
	   cvReleaseImage(&project_image22);

	   cvReleaseImage(&project_imageH00);
	   cvReleaseImage(&project_imageH01);
	   cvReleaseImage(&project_imageH02);
	   cvReleaseImage(&project_imageH11);
	   cvReleaseImage(&project_imageH12);
	   cvReleaseImage(&project_imageH22);
  }

};

int main(int argc, char** argv)
{
	 ros::init(argc, argv, "cp_projector_node");
	 ros::NodeHandle n;
	 golfcar_vision::CP_projector cp_projector_node;
    ros::spin();
    return 0;
}
