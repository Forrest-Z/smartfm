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

	  //private_nh_.param("base_frame", base_frame_, std::string("base_link"));

	  private_nh_.param("destination_frame_id", dest_frame_id_, std::string("camera_front_img"));
	  private_nh_.param("odom_frame", odom_frame_, std::string("/odom"));
	  private_nh_.param("baselink_frame", baselink_frame_, std::string("/base_link"));

	  private_nh_.param("image_height", 		image_height_,		360);
	  private_nh_.param("image_width",			image_width_,	640);

	  cloud_scan_sub_.subscribe(nh_, "/pts_rgb", 10);
	  cloud_scan_filter_ = new tf::MessageFilter<PointCloudRGB>(cloud_scan_sub_, tf_, odom_frame_, 100);
	  cloud_scan_filter_->registerCallback(boost::bind(&CP_projector::pclCallback, this, _1));
	  cloud_scan_filter_->setTolerance(ros::Duration(0.05));

	  camera_model_initialized_ = false;

	  CvSize image_size = cvSize(image_width_, image_height_);
	  color_image_ = cvCreateImage(image_size, 8,3);

	  private_nh_.param("own_image_name", own_image_name_, std::string("vehicle1_img"));
	  private_nh_.param("project_image_name", project_image_name_, std::string("vehicle2_project"));
	  private_nh_.param("merged_name", merged_name_, std::string("vehicle2_on_vehicle1"));
	  private_nh_.param("show_scale",    show_scale_,   		 1.0);

	  private_nh_.param("vehicle_length",    	vehicle_length_,   		 3.5);
	  private_nh_.param("vehicle_width",    	vehicle_width_,   		 2.0);
	  private_nh_.param("vehicle_height",    	vehicle_height_,   		 1.7);
	  vehicle_box = new vehicle_model(vehicle_length_, vehicle_width_, vehicle_height_);

	  private_nh_.param("visualize_farest_predecessor",    visualize_farest_predecessor_,   		 false);
	  private_nh_.param("predecessor_frameID", predecessor_frameID_, std::string("/robot_1/base_link"));
	  private_nh_.param("predecessor_color_mode", predecessor_color_mode_, 1);

	  predecessor_velo_sub_ = nh_.subscribe("/robot_1/vehicle_vel",10, &CP_projector::velocity_callback, this);
	  farest_velo_sub_ = nh_.subscribe("/robot_2/vehicle_vel",10, &CP_projector::farestVelo_callback, this);
  }

  void CP_projector::pclCallback(const PointCloudRGB::ConstPtr& pcl_in)
  {
	  ROS_INFO("----------CP_projector: pclCallback--111111111--------");
	  pcl_ros::transformPointCloud(dest_frame_id_, *pcl_in, pcl_batch_, tf_);
	  ROS_INFO("---pcl_batch_.points.size() %ld", pcl_batch_.points.size());

	  cvZero(color_image_);

	  if(!camera_model_initialized_)
	  {
		  ROS_WARN("camera_model not initialized yet!");
		  return;
	  }

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
				cvSet2D(color_image_, pixel.y, pixel.x, s);
			}
	  }

	  CvScalar color_plot; //CvScalar-BGR, CvRGB-RGB;
	  if(predecessor_color_mode_ == 1)color_plot=cvScalar(0,0,255);
	  else color_plot=cvScalar(255,0,0);

	  PointCloud vehicle_skeleton;
	  vehicle_skeleton.header = pcl_in->header;
	  vehicle_skeleton.header.frame_id = predecessor_frameID_.c_str();
	  vehicle_skeleton.clear();
	  vehicle_skeleton.height = 1;
	  for(size_t i=0; i<8; i++)vehicle_skeleton.push_back(vehicle_box->skeleton3D_[i]);
	  pcl_ros::transformPointCloud(dest_frame_id_, vehicle_skeleton, vehicle_skeleton, tf_);
	  for(size_t i=0; i<8; i++)
	  {
		  pcl::PointXYZ point_tmp = vehicle_skeleton.points[i];
		  cv::Point3d pt_cv(point_tmp.x, point_tmp.y, point_tmp.z);
		  cv::Point2d uv;
		  cam_model_.project3dToPixel(pt_cv, uv);
		  vehicle_box->skeletonIMG_[i] = cvPoint(uv.x, uv.y);
	  }
	  vehicle_box->DrawSkel(color_image_, color_plot, 2);

	  std_msgs::Header predecessor = vehicle_skeleton.header;
	  std_msgs::Header own_vehicle = vehicle_skeleton.header;
	  own_vehicle.frame_id = baselink_frame_;
	  predecessor_distance = distance_between_vehicles(predecessor, own_vehicle)>0.0 ? distance_between_vehicles(predecessor, own_vehicle) : predecessor_distance;
	  vehicle_box->PutInfo(color_image_, color_plot, predecessor_velocity, predecessor_distance, cvPoint(0, -5));


	  if(visualize_farest_predecessor_)
	  {
			vehicle_skeleton.header = pcl_in->header;
			vehicle_skeleton.header.frame_id = "/robot_2/base_link";
			vehicle_skeleton.clear();
			vehicle_skeleton.height = 1;
			for(size_t i=0; i<8; i++)vehicle_skeleton.push_back(vehicle_box->skeleton3D_[i]);
			pcl_ros::transformPointCloud(dest_frame_id_, vehicle_skeleton, vehicle_skeleton, tf_);
			for(size_t i=0; i<8; i++)
			{
			  pcl::PointXYZ point_tmp = vehicle_skeleton.points[i];
			  cv::Point3d pt_cv(point_tmp.x, point_tmp.y, point_tmp.z);
			  cv::Point2d uv;
			  cam_model_.project3dToPixel(pt_cv, uv);
			  vehicle_box->skeletonIMG_[i] = cvPoint(uv.x, uv.y);
			}
			vehicle_box->DrawSkel(color_image_, CV_RGB(0,0,255), 2);

			std_msgs::Header farest_predecessor = vehicle_skeleton.header;
			farest_distance = distance_between_vehicles(farest_predecessor, own_vehicle)>0.0 ? distance_between_vehicles(farest_predecessor, own_vehicle) : farest_distance;
			vehicle_box->PutInfo(color_image_, CV_RGB(0,0,255), predecessor_velocity, farest_distance, cvPoint(0, -5));
	  }

	  //cvShowImage(dest_frame_id_.c_str(), color_image_);
	  resize_show(color_image_, show_scale_, project_image_name_.c_str());
	  cvWaitKey(1);
  }

  void CP_projector::ImageCallBack( const sensor_msgs::ImageConstPtr& image_msg,
													const sensor_msgs::CameraInfoConstPtr& info_msg)
  {
        ROS_INFO("------------ImageCallBack-----------");

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
        //cvShowImage("own_image", color_image);
        resize_show(color_image, show_scale_, own_image_name_.c_str());
        IplImage* merged_image = cvCloneImage(color_image);

        cvWaitKey(1);

        int img_height 		= color_image -> height;
		int img_width  		= color_image -> width;
		for(int ih=0; ih < img_height; ih++)
		{
			for(int iw=0; iw < img_width; iw++)
			{
				CvPoint pixel;
				pixel.x = iw;
				pixel.y = ih;
				CvScalar s=cvGet2D(color_image_, pixel.y, pixel.x);
				if(s.val[0]!=0 || s.val[1]!=0 || s.val[2]!=0 )
				{
					cvSet2D(merged_image, pixel.y, pixel.x, s);
				}
			}
		}
		//cvShowImage("merged_image", merged_image);
		resize_show(merged_image, show_scale_, merged_name_.c_str());
		cvReleaseImage(&merged_image);
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

  void CP_projector::velocity_callback(const geometry_msgs::TwistStamped::ConstPtr& velo_in)
  {
	  double linear_x, linear_y;
	  linear_x = velo_in->twist.linear.x;
	  linear_y = velo_in->twist.linear.y;
	  predecessor_velocity = sqrt(linear_x*linear_x+linear_y*linear_y);
  }

  void CP_projector::farestVelo_callback(const geometry_msgs::TwistStamped::ConstPtr& velo_in)
  {
	  double linear_x, linear_y;
	  linear_x = velo_in->twist.linear.x;
	  linear_y = velo_in->twist.linear.y;
	  farest_velocity = sqrt(linear_x*linear_x+linear_y*linear_y);
  }

   CP_projector::~CP_projector()
  {
	   cvReleaseImage(&color_image_);
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
