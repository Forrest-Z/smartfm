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

	  private_nh_.param("base_frame", base_frame_, std::string("base_link"));
	  private_nh_.param("destination_frame_id", dest_frame_id_, std::string("camera_front_img"));
	  private_nh_.param("odom_frame", odom_frame_, std::string("/odom"));
	  private_nh_.param("image_height", 		image_height_,		360);
	  private_nh_.param("image_width",			image_width_,	640);

	  cloud_scan_sub_.subscribe(nh_, "/pts_rgb", 10);
	  cloud_scan_filter_ = new tf::MessageFilter<PointCloudRGB>(cloud_scan_sub_, tf_, odom_frame_, 100);
	  cloud_scan_filter_->registerCallback(boost::bind(&CP_projector::pclCallback, this, _1));
	  cloud_scan_filter_->setTolerance(ros::Duration(0.05));

	  camera_model_initialized_ = false;

	  CvSize image_size = cvSize(image_width_, image_height_);
	  color_image_ = cvCreateImage(image_size, 8,3);
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
	  cvShowImage("projected_image", color_image_);
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
        cvShowImage("own_image", color_image);

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
		cvShowImage("merged_image", merged_image);

		cvReleaseImage(&merged_image);
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
