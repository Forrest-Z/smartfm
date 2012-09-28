/*
 * created on 30/08/2012
 * by Baoxing
 * future work: write it in a general way to support multiple cameras;
 */

#include "experimental_rgb.h"


namespace golfcar_vision{
  
  experimental_rgb::experimental_rgb():
    private_nh_("~"),
    it_(nh_),
    fixedTf_inited_(false),
    process_pcl_(false)
  {
      cam_sub_ = it_.subscribeCamera("/camera_front/image_raw", 1, &experimental_rgb::ImageCallBack, this);

		odom_frame_ = "odom";
		base_frame_ = "base_link";
		
		cloud_scan_sub_.subscribe(nh_, "process_fraction", 10);
		cloud_scan_filter_ = new tf::MessageFilter<PointCloud>(cloud_scan_sub_, tf_, odom_frame_, 100);
		cloud_scan_filter_->registerCallback(boost::bind(&experimental_rgb::pclCallback, this, _1));
		cloud_scan_filter_->setTolerance(ros::Duration(0.05));
		rbg_pub_ = nh_.advertise<PointCloudRGB>("pts_rgb", 10);
  }
  
  void experimental_rgb::pclCallback(const PointCloud::ConstPtr& pcl_in)
  {
	  ROS_INFO("----------experimental_rgb::pclCallback--111111111--------");
	  
	  if(process_pcl_) ROS_WARN("experimental_rbg: last pcl batch has not been processed!!!");
	  experimental_rgb::pclXYZ_transform(odom_frame_, *pcl_in, pcl_batch_);
	  
	  ROS_INFO("---pcl_batch_.points.size() %ld", pcl_batch_.points.size());
	  process_pcl_ = true;
  }
  
  void experimental_rgb::ImageCallBack( const sensor_msgs::ImageConstPtr& image_msg,
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
        
        //get fixed transform information from "baselink" (dest) to "camera" (src);
        //pay attention to that this time the roles are interchanged between two frames;
        //right now is "camera" centered;
        if(!fixedTf_inited_)
        {
            tf::StampedTransform transform;
            try {
                ros::Time acquisition_time = info_msg->header.stamp;
                tf_.lookupTransform(cam_model_.tfFrame(), base_frame_, acquisition_time, transform);
                fixedTf_inited_ = true;
                }
				catch (tf::TransformException& ex){
                ROS_WARN("[draw_frames] TF exception:\n%s", ex.what());
                return;
					 }
					 
            tf::Transform *transform_pointer = &transform;
            src_dest_tf_ = *transform_pointer;
        }
		  
		  if(process_pcl_)
		  {
			  PointCloud 	 raw_pts;
			  PointCloudRGB rgb_pts;
			  pcl_batch_.header.frame_id = odom_frame_;
			  pcl_batch_.header.stamp = meas_time;
			  ROS_INFO("pcl_batch_.points.size() %ld", pcl_batch_.points.size());
			  pclXYZ_transform(base_frame_, pcl_batch_, raw_pts);
			  ROS_INFO("raw_pts.points.size() %ld", raw_pts.points.size());
			  curbPts_to_image(raw_pts, rgb_pts, color_image);
			  ROS_INFO("rgb_pts.points.size() %ld", rgb_pts.points.size());
			  rbg_pub_.publish(rgb_pts);
			  process_pcl_ = false;
		  }
		  else
		  {
			  ROS_INFO("image skip");
		  }
  }
  
   void experimental_rgb::curbPts_to_image(PointCloud &pts_3d,  PointCloudRGB &rgb_pts, IplImage* color_image)
   {
		rgb_pts.clear();
		rgb_pts.header = pts_3d.header;
		geometry_msgs::Pose temppose;
		temppose.position.x=0;
		temppose.position.y=0;
		temppose.position.z=0;
		temppose.orientation.x=1;
		temppose.orientation.y=0;
		temppose.orientation.z=0;
		temppose.orientation.w=0;
		tf::Pose tempTfPose;
		tf::Pose PoseInCamera;

		CvPoint pixel;
		for(size_t i=0; i<pts_3d.points.size(); i++)
		{	
			temppose.position.x=pts_3d.points[i].x;
			temppose.position.y=pts_3d.points[i].y;
			temppose.position.z=pts_3d.points[i].z;
			
			tf::poseMsgToTF(temppose, tempTfPose);
			PoseInCamera = src_dest_tf_ * tempTfPose;
         tf::Point pt = PoseInCamera.getOrigin();
			cv::Point3d pt_cv(pt.x(), pt.y(), pt.z());
         cv::Point2d uv;
         cam_model_.project3dToPixel(pt_cv, uv);
         
         pixel.x = int(floor(uv.x));
			pixel.y = int(floor(uv.y));
			
			CvScalar s;
			if(pixel.x>=640||pixel.x<0||pixel.y<0||pixel.y>=360)
			{
				s.val[0] = 0;
				s.val[1] = 0;
				s.val[2] = 255;
			}
			else 
			{
				s=cvGet2D(color_image, pixel.y, pixel.x);
			}
		   
			pcl::PointXYZRGB xyzRGB_pt;
			xyzRGB_pt.x = pts_3d.points[i].x;
			xyzRGB_pt.y = pts_3d.points[i].y;
			xyzRGB_pt.z = pts_3d.points[i].z;
			xyzRGB_pt.r = s.val[0];
			xyzRGB_pt.g = s.val[1];
			xyzRGB_pt.b = s.val[2];
			rgb_pts.points.push_back(xyzRGB_pt);
		}

	}
		
	void experimental_rgb::pclXYZ_transform(string target_frame, const PointCloud &pcl_src, PointCloud &pcl_dest)
	{
		//from current frame to "frame_id" frame: pointcloud_XYZ -> pointcloud2 -> pointcloud;
		//to manipulate the data, convert back to the format of "pointcloud_XYZ";
		sensor_msgs::PointCloud  window_tmp;
		sensor_msgs::PointCloud2 window_tmp2;
				
		pcl::toROSMsg(pcl_src, window_tmp2);
		
		sensor_msgs::convertPointCloud2ToPointCloud(window_tmp2, window_tmp);
		try
		{
			ros::Time acquisition_time = pcl_src.header.stamp;
			ros::Duration timeout(5.0 / 30);
         tf_.waitForTransform(odom_frame_, base_frame_, acquisition_time, timeout);
			tf_.transformPointCloud(target_frame, window_tmp, window_tmp);
			sensor_msgs::convertPointCloudToPointCloud2(window_tmp, window_tmp2);
			pcl::fromROSMsg(window_tmp2, pcl_dest);
		}
		catch (tf::TransformException &ex)
		{
			ROS_WARN ("Failure %s\n", ex.what()); //Print exception which was caught
			return;
		}
	}
	
  experimental_rgb::~experimental_rgb()
  {

  }
};

int main(int argc, char** argv)
{
	 ros::init(argc, argv, "experimental_rgb_node");
	 ros::NodeHandle n;
	 golfcar_vision::experimental_rgb experimental_rgb_node;
    ros::spin();
    return 0;
}
