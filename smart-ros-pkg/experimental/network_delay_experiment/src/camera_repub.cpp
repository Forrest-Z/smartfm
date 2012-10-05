#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/CvBridge.h>
#include <cstdio>

namespace golfcar_vision{

    class camera_repub {
        public:
        camera_repub();
        ~camera_repub(){};        
    
        private:
        ros::NodeHandle nh_, private_nh_;
        image_transport::ImageTransport it_;
        image_transport::CameraSubscriber cam_sub_;
        image_transport::Publisher image_pub_;
         sensor_msgs::CvBridge bridge_;
        void ImageCallBack(const sensor_msgs::ImageConstPtr& image_msg, const sensor_msgs::CameraInfoConstPtr& info_msg);
   };
   
   camera_repub::camera_repub():
		private_nh_("~"),
		it_(nh_)
  {
		cam_sub_ = it_.subscribeCamera("camera_front/image_raw", 1, &camera_repub::ImageCallBack, this);
		image_pub_ = it_.advertise("camera_front/image_repub", 1);
  };
  
  void camera_repub::ImageCallBack(const sensor_msgs::ImageConstPtr& image_msg, const sensor_msgs::CameraInfoConstPtr& info_msg)
  {
			 IplImage* color_image;
	  //get image in OpenCV format;
	  try {
			color_image = bridge_.imgMsgToCv(image_msg, "bgr8");
			}
	  catch (sensor_msgs::CvBridgeException& ex) {
			ROS_ERROR("Failed to convert image");
			return;
			}
			
			try
	 {
		image_pub_.publish(bridge_.cvToImgMsg(color_image, "bgr8"));
	 }
	catch (sensor_msgs::CvBridgeException error)
	 {
		ROS_ERROR("error");
	 }
	 
  };
  
};

int main(int argc, char** argv)
{
	 ros::init(argc, argv, "camera_repub");
	 ros::NodeHandle n;
	 golfcar_vision::camera_repub camera_repub_node;
     ros::spin();
     return 0;
}

