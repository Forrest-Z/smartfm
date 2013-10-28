#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/message_filter.h>
#include <message_filters/subscriber.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <geometry_msgs/PoseStamped.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

namespace enc = sensor_msgs::image_encodings;
using std::string;
using std::cout;
using std::endl;

class MapImgSync{

  image_transport::SubscriberFilter camera_sub_;
  tf::MessageFilter<sensor_msgs::Image>       *camera_filter_;
  string target_frame_;
  tf::TransformListener *tf_;
  ros::Publisher pose_pub_;
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
public:
  MapImgSync() : target_frame_("/map"), it_(nh_) {
    
    camera_sub_.subscribe(it_, "camera_front/image_raw", 10);
    pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("localized_pt", 10);
    tf_ = new tf::TransformListener(); 
    camera_filter_ = new tf::MessageFilter<sensor_msgs::Image>(camera_sub_, *tf_, target_frame_, 10);
    camera_filter_->registerCallback(boost::bind(&MapImgSync::imageCallback, this, _1));
    ros::spin();
  }

  void imageCallback(sensor_msgs::ImageConstPtr cam)
  { 
    geometry_msgs::PoseStamped localized_pose;
    localized_pose.header = cam->header;
    tf::StampedTransform transform;
    tf_->lookupTransform(target_frame_, localized_pose.header.frame_id, localized_pose.header.stamp, transform);
    localized_pose.header.frame_id = target_frame_;
    localized_pose.pose.position.x = transform.getOrigin().x();
    localized_pose.pose.position.y = transform.getOrigin().y();
    localized_pose.pose.position.z = transform.getOrigin().z();
    tf::Quaternion btq = transform.getRotation();
    localized_pose.pose.orientation.x = btq.x();
    localized_pose.pose.orientation.y = btq.y();
    localized_pose.pose.orientation.z = btq.z();
    localized_pose.pose.orientation.w = btq.w();
    btScalar pitch, roll, yaw;
    btQuaternion btq_temp(btq.x(), btq.y(), btq.z(), btq.w());
	  btMatrix3x3(btq_temp).getEulerYPR(yaw, pitch, roll);
    localized_pose.pose.position.z = yaw/M_PI*180;
    pose_pub_.publish(localized_pose);
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(cam, enc::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
    //cv::imshow("sync_img", cv_ptr->image);
    //cv::waitKey(3);
    cout<<cam->header.seq<<" "<<transform.getOrigin().x()<<" "<<transform.getOrigin().y()<<" "<<yaw<<endl;
    std::stringstream ss;
    ss<<cam->header.seq<<".png";
    cv::imwrite(ss.str(), cv_ptr->image);
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "MapImgSync");
  MapImgSync mis;
  return 0;
}
  
