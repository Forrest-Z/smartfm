#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>

#include <boost/foreach.hpp>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>

// A struct to hold the synchronized camera data 
// Struct to store stereo data
class StereoData
{
public:
  sensor_msgs::Image::ConstPtr image_l, image_r;
  sensor_msgs::CameraInfo::ConstPtr cam_info_l, cam_info_r;
  
  StereoData(const sensor_msgs::Image::ConstPtr &l_img, 
             const sensor_msgs::Image::ConstPtr &r_img, 
             const sensor_msgs::CameraInfo::ConstPtr &l_info, 
             const sensor_msgs::CameraInfo::ConstPtr &r_info) :
    image_l(l_img),
    image_r(r_img),
    cam_info_l(l_info),
    cam_info_r(r_info)
  {}
};

/**
 * Inherits from message_filters::SimpleFilter<M>
 * to use protected signalMessage function 
 */
template <class M>
class BagSubscriber : public message_filters::SimpleFilter<M>
{
public:
  void newMessage(const boost::shared_ptr<M const> &msg)
  {
    signalMessage(msg);
  }
};

// Callback for synchronized messages
void callback(const sensor_msgs::Image::ConstPtr &l_img, 
              const sensor_msgs::Image::ConstPtr &r_img, 
              const sensor_msgs::CameraInfo::ConstPtr &l_info,
              const sensor_msgs::CameraInfo::ConstPtr &r_info)
{
  StereoData sd(l_img, r_img, l_info, r_info);

  // Stereo dataset is class variable to store data
  stereo_dataset_.push_back(sd);
}
 
// Load bag
void loadBag(const std::string &filename)
{
  rosbag::Bag bag;
  bag.open(filename, rosbag::bagmode::Read);
  
  std::string l_cam = image_ns_ + "/left";
  std::string r_cam = image_ns_ + "/right";
  std::string l_cam_image = l_cam + "/image_raw";
  std::string r_cam_image = r_cam + "/image_raw";
  std::string l_cam_info = l_cam + "/camera_info";
  std::string r_cam_info = r_cam + "/camera_info";
  
  // Image topics to load
  std::vector<std::string> topics;
  topics.push_back(l_cam_image);
  topics.push_back(r_cam_image);
  topics.push_back(l_cam_info);
  topics.push_back(r_cam_info);
  
  rosbag::View view(bag, rosbag::TopicQuery(topics));
  
  // Set up fake subscribers to capture images
  BagSubscriber<sensor_msgs::Image> l_img_sub, r_img_sub;
  BagSubscriber<sensor_msgs::CameraInfo> l_info_sub, r_info_sub;
  
  // Use time synchronizer to make sure we get properly synchronized images
  message_filters::TimeSynchronizer<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::CameraInfo> sync(l_img_sub, r_img_sub, l_info_sub, r_info_sub, 25);
  sync.registerCallback(boost::bind(&callback, _1, _2, _3, _4));
  
  // Load all messages into our stereo dataset
  BOOST_FOREACH(rosbag::MessageInstance const m, view)
  {
    if (m.getTopic() == l_cam_image || ("/" + m.getTopic() == l_cam_image))
    {
      sensor_msgs::Image::ConstPtr l_img = m.instantiate<sensor_msgs::Image>();
      if (l_img != NULL)
        l_img_sub.newMessage(l_img);
    }
    
    if (m.getTopic() == r_cam_image || ("/" + m.getTopic() == r_cam_image))
    {
      sensor_msgs::Image::ConstPtr r_img = m.instantiate<sensor_msgs::Image>();
      if (r_img != NULL)
        r_img_sub.newMessage(r_img);
    }
    
    if (m.getTopic() == l_cam_info || ("/" + m.getTopic() == l_cam_info))
    {
      sensor_msgs::CameraInfo::ConstPtr l_info = m.instantiate<sensor_msgs::CameraInfo>();
      if (l_info != NULL)
        l_info_sub.newMessage(l_info);
    }
    
    if (m.getTopic() == r_cam_info || ("/" + m.getTopic() == r_cam_info))
    {
      sensor_msgs::CameraInfo::ConstPtr r_info = m.instantiate<sensor_msgs::CameraInfo>();
      if (r_info != NULL)
        r_info_sub.newMessage(r_info);
    }
  }
  bag.close();
}
