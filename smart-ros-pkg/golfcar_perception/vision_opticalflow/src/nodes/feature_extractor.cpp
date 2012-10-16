#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber.h>
#include <std_msgs/Empty.h>
#include <boost/thread/mutex.hpp>
#include <vision_opticalflow/Feature.h> //for message type Feature.msg

// #include <dynamic_reconfigure/server.h>

#include <vision_opticalflow/FeatureExtractor.h>

class FeatureExtractorNode
{
public:
    FeatureExtractorNode();
    
private:
    ros::NodeHandle nhp_, nh_;
    image_transport::ImageTransport it_sub_;
    image_transport::Subscriber image_sub_;
    ros::Publisher feature_pub_;
    ros::Subscriber reset_sub_;

    FeatureExtractor feature_;
    boost::mutex mutex_; // protects background_

    void imageCallback(const sensor_msgs::ImageConstPtr& image);
    void resetCallback(const std_msgs::Empty &);

    //For dynamic_reconfigure option (will use this in the future)
    //dynamic_reconfigure::Server<BackgroundExtractorConfig> server_;
    //void configCallback(BackgroundExtractorConfig & config, uint32_t level);
};

FeatureExtractorNode::FeatureExtractorNode() : nhp_("~"), it_sub_(nh_)
{
    image_sub_ = it_sub_.subscribe("image", 10, &FeatureExtractorNode::imageCallback, this);
    feature_pub_  = nhp_.advertise<vision_opticalflow::Feature>("feature", 10);

    double alpha;
    nhp_.param("alpha", alpha, 0.005);
    ROS_INFO("Good to go ...");
    //feature_.set_alpha(alpha);

    //reset_sub_ = nhp_.subscribe("reset", 1, &FeatureExtractorNode::resetCallback, this);

//     server_.setCallback( boost::bind(&FeatureExtractorNode::configCallback, this, _1, _2) );
}

void FeatureExtractorNode::imageCallback(const sensor_msgs::ImageConstPtr& image)
{
    mutex_.lock();
    cv_bridge::CvImageConstPtr cv_image = cv_bridge::toCvShare(image, "bgr8");
    //Get feature
    feature_.goodFeatures(cv_image->image);
    //Do optical flow
    feature_.opticalFlow();
    
    mutex_.unlock();
    
    //Display result
//     dispImg.image = feature_.getImg();
//     feature_pub_.publish(dispImg.toImageMsg()); //publish prev_frame_feature_ and found_frame_feature_

    //format vector for data from opencv datatype to ROS data type and publish using new defined Feature msg
    vision_opticalflow::Feature feature_msg;
    feature_msg = feature_.getFeatureToMsg(image->header);
    
    feature_pub_.publish(feature_msg);
    
    //Update frame and feature
    feature_.update_frame();
//     ROS_INFO("Published!");
}

void FeatureExtractorNode::resetCallback(const std_msgs::Empty & dummy)
{
    mutex_.lock();
    feature_.reset();
    mutex_.unlock();
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "feature_extractor");
    FeatureExtractorNode node;
    ros::spin();
    return 0;
}