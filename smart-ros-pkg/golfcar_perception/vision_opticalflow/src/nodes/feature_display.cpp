/**
 * Displays features. Subscribes to "features" and "image" topics.
*/

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/time_synchronizer.h>
#include <opencv2/highgui/highgui.hpp>

#include <vision_opticalflow/Feature.h>

class FeatureDisplayNode
{
public:
    FeatureDisplayNode();
    void spin();
private:
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::SubscriberFilter frame_sub_;
    message_filters::Subscriber<vision_opticalflow::Feature> features_sub_;

    typedef message_filters::sync_policies::ApproximateTime<
    sensor_msgs::Image, vision_opticalflow::Feature
    > MySyncPolicy;
    message_filters::Synchronizer<MySyncPolicy> synchronizer;

    std::string featuresWindowName_;

    void featuresCallback(const sensor_msgs::Image::ConstPtr & frame,
        const vision_opticalflow::Feature::ConstPtr & features_msg);
};

FeatureDisplayNode::FeatureDisplayNode()
: it_(nh_),
  frame_sub_(it_, "image", 20),
  features_sub_(nh_, "features", 20),
  synchronizer(MySyncPolicy(20), frame_sub_, features_sub_),
  featuresWindowName_("")
{
    synchronizer.registerCallback( boost::bind(&FeatureDisplayNode::featuresCallback, this, _1, _2) );
}

void FeatureDisplayNode::featuresCallback(const sensor_msgs::Image::ConstPtr & frame,
        const vision_opticalflow::Feature::ConstPtr & features_msg)
{
    cv::Point2f prev_feature_cv;
    cv::Point2f found_feature_cv;
    
    if( featuresWindowName_.length()==0 ) {
        featuresWindowName_ = features_sub_.getTopic();
        cv::namedWindow(featuresWindowName_, CV_WINDOW_NORMAL);
    }
    cv_bridge::CvImageConstPtr cvImgFrame = cv_bridge::toCvCopy(frame, "bgr8");
    cv::Mat img = cvImgFrame->image;

    for( unsigned i=0; i< features_msg->prev_feature.size(); i++ )
    {
        prev_feature_cv.x = features_msg->prev_feature[i].x;
        prev_feature_cv.y = features_msg->prev_feature[i].y;
        found_feature_cv.x = features_msg->found_feature[i].x;
        found_feature_cv.y = features_msg->found_feature[i].y;
        
        cv::line(img, prev_feature_cv, found_feature_cv, cv::Scalar(0,255,0));
    }
    cv::imshow(featuresWindowName_, img);
}

void FeatureDisplayNode::spin()
{
    while( ros::ok() )
    {
        if( featuresWindowName_.length()==0 )
            ros::Duration(0.1).sleep();
        else
            if( cv::waitKey(100)=='q' )
                return;
        ros::spinOnce();
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "feature_display", ros::init_options::AnonymousName);
    FeatureDisplayNode node;
    node.spin();
    return 0;
}