/**
 * Extract blobs from current frame and background image. Publishes extracted
 * blobs as an infrastructure_road_monitoring/Blobs message on topic "blobs".
 * Also publishes the diff image on topic "diff".
 */

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/time_synchronizer.h>
#include <opencv2/highgui/highgui.hpp>

#include <dynamic_reconfigure/server.h>
#include <infrastructure_road_monitoring/BlobExtractorConfig.h>

#include <infrastructure_road_monitoring/Blobs.h>

#include "MovingObjectDetector.h"
#include "BlobExtractor.h"
#include "data_msg_conv.h"


class BlobExtractorNode
{
public:
    BlobExtractorNode();

private:
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::SubscriberFilter frame_sub_, bgnd_sub_;

    typedef message_filters::sync_policies::ApproximateTime<
        sensor_msgs::Image, sensor_msgs::Image
      > MySyncPolicy;

    message_filters::Synchronizer<MySyncPolicy> synchronizer;

    image_transport::Publisher diff_pub_;
    ros::Publisher blobs_pub_;

    MovingObjectDetector detector;
    BlobExtractor blob_extractor;

    dynamic_reconfigure::Server<infrastructure_road_monitoring::BlobExtractorConfig> server;

    void imgCallback(const sensor_msgs::Image::ConstPtr &  frame,
            const sensor_msgs::Image::ConstPtr &  background);
    void configCallback(infrastructure_road_monitoring::BlobExtractorConfig & config, uint32_t level);
};


BlobExtractorNode::BlobExtractorNode()
: it_(nh_),
  frame_sub_(it_, "camera", 20),
  bgnd_sub_(it_, "background", 20),
  synchronizer(MySyncPolicy(20), frame_sub_, bgnd_sub_)
{
    synchronizer.registerCallback( boost::bind(&BlobExtractorNode::imgCallback, this, _1, _2) );

    diff_pub_ = it_.advertise("diff", 10);
    blobs_pub_ = nh_.advertise<infrastructure_road_monitoring::Blobs>("blobs", 10);

    server.setCallback( boost::bind(&BlobExtractorNode::configCallback, this, _1, _2) );
}

void BlobExtractorNode::imgCallback(const sensor_msgs::Image::ConstPtr & frame,
        const sensor_msgs::Image::ConstPtr & background)
{
    cv_bridge::CvImageConstPtr cvImgFrame = cv_bridge::toCvShare(frame, "bgr8");
    cv_bridge::CvImageConstPtr cvImgBgnd = cv_bridge::toCvShare(background, "bgr8");

    detector.diff(cvImgFrame->image, cvImgBgnd->image);
    blob_extractor.extract(detector.diffImg, frame->header.stamp.toSec());

    infrastructure_road_monitoring::Blobs msg;
    msg.header = frame->header;
    for(unsigned i=0; i<blob_extractor.blobs.size(); i++) {
        msg.blobs.push_back( blobDataToMsg(blob_extractor.blobs[i]) );
    }
    blobs_pub_.publish(msg);

    cv_bridge::CvImage diffImg;
    diffImg.header = frame->header;
    diffImg.encoding = "mono8";
    diffImg.image = detector.diffImg;
    diff_pub_.publish(diffImg.toImageMsg());
}

void BlobExtractorNode::configCallback(infrastructure_road_monitoring::BlobExtractorConfig & config, uint32_t level)
{
    //ROS_INFO("diff_threshold=%d, dilate_size=%d, erode_size=%d",
    //        config.diff_threshold, config.dilate_size, config.erode_size);

    detector.diff_thresh = config.diff_threshold;
    detector.dilate_size = config.dilate_size;
    detector.erode_size = config.erode_size;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "blob_extractor");
    BlobExtractorNode node;
    ros::spin();
    return 0;
}
