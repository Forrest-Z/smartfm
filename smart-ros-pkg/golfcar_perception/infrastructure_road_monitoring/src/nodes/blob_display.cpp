/**
 * Displays blobs. Subscribes to "blobs" and "image" topics.
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

#include <infrastructure_road_monitoring/Blobs.h>

#include "data_msg_conv.h"


class BlobDisplayNode
{
public:
    BlobDisplayNode();
    void spin();

private:
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::SubscriberFilter frame_sub_;
    message_filters::Subscriber<infrastructure_road_monitoring::Blobs> blobs_sub_;

    typedef message_filters::sync_policies::ApproximateTime<
        sensor_msgs::Image, infrastructure_road_monitoring::Blobs
      > MySyncPolicy;
    message_filters::Synchronizer<MySyncPolicy> synchronizer;

    std::string blobsWindowName_;

    void blobsCallback(const sensor_msgs::Image::ConstPtr & frame,
            const infrastructure_road_monitoring::Blobs::ConstPtr & blobs);
};


BlobDisplayNode::BlobDisplayNode()
: it_(nh_),
  frame_sub_(it_, "image", 20),
  blobs_sub_(nh_, "blobs", 20),
  synchronizer(MySyncPolicy(20), frame_sub_, blobs_sub_),
  blobsWindowName_("")
{
    synchronizer.registerCallback( boost::bind(&BlobDisplayNode::blobsCallback, this, _1, _2) );
}

void BlobDisplayNode::blobsCallback(const sensor_msgs::Image::ConstPtr & frame,
        const infrastructure_road_monitoring::Blobs::ConstPtr & blobs_msg)
{
    if( blobsWindowName_.length()==0 ) {
        blobsWindowName_ = blobs_sub_.getTopic();
        cv::namedWindow(blobsWindowName_, CV_WINDOW_NORMAL);
    }
    cv_bridge::CvImageConstPtr cvImgFrame = cv_bridge::toCvCopy(frame, "bgr8");
    cv::Mat img = cvImgFrame->image;

    for( unsigned i=0; i<blobs_msg->blobs.size(); i++ )
    {
        Blob blob = blobMsgToData( blobs_msg->blobs[i], 0 );
        blob.drawContour(img, CV_RGB(255,0,0));
        blob.drawCentroid(img, CV_RGB(255,0,0));
    }

    cv::imshow(blobsWindowName_, img);
}

void BlobDisplayNode::spin()
{
    while( ros::ok() )
    {
        if( blobsWindowName_.length()==0 )
            ros::Duration(0.1).sleep();
        else
            if( cv::waitKey(100)=='q' )
                return;
        ros::spinOnce();
    }
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "blob_display", ros::init_options::AnonymousName);
    BlobDisplayNode node;
    node.spin();
    return 0;
}
