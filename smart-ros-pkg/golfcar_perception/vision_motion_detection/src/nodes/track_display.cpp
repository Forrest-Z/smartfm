/**
 * Displays tracks. Subscribes to "tracks" and "image" topics.
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

#include <vision_motion_detection/Tracks.h>
#include <vision_motion_detection/TrackMatcher.h>

#include <vision_motion_detection/data_types.h>


class TrackDisplayNode
{
public:
    TrackDisplayNode();
    void spin();

private:
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::SubscriberFilter frame_sub_;
    message_filters::Subscriber<vision_motion_detection::Tracks> tracks_sub_;

    typedef message_filters::sync_policies::ApproximateTime<
        sensor_msgs::Image, vision_motion_detection::Tracks
      > MySyncPolicy;
    message_filters::Synchronizer<MySyncPolicy> synchronizer;

    std::string tracksWindowName_;

    void tracksCallback(const sensor_msgs::Image::ConstPtr & frame,
            const vision_motion_detection::Tracks::ConstPtr & tracks);
};


TrackDisplayNode::TrackDisplayNode()
: it_(nh_),
  frame_sub_(it_, "image", 20),
  tracks_sub_(nh_, "tracks", 20),
  synchronizer(MySyncPolicy(20), frame_sub_, tracks_sub_),
  tracksWindowName_("")
{
    synchronizer.registerCallback( boost::bind(&TrackDisplayNode::tracksCallback, this, _1, _2) );
}

void TrackDisplayNode::tracksCallback(const sensor_msgs::Image::ConstPtr & frame,
        const vision_motion_detection::Tracks::ConstPtr & tracks_msg)
{
    if( tracksWindowName_.length()==0 ) {
        tracksWindowName_ = tracks_sub_.getTopic();
        cv::namedWindow(tracksWindowName_, CV_WINDOW_NORMAL);
    }
    cv_bridge::CvImageConstPtr cvImgFrame = cv_bridge::toCvCopy(frame, "bgr8");
    cv::Mat img = cvImgFrame->image;

    for( unsigned i=0; i<tracks_msg->tracks.size(); i++ )
    {
        Track track = trackMsgToData( tracks_msg->tracks[i] );
        track.observations.back().drawContour(img, CV_RGB(255,0,0));
        track.observations.back().drawCentroid(img, CV_RGB(255,0,0));
        track.display(img, CV_RGB(255,0,0));
    }

    cv::imshow(tracksWindowName_, img);
}

void TrackDisplayNode::spin()
{
    while( ros::ok() )
    {
        if( tracksWindowName_.length()==0 )
            ros::Duration(0.1).sleep();
        else
            if( cv::waitKey(100)=='q' )
                return;
        ros::spinOnce();
    }
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "track_display", ros::init_options::AnonymousName);
    TrackDisplayNode node;
    node.spin();
    return 0;
}
