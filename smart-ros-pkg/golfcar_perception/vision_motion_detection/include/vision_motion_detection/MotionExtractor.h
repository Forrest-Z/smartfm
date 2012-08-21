#ifndef MOTIONEXTRACTOR_H_
#define MOTIONEXTRACTOR_H_


#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber.h>

#include <vision_motion_detection/BackgroundExtractor.h>
#include <vision_motion_detection/BlobExtractor.h>
#include <vision_motion_detection/BlobFilter.h>
#include <vision_motion_detection/TrackMatcher.h>
#include <vision_motion_detection/Tracker.h>

#include <boost/thread/mutex.hpp>

/// Combines an image subscriber, a background extractor, a blob extractor
/// and a tracker
class MotionExtractor
{
public:
    MotionExtractor();
    explicit MotionExtractor(std::string image_topic);

    /// updates the background extractor, extracts motion blobs with the blob
    /// extractor and updates the tracker
    void imageCallback(const sensor_msgs::ImageConstPtr& msg);


    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber frame_sub_;

    boost::mutex mutex_;

    BackgroundExtractor background_extractor_;
    BlobExtractor blob_extractor_;
    BlobFilterArea area_filter_;
    FixedMatcherThreshold matcher_threshold_;
    TrackMatcherNNT track_matcher_;
    Tracker tracker_;
};


#endif /* MOTIONEXTRACTOR_H_ */
