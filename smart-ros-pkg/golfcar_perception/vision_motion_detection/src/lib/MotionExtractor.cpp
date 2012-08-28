#include <cv_bridge/cv_bridge.h>
#include <vision_motion_detection/MotionExtractor.h>

MotionExtractor::MotionExtractor()
: it_(nh_), track_matcher_(matcher_threshold_), tracker_(track_matcher_)
{

}

MotionExtractor::MotionExtractor(std::string image_topic)
: it_(nh_), track_matcher_(matcher_threshold_), tracker_(track_matcher_)
{
    frame_sub_ = it_.subscribe(image_topic, 10, &MotionExtractor::imageCallback, this);
}

void MotionExtractor::imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    mutex_.lock();
    cv_bridge::CvImageConstPtr cv_image = cv_bridge::toCvShare(msg, "bgr8");
    background_extractor_.add(cv_image->image);
    std::vector<Blob> blobs = blob_extractor_.extract(cv_image->image,
            background_extractor_.getImg(), msg->header.stamp.toSec());
    area_filter_.filter(blobs);
    tracker_.update(blobs);
    mutex_.unlock();
}

