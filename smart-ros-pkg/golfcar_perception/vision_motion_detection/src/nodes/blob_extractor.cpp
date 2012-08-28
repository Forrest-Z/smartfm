/** Extracts blobs from current frame and background image.
 *
 * Publishes extracted blobs as an vision_motion_detection/Blobs
 * message on topic "blobs". Also publishes the diff image on topic "diff".
 *
 * It is possible to control the output rate by skipping some frames by either
 * specifying the desired period (parameter "period") or the number of frames
 * to skip (parameter "skip").
 *
 * If a ROI is defined, only extracts moving objects blobs from that region. The
 * ROI is obtained as a ROS param (defaults to full image), and it is updated
 * every one second (so roi_select can be used to set it).
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
#include <vision_motion_detection/BlobExtractorConfig.h>
using vision_motion_detection::BlobExtractorConfig;

#include <vision_motion_detection/Blobs.h>

#include <vision_motion_detection/data_types.h>
#include <vision_motion_detection/Polygon.h>
#include <vision_motion_detection/BlobExtractor.h>

using namespace std;

class BlobExtractorNode
{
public:
    BlobExtractorNode();

private:
    BlobExtractor blob_extractor_;

    ros::NodeHandle nh_, private_nh_;
    image_transport::ImageTransport it_;
    image_transport::SubscriberFilter frame_sub_, bgnd_sub_;
    ros::Timer timer_;

    typedef message_filters::sync_policies::ApproximateTime<
        sensor_msgs::Image, sensor_msgs::Image
      > MySyncPolicy;

    message_filters::Synchronizer<MySyncPolicy> synchronizer;

    ros::Publisher blobs_pub_;

    int skip_frames_, skip_frames_count_;
    double period_, last_time_;

    dynamic_reconfigure::Server<BlobExtractorConfig> server_;

    void imgCallback(const sensor_msgs::Image::ConstPtr &  frame,
            const sensor_msgs::Image::ConstPtr &  background);
    void configCallback(BlobExtractorConfig & config, uint32_t level);
    void timerCallback(const ros::TimerEvent &);
    void getRoi();
};


BlobExtractorNode::BlobExtractorNode()
: private_nh_("~"),
  it_(nh_),
  frame_sub_(it_, "camera", 20),
  bgnd_sub_(it_, "background", 20),
  synchronizer(MySyncPolicy(20), frame_sub_, bgnd_sub_),
  skip_frames_(0),
  skip_frames_count_(0),
  period_(0.0),
  last_time_(0.0)
{
    // check if a ROI is defined
    getRoi();

    synchronizer.registerCallback( boost::bind(&BlobExtractorNode::imgCallback, this, _1, _2) );

    blobs_pub_ = nh_.advertise<vision_motion_detection::Blobs>("blobs", 10);

    server_.setCallback( boost::bind(&BlobExtractorNode::configCallback, this, _1, _2) );

    // periodically update the ROI
    timer_ = nh_.createTimer(ros::Duration(1), boost::bind(&BlobExtractorNode::timerCallback, this, _1) );
}

// periodically updates the ROI
void BlobExtractorNode::timerCallback(const ros::TimerEvent & dummy)
{
    getRoi();
}

// Updates the ROI.
void BlobExtractorNode::getRoi()
{
    // the ROI is defined by a polygon as a ROS param. In ROS, an array
    // parameter is passed as a XmlRpcValue
    XmlRpc::XmlRpcValue xmlrpc;

    // Check if it is defined
    if( ros::param::get("~roi", xmlrpc) )
    {
        try {
            // update the polygon
            Polygon p;
            p.from_XmlRpc(xmlrpc);
            blob_extractor_.set_roi(p);
        } catch (runtime_error & e) {
            ROS_ERROR("ROI param has wrong format");
        }
    }
}

void BlobExtractorNode::imgCallback(const sensor_msgs::Image::ConstPtr & frame,
        const sensor_msgs::Image::ConstPtr & background)
{
    double t_now = frame->header.stamp.toSec();
    if( last_time_==0.0 ||
            (++skip_frames_count_ > skip_frames_ && t_now>last_time_+period_) )
    {
        //ROS_INFO("processing");
        skip_frames_count_ = 0;
        last_time_ = t_now;
    }
    else
    {
        //ROS_INFO("skipping");
        return;
    }

    cv_bridge::CvImageConstPtr cvImgFrame = cv_bridge::toCvShare(frame, "bgr8");
    cv_bridge::CvImageConstPtr cvImgBgnd = cv_bridge::toCvShare(background, "bgr8");

    std::vector<Blob> blobs = blob_extractor_.extract(cvImgFrame->image, cvImgBgnd->image, t_now);

    vision_motion_detection::Blobs msg;
    msg.header = frame->header;
    for(unsigned i=0; i<blobs.size(); i++)
        msg.blobs.push_back( blobDataToMsg(blobs[i]) );

    blobs_pub_.publish(msg);
}

void BlobExtractorNode::configCallback(BlobExtractorConfig & config, uint32_t level)
{
    //ROS_INFO("diff_threshold=%d, dilate_size=%d, erode_size=%d",
    //        config.diff_threshold, config.dilate_size, config.erode_size);

    if( config.skip!=skip_frames_ ) { skip_frames_ = config.skip; skip_frames_count_=0; }
    if( config.period!=period_ ) { period_=config.period; last_time_=0.0; }
    blob_extractor_.diff_thresh_ = config.diff_threshold;
    blob_extractor_.dilate_size_ = config.dilate_size;
    blob_extractor_.erode_size_ = config.erode_size;
    blob_extractor_.blurring_size_ = config.blurring_size;
    blob_extractor_.view_intermediate_images_ = config.debug_view;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "blob_extractor");
    BlobExtractorNode node;
    ros::spin();
    return 0;
}
