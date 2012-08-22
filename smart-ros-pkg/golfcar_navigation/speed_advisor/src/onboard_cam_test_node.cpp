/*
 * A node to test the OnboardCamPolicy. Allows to control the parameters
 * via dynamic_reconfigure.
 */

#include <dynamic_reconfigure/server.h>
#include <speed_advisor/OnboardCamPolicyConfig.h>
#include <vision_motion_detection/MotionExtractor.h>

class OnboardCamNode : public MotionExtractor
{
public:
    OnboardCamNode();

private:
    ros::NodeHandle nh_;
    ros::Timer timer_;
    dynamic_reconfigure::Server<speed_advisor::OnboardCamPolicyConfig> server_;

    void configCallback(
            speed_advisor::OnboardCamPolicyConfig & config, uint32_t level);

    void timerCallback(const ros::TimerEvent &);
};


OnboardCamNode::OnboardCamNode()
: nh_("~")
{
    server_.setCallback( boost::bind(&OnboardCamNode::configCallback, this, _1, _2) );
    timer_ = nh_.createTimer(ros::Duration(1), &OnboardCamNode::timerCallback, this);
}

void OnboardCamNode::configCallback
    (speed_advisor::OnboardCamPolicyConfig & config, uint32_t level)
{
    if( config.image_topic != frame_sub_.getTopic() )
    {
        mutex_.lock();
        frame_sub_ = it_.subscribe(config.image_topic, 10,
                &MotionExtractor::imageCallback, (MotionExtractor *)this);
        mutex_.unlock();
    }

    if( level==1 )
    {
        mutex_.lock();
        background_extractor_.reset();
        tracker_.tracks.clear();
        mutex_.unlock();
    }

    background_extractor_.set_alpha(config.alpha);

    blob_extractor_.diff_thresh_ = config.diff_threshold;
    blob_extractor_.dilate_size_ = config.dilate_size;
    blob_extractor_.erode_size_ = config.erode_size;
    blob_extractor_.blurring_size_ = config.blurring_size;
    blob_extractor_.view_intermediate_images_ = config.debug_view;

    area_filter_.threshold_ = config.area_threshold;
}

void OnboardCamNode::timerCallback(const ros::TimerEvent & dummy)
{

}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "onboard_cam_test");
    OnboardCamNode node;
    ros::spin();
}
