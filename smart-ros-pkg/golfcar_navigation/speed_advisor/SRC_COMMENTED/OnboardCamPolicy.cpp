#include <ros/ros.h>
#include "OnboardCamPolicy.h"


ExitingEALeftPolicy::ExitingEALeftPolicy()
: right_cam_(0)
{

}

ExitingEALeftPolicy::~ExitingEALeftPolicy()
{
    if( right_cam_ ) delete right_cam_;
}

void ExitingEALeftPolicy::update_dist(double d)
{
    if(d==0)
    {
        if( right_cam_==0 )
        {
            ROS_INFO("turning on the motion detection system");
            right_cam_ = new MotionExtractor("/cam_front_right");
        }
    }
}

bool ExitingEALeftPolicy::is_clear_to_go()
{

    return false;
}
