#include <vision_motion_detection/BackgroundExtraction.h>

Background::Background() : changed_(true), alpha_target_(0.005), alpha_(1.0)
{

}

void Background::reset()
{
    cv::Mat empty;
    empty.copyTo(background_);
    empty.copyTo(prev_frame_);
    changed_ = true;
    alpha_ = 1.0;
}

void Background::set_alpha(double a)
{
    alpha_target_ = a;
}

void Background::add(cv::Mat frame)
{
    changed_ = true;

    // initialize background to 1st frame
    if( background_.empty() )
    {
        frame.convertTo(background_, CV_32F);
        return;
    }

    if( prev_frame_.empty() )
    {
        prev_frame_ = frame.clone();
        return;
    }

    // make alpha converge to alpha_target
    if( fabs(alpha_-alpha_target_)<0.01 )
        alpha_ = alpha_target_;
    else
        alpha_ += (alpha_target_-alpha_) * 0.1;

    // accumulate prev frame to background
    cv::accumulateWeighted(prev_frame_, background_, alpha_);

    // store current frame
    prev_frame_ = frame.clone();
}


cv::Mat Background::getImg()
{
    if( changed_ )
    {
        background_.convertTo(backImage_, CV_8U);
        changed_ = false;
    }

    return backImage_;
}
