#include "BackgroundExtraction.h"

using namespace std;



Background::Background() : changed(true), alpha(0.005)
{

}


void Background::add(cv::Mat frame)
{
    if( prev_frame.empty() )
        prev_frame = frame.clone();

    // initialize background to 1st frame
    if( background.empty() )
        frame.convertTo(background, CV_32F);

    // accumulate prev frame to background
    cv::accumulateWeighted(prev_frame, background, alpha);

    // store current frame
    prev_frame = frame.clone();

    changed = true;
}


cv::Mat Background::getImg()
{
    if( changed )
    {
        background.convertTo(backImage, CV_8U);
        changed = false;
    }

    return backImage;
}
