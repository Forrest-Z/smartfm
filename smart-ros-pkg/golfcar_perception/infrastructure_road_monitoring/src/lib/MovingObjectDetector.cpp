#include "MovingObjectDetector.h"

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




MovingObjectDetector::MovingObjectDetector()
: diff_thresh(70),
  dilate_size(40),
  erode_size(40)
{

}

void MovingObjectDetector::diff(cv::Mat frame, cv::Mat background)
{
    // compute difference between current image and background
    cv::Mat tmp;
    cv::absdiff(background, frame, tmp);

    cvtColor(tmp, tmp, CV_BGR2GRAY);

    // apply threshold to foreground image
    cv::threshold(tmp, diffImg, diff_thresh, 255, CV_THRESH_BINARY);

    dilate();
    erode();
}

void MovingObjectDetector::dilate()
{
    //MORPH_RECT, MORPH_CROSS, MORPH_ELLIPSE
    int s = dilate_size;
    if( s<=0 ) return;
    cv::Mat element = cv::getStructuringElement( cv::MORPH_RECT,
                                       cv::Size( 2*s + 1, 2*s+1 ),
                                       cv::Point( s, s ) );
    cv::dilate( diffImg, diffImg, element );
}

void MovingObjectDetector::erode()
{
    //MORPH_RECT, MORPH_CROSS, MORPH_ELLIPSE
    int s = erode_size;
    if( s<=0 ) return;
    cv::Mat element = cv::getStructuringElement( cv::MORPH_RECT,
                                       cv::Size( 2*s + 1, 2*s+1 ),
                                       cv::Point( s, s ) );
    cv::erode( diffImg, diffImg, element );
}
