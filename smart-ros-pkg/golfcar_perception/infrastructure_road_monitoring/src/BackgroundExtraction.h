#ifndef MOVINGOBJECTDETECTOR_H_
#define MOVINGOBJECTDETECTOR_H_

#include <opencv2/opencv.hpp>

class Background
{
    cv::Mat background; ///< where accumulation is done. type CV_32F
    cv::Mat backImage; ///< the background image in 8U type (generated and retrieved by getImg() ).
    cv::Mat prev_frame; ///< buffer frames so that current frame is not accumulated directly.
    bool changed; ///< whether background and backImage are out of sync

public:
    /// accumulation weight. between 0 and 1. The lower it is, the longer it
    /// takes to 'forget' the past frames.
    double alpha;

    Background();

    void add(cv::Mat frame);

    /// convert background to 8U
    cv::Mat getImg();
};


#endif /* MOVINGOBJECTDETECTOR_H_ */
