#ifndef MOVINGOBJECTDETECTOR_H_
#define MOVINGOBJECTDETECTOR_H_

#include <opencv2/opencv.hpp>

class Background
{
    cv::Mat background_; ///< where accumulation is done. type CV_32F
    cv::Mat backImage_; ///< the background image in 8U type (generated and retrieved by getImg() ).
    cv::Mat prev_frame_; ///< buffer frames so that current frame is not accumulated directly.
    bool changed_; ///< whether background and backImage are out of sync

    /// accumulation weight. between 0 and 1. The lower it is, the longer it
    /// takes to 'forget' the past frames.
    double alpha_target_;

    /// Start with a large value of alpha and converge to alpha_target_.
    /// This is to prevent the first frame from taking too much importance.
    double alpha_;


public:
    Background();

    void add(cv::Mat frame);

    /// convert background to 8U
    cv::Mat getImg();

    void set_alpha(double);

    void reset();
};


#endif /* MOVINGOBJECTDETECTOR_H_ */
