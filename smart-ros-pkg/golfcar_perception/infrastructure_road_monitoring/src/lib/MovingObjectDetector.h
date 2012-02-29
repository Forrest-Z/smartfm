#ifndef MOVINGOBJECTDETECTOR_H_
#define MOVINGOBJECTDETECTOR_H_

#include <cv.h>

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



class MovingObjectDetector
{
public:

    unsigned diff_thresh; ///< threshold used when converting diff image to binary
    int dilate_size; ///< how much dilation should be applied
    int erode_size;  ///< how much erosion should be applied
    cv::Mat diffImg; ///< the resulting binary image.

    MovingObjectDetector();

    /// Computes the difference between frame and background, convert to grayscale,
    /// apply threshold, then dilate and erode. The resulting binary image is
    /// stored in diffImg.
    void diff(cv::Mat frame, cv::Mat background);

private:
    void dilate();
    void erode();
};

#endif /* MOVINGOBJECTDETECTOR_H_ */
