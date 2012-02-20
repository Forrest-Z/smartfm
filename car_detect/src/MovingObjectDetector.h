#ifndef MOVINGOBJECTDETECTOR_H_
#define MOVINGOBJECTDETECTOR_H_

#include <cv.h>

class Background
{
    cv::Mat background, backImage, prev_frame;
    bool changed;

public:
    float alpha;

    Background();

    void add(cv::Mat frame);

    /// convert background to 8U
    cv::Mat getImg();
};



class MovingObjectDetector
{
public:
    unsigned diff_thresh;
    int dilate_size;
    int erode_size;

    cv::Mat diffImg;

    MovingObjectDetector();

    void diff(cv::Mat gray, cv::Mat backImage);
    void dilate();
    void erode();
};

#endif /* MOVINGOBJECTDETECTOR_H_ */
