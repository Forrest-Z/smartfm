#ifndef DATA_DEF_H_
#define DATA_DEF_H_

#include <vector>
#include <opencv2/opencv.hpp>

class Feature
{
public:
    std::vector<cv::Point2f> prev_feature;       //feature at time n-1
    std::vector<cv::Point2f> found_feature;      //feature from time n-1 found at time n
};