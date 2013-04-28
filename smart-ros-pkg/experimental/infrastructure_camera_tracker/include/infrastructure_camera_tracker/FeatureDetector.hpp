#ifndef __FEATURE_DETECTOR_H__
#define __FEATURE_DETECTOR_H__

#include <opencv2/opencv.hpp>

class FeatureDetector
{
  public:
    FeatureDetector();
    ~FeatureDetector();
    std::vector<cv::Point2f> getFeatures(const cv::Mat& image);
  private:
    int max_corners;
    double quality_level;
    double min_distance;
    cv::Mat mask;
    int block_size;
    bool use_Harris_detector;
};

#endif
