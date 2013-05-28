#ifndef __FEATURE_DETECTOR_HPP__
#define __FEATURE_DETECTOR_HPP__

#include <opencv2/opencv.hpp>

class FeatureDetector
{
  public:
    FeatureDetector();
    FeatureDetector(  int maximum_corners, double quality, double minimum_distance, cv::Mat roi,
                      int block, bool use_Harris);
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
