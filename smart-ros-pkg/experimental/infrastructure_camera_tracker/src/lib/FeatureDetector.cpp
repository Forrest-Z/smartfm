#include <infrastructure_camera_tracker/FeatureDetector.hpp>

FeatureDetector::FeatureDetector()
{
    // initialize with the default values from opencv
    max_corners         = 1000;
    quality_level       = 0.01;
    min_distance        = 10;
    cv::Mat mask        = cv::Mat();
    block_size          = 3;
    use_Harris_detector = false;
};

FeatureDetector::~FeatureDetector()
{
  // do nothing
};

std::vector<cv::Point2f> FeatureDetector::getFeatures(const cv::Mat& image)
{
  std::vector<cv::Point2f> features;

  cv::Mat grayscale_image;

  cv::cvtColor(image, grayscale_image, CV_BGR2GRAY);

  cv::goodFeaturesToTrack(grayscale_image, features, max_corners, quality_level, min_distance, mask, block_size, use_Harris_detector);

  return(features);
};