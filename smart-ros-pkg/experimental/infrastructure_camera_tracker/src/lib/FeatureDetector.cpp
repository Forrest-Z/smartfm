#include <infrastructure_camera_tracker/FeatureDetector.hpp>

FeatureDetector::FeatureDetector()
{
  // initialize with the default values from opencv
  max_corners         = 1000;
  quality_level       = 0.01;
  min_distance        = 10;
  mask                = cv::Mat();
  block_size          = 3;
  use_Harris_detector = false;
};

FeatureDetector::FeatureDetector( int maximum_corners = 1000,
                                  double quality = 0.01,
                                  double minimum_distance = 10,
                                  cv::Mat roi = cv::Mat(),
                                  int block = 3,
                                  bool use_Harris = false)
{
  max_corners         = maximum_corners;
  quality_level       = quality;
  min_distance        = minimum_distance;
  mask                = roi;
  block_size          = block;
  use_Harris_detector = use_Harris;
}

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