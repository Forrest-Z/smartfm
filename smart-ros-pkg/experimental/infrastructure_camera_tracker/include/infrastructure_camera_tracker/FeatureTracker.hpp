#ifndef __FEATURE_TRACKER_HPP__
#define __FEATURE_TRACKER_HPP__

#include <opencv2/opencv.hpp>
#include <vector>

// structs to facilitate initialization of the tracker

struct detectorParameters
{
  int maximum_corners;
  double quality;
  double minimum_distance;
  cv::Mat mask;
  int block_size;
  bool use_Harris_detector;
};

struct trackerParameters
{
  cv::Size window_size;//=Size(21,21),
  int maximum_level;//=3,
  cv::TermCriteria termination_criteria;//=TermCriteria(TermCriteria::COUNT+TermCriteria::EPS, 30, 0.01),
  int flags;//=0,
  double min_eig_threshold;//=1e-4
};

class FeatureTracker
{
  public:
    FeatureTracker();

    FeatureTracker( detectorParameters detector_params,
                    trackerParameters tracker_params,
                    const cv::Mat& image); // use the image to construct the OF pyramid

    ~FeatureTracker();

    void restart( const cv::Mat& image,
                  const double& new_time);

    void update(const cv::Mat& new_image, const double& new_time);

    unsigned int getNumberActiveFeatures(void) const;

    void getActiveFeatures( std::vector<cv::Point2f>& current_position,
                            std::vector<cv::Point2f>& predicted_position,
                            const double& prediction_time ) const;
  private:
    detectorParameters detector_parameters;
    trackerParameters tracker_parameters;

    cv::Mat previous_image, current_image;
    double previous_time, current_time;

    std::vector<cv::Point2f> previous_features, current_features;
    std::vector<bool> active;
    std::vector<uint64> feature_ids;
    static uint64 id_count;

    bool predict;


    // weighted sum of vectors: result = w1*v1 + w2*v2
    static std::vector<cv::Point2f> addWeighted(  const std::vector<cv::Point2f>& v1,
                                                  const double& w1,
                                                  const std::vector<cv::Point2f>& v2,
                                                  const double& w2)
    {
      std::vector<cv::Point2f> result(v1.size());
      for (unsigned int i = 0; i < v1.size(); ++i)
      {
        result[i] = w1 * v1[i] + w2 * v2[i];
      }
      return(result);
    };

    void updateActiveFeatures(  const std::vector<cv::Point2f>& new_points,
                                const std::vector<unsigned char>& status,
                                const double& new_time);

};

#endif
