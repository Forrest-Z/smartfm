#include <infrastructure_camera_tracker/FeatureTracker.hpp>

uint64 FeatureTracker::id_count = 0;

FeatureTracker::FeatureTracker(): predict(false)
{
  detector_parameters.maximum_corners = 1000;
  detector_parameters.quality = 0.01;
  detector_parameters.minimum_distance = 10;
  detector_parameters.mask = cv::Mat();
  detector_parameters.block_size = 3;
  detector_parameters.use_Harris_detector = false;

  tracker_parameters.window_size = cv::Size(21, 21);
  tracker_parameters.maximum_level = 3;
  tracker_parameters.termination_criteria = cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30, 0.01);
  tracker_parameters.flags = 0;
  tracker_parameters.min_eig_threshold = 1e-4;
};

FeatureTracker::FeatureTracker( detectorParameters detector_params,
                                trackerParameters tracker_params,
                                const cv::Mat& image)
{

};

FeatureTracker::~FeatureTracker()
{

};

void FeatureTracker::restart( const cv::Mat& image,
                              const double& new_time)
{
  // convert image to gray scale and keep it as the current image
  cv::cvtColor(image, current_image, CV_BGR2GRAY);
  // store image's time
  current_time = new_time;
  // clean-up the feature vectors
  current_features = std::vector<cv::Point2f>();
  previous_features = std::vector<cv::Point2f>();
  // run good features to track
  cv::goodFeaturesToTrack(current_image, current_features,
                          detector_parameters.maximum_corners,
                          detector_parameters.quality,
                          detector_parameters.minimum_distance,
                          detector_parameters.mask,
                          detector_parameters.block_size,
                          detector_parameters.use_Harris_detector);
  // set
  active = std::vector<bool>(current_features.size(), true);
  // assign IDs
  feature_ids.resize(active.size());
  for (unsigned int i = 0; i < active.size(); ++i)
  {
    feature_ids[i] = ++id_count;
  }

  predict = false; // only after receiving one image will we have enough information to make a prediction
};


// Naming convention in use:
// previous  = k-1
// current   = k
// new       = k+1
void FeatureTracker::update( const cv::Mat& new_image_bgr, const double& new_time)
{
  // convert new image to grayscale
  cv::Mat new_image;
  cv::cvtColor(new_image_bgr, new_image, CV_BGR2GRAY);
  // get the currently active features, and the prediction of their location in the new image
  std::vector<cv::Point2f> current_active_features, new_features;
  this->getActiveFeatures(current_active_features, new_features, new_time);
  // compute optical flow
  std::vector<unsigned char> status(new_features.size());
  std::vector<float> err(new_features.size());
  cv::calcOpticalFlowPyrLK( current_image, new_image, current_active_features, new_features, status, err,
                            tracker_parameters.window_size, tracker_parameters.maximum_level,
                            tracker_parameters.termination_criteria, tracker_parameters.flags,
                            tracker_parameters.min_eig_threshold);
  // update features and time
  this->updateActiveFeatures(new_features, status, new_time);
  // update images
  previous_image = current_image;
  current_image = new_image;
};

unsigned int FeatureTracker::getNumberActiveFeatures(void) const
{
  return(active.size());
}

//  This method returns the currently active features by reference, storing them in the
//  'current_position' vector. The method takes as an argument 'prediction_time', the time at which
//  the positions of the currently active features are to be predicted (first order hold prediction)
void FeatureTracker::getActiveFeatures( std::vector<cv::Point2f>& current_position,
                                        std::vector<cv::Point2f>& predicted_position,
                                        const double& prediction_time=0.1 ) const
{
  std::vector<cv::Point2f> previous_position;
  if(!current_position.empty())
  {
    current_position = std::vector<cv::Point2f>();
  }

  if(!predicted_position.empty())
  {
    predicted_position = std::vector<cv::Point2f>();
  }

  // get active features
  for (unsigned int i = 0; i < active.size(); ++i)
  {
    if(active[i])
    {
      previous_position.push_back(previous_features[i]);
      current_position.push_back(current_features[i]);
    }
  }
  // predict position of active features
  if(predict)
  {
    predicted_position = addWeighted( current_position, 1.0, previous_position, -1.0);
    double delta = ((prediction_time - current_time)/(current_time - previous_time));
    predicted_position = addWeighted( current_position, 1.0, previous_position, delta);
  }
};

void FeatureTracker::updateActiveFeatures(  const std::vector<cv::Point2f>& new_points,
                                            const std::vector<unsigned char>& status,
                                            const double& new_time)
{
  previous_features = current_features;
  predict = true;   // at this point we have samples at two different time instances, which is
                    // sufficient for us to predict.
  for (unsigned int i = 0; i < new_points.size(); ++i)
  {
    if (1 == static_cast<int>(status[i])) // active feature
    {
      current_features[i] = new_points[i];
    }
    else
    {
      active[i] = false;
    }
  }

  // update time stamps
  previous_time = current_time;
  current_time = new_time;
};