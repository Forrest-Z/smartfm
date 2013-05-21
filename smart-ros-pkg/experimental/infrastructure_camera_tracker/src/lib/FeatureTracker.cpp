#include <infrastructure_camera_tracker/FeatureTracker.hpp>

uint64 FeatureTracker::id_count = 0;

FeatureTracker::FeatureTracker(): predict(false)
{
  detector_parameters.maximum_corners = 100;
  detector_parameters.quality = 0.1;
  detector_parameters.minimum_distance = 1;
  detector_parameters.mask = cv::Mat();
  detector_parameters.block_size = 3;
  detector_parameters.use_Harris_detector = true;

  tracker_parameters.window_size = cv::Size(3, 3);
  tracker_parameters.maximum_level = 5;
  tracker_parameters.termination_criteria = cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 20, 0.3);
  tracker_parameters.flags = 0; //cv::OPTFLOW_USE_INITIAL_FLOW;//1;
  tracker_parameters.min_eig_threshold = 1e-4;
};

// MISSING: constructor taking <detector_parameters> and <tracker_parameters> structs as arguments to
// initialize the detector and the tracker

FeatureTracker::~FeatureTracker()
{

};

void FeatureTracker::restart( const cv::Mat& image,
                              const double& new_time)
{
  cv::cvtColor(image, current_image, CV_BGR2GRAY);                              // convert image to gray scale and keep it as the current image

  current_time = new_time;                                                      // store image's time

  current_features = std::vector<cv::Point2f>();                                // clean-up the feature vectors
  previous_features = std::vector<cv::Point2f>();

  cv::goodFeaturesToTrack(current_image, current_features,                      // run good features to track
                          detector_parameters.maximum_corners,
                          detector_parameters.quality,
                          detector_parameters.minimum_distance);
                          // detector_parameters.mask,
                          // detector_parameters.block_size,
                          // detector_parameters.use_Harris_detector);

  active = std::vector<bool>(current_features.size(), true);                    // mark all features as active

  feature_ids.resize(active.size());                                            // assign IDs
  for (unsigned int i = 0; i < active.size(); ++i)
  {
    feature_ids[i] = ++id_count;
  }

  predict = false;                                                              // only after receiving one image will we have enough information to make a prediction
};

// Naming convention in use:
// previous  = k-1
// current   = k
// new       = k+1
void FeatureTracker::update( const cv::Mat& new_image_bgr, const double& new_time)
{
  cv::Mat new_image;
  cv::cvtColor(new_image_bgr, new_image, CV_BGR2GRAY);                          // convert new image to grayscale

  std::vector<cv::Point2f>  previous_active_features,
                            current_active_features,
                            predicted_active_features;

  // get active features and predict their position in the image received
  if(predict)
  {
    for (unsigned int i = 0; i < current_features.size(); ++i)
    {
      if(active.at(i))
      {
        previous_active_features.push_back(previous_features.at(i));
        current_active_features.push_back(current_features.at(i));
      }
    }
    predicted_active_features = addWeighted(current_active_features, 1.0,
                                            previous_active_features, -1.0);
    double delta = ((new_time - current_time)/(current_time - previous_time));
    predicted_active_features = addWeighted(current_active_features, 1.0,
                                            previous_active_features, delta);
  }
  else
  {
    for (unsigned int i = 0; i < current_features.size(); ++i)
    {
      if(active.at(i))
      {
        current_active_features.push_back(current_features.at(i));
      }
    }
    predicted_active_features = current_active_features;
  }

  std::vector<unsigned char> status(predicted_active_features.size());
  std::vector<float> err(predicted_active_features.size());
  cv::calcOpticalFlowPyrLK( current_image, new_image,                           // compute optical flow
                            current_active_features, predicted_active_features,
                            status, err);//,
                            // tracker_parameters.window_size, tracker_parameters.maximum_level,
                            // tracker_parameters.termination_criteria, tracker_parameters.flags,
                            // tracker_parameters.min_eig_threshold);

  double max = -1.0;
  for(unsigned int i = 0; i < err.size(); ++i)
    err[i] > max ? max=err[i]:max = max;
  std::cout << err.size() << " active features, maximum error="<< max << std::endl;

  updateActiveFeatures(predicted_active_features, status, new_time);            // update features and time

  previous_image = current_image;                                               // update images
  current_image = new_image;

  cv::Mat frame;
  new_image_bgr.copyTo(frame);
  for(unsigned int i = 0; i<current_features.size(); ++i)
  {
    if(active.at(i))
    {
      if(err[i]>0)
      {
        if(err[i] < 50.0)
          cv::circle(frame, current_features.at(i), static_cast<int>(err[i]), cv::Scalar(255, 0, 0, 0), 1, CV_AA, 0 );
        else
          cv::circle(frame, current_features.at(i), 50, cv::Scalar(255, 0, 255, 0), 1, CV_AA, 0 );
      }
    }
  }
  // DEBUG code:
  cv::imshow("debug", frame);
  cvWaitKey(3);
};

void FeatureTracker::updateActiveFeatures(  const std::vector<cv::Point2f>& new_points,
                                            const std::vector<unsigned char>& status,
                                            const double& new_time)
{
  previous_features = current_features;
  predict = true;                                                               // at this point we have enough samples to be able to make predictions
  unsigned int j = 0;
  for (unsigned int i = 0; i < current_features.size(); ++i)
  {
    if(active.at(i))
    {
      if (1 == static_cast<int>(status[j]))                                     // active feature
      {
        current_features[i] = new_points[j];                                    // non-active features will hold their value
      }
      else
      {
        active[i] = false;
      }
      ++j;
    }
  }

  previous_time = current_time;                                                 // update time stamps
  current_time = new_time;
};