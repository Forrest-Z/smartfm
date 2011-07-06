#ifndef __TRACK_ESTIMATION__
#define __TRACK_ESTIMATION__

// bayesian filtering
#include <filter/extendedkalmanfilter.h>
#include <wrappers/matrix/matrix_wrapper.h>
#include <model/linearanalyticsystemmodel_gaussianuncertainty.h>
#include <model/linearanalyticmeasurementmodel_gaussianuncertainty.h>
#include <pdf/analyticconditionalgaussian.h>
#include <pdf/linearanalyticconditionalgaussian.h>
#include "nonlinearanalyticconditionalgaussiantrack.h"

// TF
#include <tf/tf.h>

// msgs
#include <geometry_msgs/PoseWithCovarianceStamped.h>

//msgs for road_detection
#include "road_detection/vec_point.h"

namespace estimation
{

class TrackEstimation
{
public:
  /// constructor
  TrackEstimation();

  /// destructor
  virtual ~TrackEstimation();

  void initialize(const road_detection::vec_point& prior);

  bool isInitialized() {return filter_initialized_;};

  // The following function consists of both "prediction" and "measurement update" steps.
  //"meas_update" will be changed in this function, shows whether or not use measurement to update;
  //"delt_phi" change the value of "delt_phi_";
  //"curb_observation" update the filter, and change the value of "curb_observation";

  bool update(const road_detection::vec_point& curb_observation,float& association_gate,bool& meas_update, bool& reinitial, const float& delt_phi, const float& tx, const float& ty);

  void getEstimate(road_detection::vec_point& estimate_value, MatrixWrapper::SymmetricMatrix& estimate_covariance);

  road_detection::vec_point                                	estimate_value_;
  MatrixWrapper::SymmetricMatrix                          	estimate_covariance_;
  bool                                                      vehicles_flag_;

private:

  BFL::AnalyticSystemModelGaussianUncertainty*            	sys_model_;
  BFL::NonLinearAnalyticConditionalGaussianTrack*         	sys_pdf_;
  BFL::LinearAnalyticConditionalGaussian*                 	meas_pdf_;
  BFL::LinearAnalyticMeasurementModelGaussianUncertainty* 	meas_model_;

  BFL::Gaussian*                                          	prior_;
  BFL::ExtendedKalmanFilter*                              	filter_;

  bool 																		filter_initialized_;
 
  road_detection::vec_point											old_observation_;
  float                                                     old_forward_dis_;


}; // class

}; // namespace

#endif
