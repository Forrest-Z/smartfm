#ifndef MODT_CONST_SPEED_TRACKER_H
#define MODT_CONST_SPEED_TRACKER_H

#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <filter/extendedkalmanfilter.h>
#include <wrappers/matrix/matrix_wrapper.h>
#include <model/linearanalyticsystemmodel_gaussianuncertainty.h>
#include <model/linearanalyticmeasurementmodel_gaussianuncertainty.h>
#include <pdf/analyticconditionalgaussian.h>
#include <pdf/linearanalyticconditionalgaussian.h>
#include "nonlinearanalyticconditionalgaussianodo.h"

class constspeed_ekf_tracker
{
	public:

	BFL::AnalyticSystemModelGaussianUncertainty*            sys_model_;
	BFL::NonLinearAnalyticConditionalGaussianOdo*           sys_pdf_;
	BFL::LinearAnalyticConditionalGaussian*                 meas_pdf_;
	BFL::LinearAnalyticMeasurementModelGaussianUncertainty* meas_model_;
	BFL::Gaussian*                                          prior_;
	BFL::ExtendedKalmanFilter*                              filter_;
	MatrixWrapper::SymmetricMatrix                          predict_covariance, meas_covariance;

	constspeed_ekf_tracker();
	~constspeed_ekf_tracker();

	void set_params(double sys_noise_sig);
	void predict(ros::Time time);
	void update(double x, double y);
	void getEstimate(geometry_msgs::PoseWithCovarianceStamped& estimate);
};


#endif
