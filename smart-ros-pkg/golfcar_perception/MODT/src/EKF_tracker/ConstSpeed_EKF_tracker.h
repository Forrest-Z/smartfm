#ifndef MODT_CONST_SPEED_TRACKER_H
#define MODT_CONST_SPEED_TRACKER_H

#include <ros/ros.h>
#include <tf/tf.h>
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

	ros::Time												last_update_time;

	constspeed_ekf_tracker();
	~constspeed_ekf_tracker();

	void set_params(double sys_sig1,
					double sys_sig2,
					double sys_sig3,
					double sys_sig4,
					double sys_sig5,
					double meas_sig1,
					double meas_sig2);

	void getEstimate(ros::Time time, geometry_msgs::PoseWithCovarianceStamped& estimate);
	void update(double x, double y, ros::Time update_time);
};


#endif
