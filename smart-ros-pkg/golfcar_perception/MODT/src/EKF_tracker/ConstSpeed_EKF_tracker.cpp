#include "ConstSpeed_EKF_tracker.h"

using namespace MatrixWrapper;
using namespace BFL;
using namespace tf;
using namespace std;
using namespace ros;

constspeed_ekf_tracker::constspeed_ekf_tracker():
	predict_covariance(5),
	meas_covariance(2)
{
    // create SYSTEM MODEL
    ColumnVector sysNoise_Mu(6);  sysNoise_Mu = 0;
    SymmetricMatrix sysNoise_Cov(6); sysNoise_Cov = 0;
    for (unsigned int i=1; i<=6; i++) sysNoise_Cov(i,i) = pow(1000.0,2);
    Gaussian 	system_Uncertainty(sysNoise_Mu, sysNoise_Cov);
    sys_pdf_   = new NonLinearAnalyticConditionalGaussianOdo(system_Uncertainty);
    sys_model_ = new AnalyticSystemModelGaussianUncertainty(sys_pdf_);

    // create MEASUREMENT MODEL ODOM
    ColumnVector measNoiseOdom_Mu(2);  measNoiseOdom_Mu = 0;
    SymmetricMatrix measNoiseOdom_Cov(2);  measNoiseOdom_Cov = 0;
    for (unsigned int i=1; i<=2; i++) measNoiseOdom_Cov(i,i) = 1;
    Gaussian measurement_Uncertainty_Odom(measNoiseOdom_Mu, measNoiseOdom_Cov);
    Matrix Hodom(2,5);  Hodom = 0;
    Hodom(1,1) = 1;    Hodom(2,2) = 1;
    meas_pdf_   = new LinearAnalyticConditionalGaussian(Hodom, measurement_Uncertainty_Odom);
    meas_model_ = new LinearAnalyticMeasurementModelGaussianUncertainty(meas_pdf_);
}

void constspeed_ekf_tracker::set_params(double sys_sig1,
										double sys_sig2,
										double sys_sig3,
										double sys_sig4,
										double sys_sig5,
										double meas_sig1,
										double meas_sig2)
{
	predict_covariance = 0.0;
	meas_covariance = 0.0;

	//sys_sig1, sys_sig2: 0.1;
	//sys_sig3: M_PI/180.0*30.0;
	//sys_sig4: 1.0;
	//sys_sig5: M_PI/180*10;
	predict_covariance(1,1) = pow(sys_sig1,2);
	predict_covariance(2,2) = pow(sys_sig2,2);
	predict_covariance(3,3) = pow(sys_sig3,2);
	predict_covariance(4,4) = pow(sys_sig4,2);
	predict_covariance(5,5) = pow(sys_sig5,2);

	//meas_sig: around 0.05;
	meas_covariance(1,1) 	= pow(meas_sig1,2);
	meas_covariance(2,2) 	= pow(meas_sig2,2);
}


void constspeed_ekf_tracker::update(double x, double y, ros::Time update_time)
{
	sys_pdf_->delt_time = (update_time-last_update_time).toSec();
	sys_pdf_->AdditiveNoiseSigmaSet(predict_covariance * pow(sys_pdf_->delt_time,2));
	//this vel_desi is not actually in use;
    ColumnVector vel_desi(2); vel_desi = 0;
    filter_->Update(sys_model_, vel_desi);

    ColumnVector position_xy(2);
    position_xy(1)=x; position_xy(2)=y;
    meas_pdf_->AdditiveNoiseSigmaSet(meas_covariance);
    filter_->Update(meas_model_, position_xy);

	last_update_time = update_time;
}

void constspeed_ekf_tracker::getEstimate(ros::Time time, geometry_msgs::PoseWithCovarianceStamped& estimate)
{
	ros::Time delt_time_tmp = (time-last_update_time).toSec();
	ColumnVector state_tmp = sys_pdf_->getEstimate(delt_time_tmp);
	estimate.pose.pose.position.x = state_tmp(1);
	estimate.pose.pose.position.y = state_tmp(2);
	estimate.pose.pose.position.z = 0.0;
	tf::quaternionTFToMsg(tf::createQuaternionFromYaw(state_tmp(3)), estimate.pose.pose.orientation);
	//filter_estimate_old_vec_ = filter_->PostGet()->ExpectedValueGet();
}

constspeed_ekf_tracker::~constspeed_ekf_tracker()
{

}
