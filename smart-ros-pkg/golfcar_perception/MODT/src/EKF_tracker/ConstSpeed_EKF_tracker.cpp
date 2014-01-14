#include "ConstSpeed_EKF_tracker.h"

using namespace MatrixWrapper;
using namespace BFL;
using namespace tf;
using namespace std;
using namespace ros;

constspeed_ekf_tracker::constspeed_ekf_tracker():
	predict_covariance(5),
	meas_covariance(3)
{
    // create SYSTEM MODEL
    ColumnVector sysNoise_Mu(5);  sysNoise_Mu = 0;
    SymmetricMatrix sysNoise_Cov(5); sysNoise_Cov = 0;
    for (unsigned int i=1; i<=5; i++) sysNoise_Cov(i,i) = pow(1000.0,2);
    Gaussian 	system_Uncertainty(sysNoise_Mu, sysNoise_Cov);
    sys_pdf_   = new NonLinearAnalyticConditionalGaussianOdo(system_Uncertainty);
    sys_model_ = new AnalyticSystemModelGaussianUncertainty(sys_pdf_);

    // create MEASUREMENT MODEL ODOM
    ColumnVector measNoiseOdom_Mu(3);  measNoiseOdom_Mu = 0;
    SymmetricMatrix measNoiseOdom_Cov(3);  measNoiseOdom_Cov = 0;
    for (unsigned int i=1; i<=3; i++) measNoiseOdom_Cov(i,i) = 1;

    Gaussian measurement_Uncertainty_Odom(measNoiseOdom_Mu, measNoiseOdom_Cov);
    Matrix Hodom(3,5);  Hodom = 0;
    Hodom(1,1) = 1;    Hodom(2,2) = 1;  Hodom(3,5) = 1;
    meas_pdf_   = new LinearAnalyticConditionalGaussian(Hodom, measurement_Uncertainty_Odom);
    meas_model_ = new LinearAnalyticMeasurementModelGaussianUncertainty(meas_pdf_);
}

void constspeed_ekf_tracker::init_filter(double x, double y, double thetha)
{
    ColumnVector prior_Mu(5); prior_Mu = 0.0;
    prior_Mu(1) = x;
    prior_Mu(2) = y;
    prior_Mu(3) = thetha;

    SymmetricMatrix prior_Cov(5); prior_Cov = 0.0;
    for(int i=1; i<=5; i++)prior_Cov(i,i) = 1000000.0;
    prior_Cov(1,1) = 0.01*0.01;
    prior_Cov(2,2) = 0.01*0.01;
    prior_Cov(3,3) = (M_PI/180.0*30.0)*(M_PI/180.0*30.0);

    prior_  = new Gaussian(prior_Mu,prior_Cov);
    filter_ = new ExtendedKalmanFilter(prior_);
}

void constspeed_ekf_tracker::set_params(double sys_sig1,
										double sys_sig2,
										double sys_sig3,
										double sys_sig4,
										double sys_sig5,
										double meas_sig1,
										double meas_sig2,
										double meas_sig3)
{
	predict_covariance = 0.0;
	meas_covariance = 0.0;

	predict_covariance(1,1) = pow(sys_sig1,2);
	predict_covariance(2,2) = pow(sys_sig2,2);
	predict_covariance(3,3) = pow(sys_sig3,2);
	predict_covariance(4,4) = pow(sys_sig4,2);
	predict_covariance(5,5) = pow(sys_sig5,2);

	meas_covariance(1,1) 	= pow(meas_sig1,2);
	meas_covariance(2,2) 	= pow(meas_sig2,2);
	meas_covariance(3,3) 	= pow(meas_sig3,2);
}


void constspeed_ekf_tracker::update(double x, double y, double omega, ros::Time update_time)
{
	sys_pdf_->delt_time = (update_time-last_update_time).toSec();
	ROS_ERROR("sys_pdf delt_time %lf", sys_pdf_->delt_time );
	sys_pdf_->AdditiveNoiseSigmaSet(predict_covariance * pow(sys_pdf_->delt_time,2));

	//this vel_desi is not actually in use;
    ColumnVector vel_desi(2); vel_desi = 0;
    filter_->Update(sys_model_, vel_desi);

    ColumnVector meas(3);
    meas(1)=x; meas(2)=y; meas(3)= omega;
    meas_pdf_->AdditiveNoiseSigmaSet(meas_covariance);
    filter_->Update(meas_model_, meas);

	last_update_time = update_time;
}

void constspeed_ekf_tracker::getEstimate(ros::Time time, geometry_msgs::PoseWithCovarianceStamped& estimate)
{
	double delt_time_tmp = (time-last_update_time).toSec();

	ColumnVector state_tmp = sys_pdf_->getEstimate(delt_time_tmp);

	estimate.pose.pose.position.x = state_tmp(1);
	estimate.pose.pose.position.y = state_tmp(2);
	estimate.pose.pose.position.z = 0.0;
	tf::quaternionTFToMsg(tf::createQuaternionFromYaw(state_tmp(3)), estimate.pose.pose.orientation);
	//filter_estimate_old_vec_ = filter_->PostGet()->ExpectedValueGet();
}

constspeed_ekf_tracker::~constspeed_ekf_tracker()
{
	delete            	sys_model_;
	delete           	sys_pdf_;
	delete             	meas_pdf_;
	delete 				meas_model_;
	delete           	prior_;
	delete          	filter_;

}
