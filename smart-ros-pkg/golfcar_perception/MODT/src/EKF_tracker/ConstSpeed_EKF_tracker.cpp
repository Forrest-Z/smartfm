#include "ConstSpeed_EKF_tracker.h"

using namespace MatrixWrapper;
using namespace BFL;
using namespace tf;
using namespace std;
using namespace ros;

constspeed_ekf_tracker::constspeed_ekf_tracker()
{
    // create SYSTEM MODEL
    ColumnVector sysNoise_Mu(6);  sysNoise_Mu = 0;
    SymmetricMatrix sysNoise_Cov(6); sysNoise_Cov = 0;
    for (unsigned int i=1; i<=6; i++) sysNoise_Cov(i,i) = pow(1000.0,2);
    Gaussian 	system_Uncertainty(sysNoise_Mu, sysNoise_Cov);
    sys_pdf_   = new NonLinearAnalyticConditionalGaussianOdo(system_Uncertainty);
    sys_model_ = new AnalyticSystemModelGaussianUncertainty(sys_pdf_);

    // create MEASUREMENT MODEL ODOM
    ColumnVector measNoiseOdom_Mu(6);  measNoiseOdom_Mu = 0;
    SymmetricMatrix measNoiseOdom_Cov(6);  measNoiseOdom_Cov = 0;
    for (unsigned int i=1; i<=6; i++) measNoiseOdom_Cov(i,i) = 1;
    Gaussian measurement_Uncertainty_Odom(measNoiseOdom_Mu, measNoiseOdom_Cov);
    Matrix Hodom(6,6);  Hodom = 0;
    Hodom(1,1) = 1;    Hodom(2,2) = 1;    Hodom(6,6) = 1;
    meas_pdf_   = new LinearAnalyticConditionalGaussian(Hodom, measurement_Uncertainty_Odom);
    meas_model_ = new LinearAnalyticMeasurementModelGaussianUncertainty(meas_pdf_);


}

void constspeed_ekf_tracker::set_params(double sys_noise_sig)
{

}

void constspeed_ekf_tracker::predict()
{

}

void constspeed_ekf_tracker::update()
{

}

void constspeed_ekf_tracker::getEstimate(geometry_msgs::PoseWithCovarianceStamped& estimate)
{

}

constspeed_ekf_tracker::~constspeed_ekf_tracker()
{

}
