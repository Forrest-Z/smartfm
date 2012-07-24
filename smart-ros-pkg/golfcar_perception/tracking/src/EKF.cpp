#include "EKF.h"

#define STATE_SIZE 5
#define MEAS_SIZE 2


EKF_Param::EKF_Param() :
    x_(0), y_(0), theta_(0), v_(0), w_(0)
{
}

void EKF_Param::apply(MatrixWrapper::ColumnVector & mu) const
{
    mu(1) = x_;
    mu(2) = y_;
    mu(3) = theta_;
    mu(4) = v_;
    mu(5) = w_;
}

void EKF_Param::apply(MatrixWrapper::SymmetricMatrix & cov) const
{
    cov = 0;
    cov(1,1) = x_;
    cov(2,2) = y_;
    cov(3,3) = theta_;
    cov(4,4) = v_;
    cov(5,5) = w_;
}


EKF::EKF() :
    mu_meas_noise_(0.0), sigma_meas_noise_(0.0),
    sys_noise_Mu_(STATE_SIZE), sys_noise_Cov_(STATE_SIZE),
    system_Uncertainty_(sys_noise_Mu_, sys_noise_Cov_),
    sys_pdf_(system_Uncertainty_),
    sys_model_(&sys_pdf_),
    H_(MEAS_SIZE,STATE_SIZE), meas_noise_Mu_(MEAS_SIZE), meas_noise_Cov_(MEAS_SIZE),
    measurement_Uncertainty_(meas_noise_Mu_, meas_noise_Cov_),
    meas_pdf_(H_, measurement_Uncertainty_),
    meas_model_(&meas_pdf_),
    prior_Mu_(STATE_SIZE), prior_Cov_(STATE_SIZE), prior_cont_(prior_Mu_, prior_Cov_),
    filter_(&prior_cont_)
{
    // create matrix H for linear measurement model
    H_ = 0.0;
}


void EKF::apply_parameters()
{
    mu_system_noise_.apply(sys_noise_Mu_);
    sigma_system_noise_.apply(sys_noise_Cov_);

    meas_noise_Mu_(1) = mu_meas_noise_;
    meas_noise_Cov_(1,1) = sigma_meas_noise_;

    prior_mu_.apply(prior_Mu_);
    prior_cov_.apply(prior_Cov_);
}
