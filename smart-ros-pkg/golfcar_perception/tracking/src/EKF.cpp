#include "EKF.h"

EKF::EKF(FilterParameters parameters) :
    sys_noise_Mu_(STATE::SIZE), sys_noise_Cov_(STATE::SIZE),
    H_(MEAS::SIZE,STATE::SIZE), meas_noise_Mu_(MEAS::SIZE),
    meas_noise_Cov_(MEAS::SIZE),
    prior_Mu_(STATE::SIZE), prior_Cov_(STATE::SIZE)
{
    parameters.mu_system_noise_.apply(sys_noise_Mu_);
    parameters.sigma_system_noise_.apply(sys_noise_Cov_);

    system_Uncertainty_ = new BFL::Gaussian(sys_noise_Mu_, sys_noise_Cov_);
    sys_pdf_ = new NonLinearAnalyticConditionalGaussianMobile(*system_Uncertainty_);
    sys_model_ = new BFL::AnalyticSystemModelGaussianUncertainty(sys_pdf_);


    for( unsigned i=1; i<=MEAS::SIZE; i++ )
        for( unsigned j=1; j<=STATE::SIZE; j++ )
            H_(i,j) = 0.0;
    H_(MEAS::X,STATE::X) = H_(MEAS::Y,STATE::Y) = 1.0;

    meas_noise_Mu_(MEAS::X) = parameters.mu_meas_noise_;
    meas_noise_Mu_(MEAS::Y) = parameters.mu_meas_noise_;
    meas_noise_Cov_ = 0.0;
    meas_noise_Cov_(MEAS::X,MEAS::X) = parameters.sigma_meas_noise_;
    meas_noise_Cov_(MEAS::Y,MEAS::Y) = parameters.sigma_meas_noise_;

    measurement_Uncertainty_ = new BFL::Gaussian(meas_noise_Mu_, meas_noise_Cov_);
    meas_pdf_ = new BFL::LinearAnalyticConditionalGaussian(H_, *measurement_Uncertainty_);
    meas_model_ = new BFL::LinearAnalyticMeasurementModelGaussianUncertainty(meas_pdf_);


    parameters.prior_mu_.apply(prior_Mu_);
    parameters.prior_cov_.apply(prior_Cov_);

    prior_cont_ = new BFL::Gaussian(prior_Mu_, prior_Cov_);


    filter_ = new BFL::ExtendedKalmanFilter(prior_cont_);
}

EKF::~EKF()
{
    delete filter_;
    delete prior_cont_;
    delete meas_model_;
    delete meas_pdf_;
    delete measurement_Uncertainty_;
    delete sys_model_;
    delete sys_pdf_;
    delete system_Uncertainty_;
}

void EKF::update(double x, double y, double dt)
{
    MatrixWrapper::ColumnVector measurement(MEAS::SIZE);
    measurement(MEAS::X) = x;
    measurement(MEAS::Y) = y;
    MatrixWrapper::ColumnVector input(1);
    input(1) = dt;
    filter_->Update(sys_model_, input, meas_model_, measurement);
}

MatrixWrapper::ColumnVector EKF::get_posterior_expected_value()
{
    BFL::Pdf<MatrixWrapper::ColumnVector> * posterior = filter_->PostGet();
    return posterior->ExpectedValueGet();
}

MatrixWrapper::SymmetricMatrix EKF::get_posterior_cov()
{
    BFL::Pdf<MatrixWrapper::ColumnVector> * posterior = filter_->PostGet();
    return posterior->CovarianceGet();
}
