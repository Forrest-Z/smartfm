#include "EKF.h"

#include <fmutil/fm_math.h>

EKF::EKF(FilterParameters parameters)
{
    BFL::Gaussian system_Uncertainty = FilterMeasParam::createGaussian
            (parameters.mu_system_noise_, parameters.sigma_system_noise_);
    sys_pdf_ = new NonLinearAnalyticConditionalGaussianMobile(system_Uncertainty);
    sys_model_ = new BFL::AnalyticSystemModelGaussianUncertainty(sys_pdf_);


    BFL::Gaussian measurement_Uncertainty = FilterMeasParam::createGaussian
            (parameters.mu_meas_noise_, parameters.sigma_meas_noise_);
    H_.resize(measurement_Uncertainty.DimensionGet(), system_Uncertainty.DimensionGet());
    for( unsigned i=1; i<=H_.rows(); i++ )
        for( unsigned j=1; j<=H_.columns(); j++ )
            H_(i,j) = (i==j ? 1 : 0);
    meas_pdf_ = new BFL::LinearAnalyticConditionalGaussian(H_, measurement_Uncertainty);
    meas_model_ = new BFL::LinearAnalyticMeasurementModelGaussianUncertainty(meas_pdf_);

    prior_cont_ = FilterMeasParam::createGaussian(parameters.prior_mu_, parameters.prior_sigma_);

    filter_ = new BFL::ExtendedKalmanFilter(&prior_cont_);
}

EKF::~EKF()
{
    delete filter_;
    delete meas_model_;
    delete meas_pdf_;
    delete sys_model_;
    delete sys_pdf_;
}

void EKF::update(double x, double y, double dt)
{
    MatrixWrapper::ColumnVector measurement(H_.rows());
    measurement(MEAS::X) = x;
    measurement(MEAS::Y) = y;
    if( measurement.rows()==5 )
    {
        MatrixWrapper::ColumnVector state = get_posterior_expected_value();
        double dx = x - state(STATE::X);
        double dy = y - state(STATE::Y);
        measurement(MEAS::T) = atan2(dy,dx);
        measurement(MEAS::V) = fmutil::mag(dx,dy) / dt;
        measurement(MEAS::W) = (measurement(MEAS::T) - state(STATE::T)) / dt;
    }
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
