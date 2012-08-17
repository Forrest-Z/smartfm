#include "EKF.h"

#include <fmutil/fm_math.h>

EKF::EKF(BFL::Gaussian sys_noise, BFL::Gaussian meas_noise, BFL::Gaussian prior)
: prior_cont_(prior)
{
    sys_pdf_ = new NonLinearAnalyticConditionalGaussianMobile(sys_noise);
    sys_model_ = new BFL::AnalyticSystemModelGaussianUncertainty(sys_pdf_);

    H_.resize(meas_noise.DimensionGet(), sys_noise.DimensionGet());
    for( unsigned i=1; i<=H_.rows(); i++ )
        for( unsigned j=1; j<=H_.columns(); j++ )
            H_(i,j) = (i==j ? 1 : 0);
    meas_pdf_ = new BFL::LinearAnalyticConditionalGaussian(H_, meas_noise);
    meas_model_ = new BFL::LinearAnalyticMeasurementModelGaussianUncertainty(meas_pdf_);

    filter_ = new BFL::ExtendedKalmanFilter(&prior_cont_);
    filter_->Update(sys_model_, MatrixWrapper::ColumnVector(1));
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
    measurement(STATE::X) = x;
    measurement(STATE::Y) = y;
    if( measurement.rows()==5 )
    {
        MatrixWrapper::ColumnVector state = get_posterior_expected_value();
        double dx = x - state(STATE::X);
        double dy = y - state(STATE::Y);
        measurement(STATE::T) = atan2(dy,dx);
        measurement(STATE::V) = fmutil::mag(dx,dy) / dt;
        measurement(STATE::W) = (measurement(STATE::T) - state(STATE::T)) / dt;
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
