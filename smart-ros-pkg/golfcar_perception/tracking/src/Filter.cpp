#include "Filter.h"


BFL::Gaussian FilterParamElement::createGaussian(const FilterParamElement &mu,
                                                 const FilterParamElement &sigma)
{
    return BFL::Gaussian(mu.to_mu(), sigma.to_cov());
}

FilterStateParam::FilterStateParam() :
    FilterParamElement(5),
    x_(0), y_(0), t_(0), v_(0), w_(0)
{
}

MatrixWrapper::ColumnVector FilterStateParam::to_mu() const
{
    MatrixWrapper::ColumnVector mu(size_);
    mu(STATE::X) = x_;
    mu(STATE::Y) = y_;
    mu(STATE::T) = t_;
    mu(STATE::V) = v_;
    mu(STATE::W) = w_;
    return mu;
}

MatrixWrapper::SymmetricMatrix FilterStateParam::to_cov() const
{
    MatrixWrapper::SymmetricMatrix cov(size_);
    for( unsigned i=1; i<=size_; i++ )
        for( unsigned j=1; j<=size_; j++ )
            cov(i,j) = 0.0;

    cov(STATE::X,STATE::X) = x_;
    cov(STATE::Y,STATE::Y) = y_;
    cov(STATE::T,STATE::T) = t_;
    cov(STATE::V,STATE::V) = v_;
    cov(STATE::W,STATE::W) = w_;
    return cov;
}



FilterMeasParam::FilterMeasParam(unsigned s) :
    FilterParamElement(s), x_(0), y_(0)
{
}

MatrixWrapper::ColumnVector FilterMeasParam::to_mu() const
{
    MatrixWrapper::ColumnVector mu(size_);
    mu(MEAS::X) = x_;
    mu(MEAS::Y) = y_;
    if(size_==5)
    {
        mu(MEAS::T) = t_;
        mu(MEAS::V) = v_;
        mu(MEAS::W) = w_;
    }
    return mu;
}

MatrixWrapper::SymmetricMatrix FilterMeasParam::to_cov() const
{
    MatrixWrapper::SymmetricMatrix cov(size_);
    for( unsigned i=1; i<=size_; i++ )
        for( unsigned j=1; j<=size_; j++ )
            cov(i,j) = 0.0;

    cov(MEAS::X,MEAS::X) = x_;
    cov(MEAS::Y,MEAS::Y) = y_;
    if(size_==5)
    {
        cov(MEAS::T,MEAS::T) = t_;
        cov(MEAS::V,MEAS::V) = v_;
        cov(MEAS::W,MEAS::W) = w_;
    }
    return cov;
}

FilterParameters::FilterParameters(unsigned s) :
    mu_meas_noise_(s), sigma_meas_noise_(s)
{

}

