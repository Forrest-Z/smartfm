#include "Filter.h"

FilterParamElement::FilterParamElement() :
    x_(0), y_(0), t_(0), v_(0), w_(0)
{
}

void FilterParamElement::apply(MatrixWrapper::ColumnVector & mu) const
{
    mu(STATE::X) = x_;
    mu(STATE::Y) = y_;
    mu(STATE::T) = t_;
    mu(STATE::V) = v_;
    mu(STATE::W) = w_;
}

void FilterParamElement::apply(MatrixWrapper::SymmetricMatrix & cov) const
{
    for( unsigned i=1; i<=STATE::SIZE; i++ )
        for( unsigned j=1; j<=STATE::SIZE; j++ )
            cov(i,j) = 0.0;

    cov(STATE::X,STATE::X) = x_;
    cov(STATE::Y,STATE::Y) = y_;
    cov(STATE::T,STATE::T) = t_;
    cov(STATE::V,STATE::V) = v_;
    cov(STATE::W,STATE::W) = w_;
}

FilterParameters::FilterParameters() :
    mu_meas_noise_(0.0), sigma_meas_noise_(0.0)
{

}

