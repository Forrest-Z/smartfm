#ifndef __FILTER_H__
#define __FILTER_H__

#include <wrappers/matrix/matrix_wrapper.h>
#include <wrappers/matrix/vector_wrapper.h>

namespace STATE
{
enum VAL {X=1, Y, T, V, W, SIZE=5};
}

namespace MEAS
{
enum VAL {X=1, Y, SIZE=2};
}

class FilterParamElement
{
public:
    ///default constructor: inits to 0
    FilterParamElement();

    double x_, y_, t_, v_, w_;

    void apply(MatrixWrapper::ColumnVector & mu) const;
    void apply(MatrixWrapper::SymmetricMatrix & cov) const;
};

class FilterParameters
{
public:
    ///default constructor: inits to 0
    FilterParameters();

    FilterParamElement mu_system_noise_, sigma_system_noise_;
    double mu_meas_noise_, sigma_meas_noise_;
    FilterParamElement prior_mu_, prior_cov_;
};


class Filter
{
public:
    virtual void update(double x, double y, double dt) = 0;
    virtual MatrixWrapper::ColumnVector get_posterior_expected_value() = 0;
    virtual MatrixWrapper::SymmetricMatrix get_posterior_cov() = 0;
};

#endif //__FILTER_H__
