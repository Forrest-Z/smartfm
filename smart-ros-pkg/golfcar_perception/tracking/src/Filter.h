#ifndef __FILTER_H__
#define __FILTER_H__

#include <wrappers/matrix/matrix_wrapper.h>
#include <wrappers/matrix/vector_wrapper.h>
#include <pdf/gaussian.h>


namespace STATE
{
    enum VAL {X=1, Y, T, V, W};
}

namespace MEAS
{
    enum VAL {X=1, Y, T, V, W};
}

class FilterParamElement
{
public:
    virtual MatrixWrapper::ColumnVector to_mu() const = 0;
    virtual MatrixWrapper::SymmetricMatrix to_cov() const = 0;
    static BFL::Gaussian createGaussian(const FilterParamElement &mu, const FilterParamElement &sigma);
    virtual ~FilterParamElement() { }
protected:
    explicit FilterParamElement(unsigned s) : size_(s) { }
    unsigned size_;
};



class FilterStateParam : public FilterParamElement
{
public:
    double x_, y_, t_, v_, w_;

    ///default constructor: inits to 0
    FilterStateParam();
    virtual ~FilterStateParam() { }
    virtual MatrixWrapper::ColumnVector to_mu() const;
    virtual MatrixWrapper::SymmetricMatrix to_cov() const;
};

class FilterMeasParam : public FilterParamElement
{
public:
    double x_, y_, t_, v_, w_;

    explicit FilterMeasParam(unsigned s);
    virtual ~FilterMeasParam() { }
    virtual MatrixWrapper::ColumnVector to_mu() const;
    virtual MatrixWrapper::SymmetricMatrix to_cov() const;
};

class FilterParameters
{
public:
    FilterStateParam mu_system_noise_, sigma_system_noise_;
    FilterStateParam prior_mu_, prior_sigma_;
    FilterMeasParam mu_meas_noise_, sigma_meas_noise_;

    ///default constructor: inits to 0
    explicit FilterParameters(unsigned s);
};


class Filter
{
public:
    virtual ~Filter() { }
    virtual void update(double x, double y, double dt) = 0;
    virtual MatrixWrapper::ColumnVector get_posterior_expected_value() = 0;
    virtual MatrixWrapper::SymmetricMatrix get_posterior_cov() = 0;
};

#endif //__FILTER_H__
