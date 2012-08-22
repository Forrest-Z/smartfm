#ifndef __FILTER_H__
#define __FILTER_H__

#include <wrappers/matrix/matrix_wrapper.h>
#include <wrappers/matrix/vector_wrapper.h>
#include <pdf/gaussian.h>

namespace STATE
{
    enum VAL { X=1, Y, T, V, W, SIZE=5};
}

class Filter
{
public:
    virtual ~Filter() { }
    virtual void update(double x, double y, double dt) = 0;
    virtual MatrixWrapper::ColumnVector get_posterior_expected_value() = 0;
    virtual MatrixWrapper::SymmetricMatrix get_posterior_cov() = 0;
};

#endif //__FILTER_H__
