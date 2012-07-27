/** An implementation of an Extended Kalman Filter for tracking a vehicle.
 *
 * The model estimates the position and velocity of the particles using an extended
 * Kalman filter, as in:
 * http://people.mech.kuleuven.be/~tdelaet/bfl_doc/getting_started_guide/node18.html
 * The tutorial's code was modified so that v and w are not inputs but part of the
 * state. The input is the time delta dt.
 *
 * The vehicle model assumes that v and w will remain constant (with a large
 * variance though) and that x,y,theta follow a classic v.cos(theta) type of law.
 */

#ifndef __EKF_H__
#define __EKF_H__


// BFL includes
#include <filter/extendedkalmanfilter.h>
#include <model/linearanalyticsystemmodel_gaussianuncertainty.h>
#include <model/linearanalyticmeasurementmodel_gaussianuncertainty.h>
#include <pdf/analyticconditionalgaussian.h>
#include <pdf/linearanalyticconditionalgaussian.h>

#include "nonlinearanalyticconditionalgaussianmobile.h"
#include "Filter.h"

class EKF : public Filter
{
public:
    explicit EKF(FilterParameters);
    virtual ~EKF();

    virtual void update(double x, double y, double dt);
    virtual MatrixWrapper::ColumnVector get_posterior_expected_value();
    virtual MatrixWrapper::SymmetricMatrix get_posterior_cov();

private:
    /****************************
    * NonLinear system model    *
    ****************************/

    NonLinearAnalyticConditionalGaussianMobile *sys_pdf_;
    BFL::AnalyticSystemModelGaussianUncertainty *sys_model_;


    /*******************************
    * Initialise measurement model *
    *******************************/

    // create matrix H for linear measurement model
    MatrixWrapper::Matrix H_;

    BFL::LinearAnalyticConditionalGaussian *meas_pdf_;
    BFL::LinearAnalyticMeasurementModelGaussianUncertainty *meas_model_;


    /*************************
    * Linear prior DENSITY   *
    *************************/
    BFL::Gaussian prior_cont_;

    /*****************************
    * Construction of the Filter *
    *****************************/
    BFL::ExtendedKalmanFilter *filter_;
};


#endif //__EKF_H__
