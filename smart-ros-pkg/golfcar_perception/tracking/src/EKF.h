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
#include <bfl/filter/extendedkalmanfilter.h>
#include <bfl/model/linearanalyticsystemmodel_gaussianuncertainty.h>
#include <bfl/model/linearanalyticmeasurementmodel_gaussianuncertainty.h>
#include <bfl/pdf/analyticconditionalgaussian.h>
#include <bfl/pdf/linearanalyticconditionalgaussian.h>

#include "nonlinearanalyticconditionalgaussianmobile.h"

class EKF_Param
{
public:
    EKF_Param();
    double x_, y_, theta_, v_, w_;
    void apply(MatrixWrapper::ColumnVector & mu) const;
    void apply(MatrixWrapper::SymmetricMatrix & cov) const;
};


class EKF
{
public:
    EKF();

    EKF_Param mu_system_noise_, sigma_system_noise_;
    double mu_meas_noise_, sigma_meas_noise_;
    EKF_Param prior_mu_, prior_cov_;

    void apply_parameters();

private:
    /****************************
    * NonLinear system model    *
    ****************************/

    // create gaussian
    MatrixWrapper::ColumnVector sys_noise_Mu_;
    MatrixWrapper::SymmetricMatrix sys_noise_Cov_;
    BFL::Gaussian system_Uncertainty_;

    // create the model
    NonLinearAnalyticConditionalGaussianMobile sys_pdf_;
    BFL::AnalyticSystemModelGaussianUncertainty sys_model_;


    /*******************************
    * Initialise measurement model *
    *******************************/

    // create matrix H for linear measurement model
    MatrixWrapper::Matrix H_;

    // Construct the measurement noise (a scalar in this case)
    MatrixWrapper::ColumnVector meas_noise_Mu_;
    MatrixWrapper::SymmetricMatrix meas_noise_Cov_;
    BFL::Gaussian measurement_Uncertainty_;

    // create the measurement model
    BFL::LinearAnalyticConditionalGaussian meas_pdf_;
    BFL::LinearAnalyticMeasurementModelGaussianUncertainty meas_model_;


    /*************************
    * Linear prior DENSITY   *
    *************************/

    // Continuous Gaussian prior (for Kalman filters)
    MatrixWrapper::ColumnVector prior_Mu_;
    MatrixWrapper::SymmetricMatrix prior_Cov_;
    BFL::Gaussian prior_cont_;


public:
    /*****************************
    * Construction of the Filter *
    *****************************/
    BFL::ExtendedKalmanFilter filter_;
};


#endif //__EKF_H__
