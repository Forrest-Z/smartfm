/** Tests tracking with the BFL. Single target to make it easy.
 *
 * One target is moving in 2D (x,y). The motion is defined as a list of waypoints
 * but the exact trajectory is smoothed along those waypoints. This is done using
 * the GSL library. The observations are drawn from the true trajectory plus some
 * gaussian noise.
 *
 * The model estimates the position and velocity of the particles using an extended
 * Kalman filter, as in:
 * http://people.mech.kuleuven.be/~tdelaet/bfl_doc/getting_started_guide/node18.html
 * The tutorial's code was modified so that v and w are not inputs but part of the
 * state. The input is the time delta dt.
 *
 * The result can be visualized with GNU plot.
 */

#include <stdio.h>

#include <fmutil/fm_math.h>

#include "Path.h"
#include "EKF.h"


double waypoints[][3] = {
    {0,  0,   0},
    {10, 20,  50},
    {5,  30,  50},
    {10, 50,  25},
    {5,  100, 50},
    {10, 50,  100},
    {10, 25,  75},
    {3,  15,  85},
    {5,  50,  150},
    {15, 150, 100}
};


void log(FILE *flog, double t, Path &path, MatrixWrapper::ColumnVector &measurement, EKF &ekf)
{
    MatrixWrapper::ColumnVector state = ekf.get_posterior_expected_value();
    MatrixWrapper::SymmetricMatrix pcov = ekf.get_posterior_cov();
    double cov = 0;
    for( unsigned i=1; i<=STATE::SIZE; i++ )
        for( unsigned j=1; j<=i; j++ )
            cov += pcov(i,j);

    double e = fmutil::distance(path.get_x(t), state(STATE::X), path.get_y(t), state(STATE::Y));
    printf("t=%.1f\tp=(%.1f,%.1f)\te=%.1f\tcov=%g\n", t, path.get_x(t), path.get_y(t), e, sqrt(cov));

    fprintf(flog, "%g, %g, %g, %g, %g, %g, %g, %g, %g, %g, %g, %g, %g, %g\n", t,
            path.get_x(t), path.get_y(t), path.get_t(t), path.get_v(t), path.get_w(t),
            measurement(MEAS::X), measurement(MEAS::Y),
            state(STATE::X), state(STATE::Y), state(STATE::T), state(STATE::V),
            state(STATE::W), sqrt(cov));
}

int main()
{
    Path path(waypoints, sizeof(waypoints)/sizeof(double)/3);

    // Some measurement noise to add to the ground truth
    MatrixWrapper::ColumnVector meas_noise_Mu(MEAS::SIZE);
    meas_noise_Mu = 0.0;
    MatrixWrapper::SymmetricMatrix meas_noise_Cov(MEAS::SIZE);
    meas_noise_Cov = 0.0;
    meas_noise_Cov(MEAS::X,MEAS::X) = meas_noise_Cov(MEAS::Y,MEAS::Y) = pow(4.0, 2);
    BFL::Gaussian meas_noise(meas_noise_Mu, meas_noise_Cov);

    // set the parameters of the EKF: system noise, measurement noise, etc.
    FilterParameters params;
    params.sigma_system_noise_.x_ = params.sigma_system_noise_.y_ = pow(1,2);
    params.sigma_system_noise_.t_ = pow(5*M_PI/180,2);
    params.sigma_system_noise_.v_ = pow(1,2);
    params.sigma_system_noise_.w_ = pow(1*M_PI/180,2);
    params.sigma_meas_noise_ = pow(2,2);
    path.precompute(0);
    params.prior_mu_.x_ = path.get_x(0);
    params.prior_mu_.y_ = path.get_y(0);
    params.prior_mu_.t_ = path.get_t(0);
    params.prior_mu_.v_ = path.get_v(0);
    params.prior_mu_.w_ = path.get_w(0);
    params.prior_cov_ = params.sigma_system_noise_;

    EKF ekf(params);

    FILE *flog = fopen("datalog.csv", "w");
    fprintf(flog, "time, gt_x, gt_y, gt_t, gt_v, gt_w, obs_x, obs_y, p_x, p_y, p_t, p_v, p_w, p_cov\n");

    MatrixWrapper::ColumnVector measurement(MEAS::SIZE);
    BFL::Sample<MatrixWrapper::ColumnVector> measurement_noise(MEAS::SIZE);
    const double period = 0.5;

    for (double t=path.get_tmin()+period; t<=/*15*period*/path.get_tmax(); t+=period)
    {
        path.precompute(t);
        measurement(MEAS::X) = path.get_x(t);
        measurement(MEAS::Y) = path.get_y(t);
        meas_noise.SampleFrom(measurement_noise);
        measurement += measurement_noise.ValueGet();

        ekf.update(measurement(MEAS::X), measurement(MEAS::Y), period);

        log(flog, t, path, measurement, ekf);
    }
    fclose(flog);

    return 0;
}
