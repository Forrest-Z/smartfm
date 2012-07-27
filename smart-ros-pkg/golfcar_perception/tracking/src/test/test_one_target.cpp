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

#include <cstdio>

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


void log(FILE *flog, double t, Path &path, double obs_x, double obs_y, Filter &filter)
{
    MatrixWrapper::ColumnVector state = filter.get_posterior_expected_value();
    MatrixWrapper::SymmetricMatrix mcov = filter.get_posterior_cov();
    double cov = 0;
    for( unsigned i=1; i<=mcov.rows(); i++ )
        for( unsigned j=1; j<=i; j++ )
            cov += mcov(i,j);

    double e = fmutil::distance(path.get_x(t), state(STATE::X), path.get_y(t), state(STATE::Y));
    std::printf("t=%.1f\tp=(%.1f,%.1f)\te=%.1f\tcov=%g\n", t, path.get_x(t), path.get_y(t), e, sqrt(cov));

    std::fprintf(flog, "%g, %g, %g, %g, %g, %g, %g, %g, %g, %g, %g, %g, %g, %g\n", t,
            path.get_x(t), path.get_y(t), path.get_t(t), path.get_v(t), path.get_w(t),
            obs_x, obs_y,
            state(STATE::X), state(STATE::Y), state(STATE::T), state(STATE::V),
            state(STATE::W), sqrt(cov));
}

#define SQUARE(x) pow(x,2)

int main()
{
    const double PERIOD = 0.5;
    const unsigned MEAS_SIZE = 2; // 2 or 5

    Path path(waypoints, sizeof(waypoints)/sizeof(double)/3);

    // Some measurement noise to add to the ground truth
    FilterMeasParam meas_noise_mu(2), meas_noise_sigma(2);
    meas_noise_sigma.x_ = meas_noise_sigma.y_ = SQUARE(4);
    BFL::Gaussian meas_noise = FilterMeasParam::createGaussian(meas_noise_mu,
                                                               meas_noise_sigma);

    // set the parameters of the EKF: system noise, measurement noise, etc.
    FilterParameters params(MEAS_SIZE);
    double sv = 5, sw = fmutil::d2r(50);
    params.sigma_system_noise_.x_ = params.sigma_system_noise_.y_ = SQUARE(sv*PERIOD);
    params.sigma_system_noise_.t_ = SQUARE(sw*PERIOD);
    params.sigma_system_noise_.v_ = SQUARE(sv);
    params.sigma_system_noise_.w_ = SQUARE(sw);

    params.sigma_meas_noise_.x_ = params.sigma_meas_noise_.y_ = SQUARE(2);
    if( MEAS_SIZE == 5 )
    {
        params.sigma_meas_noise_.t_ = SQUARE(sw*PERIOD);
        params.sigma_meas_noise_.v_ = SQUARE(sv);
        params.sigma_meas_noise_.w_ = SQUARE(sw);
    }

    path.precompute(0);
    params.prior_mu_.x_ = path.get_x(0);
    params.prior_mu_.y_ = path.get_y(0);
    params.prior_mu_.t_ = path.get_t(0);
    params.prior_mu_.v_ = path.get_v(0);
    params.prior_mu_.w_ = path.get_w(0);
    params.prior_sigma_.x_ = params.sigma_system_noise_.x_ * 10;
    params.prior_sigma_.y_ = params.sigma_system_noise_.y_ * 10;
    params.prior_sigma_.t_ = params.sigma_system_noise_.t_ * 10;
    params.prior_sigma_.v_ = params.sigma_system_noise_.v_ * 10;
    params.prior_sigma_.w_ = params.sigma_system_noise_.w_ * 10;

    EKF ekf(params);

    std::FILE *flog = std::fopen("datalog.csv", "w");
    std::fprintf(flog, "time, gt_x, gt_y, gt_t, gt_v, gt_w, obs_x, obs_y, p_x, p_y, p_t, p_v, p_w, p_cov\n");

    MatrixWrapper::ColumnVector measurement(2);
    BFL::Sample<MatrixWrapper::ColumnVector> measurement_noise(2);

    for (double t=path.get_tmin()+PERIOD; t<=/*15*PERIOD*/path.get_tmax(); t+=PERIOD)
    {
        path.precompute(t);
        measurement(MEAS::X) = path.get_x(t);
        measurement(MEAS::Y) = path.get_y(t);
        meas_noise.SampleFrom(measurement_noise);
        measurement += measurement_noise.ValueGet();

        ekf.update(measurement(MEAS::X), measurement(MEAS::Y), PERIOD);

        log(flog, t, path, measurement(MEAS::X), measurement(MEAS::Y), ekf);
    }

    std::fclose(flog);

    return 0;
}
