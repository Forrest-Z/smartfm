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
    /* This controls the updating step:
     * 2: observe only x and y and let the filter guess the rest
     * 5: we observe only x and y but compute t, v and w from the change with
     *    the expected value.
     */
    const unsigned MEAS_SIZE = 2;

    const double PERIOD = 0.5;

    Path path(waypoints, sizeof(waypoints)/sizeof(double)/3);

    // Some measurement noise to add to the ground truth
    MatrixWrapper::SymmetricMatrix obs_noise_cov(2);
    obs_noise_cov(STATE::X,STATE::X) = obs_noise_cov(STATE::Y,STATE::Y) = SQUARE(4);
    BFL::Gaussian obs_noise(MatrixWrapper::ColumnVector(2), obs_noise_cov);

    // set the parameters of the EKF: system noise, measurement noise, etc.

    // system noise: mu=0
    double sv = .1, sw = fmutil::d2r(1);
    MatrixWrapper::SymmetricMatrix Q(STATE::SIZE);
    Q(STATE::X,STATE::X) = Q(STATE::Y,STATE::Y) = SQUARE(sv*PERIOD);
    Q(STATE::T,STATE::T) = SQUARE(sw*PERIOD);
    Q(STATE::V,STATE::V) = SQUARE(sv);
    Q(STATE::W,STATE::W) = SQUARE(sw);
    BFL::Gaussian sys_noise(MatrixWrapper::ColumnVector(STATE::SIZE), Q);

    MatrixWrapper::SymmetricMatrix R(MEAS_SIZE);
    R(STATE::X,STATE::X) = R(STATE::Y,STATE::Y) = SQUARE(10);
    if( MEAS_SIZE == STATE::SIZE )
    {
        R(STATE::T,STATE::T) = SQUARE(sw*PERIOD) * 10;
        R(STATE::V,STATE::V) = SQUARE(sv) * 10;
        R(STATE::W,STATE::W) = SQUARE(sw) * 10;
    }
    BFL::Gaussian meas_noise(MatrixWrapper::ColumnVector(MEAS_SIZE), R);

    path.precompute(0);
    MatrixWrapper::ColumnVector prior_mu(STATE::SIZE);
    prior_mu(STATE::X) = path.get_x(0);
    prior_mu(STATE::Y) = path.get_y(0);
    prior_mu(STATE::T) = path.get_t(0);
    prior_mu(STATE::V) = path.get_v(0);
    prior_mu(STATE::W) = path.get_w(0);
    MatrixWrapper::SymmetricMatrix P(STATE::SIZE);
    P = 0.01;
    BFL::Gaussian prior(prior_mu, P);

    EKF ekf(sys_noise, meas_noise, prior);

    std::FILE *flog = std::fopen("datalog.csv", "w");
    std::fprintf(flog, "time, gt_x, gt_y, gt_t, gt_v, gt_w, obs_x, obs_y, p_x, p_y, p_t, p_v, p_w, p_cov\n");

    MatrixWrapper::ColumnVector measurement(obs_noise.DimensionGet());
    BFL::Sample<MatrixWrapper::ColumnVector> measurement_noise(obs_noise.DimensionGet());

    for (double t=path.get_tmin()+PERIOD; t<=/*15*PERIOD*/path.get_tmax(); t+=PERIOD)
    {
        path.precompute(t);
        measurement(STATE::X) = path.get_x(t);
        measurement(STATE::Y) = path.get_y(t);
        obs_noise.SampleFrom(measurement_noise);
        measurement += measurement_noise.ValueGet();

        ekf.update(measurement(STATE::X), measurement(STATE::Y), PERIOD);

        log(flog, t, path, measurement(STATE::X), measurement(STATE::Y), ekf);
    }

    std::fclose(flog);

    return 0;
}
