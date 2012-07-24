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

#include <fstream>

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

struct DataLogEntry
{
    double time;

    // ground truth
    double gt_x, gt_y, gt_v, gt_w;

    // measured
    double obs_x, obs_y;

    // predicted
    double p_x, p_y, p_v, p_w;
};


std::vector<DataLogEntry> datalog;

int main()
{
    Path path(waypoints, sizeof(waypoints)/sizeof(double)/3);
    std::ofstream file("trajectory.dat");
    file <<"#m=1,S=0" <<std::endl;
    path.print_path_xy(file);
    path.precompute(0);

    EKF ekf;
    ekf.sigma_system_noise_.x_ = ekf.sigma_system_noise_.y_ = pow(1,2);
    ekf.sigma_system_noise_.theta_ = pow(5*M_PI/180,2);
    ekf.sigma_system_noise_.v_ = pow(1,2);
    ekf.sigma_system_noise_.v_ = pow(1*M_PI/180,2);
    ekf.sigma_meas_noise_ = pow(2,2);
    ekf.prior_mu_.x_ = path.get_x(0);
    ekf.prior_mu_.y_ = path.get_y(0);
    ekf.prior_mu_.theta_ = path.get_theta(0);
    ekf.prior_mu_.v_ = path.get_v(0);
    ekf.prior_mu_.w_ = path.get_w(0);
    ekf.prior_cov_ = ekf.sigma_system_noise_;
    ekf.apply_parameters();

    return 0;
}
