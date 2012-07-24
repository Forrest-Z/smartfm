#include <cassert>
#include <cmath>

#include "Path.h"

Path::Path(double waypoints[][3], unsigned n)
{
    n_pts_ = n;
    t_ = new double[n_pts_];
    x_ = new double[n_pts_];
    y_ = new double[n_pts_];
    double t = 0;
    for( unsigned i=0; i<n_pts_; i++ )
    {
        assert(i==0 || waypoints[i][0]>0);
        t += waypoints[i][0];
        t_[i] = t;
        x_[i] = waypoints[i][1];
        y_[i] = waypoints[i][2];
    }

    x_acc_ = gsl_interp_accel_alloc();
    y_acc_ = gsl_interp_accel_alloc();
    x_spline_ = gsl_spline_alloc(gsl_interp_cspline, n_pts_);
    y_spline_ = gsl_spline_alloc(gsl_interp_cspline, n_pts_);
    gsl_spline_init(x_spline_, t_, x_, n_pts_);
    gsl_spline_init(y_spline_, t_, y_, n_pts_);

    buf_interp_.t = -1;
}

Path::~Path()
{
    delete[] t_;
    delete[] x_;
    delete[] y_;
    gsl_spline_free(x_spline_);
    gsl_spline_free(y_spline_);
    gsl_interp_accel_free(x_acc_);
    gsl_interp_accel_free(y_acc_);
}

double Path::get_tmin() const
{
    return t_[0];
}

double Path::get_tmax() const
{
    return t_[n_pts_-1];
}

#define CHECK_T_(t) do{assert(t>=t_[0]); assert(t<=t_[n_pts_-1]);} while(0)

void Path::precompute(double t)
{
    CHECK_T_(t);
    buf_interp_.x  = gsl_spline_eval(x_spline_, t, x_acc_);
    buf_interp_.y  = gsl_spline_eval(y_spline_, t, y_acc_);
    buf_interp_.dx = gsl_spline_eval_deriv(x_spline_, t, x_acc_);
    buf_interp_.dx = gsl_spline_eval_deriv(y_spline_, t, y_acc_);
    buf_interp_.t = t;
}

double Path::get_x(double t)
{
    if( t==buf_interp_.t ) return buf_interp_.x;
    CHECK_T_(t);
    return gsl_spline_eval(x_spline_, t, x_acc_);
}

double Path::get_y(double t)
{
    if( t==buf_interp_.t ) return buf_interp_.y;
    CHECK_T_(t);
    return gsl_spline_eval(y_spline_, t, y_acc_);
}

double Path::get_theta(double t)
{
    double dx=0, dy=0;
    if( t==buf_interp_.t ) {
        dx = buf_interp_.dx;
        dy = buf_interp_.dy;
    }
    else {
        CHECK_T_(t);
        dx = gsl_spline_eval_deriv(x_spline_, t, x_acc_);
        dy = gsl_spline_eval_deriv(y_spline_, t, y_acc_);
    }
    return atan2(dy, dx);
}

double Path::get_v(double t)
{
    double dx=0, dy=0;
    if( t==buf_interp_.t ) {
        dx = buf_interp_.dx;
        dy = buf_interp_.dy;
    }
    else {
        CHECK_T_(t);
        dx = gsl_spline_eval_deriv(x_spline_, t, x_acc_);
        dy = gsl_spline_eval_deriv(y_spline_, t, y_acc_);
    }
    return sqrt(dx*dx + dy*dy);
}

double Path::get_w(double t)
{
    // see http://en.wikipedia.org/wiki/Curvature#Local_expressions_2
    // to compute the curvature k
    // then w = k*v

    double dx=0, dy=0;
    if( t==buf_interp_.t ) {
        dx = buf_interp_.dx;
        dy = buf_interp_.dy;
    }
    else {
        CHECK_T_(t);
        dx = gsl_spline_eval_deriv(x_spline_, t, x_acc_);
        dy = gsl_spline_eval_deriv(y_spline_, t, y_acc_);
    }
    double ddx = gsl_spline_eval_deriv2(x_spline_, t, x_acc_);
    double ddy = gsl_spline_eval_deriv2(y_spline_, t, y_acc_);

    return (dx*ddy - dy*ddx)/(dx*dx+dy*dy);
}

void Path::print_path_xy(std::ostream & stream)
{
    for(double t=t_[0]; t<=t_[n_pts_-1]; t+=0.1)
    {
        precompute(t);
        stream <<get_x(t) <<" " <<get_y(t) <<std::endl;
    }
}

void Path::print_waypoints_xy(std::ostream & stream) const
{
    for(unsigned i=0; i<n_pts_; i++)
        stream <<x_[i] <<" " <<y_[i] <<std::endl;
}
