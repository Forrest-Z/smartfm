#ifndef __PATH_H__
#define __PATH_H__

#include <gsl/gsl_spline.h>
#include <ostream>

class Path
{
public:

    /// waypoints are defined as a sequence of t,x,y tupples:
    /// t is actually the time from the previous waypoint, in seconds.
    Path(double waypoints[][3], unsigned n_pts);

    ~Path();

    /// If you are going to retrieve several values from the path for time t,
    /// it makes sense to call this function first to speed up a bit.
    /// This is optional though.
    void precompute(double t);

    double get_tmin() const;
    double get_tmax() const;
    double get_x(double t);
    double get_y(double t);
    double get_theta(double t);
    double get_v(double t);
    double get_w(double t);

    void print_path_xy(std::ostream &);
    void print_waypoints_xy(std::ostream &) const;

private:
    double *t_, *x_, *y_;
    unsigned n_pts_;
    gsl_interp_accel *x_acc_, *y_acc_;
    gsl_spline *x_spline_, *y_spline_;

    // buffered interpolation for efficiency
    struct
    {
        double t;
        double x;
        double y;
        double dx;
        double dy;
    } buf_interp_;
};

#endif //__PATH_H__
