# include "spline_fitting.h"

namespace golfcar_semantics{

	spline_fitting::spline_fitting (vector<CvPoint> raw_input, double map_ratio, double control_length)
	{
		raw_points_ = raw_input;
		map_ratio_ = map_ratio;
		control_length_ = control_length;

		data_point_Num_ = raw_points_.size();
		s_ = gsl_vector_alloc(data_point_Num_);
		x_ = gsl_vector_alloc(data_point_Num_);
		y_ = gsl_vector_alloc(data_point_Num_);
		w_ = gsl_vector_alloc(data_point_Num_);

		calc_road_length();

		breaks_Num_ = (size_t)road_length_/control_length_;
		ROS_INFO("road_length_ %3f, control_length %3f, break_num %ld", road_length_, control_length_, breaks_Num_ );
		if(breaks_Num_<2) breaks_Num_ = 2;

		coeffs_Num_ = breaks_Num_ + 2;
		bwx_ = gsl_bspline_alloc(4, breaks_Num_);
		Bx_ = gsl_vector_alloc(coeffs_Num_);
		Xx_ = gsl_matrix_alloc(data_point_Num_, coeffs_Num_);
		cx_ = gsl_vector_alloc(coeffs_Num_);
		covx_ = gsl_matrix_alloc(coeffs_Num_, coeffs_Num_);
		mwx_ = gsl_multifit_linear_alloc(data_point_Num_, coeffs_Num_);

		bwy_ = gsl_bspline_alloc(4, breaks_Num_);
		By_ = gsl_vector_alloc(coeffs_Num_);
		Xy_ = gsl_matrix_alloc(data_point_Num_, coeffs_Num_);
		cy_ = gsl_vector_alloc(coeffs_Num_);
		covy_ = gsl_matrix_alloc(coeffs_Num_, coeffs_Num_);
		mwy_ = gsl_multifit_linear_alloc(data_point_Num_, coeffs_Num_);
	}

	void spline_fitting::cubicSpline_fitting()
	{
		gsl_bspline_knots_uniform(0.0, road_length_, bwx_);
		gsl_bspline_knots_uniform(0.0, road_length_, bwy_);

		// construct the fit matrix Xx, Xy;
		for (size_t i = 0; i < data_point_Num_; ++i)
		 {
		   double si = gsl_vector_get(s_, i);

		   /* compute B_j(si) for all j */
		   //printf("s: %3f\t", si);

		   gsl_bspline_eval(si, Bx_, bwx_);
		   gsl_bspline_eval(si, By_, bwy_);

		   /* fill in row i of X */
		   for (size_t j = 0; j < coeffs_Num_; ++j)
			 {
			   double Bjx = gsl_vector_get(Bx_, j);
			   double Bjy = gsl_vector_get(By_, j);
			   gsl_matrix_set(Xx_, i, j, Bjx);
			   gsl_matrix_set(Xy_, i, j, Bjy);
			 }
		 }

		/* do the fit */
		gsl_multifit_wlinear(Xx_, w_, x_, cx_, covx_, &chisqx_, mwx_);
		gsl_multifit_wlinear(Xy_, w_, y_, cy_, covy_, &chisqy_, mwy_);

		//output fitted spline points;
		for (size_t i = 0; i < data_point_Num_; ++i)
		{
			double si, xi, yi, xerr, yerr;
			si = gsl_vector_get(s_, i);
			gsl_bspline_eval(si, Bx_, bwx_);
			gsl_bspline_eval(si, By_, bwy_);

			gsl_multifit_linear_est(Bx_, cx_, covx_, &xi, &xerr);
			gsl_multifit_linear_est(By_, cy_, covy_, &yi, &yerr);

			CvPoint output_point;
			output_point.x = (int)xi;
			output_point.y = (int)yi;
			output_points_.push_back(output_point);
		}
	}

	void spline_fitting::calc_road_length()
	{
		size_t i = 0;
		road_length_ = 0.0;
		CvPoint current_point = raw_points_[0];
		gsl_vector_set(s_, i, road_length_);
		gsl_vector_set(x_, i, double(current_point.x));
		gsl_vector_set(y_, i, double(current_point.y));
		gsl_vector_set(w_, i, 1.0);

		for(i=1; i<raw_points_.size(); i++)
		{
			double deltx = double(raw_points_[i].x-raw_points_[i-1].x);
			double delty = double(raw_points_[i].y-raw_points_[i-1].y);
			double dist_tmp = sqrt(deltx*deltx+delty*delty)*map_ratio_;
			road_length_  = road_length_ + dist_tmp;

			current_point = raw_points_[i];
			gsl_vector_set(s_, i, road_length_);
			gsl_vector_set(x_, i, double(current_point.x));
			gsl_vector_set(y_, i, double(current_point.y));
			gsl_vector_set(w_, i, 1.0);
		}
	}

	spline_fitting::~spline_fitting()
	{
		gsl_bspline_free(bwx_);	gsl_bspline_free(bwy_);
		gsl_vector_free(Bx_);	gsl_vector_free(By_);
		gsl_vector_free(s_);	gsl_vector_free(w_);
		gsl_vector_free(x_);	gsl_vector_free(y_);
		gsl_matrix_free(Xx_);	gsl_matrix_free(Xy_);
		gsl_vector_free(cx_);	gsl_vector_free(cy_);
		gsl_matrix_free(covx_);	gsl_matrix_free(covy_);
		gsl_multifit_linear_free(mwx_);
		gsl_multifit_linear_free(mwy_);
	}
};

