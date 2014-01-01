#ifndef RANSAC_MODEL_LSHAPE_H
#define RANSAC_MODEL_LSHAPE_H

//#include <Eigen/Dense>
#include <ros/ros.h>
#include <mrpt/base.h>
#include <mrpt/gui.h>
#include <mrpt/utils.h>
#include <mrpt/math.h>
#include <sensor_msgs/PointCloud.h>

using namespace mrpt;
using namespace mrpt::utils;
using namespace mrpt::math;
using namespace mrpt::random;


class Lshape {
	
	public:
		Lshape(){};
		inline Lshape(const TPoint2D &p1, const TPoint2D &p2, const TPoint2D &p3) throw(std::logic_error);
		inline Lshape(double A,double B,double C, double D);
		~Lshape(){};
		double distance(const TPoint2D &point) const;
		bool check_degenerate(const TPoint2D &p1, const TPoint2D &p2, const TPoint2D &p3);
		double coefs[4];
};

namespace mrpt
{
	namespace math
	{
		template <typename T>
		void  ransacLshape_fit(
			const CMatrixTemplateNumeric<T> &allData,
			const vector_size_t  &useIndices,
			vector< CMatrixTemplateNumeric<T> > &fitModels );

		template <typename T>
		void ransacLshape_distance(
			const CMatrixTemplateNumeric<T> &allData,
			const vector< CMatrixTemplateNumeric<T> > & testModels,
			const T distanceThreshold,
			unsigned int & out_bestModelIndex,
			vector_size_t & out_inlierIndices );

			//Return "true" if the selected points are a degenerate (invalid) case.
			template <typename T>
			bool ransacLshape_degenerate(
				const CMatrixTemplateNumeric<T> &allData,
				const mrpt::vector_size_t &useIndices );

			template <typename NUMTYPE>
			void ransac_detect_Lshape(
				const Eigen::Matrix<NUMTYPE,Eigen::Dynamic,1>  &x,
				const Eigen::Matrix<NUMTYPE,Eigen::Dynamic,1>  &y,
				std::vector<std::pair<mrpt::vector_size_t,Lshape> >   	   &out_detected_models,
				const double           threshold,
				const size_t           min_inliers_for_valid_model
				);
	} // end namespace
} // end namespace

#endif
