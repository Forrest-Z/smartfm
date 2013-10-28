#ifndef RANSAC_2DLINES_H
#define RANSAC_2DLINES_H
#include <mrpt/base.h>  // Precompiled headers

using namespace mrpt;
using namespace mrpt::utils;
using namespace mrpt::math;
using namespace mrpt::random;

//customized 2d lines detection, 
//just chagne the format of "out_detected_lines"
namespace mrpt
{
	namespace math
	{
		template <typename T>
		void  ransac2Dline_fit(
			const CMatrixTemplateNumeric<T> &allData,
			const vector_size_t  &useIndices,
			vector< CMatrixTemplateNumeric<T> > &fitModels )
		{
			ASSERT_(useIndices.size()==2);

			TPoint2D  p1( allData(0,useIndices[0]),allData(1,useIndices[0]) );
			TPoint2D  p2( allData(0,useIndices[1]),allData(1,useIndices[1]) );

			try
			{
				TLine2D  line(p1,p2);
				fitModels.resize(1);
				CMatrixTemplateNumeric<T> &M = fitModels[0];

				M.setSize(1,3);
				for (size_t i=0;i<3;i++)
					M(0,i)=line.coefs[i];
			}
			catch(exception &)
			{
				fitModels.clear();
				return;
			}
		}


		template <typename T>
		void ransac2Dline_distance(
			const CMatrixTemplateNumeric<T> &allData,
			const vector< CMatrixTemplateNumeric<T> > & testModels,
			const T distanceThreshold,
			unsigned int & out_bestModelIndex,
			vector_size_t & out_inlierIndices )
		{
			out_inlierIndices.clear();
			out_bestModelIndex = 0;

			if (testModels.empty()) return; // No model, no inliers.

			ASSERTMSG_( testModels.size()==1, format("Expected testModels.size()=1, but it's = %u",static_cast<unsigned int>(testModels.size()) ) )
			const CMatrixTemplateNumeric<T> &M = testModels[0];

			ASSERT_( size(M,1)==1 && size(M,2)==3 )

			TLine2D  line;
			line.coefs[0] = M(0,0);
			line.coefs[1] = M(0,1);
			line.coefs[2] = M(0,2);

			const size_t N = size(allData,2);
			out_inlierIndices.reserve(100);
			for (size_t i=0;i<N;i++)
			{
				const double d = line.distance( TPoint2D( allData.get_unsafe(0,i),allData.get_unsafe(1,i) ) );
				if (d<distanceThreshold)
					out_inlierIndices.push_back(i);
			}
		}

		/** Return "true" if the selected points are a degenerate (invalid) case.
		  */
		template <typename T>
		bool ransac2Dline_degenerate(
			const CMatrixTemplateNumeric<T> &allData,
			const mrpt::vector_size_t &useIndices )
		{
			return false;
		}
		
		template <typename NUMTYPE>
   void ransac_detect_2dLines(
	const Eigen::Matrix<NUMTYPE,Eigen::Dynamic,1>  &x,
	const Eigen::Matrix<NUMTYPE,Eigen::Dynamic,1>  &y,
	std::vector<std::pair<mrpt::vector_size_t,TLine2D> >   &out_detected_lines,
	const double           threshold,
	const size_t           min_inliers_for_valid_line
	)
  {
	ASSERT_(x.size()==y.size())

	out_detected_lines.clear();

	if (x.empty())
		return;

	// The running lists of remaining points after each plane, as a matrix:
	CMatrixTemplateNumeric<NUMTYPE> remainingPoints( 2, x.size() );
	remainingPoints.insertRow(0,x);
	remainingPoints.insertRow(1,y);

	// ---------------------------------------------
	// For each line:
	// ---------------------------------------------
	while (size(remainingPoints,2)>=2)
	{
		mrpt::vector_size_t				this_best_inliers;
		CMatrixTemplateNumeric<NUMTYPE> this_best_model;

		math::RANSAC_Template<NUMTYPE>::execute(
			remainingPoints,
			ransac2Dline_fit,
			ransac2Dline_distance,
			ransac2Dline_degenerate,
			threshold,
			2,  // Minimum set of points
			this_best_inliers,
			this_best_model,
			false, // Verbose
			0.99999  // Prob. of good result
			);

		// Is this plane good enough?
		if (this_best_inliers.size()>=min_inliers_for_valid_line)
		{
			// Add this plane to the output list:
			out_detected_lines.push_back(
				std::make_pair<mrpt::vector_size_t,TLine2D>(
					this_best_inliers,
					TLine2D(this_best_model(0,0), this_best_model(0,1),this_best_model(0,2) )
					) );

			out_detected_lines.rbegin()->second.unitarize();

			// Discard the selected points so they are not used again for finding subsequent planes:
			remainingPoints.removeColumns(this_best_inliers);
		}
		else
		{
			break; // Do not search for more planes.
		}
	}

 }

	} // end namespace
} // end namespace




#endif
