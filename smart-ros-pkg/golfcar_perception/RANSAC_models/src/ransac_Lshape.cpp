/*******************************************************************************************************************
 * author: 		baoxing.qin
 * data:		2013-12-31
 *
 * description: there are two main parts in this cpp file;
 *
 * 				1st: the class of "Lshape", learned from the TLine2D in MRPT; some basic functions are designed;
 *					 a. the two constructors: from selected points, and from parameters;
 *					 b. the distance functions of a point to this L-shape;
 *					 c. check_degenerate function to check the points;
 *
 *				2nd: the RANSAC functions for this particular L-shape model; 4 functions are designed;
 *					 a. the fitting function to get the model;
 *					 b. the distance function to check whether a point is inside the L-shape;
 *				     c. the degenerate function to deal with the degeneration cases;
 *				     d. the detection functions that call the above functions and do the main job;
 *
 *******************************************************************************************************************
 */

#include "ransac_Lshape.h"

//2 lines in Lshape: line1: AX+BY+C=0; line2: BX-AY+D=0;

//two constructors are needed;

//the 1st is for construction from selected points;
//the order of the points does matter:
//a. p1 & p2 form the 1st line;
//b. p3 determines the 2nd line perpendicular to the 1st one;

Lshape::Lshape(const TPoint2D &p1, const TPoint2D &p2, const TPoint2D &p3) throw(std::logic_error)
{
	if(check_degenerate(p1, p2, p3))throw std::logic_error("degenerate cases");
	else
	{
		TLine2D  line1(p1,p2);

		coefs[0]=line1.coefs[0];
		coefs[1]=line1.coefs[1];
		coefs[2]=line1.coefs[2];

		//D=AY-BX;
		coefs[3]= line1.coefs[0]*p3.y-line1.coefs[1]*p3.x;
		ROS_DEBUG("Lshape model constructed: %3f, %3f, %3f, %3f", coefs[0],coefs[1],coefs[2],coefs[3]);
	}
}

//the 2nd is for construction from parameters;
Lshape::Lshape(double A,double B,double C, double D)
{
	coefs[0]=A;
	coefs[1]=B;
	coefs[2]=C;
	coefs[3]=D;
	ROS_DEBUG("Lshape parameter: %3f, %3f, %3f, %3f", coefs[0],coefs[1],coefs[2],coefs[3]);
}

double Lshape::distance(const TPoint2D &point) const
{
	TLine2D  line1(coefs[0],coefs[1],coefs[2]);
	TLine2D  line2(coefs[1],-coefs[0],coefs[3]);

	double dist1 = line1.distance(point);
	double dist2 = line2.distance(point);
	double smaller_dist = dist1<dist2?dist1:dist2;
	return smaller_dist;
}

bool Lshape::check_degenerate(const TPoint2D &p1, const TPoint2D &p2, const TPoint2D &p3)
{
	if(p1==p2||p2==p3||p1==p3)return false;
	else
	{
		//check whether it is a blunt angle between line "p2-p1" and line "p2-p3";
		//it needs to be a blunt angle if we want a "L-shape" with line1 "p1-p2" and line2 "passing p3, perpendicular to line1"
		double direct1[2], direct2[2];
		direct1[0]=p1.x-p2.x;
		direct1[1]=p1.y-p2.y;
		direct2[0]=p3.x-p2.x;
		direct2[1]=p3.y-p2.y;
		double cosine_angle = direct1[0]*direct2[0]+direct1[1]*direct2[1];
		if(cosine_angle>0) return false;
		else return true;
	}
}

namespace mrpt
{
	namespace math
	{
		template <typename T>
		void  ransacLshape_fit(
			const CMatrixTemplateNumeric<T> &allData,
			const vector_size_t  &useIndices,
			vector< CMatrixTemplateNumeric<T> > &fitModels )
		{
			ASSERT_(useIndices.size()==3);

			TPoint2D  p1( allData(0,useIndices[0]),allData(1,useIndices[0]) );
			TPoint2D  p2( allData(0,useIndices[1]),allData(1,useIndices[1]) );
			TPoint2D  p3( allData(0,useIndices[2]),allData(1,useIndices[2]) );
			
			try
			{
				Lshape  Lshape_object(p1,p2,p3);
				fitModels.resize(1);
				CMatrixTemplateNumeric<T> &M = fitModels[0];
				M.setSize(1,4);
				for (size_t i=0;i<4;i++)M(0,i)=Lshape_object.coefs[i];
			}
			catch(exception &)
			{
				fitModels.clear();
				return;
			}
		}

		template <typename T>
		void ransacLshape_distance(
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

				ASSERT_( size(M,1)==1 && size(M,2)==4 )

				Lshape  Lshape_object( M(0,0), M(0,1), M(0,2), M(0,3));

				const size_t N = size(allData,2);
				out_inlierIndices.reserve(1000);
				for (size_t i=0;i<N;i++)
				{
					const double d = Lshape_object.distance( TPoint2D( allData.get_unsafe(0,i),allData.get_unsafe(1,i) ) );
					if (d<distanceThreshold) out_inlierIndices.push_back(i);
				}
			}

			//Return "true" if the selected points are a degenerate (invalid) case.
			template <typename T>
			bool ransacLshape_degenerate(
				const CMatrixTemplateNumeric<T> &allData,
				const mrpt::vector_size_t &useIndices )
			{
				//in the case where three points forms one horizontal and one vertical;
				TPoint2D  p1( allData(0,useIndices[0]),allData(1,useIndices[0]) );
				TPoint2D  p2( allData(0,useIndices[1]),allData(1,useIndices[1]) );
				TPoint2D  p3( allData(0,useIndices[2]),allData(1,useIndices[2]) );
				if(Lshape::check_degenerate(p1, p2, p3))return true;
				else return false;
			}

			template <typename NUMTYPE>
			void ransac_detect_Lshape(
				const Eigen::Matrix<NUMTYPE,Eigen::Dynamic,1>  &x,
				const Eigen::Matrix<NUMTYPE,Eigen::Dynamic,1>  &y,
				std::pair<mrpt::vector_size_t,Lshape>   	   &out_detected_model,
				const double           threshold,
				const size_t           min_inliers_for_valid_model
				)
			{
				ASSERT_(x.size()==y.size())
				if (x.empty())
					return;

				// The running lists of remaining points after each plane, as a matrix:
				CMatrixTemplateNumeric<NUMTYPE> undercheckPoints( 2, x.size() );
				undercheckPoints.insertRow(0,x);
				undercheckPoints.insertRow(1,y);

				if (size(undercheckPoints,2)>=3)
				{
					mrpt::vector_size_t				this_best_inliers;
					CMatrixTemplateNumeric<NUMTYPE> this_best_model;

					math::RANSAC_Template<NUMTYPE>::execute(
						undercheckPoints,
						ransacLshape_fit,
						ransacLshape_distance,
						ransacLshape_degenerate,
						threshold,
						3,  // Minimum set of points
						this_best_inliers,
						this_best_model,
						true, // Verbose
						0.999,  // Prob. of good result
						100
						);

					// Is this model good enough?
					if (this_best_inliers.size()>=min_inliers_for_valid_model)
					{
						// Add this plane to the output list:
						out_detected_model = std::make_pair<mrpt::vector_size_t,Lshape>(
											this_best_inliers,
											Lshape(this_best_model(0,0), this_best_model(0,1), this_best_model(0,2), this_best_model(0,3))
											);
					}
				}
			}//end function;
	} // end namespace
} // end namespace
