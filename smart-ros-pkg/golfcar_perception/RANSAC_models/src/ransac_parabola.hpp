#ifndef RANSAC_PARABOLA_H
#define RANSAC_PARABOLA_H

//#include <Eigen/Dense>
#include <mrpt/base.h>
#include <mrpt/gui.h>
#include <sensor_msgs/PointCloud.h>

#define THIRD 0.333333333333333
#define ROOTTHREE 1.73205080756888      

using namespace mrpt;
using namespace mrpt::utils;
using namespace mrpt::gui;
using namespace mrpt::math;
using namespace mrpt::random;

//----------------------------------------------------------------------------------------
// this part is used to calculate the nearest distance from a point to one parabola line;
//----------------------------------------------------------------------------------------
// http://www.dreamincode.net/code/snippet2915.htm
// this function returns the cube root if x were a negative number aswell
double cubeRoot(double x)
{
	if (x < 0)
		return -pow(-x, THIRD);
	else
		return pow(x, THIRD);
}

std::vector<double> solveCubic(double a, double b, double c, double d)
{
	// find the discriminant
	double f, g, h;
	std::vector<double> three_roots;
	f = (3 * c / a - pow(b, 2) / pow(a, 2)) / 3;
	g = (2 * pow(b, 3) / pow(a, 3) - 9 * b * c / pow(a, 2) + 27 * d / a) / 27;
	h = pow(g, 2) / 4 + pow(f, 3) / 27;
	// evaluate discriminant
	if (f == 0 && g == 0 && h == 0)
	{
		// 3 equal roots
		double x;
		// when f, g, and h all equal 0 the roots can be found by the following line
		x = -cubeRoot(d / a);
		// print solutions
		three_roots.push_back(x);
	}
	else if (h <= 0)
	{
		// 3 real roots
		double q, i, j, k, l, m, n, p;
		// complicated maths making use of the method
		i = pow(pow(g, 2) / 4 - h, 0.5);
		j = cubeRoot(i);
		k = acos(-(g / (2 * i)));
		m = cos(k / 3);
		n = ROOTTHREE * sin(k / 3);
		p = -(b / (3 * a));
		// print solutions

		double root1= 2 * j * m + p;
		double root2= -j * (m + n) + p;
		double root3= -j * (m - n) + p;

		three_roots.push_back(root1);
		three_roots.push_back(root2);
		three_roots.push_back(root3);
	}
	else if (h > 0)
	{
		// 1 real root and 2 complex roots
		double r, s, t, u, p;
		// complicated maths making use of the method
		r = -(g / 2) + pow(h, 0.5);
		s = cubeRoot(r);
		t = -(g / 2) - pow(h, 0.5);
		u = cubeRoot(t);
		p = -(b / (3 * a));
		// print solutions
		double real_root = (s + u) + p;
		three_roots.push_back(real_root);
	}
	return three_roots;
}



class parabola {
	
	public:
		parabola(){};
		
		//----------------------------add explanation later---------------------------- 
		// y = ax^2+bx+c;
		//-----------------------------------------------------------------------------
		inline parabola(const TPoint2D &p1, const TPoint2D &p2, const TPoint2D &p3) throw(std::logic_error)
		{
			//printf("p1: (%1f, %1f), p2: (%1f, %1f), p3: (%1f, %1f)", p1.x, p1.y, p2.x, p2.y, p3.x, p3.y);

			if (p1==p2||p1==p3||p2==p3) 
			{
				printf("p1:%5f\t,%5f\n",p1.x,p1.y);
				printf("p2:%5f\t,%5f\n",p2.x,p2.y);
				printf("p3:%5f\t,%5f\n",p3.x,p3.y);
				throw logic_error("two points are the same");
			}
			TPoint2D p1_tmp,  p2_tmp, p3_tmp;

			if((p2.x-p1.x)*(p2.x-p3.x)*(p3.x-p1.x)==0)
			{
				//to shift x and y;
				coefs[3] = 1.0;
				p1_tmp.x = p1.y;
				p1_tmp.y = p1.x;
				p2_tmp.x = p2.y;
				p2_tmp.y = p2.x;
				p3_tmp.x = p3.y;
				p3_tmp.y = p3.x;
			}
			else 
			{
				coefs[3] = 0.0;
				p1_tmp.x = p1.x;
				p1_tmp.y = p1.y;
				p2_tmp.x = p2.x;
				p2_tmp.y = p2.y;
				p3_tmp.x = p3.x;
				p3_tmp.y = p3.y;
			}
			//printf("p1_tmp: (%1f, %1f), p2_tmp: (%1f, %1f), p3_tmp: (%1f, %1f)", p1_tmp.x, p1_tmp.y, p2_tmp.x, p2_tmp.y, p3_tmp.x, p3_tmp.y);			
			
			double A11 = p1_tmp.x * p1_tmp.x;
			double A12 = p1_tmp.x;
			double A13 = 1.0;
		
			double A21 = p2_tmp.x * p2_tmp.x;
			double A22 = p2_tmp.x;
			double A23 = 1.0;
		
			double A31 = p3_tmp.x * p3_tmp.x;
			double A32 = p3_tmp.x;
			double A33 = 1.0;
		
			double b1  = p1_tmp.y;
			double b2  = p2_tmp.y;
			double b3  = p3_tmp.y;
		
			Eigen::Matrix3d A;
			Eigen::Vector3d b;
		
			A << A11, A12, A13,
			 	A21, A22, A23,
			 	A31, A32, A33;
			b << b1, b2, b3;
		
			Eigen::Vector3d x = A.colPivHouseholderQr().solve(b);
			coefs[0] = x(0);
			coefs[1] = x(1);
			coefs[2] = x(2);
					
			//printf("\n parabola parameter: %3f, %3f, %3f, %3f", coefs[0],coefs[1],coefs[2],coefs[3]);

			approx_points.clear();
			for(int pixNum=0; pixNum<640; pixNum++)
			{
				TPoint2D pixel_tmp;
				pixel_tmp.x = double(pixNum);
				pixel_tmp.y = coefs[0]*pixel_tmp.x*pixel_tmp.x+coefs[1]*pixel_tmp.x+coefs[2];
				approx_points.push_back(pixel_tmp);
			}
		}
		
		inline parabola(double A,double B,double C, double D)
		{
			coefs[0]=A;
			coefs[1]=B;
			coefs[2]=C;
			coefs[3]=D;

			approx_points.clear();
			for(int pixNum=0; pixNum<640; pixNum++)
			{
				TPoint2D pixel_tmp;
				pixel_tmp.x = double(pixNum);
				pixel_tmp.y = coefs[0]*pixel_tmp.x*pixel_tmp.x+coefs[1]*pixel_tmp.x+coefs[2];
				//printf("approx point (%1f, %1f)\t", pixel_tmp.x, pixel_tmp.y);
				approx_points.push_back(pixel_tmp);
			}
			//printf("\n 2-parabola parameter: %3f, %3f, %3f, %3f", coefs[0],coefs[1],coefs[2],coefs[3]);
		}
		
		~parabola(){};
		
		/*
		//-----------------------------------------------------------------------------------------------
		//1st approach: calculate the nearest distance by a set of approximated lines for the parabola;
		//speed: slow;
		//-----------------------------------------------------------------------------------------------
		double distance(const TPoint2D &point) const
		{
			TPoint2D point_tmp;
			if(coefs[3]==1.0)
			{
				point_tmp.x = point.y;
				point_tmp.y = point.x;
			}
			else
			{
				point_tmp.x = point.x;
				point_tmp.y = point.y;
			}
			
			//in case of "straight line", a=0, y = bx+c; 
			//http://mathworld.wolfram.com/Point-LineDistance2-Dimensional.html
			//for "bx-y+c=0";
			if(coefs[0]==0.0)
			{
				double nominator = fabs(coefs[1]*point_tmp.x-point_tmp.y+coefs[2]);
				double denominator = sqrt(coefs[1]*coefs[1]+1);;
				return (nominator/denominator);
			}
			//in case of real "parabola line"
			else
			{
				double shortest_dis = 1000.0;

				ASSERT_(approx_points.size()>=2);
				for(size_t point_num=0; point_num<approx_points.size()-1;point_num++)
				{
					
					//http://stackoverflow.com/questions/849211/shortest-distance-between-a-point-and-a-line-segment;
					double distance_tmp;
					TPoint2D begin_pt = approx_points[point_num];
					TPoint2D end_pt = approx_points[point_num+1];

					//distance^2;
					double seg_distance = (begin_pt.x-end_pt.x)*(begin_pt.x-end_pt.x)+
												(begin_pt.y-end_pt.y)*(begin_pt.y-end_pt.y);
					ASSERT_(seg_distance>0.0);
					double innerdot = ((end_pt.x-begin_pt.x)*(point_tmp.x-begin_pt.x)
										+(end_pt.y-begin_pt.y)*(point_tmp.y-begin_pt.y))/seg_distance;
					if(innerdot<=0.0)
					{distance_tmp = sqrt((point_tmp.x-begin_pt.x)*(point_tmp.x-begin_pt.x)+
								 	 		(point_tmp.y-begin_pt.y)*(point_tmp.y-begin_pt.y));}
					else if (innerdot>=1.0)
					{distance_tmp = sqrt((point_tmp.x-end_pt.x)*(point_tmp.x-end_pt.x)+
								 	 		(point_tmp.y-end_pt.y)*(point_tmp.y-end_pt.y));}
					else
					{
						TLine2D seg_tmp(begin_pt,end_pt);
						distance_tmp = seg_tmp.distance(point_tmp);
					}

					if(distance_tmp<shortest_dis) shortest_dis=distance_tmp;
				}
				//printf("(%1f,%1f), nearest %1f\t", point_tmp.x, point_tmp.y, shortest_dis );
				return shortest_dis;
			}
		}
		*/

		//2nd approach: calculate the nearest point on the parabola;
		//calculation is fast; this method is not easy to be applied to higher order curves;
		//-----------------add more explanation here--------------
		//calculate its nearest distance to the parabola line;
		//--------------------------------------------------------
		double distance(const TPoint2D &point) const
		{
			TPoint2D point_tmp;
			if(coefs[3]==1.0)
			{
				point_tmp.x = point.y;
				point_tmp.y = point.x;
			}
			else
			{
				point_tmp.x = point.x;
				point_tmp.y = point.y;
			}
			
			//in case of "straight line", a=0, y = bx+c; 
			//http://mathworld.wolfram.com/Point-LineDistance2-Dimensional.html
			//for "bx-y+c=0";
			if(coefs[0]==0.0)
			{
				double nominator = fabs(coefs[1]*point_tmp.x-point_tmp.y+coefs[2]);
				double denominator = sqrt(coefs[1]*coefs[1]+1);;
				return (nominator/denominator);
			}
			//in case of real "parabola line"
			else
			{
				double a_tmp = 2*coefs[0]*coefs[0];
				double b_tmp = 3*coefs[0]*coefs[1];
				double c_tmp = 2*coefs[0]*coefs[2]+coefs[1]*coefs[1]-2*coefs[0]*point_tmp.y+1;
				double d_tmp = coefs[1]*coefs[2]-coefs[1]*point_tmp.y-point_tmp.x;
				double nearest_dist;
				std::vector<double> root_tmp = solveCubic(a_tmp,b_tmp,c_tmp,d_tmp);
				if(root_tmp.size()==1)
				{
					TPoint2D nearest_pt;
					nearest_pt.x = root_tmp[0];
					nearest_pt.y = coefs[0]*nearest_pt.x*nearest_pt.x+coefs[1]*nearest_pt.x+coefs[2];
					nearest_dist = sqrt((nearest_pt.x-point_tmp.x)*(nearest_pt.x-point_tmp.x)+
											(nearest_pt.y-point_tmp.y)*(nearest_pt.y-point_tmp.y));
				}
				else
				{
					ASSERT_(root_tmp.size()==3);
					TPoint2D nearest_pt1;
					TPoint2D nearest_pt2;
					TPoint2D nearest_pt3;
					nearest_pt1.x = root_tmp[0];
					nearest_pt1.y = coefs[0]*nearest_pt1.x*nearest_pt1.x+coefs[1]*nearest_pt1.x+coefs[2];
					nearest_pt2.x = root_tmp[1];
					nearest_pt2.y = coefs[0]*nearest_pt2.x*nearest_pt2.x+coefs[1]*nearest_pt2.x+coefs[2];
					nearest_pt3.x = root_tmp[2];
					nearest_pt3.y = coefs[0]*nearest_pt3.x*nearest_pt3.x+coefs[1]*nearest_pt3.x+coefs[2];
					double nearest_dist1 = sqrt((nearest_pt1.x-point_tmp.x)*(nearest_pt1.x-point_tmp.x)+
											(nearest_pt1.y-point_tmp.y)*(nearest_pt1.y-point_tmp.y));
					double nearest_dist2 = sqrt((nearest_pt2.x-point_tmp.x)*(nearest_pt2.x-point_tmp.x)+
											(nearest_pt2.y-point_tmp.y)*(nearest_pt2.y-point_tmp.y));
					double nearest_dist3 = sqrt((nearest_pt3.x-point_tmp.x)*(nearest_pt3.x-point_tmp.x)+
											(nearest_pt3.y-point_tmp.y)*(nearest_pt3.y-point_tmp.y));
											
					nearest_dist  = max(nearest_dist1,nearest_dist2);
					nearest_dist = max(nearest_dist,nearest_dist3);
				}
				return nearest_dist;
			}
		}
		
		double coefs[4];
		vector <TPoint2D> approx_points;
};

namespace mrpt
{
	namespace math
	{
		template <typename T>
		void  ransacParabola_fit(
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
				parabola  bola_line(p1,p2,p3);
				fitModels.resize(1);
				CMatrixTemplateNumeric<T> &M = fitModels[0];

				M.setSize(1,4);
				
				for (size_t i=0;i<4;i++)
					M(0,i)=bola_line.coefs[i];
			}
			catch(exception &)
			{
				fitModels.clear();
				return;
			}
		}


		template <typename T>
		void ransacParabola_distance(
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

			if(M(0,3)==1.0)printf("inverse the points\n");

			parabola  line( M(0,0), M(0,1), M(0,2), M(0,3));

			if(M(0,3)==1.0 && line.coefs[3]!=1.0)printf("inverse the points\n");

			const size_t N = size(allData,2);
			out_inlierIndices.reserve(100);
			for (size_t i=0;i<N;i++)
			{
				const double d = line.distance( TPoint2D( allData.get_unsafe(0,i),allData.get_unsafe(1,i) ) );
				if (d<distanceThreshold)
					out_inlierIndices.push_back(i);
			}
		}

	    //Return "true" if the selected points are a degenerate (invalid) case.
		template <typename T>
		bool ransacParabola_degenerate(
			const CMatrixTemplateNumeric<T> &allData,
			const mrpt::vector_size_t &useIndices )
		{
			//in the case where three points forms one horizontal and one vertical;
			TPoint2D  p1( allData(0,useIndices[0]),allData(1,useIndices[0]) );
			TPoint2D  p2( allData(0,useIndices[1]),allData(1,useIndices[1]) );
			TPoint2D  p3( allData(0,useIndices[2]),allData(1,useIndices[2]) );
			if((p2.x-p1.x)*(p2.x-p3.x)*(p3.x-p1.x)==0&&(p2.y-p1.y)*(p2.y-p3.y)*(p3.y-p1.y)==0) return true;
			else {return false;}
		}

		/*---------------------------------------------------------------
						ransac_detect_parabola
		 ---------------------------------------------------------------*/

		template <typename NUMTYPE>
		void ransac_detect_parabolas(
			const Eigen::Matrix<NUMTYPE,Eigen::Dynamic,1>  &x,
			const Eigen::Matrix<NUMTYPE,Eigen::Dynamic,1>  &y,
			std::vector<std::pair<mrpt::vector_size_t,parabola> >   &out_detected_lines,
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
			while (size(remainingPoints,2)>=3)
			{
				mrpt::vector_size_t				this_best_inliers;
				CMatrixTemplateNumeric<NUMTYPE> this_best_model;

				math::RANSAC_Template<NUMTYPE>::execute(
					remainingPoints,
					ransacParabola_fit,
					ransacParabola_distance,
					ransacParabola_degenerate,
					threshold,
					3,  // Minimum set of points
					this_best_inliers,
					this_best_model,
					true, // Verbose
					0.999,  // Prob. of good result
					100		// non-lane contours will take much time to process; 
					);

				// Is this parabola good enough?
				if (this_best_inliers.size()>=min_inliers_for_valid_line)
				{
					// Add this plane to the output list:
					out_detected_lines.push_back(
						std::make_pair<mrpt::vector_size_t,parabola>(
							this_best_inliers,
							parabola(this_best_model(0,0), this_best_model(0,1),
										this_best_model(0,2), this_best_model(0,3))
							) );

					//out_detected_lines.rbegin()->second.unitarize();

					// Discard the selected points so they are not used again for finding subsequent planes:
					remainingPoints.removeColumns(this_best_inliers);
				}
				else
				{
					break; // Do not search for more planes.
				}
			}

		}//end function;


	} // end namespace
} // end namespace




#endif
