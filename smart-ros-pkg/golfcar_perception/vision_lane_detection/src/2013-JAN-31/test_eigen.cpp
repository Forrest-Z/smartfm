#include "lane_marker_common.h"
//#include <Eigen/Dense>
#include <mrpt/base.h>
#include <mrpt/slam.h>
#include <mrpt/gui.h>
#include <sensor_msgs/PointCloud.h>

using namespace mrpt;
using namespace mrpt::utils;
using namespace mrpt::gui;
using namespace mrpt::math;
using namespace mrpt::random;

int main(int argc, char** argv) 
{
		TPoint2D p1, p2, p3;
		p1.x = 386.0;
		p1.y = 286.5;
		p2.x = 387.0;
		p2.y = 319.0;
		p3.x = 387.0;
		p3.y = 320.0;
		
		double coefs[4];
		
		printf("p1: (%1f, %1f), p2: (%1f, %1f), p3: (%1f, %1f)", p1.x, p1.y, p2.x, p2.y, p3.x, p3.y);

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
		printf("p1_tmp: (%1f, %1f), p2_tmp: (%1f, %1f), p3_tmp: (%1f, %1f)", 
				p1_tmp.x, p1_tmp.y, p2_tmp.x, p2_tmp.y, p3_tmp.x, p3_tmp.y);			
		
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
		printf("\n %2f, %2f, %2f \n, %2f, %2f, %2f \n, %2f, %2f, %2f\n", A11, A12, A13, A21, A22, A23, A31, A32, A33);
		printf("%2f, %2f, %2f \n", b1, b2, b3);
		Eigen::Vector3d x = A.colPivHouseholderQr().solve(b);
		coefs[0] = x(0);
		coefs[1] = x(1);
		coefs[2] = x(2);
				
		printf("parabola parameter: %3f, %3f, %3f, %3f", coefs[0],coefs[1],coefs[2],coefs[3]);
		return 0;
}
