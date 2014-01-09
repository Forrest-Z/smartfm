#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>

#include <mrpt/base.h>
#include <mrpt/utils.h>
#include <mrpt/slam.h>
#include "ransac_Lshape.hpp"

/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
 
int main(int argc, char **argv)
{
  mrpt::dynamicsize_vector<double> xs, ys;
  	randomGenerator.randomize();

	// Generate random points in 2D
	// ------------------------------------
	const size_t N_LINES = 4;

	const size_t N_line = 30;
	const size_t N_noise = 50;

	const double LINE_EQ[N_LINES][3]={ 
		{ 1,-1, -2 },
		{ 1,+1.5, -1 },
		{ 0,-1, +2 },
		{ 0.5,-0.3, +1 } };

	for (size_t p=0;p<N_LINES;p++)
	{
		for (size_t i=0;i<N_line;i++)
		{
			const double xx = randomGenerator.drawUniform(-10,10);
			const double yy = randomGenerator.drawGaussian1D(0,0.05)  -(LINE_EQ[p][2]+LINE_EQ[p][0]*xx)/LINE_EQ[p][1];
			xs.push_back(xx);
			ys.push_back(yy);
		}
	}

	for (size_t i=0;i<N_noise;i++)
	{
		xs.push_back( randomGenerator.drawUniform(-15,15));
		ys.push_back( randomGenerator.drawUniform(-15,15));
	}
	
  std::vector<std::pair<mrpt::vector_size_t,Lshape> >   out_detected_lines;
  
  ransac_detect_Lshape(xs,ys,out_detected_lines,0.1,20);
  cout<<"out_detected_lines"<<out_detected_lines.front().second.coefs[0]<<","<<out_detected_lines.front().second.coefs[1]<<","<<endl;
  
  	// Run RANSAC
	// ------------------------------------
	vector<pair<size_t,TLine2D > >   detectedLines;
	const double DIST_THRESHOLD = 0.2;

	CTicTac	tictac;

	ransac_detect_2D_lines(xs,ys,detectedLines,DIST_THRESHOLD,20 );

	// Display output:
	cout << "RANSAC method: ransac_detect_2D_lines" << endl;
	cout << " Computation time: " << tictac.Tac()*1000.0 << " ms" << endl;
	cout << " " << detectedLines.size() << " lines detected." << endl;


	// Show GUI
	// --------------------------
	mrpt::gui::CDisplayWindowPlots  win2("Set of points", 500,500);
	win2.plot(xs,ys,".b4","points");
	unsigned int n=0;
	vector_double lx(2),ly(2);
	lx[0] = -15;
	lx[1] = 15;
	
	for (vector_double::Index q=0;q<lx.size();q++)
		ly[q] = -(out_detected_lines.front().second.coefs[2]+out_detected_lines.front().second.coefs[0]*lx[q])/out_detected_lines.front().second.coefs[1];
	win2.plot(lx,ly,"r-1",format("line_%u",n++));
	
	for (vector_double::Index q=0;q<lx.size();q++)
		ly[q] = -(out_detected_lines.front().second.coefs[3]+out_detected_lines.front().second.coefs[1]*lx[q])/-out_detected_lines.front().second.coefs[0];
	win2.plot(lx,ly,"r-1",format("line_%u",n++));
	
	win2.axis_fit();
	win2.axis_equal();

	win2.waitForKey();

  return 0;
}

