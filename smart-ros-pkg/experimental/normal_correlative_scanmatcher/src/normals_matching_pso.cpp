/*
 * normals_matching_pso.cpp
 *
 *  Created on: Jan 24, 2013
 *      Author: demian
 */
#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <fmutil/fm_stopwatch.h>
#include <fmutil/fm_math.h>
#include <pcl/point_types.h>
#include <pcl/ros/conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include "RasterMapPCL.h"
#include <fstream>
#include <boost/thread/thread.hpp>
#include "pcl_downsample.h"
#include "NormalsCorrelativeMatchingProblem.h"
#include "pso_def.h"

int main(int argc, char** argv)
{
	pcl::PointCloud<pcl::PointNormal> input_cloud, matching_cloud;
	pcl::io::loadPCDFile(argv[1], input_cloud);
	pcl::io::loadPCDFile(argv[2], matching_cloud);

	RNG_GENERATOR::rng_srand();
	RNG_GENERATOR::rng_warm_up();
	Problem::init(input_cloud, matching_cloud);


	fmutil::Stopwatch sw2("pso");


	//PSO pso;
	//pso.run(1);

	// Some display
	//std::cout << "Best particle : " << pso.getBest().getPosition(0) << " "<< pso.getBest().getPosition(1)<<" "<<pso.getBest().getPosition(2)/M_PI*180.<<" "<<1-pso.getBest().getFitness()<< std::endl;

	double best_found = 1;
	double best_pose[3]={0,0,0};
	for(double rotation=-180; rotation<180; rotation+=Problem::rotate_res_)
	{
		for(double offset_x=-20; offset_x<20; offset_x+=Problem::trans_res_)
		{
			for(double offset_y=-20; offset_y<20; offset_y+=Problem::trans_res_)
			{
				double manual_pose[] = {offset_x, offset_y, rotation/180.*M_PI};
				double score_now = Problem::evaluate(manual_pose);
				if(score_now<best_found)
				{
					for(int i=0; i<3; i++)
					best_pose[i]=manual_pose[i];
					best_found=score_now;
				}
				//cout<<score_now<<" ";
			}
			//cout<<endl;

		}
		cout<<rotation<<" "<<best_pose[0]<<" "<<best_pose[1]<<" "<<best_pose[2]/M_PI*180.<<" "<<best_found<<"          \xd"<<flush;
	}
	cout<<endl;
	sw2.end();

	//exit(0);
}
