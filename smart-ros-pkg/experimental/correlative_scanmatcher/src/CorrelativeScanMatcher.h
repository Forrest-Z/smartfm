/*
 * CorrelativeScanMatcher.h
 *
 *  Created on: Jul 14, 2012
 *      Author: demian
 */

#ifndef CORRELATIVESCANMATCHER_H_
#define CORRELATIVESCANMATCHER_H_

#include "GridMap.h"

class CorrelativeScanMatcher
{
	GridMap gm_low_, gm_, gm_hd_;
	ros::Publisher grid_pub_;
public:
	CorrelativeScanMatcher() : gm_(0.5, 0.1), gm_hd_(0.05, 0.1), gm_low_(4.0, 0.1)
	{
		ros::NodeHandle nh;
		grid_pub_ = nh.advertise<sensor_msgs::PointCloud>("grid_map", 10);
	}

	simple_pose findBestMatchMultiRes(const pcl::PointCloud<pcl::PointXYZ> &pc)
	{
		simple_pose best_pose;
		int max_score=0;
		//gm_low_.findBestMatchRotateFirst(pc, best_pose, max_score, 20.0, 4.0, 70.0, 10.0, false);
		//cout<<"("<<best_pose.x <<","<<best_pose.y<<","<<best_pose.t/M_PI*180<<":"<<max_score<<")";
		gm_.findBestMatchRotateFirst(pc, best_pose, max_score, 4.0, 0.5, 10.0, 2.5, false, best_pose);
		cout<<"("<<best_pose.x <<","<<best_pose.y<<","<<best_pose.t/M_PI*180<<":"<<max_score<<")";
		gm_hd_.findBestMatchRotateFirst(pc, best_pose, max_score, 0.5, 0.25, 2.5, 0.25, false, best_pose);
		cout<<"("<<best_pose.x <<","<<best_pose.y<<","<<best_pose.t/M_PI*180<<":"<<max_score<<")";
		gm_hd_.findBestMatchRotateFirst(pc, best_pose, max_score, 0.25, 0.05, 0.25, 0.05, false, best_pose);
		cout<<"("<<best_pose.x <<","<<best_pose.y<<","<<best_pose.t/M_PI*180<<":"<<max_score<<")"<<endl;
		return best_pose;
	}

	void updatePriorMap(const pcl::PointCloud<pcl::PointXYZ> &pc)
	{
		gm_low_.buildMapDirectDraw(pc);
		gm_.buildMapDirectDraw(pc);
		gm_hd_.buildMapDirectDraw(pc);
		sensor_msgs::PointCloud grid_map_pc;
		grid_map_pc.header = pc.header;

		gm_hd_.getMap(grid_map_pc);
		grid_pub_.publish(grid_map_pc);
	}
};
#endif /* CORRELATIVESCANMATCHER_H_ */
