/*
 * distance_metric.h
 *
 *  Created on: Apr 12, 2013
 *      Author: liuwlz
 */

/* Generate metric map based on the distance to obstacle, i.e. road curb, vehicle or pedestrain
 *
 *Algorithm is based on the dynamic ros voronoi package http://www.ros.org/wiki/dynamicvoronoi
 */

#ifndef DISTANCE_METRIC_H_
#define DISTANCE_METRIC_H_

#include <ros/ros.h>
#include <limits.h>
#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/PointCloud.h>
#include <queue>
#include <Metric_Map/BucketedQueue.hpp>

using namespace std;

namespace MPAV{

	#define invalidObstID SHRT_MAX;

	class DistMetric{
	private:
		struct mapPts{
			double dist, sqdist;
			int obstX, obstY;
			bool fwProcessed;
			bool needRaise;
			bool isSurround;
			bool norminalLane;
			bool midRroad;
		};

		BucketPrioQueue open;
		vector<bool> obsMap;
		vector<mapPts> ptsCell;
		vector<INTPOINT> addList;
		int width, height;
		int Max_Dist;

		nav_msgs::OccupancyGrid dist_map;

	public:
		DistMetric();
		~DistMetric();

		void initObstacle(nav_msgs::OccupancyGrid map);
		bool checkNearestEightCells(int x, int y);
		void setProcessedObst(int x, int y);
		bool isOccupied(int x, int y, mapPts pts);
		void updateDistMap();
		void setMaxDist(int dist){Max_Dist = dist;}
		void getDistMap(nav_msgs::OccupancyGrid &dist_map_out);
		void updateCost(nav_msgs::OccupancyGrid &local_cost_map);
	};
}
#endif /* DISTANCE_METRIC_H_ */
