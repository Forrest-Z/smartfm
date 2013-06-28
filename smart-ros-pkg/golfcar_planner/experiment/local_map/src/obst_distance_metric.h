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
#include "bucketedqueue.h"

using namespace std;

#define invalidObstID SHRT_MAX;

class obsDistMetric{

private:
	nav_msgs::OccupancyGrid local_map;

	struct mapPts{
		double dist;
		int sqdist;
		int obstX;
		int obstY;
		bool fwProcessed;
		bool needRaise;
		bool midRroad;
		bool isSurround;
		bool norminalLane;
	};

	BucketPrioQueue open;
	vector<bool> obsMap;
	vector<mapPts> ptsCell;
	vector<INTPOINT> addList;
	int width;
	int height;

public:
	obsDistMetric();
	~obsDistMetric();

	void initObstMap(nav_msgs::OccupancyGrid map);
	bool checkNearestEightCells(int x, int y);
	void setProcessedObst(int x, int y);
	bool isOccupied(int x, int y, mapPts pts);
	void updateDistMap();
	void updateCost(nav_msgs::OccupancyGrid &local_cost_map);
};
#endif /* DISTANCE_METRIC_H_ */
