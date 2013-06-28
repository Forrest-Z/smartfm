/*
 * distance_metric.cpp
 *
 *  Created on: Apr 12, 2013
 *      Author: liuwlz
 */

#include "obst_distance_metric.h"

bool debug_flag = false;

obsDistMetric::obsDistMetric(){

}

obsDistMetric::~obsDistMetric(){

}

//TODO: Initialize the map with label as obst, norminal lane etc.
void obsDistMetric::initObstMap(nav_msgs::OccupancyGrid map){

	local_map = map;
	width = local_map.info.width;
	height = local_map.info.height;
	updateDistMap();

	if (debug_flag)
		ROS_INFO("Metric Map initialised");

	obsMap.resize(width*height);
	ptsCell.resize(width*height);

	if (debug_flag)
		ROS_INFO("Metric Map initialised_1 , %d, %d, %d", width, height, ptsCell.size());

	mapPts pts;

	pts.dist = INFINITY;
	pts.sqdist = INT_MAX;
	pts.obstX = invalidObstID;
	pts.obstY = invalidObstID;
	pts.needRaise = false;
	pts.midRroad = false;
	pts.isSurround = true;
	pts.norminalLane = false;
	pts.fwProcessed = false;

	if (debug_flag)
		ROS_INFO("Metric Map initialised_3");

	//initialise all the pts for further classification;
	for (int i = 0; i < ptsCell.size(); i++)
		ptsCell[i] = pts;

	if (debug_flag)
		ROS_INFO("Metric Map initialised_2");

	for (int x = 0; x < width; x++){
		for (int y = 0; y < height; y++){
			unsigned int map_index = y * width + x;
			if (local_map.data[map_index] == 0)
				obsMap[map_index] = true;
			else {
				obsMap[map_index] = false;
				if (local_map.data[map_index] == 107)
					ptsCell[map_index].norminalLane = true;
			}
		}
	}
	/* Find the road centre, and separate the obstacle cells into whether surround or not.
	 * Only the pts that not surrounded will be processed;
	 */
	for (int x = 0; x < width; x++){
		for (int y = 0; y < height; y++){
			unsigned int map_index = y * width +x;
			if (checkNearestEightCells(x, y) && !obsMap[map_index]){

				if(debug_flag)
					ROS_INFO("Find road centre");
				obsMap[map_index] = true;
				ptsCell[map_index].midRroad = true;
			}
			if (obsMap[map_index]){
				if (ptsCell[map_index].isSurround){
					ptsCell[map_index].dist = 0;
					ptsCell[map_index].sqdist = 0;
					ptsCell[map_index].obstX = x;
					ptsCell[map_index].obstY = y;
				}
				else
					setProcessedObst(x,y);
			}
		}
	}
}

bool obsDistMetric::checkNearestEightCells(int x, int y){

	//if (debug_flag)
		//ROS_INFO("Check insurround cells");
	unsigned int index = y * width +x;

	int count = 0;

	for (int i = x - 1 ; i <= x + 1; i++){
		if (i < 0 || i > width -1)
			continue;
		for (int j = y - 1; j <= y + 1 ; j++){
			if (i == x && j ==y)
				continue;
			if (j < 0 || j > height -1)
				continue;
			unsigned int neigh_index = j * width + i;
			if (local_map.data[neigh_index] != local_map.data[index]){
				ptsCell[index].isSurround = false;
				if (!obsMap[neigh_index])
					count ++;
			}
		}
	}
	if (count > 0)
		return true;
	else
		return false;
}

void obsDistMetric::setProcessedObst(int x, int y){

	//if (debug_flag)
		//ROS_INFO("Set Processed Obst");
	unsigned int index = y * width + x;
	ptsCell[index].dist = 0;
	ptsCell[index].sqdist = 0;
	ptsCell[index].obstX = x;
	ptsCell[index].obstY = y;
	ptsCell[index].fwProcessed = true;
	open.push(0,INTPOINT(x,y));
}

bool obsDistMetric::isOccupied(int x, int y, mapPts pts){
	return (pts.obstX == x && pts.obstY == y);
}


/* @Algorithm introduced in http://www.informatik.uni-freiburg.de/~lau/dynamicvoronoi/
 * @Raise function is not used currently, will be further employed for efficient incremental update
 * to deal with dynamic obstacles;
 *
 */
void obsDistMetric::updateDistMap(){

	if (debug_flag)
		ROS_INFO("Update Map");

	while (!open.empty()){
		INTPOINT coord = open.pop();
		int x = coord.x;
		int y = coord.y;

		mapPts pts = ptsCell[y * width + x];
		if (!pts.fwProcessed)
			continue;

		// Start of Raise function
		if (pts.needRaise){
			for (int i = x-1; i <= x + 1; i++){
				if (i < 0 || i > width-1 )
					continue;
				for (int j = y -1 ; j <= y +1; j++){
					if (i == x && j == y)
						continue;
					if (j < 0 || j > height -1)
						continue;
					mapPts pts_near = ptsCell[j * width + i];
					int ox = pts_near.obstX;
					int oy = pts_near.obstY;
					if (pts_near.obstX != SHRT_MAX && !pts_near.needRaise){
						if (!isOccupied(ox, oy, ptsCell[oy * width + ox])){
							open.push(pts.sqdist, INTPOINT(i,j));
							pts_near.fwProcessed = true;
							pts_near.needRaise = true;
							pts_near.obstX = invalidObstID;
							pts_near.obstY = invalidObstID;
							pts_near.sqdist = INT_MAX;
							pts_near.dist = INFINITY;
							ptsCell[j*width+i] = pts_near;
						}
						else{
							if (!pts_near.fwProcessed){
								open.push(pts_near.sqdist, INTPOINT(i,j));
								pts_near.fwProcessed = true;
								ptsCell[j*width+i] = pts_near;
							}
						}
					}
				}
			}
			pts.needRaise = false;
			ptsCell[y*width + x] = pts;
		}

		//End of Raise function

		else if (pts.obstX != SHRT_MAX && isOccupied(pts.obstX, pts.obstY, ptsCell[pts.obstY * width + pts.obstX])){

			//if (debug_flag)
				//ROS_INFO("Update Map _ 4, %d , %d", x, y);

			pts.fwProcessed = false;
			for (int i = x-1; i <= x + 1; i++){
				if (i < 0 || i > width-1 )
					continue;
				for (int j = y -1 ; j <= y +1; j++){
					if (j < 0 || j > height -1)
						continue;
					if (i == x && j == y)
						continue;

					//if (debug_flag)
						//ROS_INFO("Update Map _ 5, %d , %d", i, j);

					mapPts pts_near = ptsCell[j * width + i];
					int ox = pts_near.obstX;
					int oy = pts_near.obstY;
					if (!pts_near.needRaise){
						int distX = i - pts.obstX;
						int distY = j - pts.obstY;
						int newSqDist = distX*distX + distY*distY;
						bool overwrite = (newSqDist < pts_near.sqdist);
						if (newSqDist == pts_near.sqdist){
							if (pts_near.obstX == SHRT_MAX || !isOccupied(ox, oy, ptsCell[oy * width + ox])){
								overwrite = true;
							}
						}
						if (overwrite){
							open.push(newSqDist, INTPOINT(i,j));
							pts_near.fwProcessed = true;
							pts_near.sqdist = newSqDist;
							pts_near.dist = sqrt((double)newSqDist);
							pts_near.obstX = pts.obstX;
							pts_near.obstY = pts.obstY;
						}
						ptsCell[j * width + i ] = pts_near;
					}
				}
			}
		}
		ptsCell[y*width + x] = pts;
	}

	if (debug_flag)
		ROS_INFO("Update Map End");
}

void obsDistMetric::updateCost(nav_msgs::OccupancyGrid &local_cost_map){

	if (debug_flag)
		ROS_INFO("Update metric value");

	local_cost_map = local_map;

	int max_sqdist = 0;
	double cost;
	mapPts pts;

	//Check max sqdist for normalise the cost
	for (int x = 0 ; x < width ; x++){
		for (int y = 0; y < height; y++){
			unsigned int map_index = y * width + x;
			pts = ptsCell[map_index];
			if (pts.sqdist > max_sqdist)
				max_sqdist = pts.sqdist;
		}
	}

	if (debug_flag)
		ROS_INFO("Update metric value _max, %d, _min , %d", max_sqdist);

	for (int x = 0 ; x < width ; x++){
		for (int y = 0; y < height; y++){
			unsigned int map_index = y * width + x;
			pts = ptsCell[map_index];
			double dist;
			unsigned int metric;

			//obstacle cells
			if (pts.sqdist == 0)
				dist = 0;
			//right lane cells
			else if (!pts.norminalLane && pts.sqdist != 0 )
				dist = sqrt(max_sqdist)/2 + sqrt(pts.sqdist);
			//left lane cells
			else
				dist = sqrt(pts.sqdist);
			//road centre, took as obst initially
			if (pts.midRroad)
				dist = sqrt(max_sqdist);

			metric = (int) 127 * (dist/(1.5*sqrt(max_sqdist)));
			local_cost_map.data[map_index] = (127 - metric);
		}
	}
}
