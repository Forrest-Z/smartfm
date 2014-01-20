/*
 * Dist_Map.cpp
 *
 *  Created on: Dec 19, 2013
 *      Author: liuwlz
 */

#include <Metric_Map/Dist_Map.hpp>

namespace MPAV{

	DistMetric::DistMetric(){
		Max_Dist = 100;
	}

	DistMetric::~DistMetric(){
		dist_map.data.clear();
		ptsCell.clear();
	}

	//TODO: Initialize the map with label as obst, norminal lane etc.
	void DistMetric::initObstacle(nav_msgs::OccupancyGrid map){
		dist_map = map;
		width = dist_map.info.width;
		height = dist_map.info.height;

		obsMap.resize(width*height);
		ptsCell.resize(width*height);

		mapPts pts;
		pts.dist = INFINITY;
		pts.sqdist = INT_MAX;
		pts.obstX = invalidObstID;
		pts.obstY = invalidObstID;
		pts.needRaise = false;
		pts.midRroad = false;
		pts.isSurround = true;
		pts.fwProcessed = false;

		for (unsigned int i = 0; i < ptsCell.size(); i++)
			ptsCell[i] = pts;

		for (int x = 0; x < width; x++){
			for (int y = 0; y < height; y++){
				unsigned int map_index = y * width + x;
				if (dist_map.data[map_index] == 0)
					obsMap[map_index] = true;
				else {
					obsMap[map_index] = false;
					if (dist_map.data[map_index] == 107)
						ptsCell[map_index].norminalLane = true;
				}
			}
		}
		/* Find the road centre, and separate the obstacle cells into whether surround or not.
		 * Only the pts that not surrounded will be processed;
		 */
		int count = 0;
		for (int x = 0; x < width; x++){
			for (int y = 0; y < height; y++){
				unsigned int map_index = y * width +x;
				if (checkNearestEightCells(x, y) && !obsMap[map_index]){
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
					else{
						setProcessedObst(x,y);
						count++;
					}
				}
			}
		}
	}

	bool DistMetric::checkNearestEightCells(int x, int y){

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
				if (dist_map.data[neigh_index] != dist_map.data[index]){
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

	void DistMetric::setProcessedObst(int x, int y){
		unsigned int index = y * width + x;
		ptsCell[index].dist = 0;
		ptsCell[index].sqdist = 0;
		ptsCell[index].obstX = x;
		ptsCell[index].obstY = y;
		ptsCell[index].fwProcessed = true;
		open.push(0,INTPOINT(x,y));
	}

	bool DistMetric::isOccupied(int x, int y, mapPts pts){
		return (pts.obstX == x && pts.obstY == y);
	}

	/* @Algorithm introduced in http://www.informatik.uni-freiburg.de/~lau/dynamicvoronoi/
	 * @Raise function is not used currently, will be further employed for efficient incremental update
	 * to deal with dynamic obstacles;
	 */
	void DistMetric::updateDistMap(){
		while (!open.empty()){
			INTPOINT coord = open.pop();
			int x = coord.x;
			int y = coord.y;

			mapPts pts = ptsCell[y * width + x];
			if (!pts.fwProcessed)
				continue;

			// Start of Raise function
			if (pts.needRaise){
				ROS_INFO("Raise");
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
				pts.fwProcessed = false;
				for (int i = x-1; i <= x + 1; i++){
					if (i < 0 || i > width-1 )
						continue;
					for (int j = y -1 ; j <= y +1; j++){
						if (j < 0 || j > height -1)
							continue;
						if (i == x && j == y)
							continue;

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
							if (overwrite && (newSqDist <Max_Dist*Max_Dist)){
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
	}

	void DistMetric::getDistMap(nav_msgs::OccupancyGrid &dist_map_out){
		mapPts temp_pts;
		for (int x = 0 ; x < width ; x++){
			for (int y = 0; y < height; y++){
				unsigned int map_index = y * width + x;
				temp_pts = ptsCell[map_index];
				if (temp_pts.dist < Max_Dist)
					dist_map_out.data[map_index] = (int) (temp_pts.dist/Max_Dist*100);
			}
		}
	}

	void DistMetric::updateCost(nav_msgs::OccupancyGrid &local_cost_map){

		local_cost_map = dist_map;
		double max_dist = 0;
		mapPts pts;

		for (int x = 0 ; x < width ; x++){
			for (int y = 0; y < height; y++){
				unsigned int map_index = y * width + x;
				pts = ptsCell[map_index];
				if (pts.dist > max_dist)
					max_dist = pts.dist;
				//if (pts.dist != 0)
				//	ROS_INFO("Curr_dist: %f, Max_dist: %f", pts.dist ,max_dist);
			}
		}

		for (int x = 0 ; x < width; x++){
			for (int y = 0; y < height; y++){
				unsigned int map_index = y * width + x;
				pts = ptsCell[map_index];
				double dist;
				unsigned int metric;

				//obstacle cells
				if (pts.dist == 0)
					dist = 0;
				//right lane cells
				else if (!pts.norminalLane && pts.dist != 0 )
					dist = max_dist/2 + pts.dist;
				//left lane cells
				else
					dist = pts.dist;
				//road centre, took as obst initially
				if (pts.midRroad)
					dist = max_dist;

				metric = (int) 127 * (dist/(1.5*max_dist));
				local_cost_map.data[map_index] = (127 - metric);

				if(x == 0 || x==(width-1) || y == 0 || y == (height-1))
					local_cost_map.data[map_index] = 127;
			}
		}
	}
}

