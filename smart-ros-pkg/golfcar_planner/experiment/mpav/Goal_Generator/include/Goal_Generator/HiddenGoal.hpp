/*
 * HiddenGoal.hpp
 *
 *  Created on: Nov 22, 2013
 *      Author: liuwlz
 */

#ifndef HIDDENGOAL_HPP_
#define HIDDENGOAL_HPP_

#include <vector>
#include <assert.h>

#include <nav_msgs/OccupancyGrid.h>

#include <Goal_Generator/Goal.hpp>
#include <Metric_Map/Dist_Map.hpp>
#include <MPAVUtil/Range.hpp>

namespace MPAV{

	typedef geometry_msgs::PoseStamped Pose;

	class HiddenGoal:public Goal{

	public:
		HiddenGoal();
		~HiddenGoal();

		int HiddenGoalProb(double prob, double distObst, DistMetric* dist_map);
		int HiddenGoalLearning();
	};

	HiddenGoal::HiddenGoal():Goal(){

	}

	HiddenGoal::~HiddenGoal(){

	}

	/**
	 * This function returns the HiddenGoal for naive CP cost map generation:
	 * ---Based on the distance map, the goal is located in the circle whose origin is obstacle and radius is given
	 * 		as max_dist*prob.
	 * @param prob
	 * @param dist_map
	 * @return
	 */
	int HiddenGoal::HiddenGoalProb(double prob, double distObst, DistMetric *dist_map){

		double max_dist, candidate_dist;
		vector<int> candidateIndex;
		vector<double> distMap;
		Pose temp_goal;
		temp_goal.header.frame_id = local_frame;

		dist_map->getDistMap(distMap);

		max_dist = *max_element(distMap.begin(), distMap.end());
		assert(max_dist > 0);

		candidate_dist = max_dist*prob;
		for (vector<double>::iterator it = distMap.begin(); it != distMap.end(); it++ )
			if (WithIn((double)(*it), (double)candidate_dist, 0.1))
				candidateIndex.push_back((int)(it - distMap.begin()));

		for (vector<int>::iterator it= candidateIndex.begin(); it!= candidateIndex.end(); it++){
			int x = (*it) % dist_map->dist_map.info.width;
			int y = (*it) / dist_map->dist_map.info.width;
			/**Calculate the distance between candidate points and robots' position*/
			double dist2Vehicle = Distance((double)x, (double)y, (double)dist_map->dist_map.info.width/2, (double)dist_map->dist_map.info.height/4)
					* dist_map->dist_map.info.resolution ;
			if (!WithIn(dist2Vehicle, distObst, 0.1)){
				candidateIndex.erase(it);
			}
			else{
				double x = candidateIndex[0] % dist_map->dist_map.info.width * dist_map->dist_map.info.resolution;;
				double y = candidateIndex[0] / dist_map->dist_map.info.width * dist_map->dist_map.info.resolution;;
				temp_goal.pose.position.x = x;
				temp_goal.pose.position.y = y;
				temp_goal.pose.orientation.z = sqrt(2.)/2;
				temp_goal.pose.orientation.w = sqrt(2.)/2;
				Pose temp_goal_global;
				if (transformGoalPose(temp_goal, temp_goal_global))
					goal_candidate.push_back(temp_goal_global);
			}
		}
		return candidateIndex.size() > 0 ? 1: 0;
	}
}


#endif /* HIDDENGOAL_HPP_ */
