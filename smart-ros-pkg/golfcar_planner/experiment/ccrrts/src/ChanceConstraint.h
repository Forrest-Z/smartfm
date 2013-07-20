/*
 * ChanceConstraint.h
 *
 *  Created on: Jul 17, 2013
 *      Author: liuwlz
 *
 * 	Risk evaluation for RRT* vertex: 1) Process obstacle info
 */



#ifndef CHANCECONSTRAINT_H_
#define CHANCECONSTRAINT_H_

#include <ros/ros.h>
#include <ros/console.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/ChannelFloat32.h>
#include <geometry_msgs/PolygonStamped.h>
#include <obstacle_tracking/obst_info.h>
#include <ccrrts/constraint.h>
#include <ccrrts/obsts_cst.h>

#include <algorithm>
#include <vector>

using namespace std;

struct LineINFO{
	double a[2];
	double b;
};

typedef vector<geometry_msgs::PolygonStamped> POLYS;
typedef geometry_msgs::Point32 POINT;
//Static obstacle first, further improving it to dynamic obstacle (Using the particle tracking result)
class ChanceConstraint{

	ros::NodeHandle nh_;
	ros::Subscriber obst_info_sub_;
	ros::Publisher obst_cst_pub_;
	obstacle_tracking::obst_info infos;
	ccrrts::obsts_cst csts;

public:
	ChanceConstraint();
	~ChanceConstraint();
	//Allocate the address and other things
	void Initialize(POLYS polys);
	void ObstInfoCallBack(const obstacle_tracking::obst_info obst_info);
	//Extract the constraints that form the polygon
	void PolygonAnalyze(geometry_msgs::PolygonStamped poly, ccrrts::constraint &cons);
};

#endif /* CHANCECONSTRAINT_H_ */
