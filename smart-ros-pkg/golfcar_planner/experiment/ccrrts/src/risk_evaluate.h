/*
 * risk_evaluate.h
 *
 *  Created on: Jul 12, 2013
 *      Author: liuwlz
 */
/*
 * Risk evaluation for RRT* vertex: 1) Process obstacle info, 2) Risk evaluation using Chance Constrain
 */

#ifndef RISK_EVALUATE_H_
#define RISK_EVALUATE_H_

//Assume point robot first

#include <ros/ros.h>
#include <ros/console.h>
#include <ccrrts/constraint.h>
#include <ccrrts/obsts_cst.h>
#include <ccrrts/obst_risk.h>
#include <boost/math/special_functions/erf.hpp>
#include <tf/transform_listener.h>
#include <tf/tf.h>

using namespace std;;


#define RISK_LEVEL 0.3

struct State{
	double x;
	double y;
	double r;
	double conv_x;
	double conv_y;
};

class RiskEvaluate{

	ros::NodeHandle nh_;
	ros::Subscriber cst_sub_;
	ros::Publisher risk_pub_;
	ccrrts::obst_risk risk;
	State vehicle_pose;
	tf::TransformListener tf_;
	tf::Stamped<tf::Pose> local_pose_;

public:
	RiskEvaluate();
	~RiskEvaluate();
	void ConstraintCallBack(const ccrrts::obsts_cst cst);
	void RiskAssess(ccrrts::obsts_cst obst_cst);
	bool GetVehicelPose();

};

#endif /* RISK_EVALUATE_H_ */
