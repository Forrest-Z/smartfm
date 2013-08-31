/*
 * risk_evaluate.cpp
 *
 *  Created on: Jul 12, 2013
 *      Author: liuwlz
 */


#include "risk_evaluate.h"

RiskEvaluate::RiskEvaluate(){
	cst_sub_ = nh_.subscribe("cst_info", 1, &RiskEvaluate::ConstraintCallBack, this);
	risk_pub_ = nh_.advertise<ccrrts::obst_risk>("obst_risk", 1);
}

RiskEvaluate::~RiskEvaluate(){

}

void RiskEvaluate::ConstraintCallBack(const ccrrts::obsts_cst cst){
	risk.obst_id.clear();
	risk.risk.clear();
	constraints = cst;
	RiskAssess(constraints);
	//ROS_INFO("CallBack");
}

//Implement robot risk evaluate w.r.t. each obstacle polygon
void RiskEvaluate::RiskAssess(ccrrts::obsts_cst obst_cst){

	//ROS_INFO("RiskAssess");
	while (!GetVehicelPose()){
		ros::spinOnce();
		ROS_INFO("Waiting for Vehilce Pose");
	}
	vehicle_pose.x = local_pose_.getOrigin().x();
	vehicle_pose.y = local_pose_.getOrigin().y();
	double roll, pitch, yaw;
	tf::Matrix3x3(local_pose_.getRotation()).getRPY(roll, pitch, yaw);
	vehicle_pose.r = yaw;
	//Simply assign a value, To be optimised
	vehicle_pose.conv_x = 0.5;
	vehicle_pose.conv_y = 2.0;

	for(vector<ccrrts::constraint>::iterator i = obst_cst.obsts_cst.begin(); i != obst_cst.obsts_cst.end(); i++ ){
		double min_risk = 1.0;
		for (int j = 0; j < i->serial_no.size(); j++){
			double a_1 = i->a_1[j];
			double a_2 = i->a_2[j];
			double b = i->b[j];
			double temp_1 = a_1*vehicle_pose.x + a_2*vehicle_pose.y - b;
			double temp_2 = sqrt(2.0*(a_1*a_1*vehicle_pose.conv_x + a_2*a_2*vehicle_pose.conv_y));
			double risk = (1 - boost::math::erf(temp_1/temp_2))/2.0;

			//cout <<"No.  "<<j<<" x " << vehicle_pose.x << "y" << vehicle_pose.y <<endl;
			if (risk < min_risk)
				min_risk = risk;
		}
		risk.obst_id.push_back(i->obst_id);
		risk.risk.push_back(min_risk);
		//cout<<"rise_size"<<risk.risk.size()<<endl;
	}
	risk_pub_.publish(risk);
}

void RiskEvaluate::CalcaulateRisk(ccrrts::obsts_cst obst_cst, State pose, vector<double>& pose_risk){
	if (obst_cst.obsts_cst.size() !=0 ){
		for(vector<ccrrts::constraint>::iterator i = obst_cst.obsts_cst.begin(); i != obst_cst.obsts_cst.end(); i++ ){
			double min_risk = 1.0;
			for (int j = 0; j < i->serial_no.size(); j++){
				double a_1 = i->a_1[j];
				double a_2 = i->a_2[j];
				double b = i->b[j];
				double temp_1 = a_1*pose.x + a_2*pose.y - b;
				double temp_2 = sqrt(2.0*( (a_1*a_1*pose.conv_x) + (a_2*a_2*pose.conv_y)
						+ (2*a_1*a_2*pose.conv_xy) ) );
				double risk = (1 - boost::math::erf(temp_1/temp_2))/2.0;

/*
					ROS_INFO("Obst_ID:%d", i->obst_id);
					ROS_INFO("Sample Pose: x:%f, y:%f, r:%f, con_x:%f, con_y:%f",pose.x, pose.y, pose.r, pose.conv_x, pose.conv_y, pose.conv_xy);
					ROS_INFO("Temp_1: %f,   temp_2, %f, temp_1/temp_2: %f", temp_1, temp_2, temp_1/temp_2);
					ROS_INFO("Constrain: a_1:%f, 1_2:%f, b:%f",a_1, a_2, b);

				ROS_INFO("Risk: %f", risk);
*/
				if (risk < min_risk)
					min_risk = risk;
			}
			pose_risk.push_back(min_risk);
			//ROS_INFO("Check Risk StateIn: x: %f, y:%f, x_cov: %f, y_cov: %f, xy_cov: %f",
			//		pose.x, pose.y, pose.conv_x, pose.conv_y, pose.conv_xy);
			//ROS_INFO("Calculate risk, %f" , min_risk);
		}
	}
	else
		pose_risk.push_back(0.0);
}

bool RiskEvaluate::GetVehicelPose(){

	local_pose_.setIdentity();
	tf::Stamped<tf::Pose> robot_pose;
	robot_pose.setIdentity();
	robot_pose.frame_id_ = "/base_link";
	robot_pose.stamp_ = ros::Time();
	ros::Time current_time = ros::Time::now();

	try {
		tf_.transformPose("/local_map", robot_pose, local_pose_);
	}
	catch(tf::LookupException& ex) {
		ROS_ERROR("No Transform available Error: %s\n", ex.what());
		return false;
	}
	catch(tf::ConnectivityException& ex) {
		ROS_ERROR("Connectivity Error: %s\n", ex.what());
		return false;
	}
	catch(tf::ExtrapolationException& ex) {
		ROS_ERROR("Extrapolation Error: %s\n", ex.what());
		return false;
	}
	return true;
}

/*
int main(int argc, char** argv){
	ros::init(argc, argv, "risk_assess");
	RiskEvaluate risk_evaluate;
	ros::spin();
	return 0;
}
*/
