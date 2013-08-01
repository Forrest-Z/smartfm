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
    while(!GetVehicelPose()){
        ros::spinOnce();
        ROS_INFO("Waiting for Robot pose");
    }
    ros::spin();
}

RiskEvaluate::~RiskEvaluate(){

}

//TODO: Implement inverse error function
/*
inline double InverseErrorFunc(double ){
	double result = 0;
	return result;
}
*/

void RiskEvaluate::ConstraintCallBack(const ccrrts::obsts_cst cst){
	risk.obst_id.clear();
	risk.risk.clear();
	RiskAssess(cst);
}

//TODO:Implement risk evaluate w.r.t. each obstacle polygon
void RiskEvaluate::RiskAssess(ccrrts::obsts_cst obst_cst){
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
			//Check the temp_1, which should not be minus
			double temp_1 = a_1*vehicle_pose.x + a_2*vehicle_pose.y - b;
			double temp_2 = sqrt(2.0*(a_1*a_1*vehicle_pose.conv_x + a_2*a_2*vehicle_pose.conv_y));
			double risk = (1 - boost::math::erf(temp_1/temp_2))/2.0;

			//cout <<"No.  "<<j<<" x " << vehicle_pose.x << "y" << vehicle_pose.y <<endl;

			//cout <<" 1_ "<<temp_1<<", 2_ "<<temp_2<< "temp "<< temp_1/sqrt(temp_2) << " erf :"<< boost::math::erf(temp_1/temp_2)<< " risk "<< risk<<endl;
			if (risk < min_risk)
				min_risk = risk;
		}
		risk.obst_id.push_back(i->obst_id);
		risk.risk.push_back(min_risk);
		//cout<<"rise_size"<<risk.risk.size()<<endl;
	}
	risk_pub_.publish(risk);
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

int main(int argc, char** argv){
	ros::init(argc, argv, "risk_assess");
	RiskEvaluate ris_evaluate;
	ros::spin();
	return 0;
}
