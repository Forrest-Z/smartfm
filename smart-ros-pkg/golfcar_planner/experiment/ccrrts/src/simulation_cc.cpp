/*
 * simulation_cc.cpp
 *
 *  Created on: Aug 17, 2013
 *      Author: liuwlz
 */


#include <ros/ros.h>
#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/Point32.h>
#include <obstacle_tracking/obst_info.h>

class SimObst{
public:
	SimObst();
	~SimObst();

	ros::NodeHandle nh;
	ros::Timer obst_pub_timer;
	ros::Publisher obst_info_pub;
	obstacle_tracking::obst_info obst;

	void PublishObstInfo(const ros::TimerEvent &e);
};

SimObst::SimObst(){
	obst_pub_timer = nh.createTimer(ros::Duration(0.1), &SimObst::PublishObstInfo, this);
	obst_info_pub = nh.advertise<obstacle_tracking::obst_info>("obst_info", 1);
	ros::spin();
}

SimObst::~SimObst(){

}

void SimObst::PublishObstInfo(const ros::TimerEvent &e){


	geometry_msgs::Point32 temp_pts_1, temp_pts_2, temp_pts_3, temp_pts_4;
	geometry_msgs::PolygonStamped temp_poly_1, temp_poly_2;
	//For first obst
	temp_pts_1.x = 10.0; temp_pts_1.y = 16.5; temp_pts_1.z = 0;
	temp_pts_2.x = 10.0; temp_pts_2.y = 18.5; temp_pts_2.z = 0;
	temp_pts_3.x = 12.0; temp_pts_3.y = 18.5; temp_pts_3.z = 0;
	temp_pts_4.x = 12.0; temp_pts_4.y = 16.5; temp_pts_4.z = 0;

	temp_poly_1.header.stamp = ros::Time::now();
	temp_poly_1.header.frame_id = "map";
	temp_poly_1.polygon.points.push_back(temp_pts_1);
	temp_poly_1.polygon.points.push_back(temp_pts_2);
	temp_poly_1.polygon.points.push_back(temp_pts_3);
	temp_poly_1.polygon.points.push_back(temp_pts_4);
	obst.veh_polys.push_back(temp_poly_1);

	temp_pts_1.x = 13.5; temp_pts_1.y = 16.5; temp_pts_1.z = 0;
	temp_pts_2.x = 13.5; temp_pts_2.y = 18.5; temp_pts_2.z = 0;
	temp_pts_3.x = 20.0; temp_pts_3.y = 18.5; temp_pts_3.z = 0;
	temp_pts_4.x = 20.0; temp_pts_4.y = 16.5; temp_pts_4.z = 0;
	temp_poly_2.polygon.points.push_back(temp_pts_1);
	temp_poly_2.polygon.points.push_back(temp_pts_2);
	temp_poly_2.polygon.points.push_back(temp_pts_3);
	temp_poly_2.polygon.points.push_back(temp_pts_4);
	temp_poly_2.header.stamp = ros::Time::now();
	temp_poly_2.header.frame_id = "map";
	obst.veh_polys.push_back(temp_poly_2);

	obst_info_pub.publish(obst);
	obst.veh_polys.clear();
}

int main(int argc, char** argv){
	ros::init(argc, argv, "sim_obst_info");
	SimObst simObst;
	ros::spin();
	return 0;
}
