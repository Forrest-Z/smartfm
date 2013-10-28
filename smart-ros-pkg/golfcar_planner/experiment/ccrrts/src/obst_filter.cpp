/*
 * obst_filter.cpp
 *
 *  Created on: Aug 27, 2013
 *      Author: liuwlz
 */

/*
 * obst_filter.cpp
 *
 * Created on: Aug 27, 2013
 * Author: liuwlz
 */

#include <ros/ros.h>
#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/Point32.h>
#include <obstacle_tracking/obst_info.h>

class ObstFilter{
public:
	ObstFilter();
	~ObstFilter();

	ros::NodeHandle nh;
	ros::Timer obst_pub_timer;
	ros::Subscriber obst_info_sub;
	ros::Publisher obst_info_pub;
	obstacle_tracking::obst_info obst_info;
	bool obst_filted;

	void obstInfoCallBack(const obstacle_tracking::obst_info obst);
	void PublishCstInfo(const ros::TimerEvent &e);
};

ObstFilter::ObstFilter(){

	obst_filted = false;

	obst_info_sub = nh.subscribe("obst_info", 2, &ObstFilter::obstInfoCallBack, this);
	obst_pub_timer = nh.createTimer(ros::Duration(0.05), &ObstFilter::PublishCstInfo, this);
	obst_info_pub = nh.advertise<obstacle_tracking::obst_info>("obst_info_exp", 1);
	ros::spin();
}

ObstFilter::~ObstFilter(){
}

#if(0)
//TODO: Figure out proper filter
void ObstFilter::obstInfoCallBack(const obstacle_tracking::obst_info obst){
	if (obst.veh_polys.size() == 2 && !obst_filted){
		geometry_msgs::PolygonStamped veh_1, veh_2;
		veh_1 = obst.veh_polys[0];
		veh_2 = obst.veh_polys[1];

		geometry_msgs::Point32 veh_pts_1;
		geometry_msgs::Point32 veh_pts_2;

		for (int i = 0 ; i < 4; i++){
			veh_pts_1.x += veh_1.polygon.points[i].x;
			veh_pts_1.y += veh_1.polygon.points[i].y;
			veh_pts_2.x += veh_2.polygon.points[i].x;
			veh_pts_2.y += veh_2.polygon.points[i].y;
		}

		if (veh_pts_1.x - veh_pts_2.x < 1.0){
			obst_info = obst;
			obst_filted = true;
		}
	}
}
#else if
void ObstFilter::obstInfoCallBack(const obstacle_tracking::obst_info obst){
	if (obst.veh_polys.size() == 2 && !obst_filted){
		geometry_msgs::PolygonStamped veh_1, veh_2;
		veh_1 = obst.veh_polys[0];
		veh_2 = obst.veh_polys[1];

		geometry_msgs::Point32 veh_pts_1;
		geometry_msgs::Point32 veh_pts_2;

		for (int i = 0 ; i < 4; i++){
			veh_pts_1.x += veh_1.polygon.points[i].x;
			veh_pts_1.y += veh_1.polygon.points[i].y;
			veh_pts_2.x += veh_2.polygon.points[i].x;
			veh_pts_2.y += veh_2.polygon.points[i].y;
		}

		if (veh_pts_1.x - veh_pts_2.x < 1.0){
			obst_info = obst;
			obst_filted = true;
		}
	}
}

#endif

void ObstFilter::PublishCstInfo(const ros::TimerEvent &e){
	if (obst_filted)
		obst_info_pub.publish(obst_info);
}


int main(int argc, char** argv){
	ros::init(argc, argv, "obst_filter");
	ObstFilter ObstFilter;
	ros::spin();
	return 0;
}



