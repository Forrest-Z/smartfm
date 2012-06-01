/*
 * pip_test.cpp
 *
 *  Created on: Jun 1, 2012
 *      Author: demian
 */
#include <ros/ros.h>
#include <geometry_msgs/PolygonStamped.h>
#include <sensor_msgs/PointCloud.h>
#include "pnpoly.h"



int main(int argc, char** argv)
{
	ros::init(argc, argv, "pip_test");
	ros::Publisher neg_pts, zero_pts, pos_pts, boundary_pub;
	ros::NodeHandle nh;
	boundary_pub = nh.advertise<geometry_msgs::PolygonStamped>("ped_boundary",1, true);
	neg_pts = nh.advertise<sensor_msgs::PointCloud>("neg_pts", 1, true);
	zero_pts = nh.advertise<sensor_msgs::PointCloud>("zero_pts", 1, true);
	pos_pts = nh.advertise<sensor_msgs::PointCloud>("pos_pts", 1, true);
	double polygon[][2] = {{662.155,3136.776},{602.0509,3019.346},{574.7768,3021.24},{567.4532,3005.204},{576.1658,2988.158},{584.6258,2983.612},{588.1613,2974.773},{566.1905,2920.351},{533.108,2843.074},{605.0814,2815.8},{653.8212,2933.736},{649.7806,2937.271},{660.1347,2961.767},{679.3276,2953.939},{694.4799,2983.738},{672.7616,2996.87},{677.8124,3007.729},{680.3377,3006.214},{733.6233,3109.25},{662.155,3136.776}};
	//double polygon[][2] = {{600, 2900}, {675, 3000},{625, 3100},  {650, 2900}, {600, 2900}};
	int npts = NPTS(polygon);
	geometry_msgs::PolygonStamped boundary_msg;
	vector<Point> boundary;
	boundary_msg.header.stamp = ros::Time::now();
	boundary_msg.header.frame_id = "/map";
	boundary_msg.header.seq = 1;
	for (int i = 0; i < npts; i++)
	{
		Point l; l.x = polygon[i][0]/10.0; l.y = polygon[i][1]/10.0;
		Point32 l32; l32.x = l.x; l32.y = l.y;
		boundary.push_back(l);
		boundary_msg.polygon.points.push_back(l32);
	}
	boundary_pub.publish(boundary_msg);

	sensor_msgs::PointCloud neg_pc, zero_pc, pos_pc;
	neg_pc.header = zero_pc.header = pos_pc.header = boundary_msg.header;
	srand(time(NULL));
	for(int i=0; i<1000; i++)
	{
		Point rnd_pt;
		rnd_pt.x = 25.0f * rand() / (RAND_MAX + 1.0f) + 50.0;
		rnd_pt.y = 40.0f * rand() / (RAND_MAX + 1.0f) + 280.0;
		int pts_decision = pointInPolygon(rnd_pt, boundary);
		Point32 pt_cat;
		pt_cat.x = rnd_pt.x;
		pt_cat.y = rnd_pt.y;
		switch (pts_decision)
		{
		case -1:
			neg_pc.points.push_back(pt_cat);
			break;
		case 0:
			zero_pc.points.push_back(pt_cat);
			break;
		case 1:
			pos_pc.points.push_back(pt_cat);
			break;
		}

	}
	neg_pts.publish(neg_pc); zero_pts.publish(zero_pc); pos_pts.publish(pos_pc);

	ros::spin();

	return 0;
}
