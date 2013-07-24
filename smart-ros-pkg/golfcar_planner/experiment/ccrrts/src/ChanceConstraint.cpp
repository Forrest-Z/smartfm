/*
 * ChanceConstraint.cpp
 *
 *  Created on: Jul 17, 2013
 *      Author: liuwlz
 */


#include "ChanceConstraint.h"

bool debug = false;

ChanceConstraint::ChanceConstraint(){
	obst_info_sub_= nh_.subscribe("obst_info",1,&ChanceConstraint::ObstInfoCallBack, this);
	obst_cst_pub_ = nh_.advertise<ccrrts::obsts_cst>("cst_info", 1);
}

ChanceConstraint::~ChanceConstraint(){

}

inline double CheckRotation(POINT pts_1, POINT pts_2, POINT pts_3){
	return ( pts_2.x - pts_1.x)*(pts_3.y - pts_1.y) - (pts_2.y - pts_1.y)*(pts_3.x - pts_1.x);
}

inline bool CheckConvex(geometry_msgs::Polygon poly){
	bool convex = false;
	return convex;
}

inline LineINFO ConstraintAnalyze(sensor_msgs::PointCloud line_pts){
	double x_1 = line_pts.points[0].x;
	double x_2 = line_pts.points[1].x;
	double y_1 = line_pts.points[0].y;
	double y_2 = line_pts.points[1].y;
	LineINFO line_info;
	line_info.a[0] = y_2-y_1;
	line_info.a[1] = x_1-x_2;
	line_info.b = (y_2-y_1)*x_1 - (x_2-x_1)*y_1;
	return line_info;
}

void ChanceConstraint::ObstInfoCallBack(const obstacle_tracking::obst_info obst_info){
	infos = obst_info;
	csts.obsts_cst.clear();
	Initialize(infos.veh_polys);
	csts.head.stamp = ros::Time::now();
	obst_cst_pub_.publish(csts);
}

void ChanceConstraint::Initialize(POLYS polys){
	ROS_DEBUG("Initialise");
	for (unsigned int i = 0; i < polys.size(); i++){
		ccrrts::constraint constraint;
		constraint.obst_id = i;
		PolygonAnalyze(polys[i], constraint);
		csts.obsts_cst.push_back(constraint);
	}
}

void ChanceConstraint::PolygonAnalyze(geometry_msgs::PolygonStamped poly, ccrrts::constraint &cst){
	/*
	 * Convex check first, now assume it is convex for easy startup
	 */
	if (CheckConvex(poly.polygon)){
		ROS_INFO("Non_convex polygon");
		return;
	}

	vector<geometry_msgs::Point32> pts;
	for (vector<geometry_msgs::Point32>::iterator i = poly.polygon.points.begin(); i != poly.polygon.points.end(); i++)
		pts.push_back(*i);
	//Check rotation and make sure rotate in counter-clockwise direction
	double ccw = CheckRotation(pts[0], pts[1], pts[2]);

	if (ccw < 0){
		reverse(pts.begin(), pts.end());
	}
	/*
	cout << "Test"<< pts[0].x <<" "<<pts[0].y <<endl;
	cout << "Test"<< pts[1].x <<" "<<pts[1].y <<endl;
	cout << "Test"<< pts[2].x <<" "<<pts[2].y <<endl;
	cout << "Test"<< pts[3].x <<" "<<pts[3].y <<endl;
	*/
	if (debug)
		cout << "Chance Constraint analysis"<<endl;

	for(unsigned int i = 0; i < pts.size(); i++){
		cst.serial_no.push_back(i);
		sensor_msgs::PointCloud line_pts;
		if (i < pts.size()-1){
			line_pts.points.push_back(pts[i]);
			line_pts.points.push_back(pts[i+1]);
		}
		else{
			line_pts.points.push_back(pts[i]);
			line_pts.points.push_back(pts[0]);
		}
		LineINFO line_info = ConstraintAnalyze(line_pts);
		cst.a_1.push_back(line_info.a[0]);
		cst.a_2.push_back(line_info.a[1]);
		cst.b.push_back(line_info.b);
	}
}

// How to distinguish the different norms
int main (int argc, char** argv){
	ros::init(argc, argv, "constraint_process");
	ChanceConstraint constrain;
	ros::spin();
	return 0;
}
