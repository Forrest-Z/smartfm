/*
 * laser_vehicle.cpp
 *
 *  Created on: Jul 9, 2012
 *      Author: demian
 */

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <geometry_msgs/PoseStamped.h>
#include <fmutil/fm_math.h>
#include <fmutil/fm_stopwatch.h>

#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

#include "pcl_ros/point_cloud.h"
#include "pcl/ros/conversions.h"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/centroid.h>
#include <pcl/common/eigen.h>

using namespace std;

ros::Publisher *pub_, *vehicle_pub_;
tf::TransformBroadcaster *tf_broadcaster_;
string target_frame_id_;

inline double deg_between_pt(geometry_msgs::Point32 p1, geometry_msgs::Point32 p2)
{
	return atan2(p1.y-p2.y, p1.x - p2.x)/M_PI*180+180;
}

struct line_segments
{
	vector<geometry_msgs::Point32> pts;
	double angle;
	geometry_msgs::Point32 centroid;
	void getDetails()
	{
		angle = fabs((deg_between_pt(pts[0], pts[pts.size()-1])));

		centroid.x = (pts[0].x + pts[pts.size()-1].x)/2.0;
		centroid.y = (pts[0].y + pts[pts.size()-1].y)/2.0;
	}
};

bool sort_degree(const line_segments& s1, const line_segments& s2)
{
	return fabs(s1.angle-90) < fabs(s2.angle-90);
}

bool sort_dist(const line_segments& s1, const line_segments& s2)
{
	return fmutil::distance(s1.centroid.x, s1.centroid.y, 0.0, 0.0) < fmutil::distance(s2.centroid.x, s2.centroid.y, 0.0, 0.0);
}

bool sort_y(const geometry_msgs::Point32& p1, const geometry_msgs::Point32& p2)
{
	return p1.y<p2.y;
}

bool findYawLeastSquare(vector<geometry_msgs::Point32> p, double &yaw)
{
	//adapted from www.ccas.ru/mmes/educat/lab04k/02/least-squares.c
	  double SUMx, SUMy, SUMxy, SUMxx, slope,
	         y_intercept;
	  SUMx = 0; SUMy = 0; SUMxy = 0; SUMxx = 0;
	  for (size_t i=0; i<p.size(); i++) {
	    SUMx = SUMx + p[i].x;
	    SUMy = SUMy + p[i].y;
	    SUMxy = SUMxy + p[i].x*p[i].y;
	    SUMxx = SUMxx + p[i].x*p[i].x;
	  }
	  slope = ( SUMx*SUMy - p.size()*SUMxy ) / ( SUMx*SUMx - p.size()*SUMxx );
	  y_intercept = ( SUMy - slope*SUMx ) / p.size();

	  double y1 = y_intercept, x1 = 0;
	  double y2 = slope + y_intercept, x2 = 1;
	  yaw = atan2(y1-y2, x1-x2);
	  return slope>=0;
}

void laserCallback(sensor_msgs::LaserScan scan)
{
	vector<geometry_msgs::Point32> pts;
	for(size_t i=0; i<scan.ranges.size(); i++)
	{
		if(scan.ranges[i] >= scan.range_max) continue;
		geometry_msgs::Point32 p;
		double t = scan.angle_min+(i*scan.angle_increment);
		p.x = scan.ranges[i]*cos(t);
		p.y = scan.ranges[i]*sin(t);
		pts.push_back(p);
	}
	if(pts.size()<=1) return;

	//get the first angle of the segments and start segmentation
	double angle_pre = deg_between_pt(pts[0], pts[1]);
	vector< line_segments > segmented_line;
	line_segments first_segment;
	first_segment.pts.push_back(pts[0]);
	first_segment.pts.push_back(pts[1]);
	segmented_line.push_back(first_segment);

	unsigned int segment_no = 0;
	for(size_t i=1; i<pts.size()-1; i++)
	{
		double angle = deg_between_pt(pts[i], pts[i+1]);
		if(fabs(angle - angle_pre) < 45)
			segmented_line[segment_no].pts.push_back(pts[i+1]);
		else
		{
			//get the angle and centroid of the last segment
			segmented_line[segmented_line.size()-1].getDetails();

			//then start construction of next segment
			line_segments new_segment;
			new_segment.pts.push_back(pts[i+1]);
			segmented_line.push_back(new_segment);
			segment_no++;
		}
		angle_pre = angle;
	}
	segmented_line[segmented_line.size()-1].getDetails();

	//sort the segment so that segments that is closer to the vehicle get selected
	sort(segmented_line.begin(), segmented_line.end(), sort_dist);
	sensor_msgs::PointCloud pc;
	pc.header = scan.header;

	//remove segments contain less than 2 points which are usually the outlier
	for(size_t i=0; i<segmented_line.size();)
	{
		if(segmented_line[i].pts.size()<=2) segmented_line.erase(segmented_line.begin()+i);
		else i++;
	}

	pc.points = segmented_line[0].pts;
	double first_angle = segmented_line[0].angle;
	geometry_msgs::Point32 first_centroid = segmented_line[0].centroid;

	//Finally gather all the pieces together
	for(size_t i=1; i<segmented_line.size(); i++)
	{
		if(fabs(segmented_line[i].angle - first_angle) < 30 && fmutil::distance(segmented_line[i].centroid.x, segmented_line[i].centroid.y, first_centroid.x, first_centroid.y)<2.0)
			pc.points.insert(pc.points.begin(), segmented_line[i].pts.begin(), segmented_line[i].pts.end());
	}

	//Obtain the vehicle's final position and orientation
	pcl::PointCloud<pcl::PointXYZ> pcl;
	sensor_msgs::PointCloud2 pc2;
	sensor_msgs::convertPointCloudToPointCloud2(pc,  pc2);
	pcl::fromROSMsg(pc2, pcl);
	pcl::PointXYZ pt_max, pt_min;
	pcl::getMinMax3D(pcl, pt_min, pt_max);
	geometry_msgs::Point vehicle_xy;
	double vehicle_yaw;
	if(findYawLeastSquare(pc.points, vehicle_yaw))
		vehicle_yaw += M_PI/2;
	else vehicle_yaw -= M_PI/2;
	vehicle_xy.x = (pt_max.x+pt_min.x)/2.0 + 0.45*cos(vehicle_yaw);
	vehicle_xy.y = (pt_max.y+pt_min.y)/2.0 + 0.45*sin(vehicle_yaw);

	tf::Quaternion q;
	q.setRPY(0, 0, vehicle_yaw);
	geometry_msgs::Quaternion vehicle_quat;
	tf::quaternionTFToMsg(q, vehicle_quat);

	geometry_msgs::PoseStamped vehicle_pose;
	vehicle_pose.header = scan.header;
	geometry_msgs::Pose temp_pose;
	temp_pose.position = vehicle_xy;
	temp_pose.orientation = vehicle_quat;
	vehicle_pose.pose = temp_pose;
	ros::NodeHandle n("~");
	n.getParam("target_frame", target_frame_id_);
	tf::StampedTransform trans(tf::Transform(), vehicle_pose.header.stamp, vehicle_pose.header.frame_id, target_frame_id_);
	tf::poseMsgToTF(vehicle_pose.pose, trans);
	tf_broadcaster_->sendTransform(trans);
	vehicle_pub_->publish(vehicle_pose);
	pub_->publish(pc);
}

int main(int argc, char** argcv)
{
	ros::init(argc, argcv, "laser_vehicle");
	ros::NodeHandle nh;
	ros::Subscriber sub = nh.subscribe("base_scan", 10, &laserCallback);
	ros::Publisher pub = nh.advertise<sensor_msgs::PointCloud>("filtered_pts", 10);
	ros::Publisher vehicle_pub = nh.advertise<geometry_msgs::PoseStamped>("vehicle_pose", 10);
	pub_ = &pub;
	vehicle_pub_ = &vehicle_pub;
	tf::TransformBroadcaster tf_broadcaster;
	tf_broadcaster_ = &tf_broadcaster;
	ros::spin();

}
