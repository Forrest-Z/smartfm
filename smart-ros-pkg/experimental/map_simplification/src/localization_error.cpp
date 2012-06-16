/*
 * localization_error.cpp
 *
 *  Created on: Jun 13, 2012
 *      Author: demian
 */

/*
 * Listen to amcl_pose and base_pose_ground_truth
 * and obtain errors on both x-y axis with respect
 * to the vehicle
 */

#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>

#include <tf/transform_datatypes.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/time_synchronizer.h>

#include <fmutil/fm_math.h>

#include <sensor_msgs/PointCloud.h>

#include <map_simplification/error.h>

using std::cout;
using std::endl;
using namespace message_filters;

struct error_info
{
	double value;
	geometry_msgs::Point pos;
	error_info() : value(0.){}
};

struct error_summary
{
	error_info max_lon, max_lat, max_abs;
	double ave_lon, ave_lat, ave_abs;
	int count;
	error_summary() : count(0), ave_lon(0.), ave_lat(0.), ave_abs(0.){}
};

class localization_error
{
public:
	localization_error();
	~localization_error();

private:
	void syncCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr amcl, const nav_msgs::OdometryConstPtr ground_truth);
	void updateErrorSummary(map_simplification::error& er);
	void compareErrorUpdate(error_info &previous_record, double current_record, geometry_msgs::Point pose);
	message_filters::Subscriber<geometry_msgs::PoseWithCovarianceStamped> amcl_pose_sub_;
	message_filters::Subscriber<nav_msgs::Odometry> ground_truth_sub_;
	ros::Publisher error_pub_, lat_error_pub_, lon_error_pub_, amcl_point_pub_;
	sensor_msgs::PointCloud lat_error, lon_error;
	error_summary e_sum;

};

localization_error::~localization_error()
{
	cout<<"Summary: "<<endl;
	cout<<"Average abs_error = "<<e_sum.ave_abs/e_sum.count<<endl;
	cout<<"Average lon_error = "<<e_sum.ave_lon/e_sum.count<<endl;
	cout<<"Average lat_error = "<<e_sum.ave_lat/e_sum.count<<endl;
	cout<<"Maximum abs_error = "<<e_sum.max_abs.value<<" at "<<e_sum.max_abs.pos.x<<", "<<e_sum.max_abs.pos.y<<endl;
	cout<<"Maximum lon_error = "<<e_sum.max_lon.value<<" at "<<e_sum.max_lon.pos.x<<", "<<e_sum.max_lon.pos.y<<endl;
	cout<<"Maximum lat_error = "<<e_sum.max_lat.value<<" at "<<e_sum.max_lat.pos.x<<", "<<e_sum.max_lat.pos.y<<endl;
}

localization_error::localization_error()
{
    ros::NodeHandle nh;
    amcl_pose_sub_.subscribe(nh, "amcl_pose", 20);
    ground_truth_sub_.subscribe(nh, "base_pose_ground_truth", 20);
    error_pub_ = nh.advertise<map_simplification::error>("map_error", 10);
    lat_error_pub_ = nh.advertise<sensor_msgs::PointCloud>("lat_error", 10, true);
    lon_error_pub_ = nh.advertise<sensor_msgs::PointCloud>("lon_error", 10, true);
    amcl_point_pub_ = nh.advertise<sensor_msgs::PointCloud>("amcl_point", 10);
    typedef sync_policies::ExactTime<geometry_msgs::PoseWithCovarianceStamped, nav_msgs::Odometry> syncPolicy;
    Synchronizer<syncPolicy> sync(syncPolicy(10), amcl_pose_sub_, ground_truth_sub_);
    //message_filters::TimeSynchronizer<geometry_msgs::PoseWithCovarianceStamped, nav_msgs::Odometry> syncPolicy(amcl_pose_sub_, ground_truth_sub_, 20);
    sync.registerCallback(boost::bind(&localization_error::syncCallback,this, _1, _2));
    ros::spin();
}

inline double getYaw(geometry_msgs::Quaternion orient)
{
    double y, p, r;
    btQuaternion btq(orient.x, orient.y, orient.z, orient.w);
    btMatrix3x3(btq).getEulerYPR(y, p, r);
    return y;
}

void localization_error::compareErrorUpdate(error_info &previous_record, double current_record, geometry_msgs::Point pose)
{
	current_record = fabs(current_record);
	if(previous_record.value < current_record)
	{
		previous_record.value = current_record;
		previous_record.pos = pose;
	}
}

void localization_error::updateErrorSummary(map_simplification::error& er)
{
	//average will be retrieve during destructor;
	e_sum.count++;
	e_sum.ave_lat += fabs(er.lat_error);
	e_sum.ave_lon += fabs(er.lon_error);
	e_sum.ave_abs += fabs(er.abs_error);
	compareErrorUpdate(e_sum.max_abs, er.abs_error, er.ground_truth);
	compareErrorUpdate(e_sum.max_lat, er.lat_error, er.ground_truth);
	compareErrorUpdate(e_sum.max_lon, er.lon_error, er.ground_truth);
}

void localization_error::syncCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr amcl, const nav_msgs::OdometryConstPtr ground_truth)
{
    geometry_msgs::Point p1 = amcl->pose.pose.position;
    geometry_msgs::Point p2 = ground_truth->pose.pose.position;
    double error = fmutil::distance(p1, p2);
    double amcl_yaw = getYaw(amcl->pose.pose.orientation);
    double ground_yaw = getYaw(ground_truth->pose.pose.orientation);

	map_simplification::error error_msg;
	error_msg.yaw_error = ground_yaw - amcl_yaw;
	error_msg.abs_error = error;
	error_msg.header = amcl->header;
	error_msg.amcl_pose = p1;
	error_msg.ground_truth = p2;
	geometry_msgs::Point rel_pt;
	rel_pt.x = p2.x-p1.x; rel_pt.y = p2.y-p1.y;
	error_msg.lon_error = rel_pt.x * cos(amcl_yaw) + rel_pt.y * sin(amcl_yaw);
	error_msg.lat_error = - rel_pt.x * sin(amcl_yaw) + rel_pt.y * cos(amcl_yaw);
	error_pub_.publish(error_msg);
	updateErrorSummary(error_msg);

    sensor_msgs::PointCloud amcl_pts;
    amcl_pts.header = lon_error.header = lat_error.header = amcl->header;
    amcl_pts.header.stamp = lon_error.header.stamp = lat_error.header.stamp = ros::Time::now();
    geometry_msgs::Point32 xy_error;
    xy_error.x = p2.x; xy_error.y = p2.y;
    xy_error.z = fabs(error_msg.lon_error);
    lon_error.points.push_back(xy_error);
    xy_error.z = fabs(error_msg.lat_error);
    lat_error.points.push_back(xy_error);
    lat_error_pub_.publish(lat_error);
    lon_error_pub_.publish(lon_error);
    geometry_msgs::Point32 amcl_pt; amcl_pt.x = p1.x; amcl_pt.y = p1.y;
    amcl_pts.points.push_back(amcl_pt);
    amcl_point_pub_.publish(amcl_pts);
}

int main(int argc, char** argcv)
{
    ros::init(argc, argcv, "localization_error");
    localization_error le;

    return 0;
}
