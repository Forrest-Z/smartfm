/*
 * matcher.cpp
 *
 *  Created on: Jul 13, 2012
 *      Author: demian
 */

//#include "RasterMapPCL.h"
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <iostream>
#include <fstream>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>


#include <fmutil/fm_math.h>
#include <fmutil/fm_stopwatch.h>
#include <cv.h>
#include <highgui.h>
using namespace std;
#include "renderMap.h"


ros::Publisher *odo_pub_, *src_pc_pub_, *dst_pc_pub_;
tf::TransformBroadcaster *tf_broadcaster_;
tf::StampedTransform odo_transform_;
//RasterMapPCL *csm_;
//ofstream *myfile_;
bool initialized=false;
int pcCb_count=0;
sensor_msgs::PointCloud pre_pc_;
geometry_msgs::Point starting_pt_;
ofstream *myfile_;
double odometer_ = 0.0;
double next_write_pose_ = 0.0;
void pcCallback(const sensor_msgs::PointCloudConstPtr& pc_ptr, const nav_msgs::OdometryConstPtr& odom)
{
	sensor_msgs::PointCloud pc = *pc_ptr;
	pcCb_count++;
	//if(pcCb_count%10!=0) return;

	if(!initialized)
	{
		starting_pt_ = odom->pose.pose.position;
		initialized = true;
	}
	odometer_ += fmutil::distance(starting_pt_, odom->pose.pose.position);
	starting_pt_ = odom->pose.pose.position;
	cout<<odometer_<<"             \xd"<<flush;
	if( odometer_ >= next_write_pose_)
	{

		btScalar pitch, roll, yaw;
		geometry_msgs::Quaternion orientation = odom->pose.pose.orientation;
		btQuaternion btq(orientation.x, orientation.y, orientation.z, orientation.w);
		btMatrix3x3(btq).getEulerYPR(yaw, pitch, roll);

		*myfile_ << odom->pose.pose.position.x << " " << odom->pose.pose.position.y <<" " << odom->pose.pose.position.z << " " << roll <<" " << pitch<< " "<<yaw<<" ";
		*myfile_ << pc.header.stamp.toNSec()<<" "<<pc.points.size();
		for(size_t i=0; i<pc.points.size(); i++)
			*myfile_<<" "<<pc.points[i].x<<" "<<pc.points[i].y<<" "<<pc.points[i].z;
		*myfile_ <<endl;

		next_write_pose_ += 2.0;
		cout<<odometer_<<" write \xd"<<flush;
	}
	cout<<flush;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "grid_map");

    ros::NodeHandle nh;

    message_filters::Subscriber<sensor_msgs::PointCloud> pc_sub(nh, "pc_legacy_out", 1);
    message_filters::Subscriber<nav_msgs::Odometry> odom_sub(nh, "odom", 1);

    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud, nav_msgs::Odometry> MySyncPolicy;
    // ApproximateTime takes a queue size as its constructor argument, hence MySyncPolicy(10)
    message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), pc_sub, odom_sub);
    sync.registerCallback(boost::bind(&pcCallback, _1, _2));

    ros::Publisher odo_pub = nh.advertise<nav_msgs::Odometry>("scan_odo", 10);
    odo_pub_ = &odo_pub;

    ros::Publisher src_pc_pub = nh.advertise<sensor_msgs::PointCloud>("src_pc", 10);
    src_pc_pub_ = &src_pc_pub;
    ros::Publisher dst_pc_pub = nh.advertise<sensor_msgs::PointCloud>("dst_pc", 10);
    dst_pc_pub_ = &dst_pc_pub;

//    RasterMapPCL csm;
//    csm_ = &csm;
    tf::TransformBroadcaster tf_broadcast;
    tf_broadcaster_ = &tf_broadcast;
    odo_transform_.setRotation(tf::Quaternion::getIdentity());

    ofstream myfile;
    stringstream ss;
    ss<<"frontEndData_"<<ros::WallTime::now().toNSec()<<".txt";
    myfile.open (ss.str().c_str());
    myfile_ = &myfile;
    ros::spin();
    myfile_->close();

}
