/*
 * matcher.cpp
 *
 *  Created on: Jul 13, 2012
 *      Author: demian
 */

#include "RasterMapPCL.h"
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/point_cloud_conversion.h>


ros::Publisher *odo_pub_;
tf::TransformBroadcaster *tf_broadcaster_;
tf::StampedTransform odo_transform_;
RasterMapPCL *csm_;
bool initialized=false;
int pcCb_count=0;
void pcCallback(const sensor_msgs::PointCloud& pc)
{
	pcCb_count++;
	//if(pcCb_count%10!=0) return;
	fmutil::Stopwatch sw("Overall matching time");
	sw.start();
	sensor_msgs::PointCloud src_pc = pc;
	//sensor_msgs::convertPointCloud2ToPointCloud(pc, src_pc);
	if(initialized)
	{
		transform_info best_pose;
		best_pose = csm_->getBestTf(src_pc);

		cv::Mat cov = best_pose.covariance;
		cout<<"cov_x="<<sqrt(cov.at<float>(0,0))<<" cov_y="<<sqrt(cov.at<float>(1,1))<<" cov_t="<<sqrt(cov.at<float>(2,2))/M_PI*180<<endl;

		tf::Transform new_transform;
		tf::Vector3 origin(best_pose.translation_2d.x, best_pose.translation_2d.y, 0.0);
		new_transform.setOrigin(origin);
		new_transform.setRotation(tf::createQuaternionFromYaw(best_pose.rotation));
		odo_transform_.mult(odo_transform_, new_transform);

		tf::Quaternion orientation = odo_transform_.getRotation();
		btQuaternion btq(orientation.x(), orientation.y(), orientation.z(), orientation.w());
		btScalar pitch, roll, yaw;
		btMatrix3x3(btq).getEulerYPR(yaw, pitch, roll);
		double scan_odo_x=0, scan_odo_y=0, scan_odo_t=0;
		scan_odo_t = yaw;

		scan_odo_x = odo_transform_.getOrigin().x();// best_pose.x * cos(scan_odo_t);
		scan_odo_y = odo_transform_.getOrigin().y();//+= best_pose.x * sin(scan_odo_t);



		//cout<<"Odo now at "<< scan_odo_x <<", "<<scan_odo_y<<", "<<scan_odo_t/M_PI*180<<endl;
		geometry_msgs::Pose pose;
		pose.position.x = scan_odo_x;
		pose.position.y = scan_odo_y;
		pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, 0, scan_odo_t);
		tf::StampedTransform trans(odo_transform_, pc.header.stamp, "scan_odo", "base_link");
		//tf::poseMsgToTF(pose, trans);
		tf_broadcaster_->sendTransform(trans);

		nav_msgs::Odometry odo;
		odo.header = pc.header;
		odo.header.frame_id = "scan_odo";
		odo.pose.pose = pose;
		odo_pub_->publish(odo);
		//cout<<endl;
	}

	csm_->setInputPts(src_pc);
	initialized = true;
	sw.end(true);cout<<endl;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "grid_map");

    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("pc_out", 10, &pcCallback);

    ros::Publisher odo_pub = nh.advertise<nav_msgs::Odometry>("scan_odo", 10);
    odo_pub_ = &odo_pub;

    RasterMapPCL csm;
    csm_ = &csm;
    tf::TransformBroadcaster tf_broadcast;
    tf_broadcaster_ = &tf_broadcast;
    odo_transform_.setRotation(tf::Quaternion::getIdentity());

    ros::spin();
}
