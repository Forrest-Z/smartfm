/*
 * vehicles_transform.cpp
 *
 *  Created on: Jul 10, 2012
 *      Author: demian
 */

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/LaserScan.h>
#include <fmutil/fm_math.h>
#include <geometry_msgs/PoseArray.h>

using namespace std;

tf::TransformListener *listener_;
ros::Publisher *detected_poses_;
void scanCallback(sensor_msgs::LaserScan scan)
{
	//look for latest available transform from 2 different robots
	//everything based on robot_2's base_link
	tf::StampedTransform robot01_tf, robot12_tf, robot02_tf, trueRobot02_tf, trueRobot12_tf;
	vector<tf::StampedTransform> detected_robot_tf, true_robot_tf, combined_robot_tf;
	const int robot_number = 7;


	stringstream last_robot_frame;
	last_robot_frame<<"/robot_"<<robot_number-1<<"/base_link";
	for(int i=0; i<robot_number-1; i++)
	{
		stringstream detected_frame, base_frame, robotinfront_true_frame;
		tf::StampedTransform detected_robot_trans, true_robot_trans;
		detected_frame<<"/robot_"<<i<<"_"<<i+1;
		base_frame<<"/robot_"<<i+1<<"/base_link";
		robotinfront_true_frame<<"/robot_"<<i<<"/base_link";
		try
		{
			//cout<<"Detected frame "<<i<<" "<<detected_frame.str()<<" "<< base_frame.str()<<endl;
			//cout<<"True frame "<<robotinfront_true_frame.str()<< " "<<last_robot_frame.str()<<endl;
			listener_->lookupTransform(base_frame.str(), detected_frame.str(),
					ros::Time(0), detected_robot_trans);
			detected_robot_tf.push_back(detected_robot_trans);
			listener_->lookupTransform(last_robot_frame.str(), robotinfront_true_frame.str(),
					ros::Time(0), true_robot_trans);
			true_robot_tf.push_back(true_robot_trans);
			//cout<<"True robot pose "<< true_robot_tf[true_robot_tf.size()-1].getOrigin().x()<<" "<<true_robot_tf[true_robot_tf.size()-1].getOrigin().y()<<endl;
		}
		catch (tf::LookupException ex)
		{
			cout<<ex.what()<<endl; return;
		}
		catch (tf::ExtrapolationException ex)
		{
			cout<<ex.what()<<endl; return;
		}
		catch (tf::ConnectivityException ex)
		{
			cout<<ex.what()<<endl; return;
		}
	}

	//cout<<detected_robot_tf.size()<<endl;
	//get the combined tf by performing robot02_tf.mult(robot01_tf, robot12_tf);
	combined_robot_tf.resize(robot_number-1);
	for(int i=0; i<robot_number-1; i++)
	{
		//cout<<"For tf "<<i<<": "<<endl;
		if((robot_number-2-i)>=1)
		{
			//cout<<i<<" mult "<<i+1<<endl;
			combined_robot_tf[i].mult(detected_robot_tf[i+1], detected_robot_tf[i]);
			for(int j=i+2; j<robot_number-1; j++)
			{
				//cout<<" mult "<<j<<endl;
				combined_robot_tf[i].mult(detected_robot_tf[j], combined_robot_tf[i]);
			}
		}
		else combined_robot_tf[i]=detected_robot_tf[i];
		//cout<<endl;
	}

	assert(robot_number>2);
	assert(combined_robot_tf.size() == true_robot_tf.size());

	vector<double> dist_error, yaw_error;
	geometry_msgs::PoseArray detected_poses;
	detected_poses.header = scan.header;
	stringstream header;
	header<<"/robot_"<<robot_number-1<<"/base_link";
	detected_poses.header.frame_id = header.str();

	for(int i=0; i<robot_number-1; i++)
	{
		geometry_msgs::Pose detected_pose;
		tf::poseTFToMsg(combined_robot_tf[i], detected_pose);
		detected_poses.poses.push_back(detected_pose);
		tf::Vector3 true_robot_pose = true_robot_tf[i].getOrigin();
		tf::Vector3 detect_robot_pose = combined_robot_tf[i].getOrigin();
		double true_robot_yaw = tf::getYaw(true_robot_tf[i].getRotation());
		double detect_robot_yaw = tf::getYaw(combined_robot_tf[i].getRotation());
		dist_error.push_back(fmutil::distance(true_robot_pose.x(), true_robot_pose.y(), detect_robot_pose.x(), detect_robot_pose.y()));
		yaw_error.push_back(fabs(true_robot_yaw - detect_robot_yaw));
	}
	for(size_t i=0; i<dist_error.size(); i++)
		cout<<dist_error[i]<<" ";
	for(size_t i=0; i<yaw_error.size(); i++)
		cout<<yaw_error[i]<<" ";
	cout<<endl;

	detected_poses_->publish(detected_poses);
}

int main(int argc, char** argcv)
{
	ros::init(argc, argcv, "vehicle_transform");
	ros::NodeHandle nh;
	tf::TransformListener listener;
	listener_ = &listener;
	ros::Publisher pub = (nh.advertise<geometry_msgs::PoseArray>("detected_poses", 10));
	detected_poses_ = &pub;
	ros::Subscriber sub = nh.subscribe("robot_2/base_scan", 10, &scanCallback);
	ros::spin();

}
