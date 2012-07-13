/*
 * matcher.cpp
 *
 *  Created on: Jul 13, 2012
 *      Author: demian
 */

#include "GridMap.h"

GridMap *gm, *gm_hd;
ros::Publisher *pub_;
tf::TransformBroadcaster *tf_broadcaster_;
bool initialized=false;
double scan_odo_x=0, scan_odo_y=0, scan_odo_t=0;
void pcCallback(const sensor_msgs::PointCloud2& pc)
{
	fmutil::Stopwatch sw("Overall matching time");
	sw.start();

	if(initialized)
	{
		int max_score=0;
		simple_pose best_pose;
		//gm->findBestMatchBruteForce(pc, best_pose, max_score);
		gm_hd->findBestMatchRotateFirst(pc, best_pose, max_score, 2.0, 1.0, 7.0, 2.0);
		cout<<"Max score at "<< best_pose.x <<", "<<best_pose.y<<", "<<best_pose.t<<": "<<max_score<<endl;
		gm_hd->findBestMatchRotateFirst(pc, best_pose, max_score, 1.0, 0.25, 2.0, 0.5, false, best_pose);
		cout<<"New finer score at "<< best_pose.x <<", "<<best_pose.y<<", "<<best_pose.t<<": "<<max_score<<endl;
		gm_hd->findBestMatchRotateFirst(pc, best_pose, max_score, 0.25, 0.05, 0.5, 0.1, false, best_pose);
		cout<<"New finer score at "<< best_pose.x <<", "<<best_pose.y<<", "<<best_pose.t<<": "<<max_score<<endl;
		gm_hd->findBestMatchRotateFirst(pc, best_pose, max_score, 0.05, 0.01, 0.1, 0.02, false, best_pose);
		cout<<"New finer score at "<< best_pose.x <<", "<<best_pose.y<<", "<<best_pose.t<<": "<<max_score<<endl;
		scan_odo_t += best_pose.t;
		scan_odo_x += best_pose.x * cos(scan_odo_t);
		scan_odo_y += best_pose.x * sin(scan_odo_t);



		cout<<"Odo now at "<< scan_odo_x <<", "<<scan_odo_y<<", "<<scan_odo_t<<endl;
		geometry_msgs::Pose pose;
		pose.position.x = scan_odo_x;
		pose.position.y = scan_odo_y;
		pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, 0, scan_odo_t);
		tf::StampedTransform trans(tf::Transform(), pc.header.stamp, "scan_odo", "base_link");
		tf::poseMsgToTF(pose, trans);
		tf_broadcaster_->sendTransform(trans);
		cout<<endl;
	}

	gm->buildMapDirectDraw(pc);
	gm_hd->buildMapDirectDraw(pc);
	sensor_msgs::PointCloud grid_map_pc;
	grid_map_pc.header = pc.header;

	gm->getMap(grid_map_pc);
	pub_->publish(grid_map_pc);
	initialized = true;
	sw.end(true);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "grid_map");
    gm = new GridMap(0.5, 0.1);
    gm_hd = new GridMap(0.05, 0.1);
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("pc_out", 10, &pcCallback);
    ros::Publisher pub = nh.advertise<sensor_msgs::PointCloud>("grid_map", 10);
    pub_ = &pub;

    tf::TransformBroadcaster tf_broadcast;
    tf_broadcaster_ = &tf_broadcast;
    ros::spin();
}
