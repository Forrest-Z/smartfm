/*
 * PedPomdpNode.h
 *
 */

#ifndef PEDESTRIAN_MOMDP_H_
#define PEDESTRIAN_MOMDP_H_

#include "momdp.h"

class PedPomdpNode
{
public:
    PedPomdpNode();
    ~PedPomdpNode();
    void robotPoseCallback(geometry_msgs::PoseWithCovarianceStamped odo);
    void speedCallback(nav_msgs::Odometry odo);
    void pedPoseCallback(ped_momdp_sarsop::ped_local_frame_vector);
    void moveSpeedCallback(geometry_msgs::Twist speed);
	void publishPath();
    ros::Subscriber speedSub_, pedSub_, scanSub_, move_base_speed_;
	ros::Publisher pc_pub,path_pub;
    ped_momdp* momdp;

	bool pathPublished;
};

#endif /* PEDESTRIAN_MOMDP_H_ */
