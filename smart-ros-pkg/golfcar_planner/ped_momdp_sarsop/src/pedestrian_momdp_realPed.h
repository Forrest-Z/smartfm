/*
 * pedestrian_momdp.h
 *
 *  Created on: Sep 15, 2011
 *      Author: golfcar
 */

#ifndef PEDESTRIAN_MOMDP_H_
#define PEDESTRIAN_MOMDP_H_

#include "momdp.h"

class pedestrian_momdp 
{
public:
    pedestrian_momdp();
    ~pedestrian_momdp();
    void robotPoseCallback(geometry_msgs::PoseWithCovarianceStamped odo);
    void speedCallback(nav_msgs::Odometry odo);
    void pedPoseCallback(ped_momdp_sarsop::ped_local_frame_vector);   
    void moveSpeedCallback(pnc_msgs::move_status status);

    ros::Subscriber speedSub_, pedSub_, scanSub_, move_base_speed_;
    ped_momdp* momdp;
};

#endif /* PEDESTRIAN_MOMDP_H_ */
