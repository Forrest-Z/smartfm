/*
 * nearest_neighbor_tracking.cpp
 *
 *  Created on: Sep 24, 2012
 *      Author: demian
 */
#include <geometry_msgs/PoseStamped.h>
#include <fmutil/fm_math.h>
#include <fmutil/fm_filter.h>
class NearestNeighborTracking
{
    geometry_msgs::PoseStamped cur_pose_;
    fmutil::LowPassFilter *filter_;

    double getYawFromQuadMsg(geometry_msgs::Quaternion quad_msg)
    {
        double y,p,r;
        tf::Quaternion quad;
        tf::quaternionMsgToTF(quad_msg, quad);
        btMatrix3x3(quad.asBt()).getEulerYPR(y,p,r);
        return y;
    }

    geometry_msgs::Quaternion getQuadMsgFromYaw(double yaw)
    {
        tf::Quaternion quad;
        quad.setRPY(0,0,yaw);
        geometry_msgs::Quaternion quadmsg;
        tf::quaternionTFToMsg(quad, quadmsg);
        return quadmsg;
    }
public:
    NearestNeighborTracking(geometry_msgs::PoseStamped pose_init): cur_pose_(pose_init)
    {
        cout<<"Initialized with pose "<<cur_pose_.pose.position.x<<" "<<cur_pose_.pose.position.y<<endl;
        filter_ = new fmutil::LowPassFilter(0.2);
    }

    geometry_msgs::PoseStamped updateMeasurement(vector<geometry_msgs::PoseStamped> possible_poses)
    {
        double dist_now = 1e9;
        geometry_msgs::PoseStamped nearest_pose;
        cout<<"-------------start----------"<<endl;
        for(size_t i=0; i<possible_poses.size(); i++)
        {
            double dist =fmutil::distance<geometry_msgs::Point>(possible_poses[i].pose.position, cur_pose_.pose.position);

            if(dist<dist_now)
            {
                nearest_pose = possible_poses[i];
                dist_now = dist;
                cout<<dist_now<<" "<<dist<<" "<<possible_poses[i].pose.position.x<<" "<<possible_poses[i].pose.position.y<<" "<<getYawFromQuadMsg(possible_poses[i].pose.orientation)<<endl;
            }
        }
        if(dist_now<6.0) cur_pose_ = nearest_pose;
        else cout<<"Not updating pose"<<endl;
        cout<<"-------------end----------"<<endl;



        double yaw = getYawFromQuadMsg(cur_pose_.pose.orientation);
        yaw = filter_->filter(cur_pose_.header.stamp.toSec(), yaw);
        cur_pose_.pose.orientation = getQuadMsgFromYaw(yaw);
        cout<<"New position= "<<cur_pose_.pose.position.x<<" "<<cur_pose_.pose.position.y<<" "<<yaw<<endl;
        return cur_pose_;

    }

};
