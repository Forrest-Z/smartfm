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
    fmutil::LowPassFilter *filter_yaw_;
    fmutil::LowPassFilter *filter_x_;
    fmutil::LowPassFilter *filter_y_;

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
        filter_yaw_ = new fmutil::LowPassFilter(0.4);
        filter_x_ = new fmutil::LowPassFilter(0.2);
        filter_y_ = new fmutil::LowPassFilter(0.2);
    }

    geometry_msgs::PoseStamped updateMeasurement(vector<geometry_msgs::PoseStamped> possible_poses)
    {
        double dist_now = 1e9;
        geometry_msgs::PoseStamped nearest_pose;
        cout<<"-------------start----------"<<endl;
        for(size_t i=0; i<possible_poses.size(); i++)
        {
            double dist =fmutil::distance<geometry_msgs::Point>(possible_poses[i].pose.position, cur_pose_.pose.position);
            double yaw = getYawFromQuadMsg(possible_poses[i].pose.orientation);
            //add a little yaw cost into dist function such that when the same dist
            dist+=0.1*fabs(yaw);
            if(dist<dist_now)
            {
                nearest_pose = possible_poses[i];
                dist_now = dist;
                cout<<dist_now<<" "<<dist<<" "<<possible_poses[i].pose.position.x<<" "<<possible_poses[i].pose.position.y<<" "<<getYawFromQuadMsg(possible_poses[i].pose.orientation)<<endl;
            }
        }
        if(dist_now<3.0) cur_pose_ = nearest_pose;
        else cout<<"Not updating pose"<<endl;
        cout<<"-------------end----------"<<endl;



        double yaw = getYawFromQuadMsg(cur_pose_.pose.orientation);
        double yaw_filtered = filter_yaw_->filter(cur_pose_.header.stamp.toSec(), yaw);
        double x_filtered = filter_x_->filter(cur_pose_.header.stamp.toSec(), cur_pose_.pose.position.x);
        double y_filtered = filter_y_->filter(cur_pose_.header.stamp.toSec(), cur_pose_.pose.position.y);
        if(isnan(yaw_filtered))
        {
            cout<<"isnan detected, resetting filter"<<endl;
            filter_yaw_->reset();
            yaw_filtered = filter_yaw_->filter(cur_pose_.header.stamp.toSec(), yaw);
        }
        cur_pose_.pose.orientation = getQuadMsgFromYaw(yaw_filtered);
        cur_pose_.pose.position.x = x_filtered;
        cur_pose_.pose.position.y = y_filtered;
        cout<<"New position= "<<cur_pose_.pose.position.x<<" "<<cur_pose_.pose.position.y<<" "<<yaw_filtered<<endl;
        return cur_pose_;

    }

};
