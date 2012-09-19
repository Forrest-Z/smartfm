/*
 * carmen.h
 *
 *  Created on: Sep 7, 2012
 *      Author: demian
 */

#ifndef CARMEN_H_
#define CARMEN_H_

#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <tf/transform_listener.h>
#include <tf/message_filter.h>
#include <message_filters/subscriber.h>
#include <laser_geometry/laser_geometry.h>
#include <fmutil/fm_math.h>
#include <fmutil/fm_stopwatch.h>
#include "sensorstream/CarmenLog.h"
#include <iostream>
#include <fstream>

class ConvertLaserCarmen
{
    double min_dist_;
    bool initialized_, writefile_;
    tf::StampedTransform last_transform_;

    message_filters::Subscriber<sensor_msgs::LaserScan> laser_scan_sub_;
    tf::MessageFilter<sensor_msgs::LaserScan>       *laser_scan_filter_;

    ros::Publisher laser_pub_;
    tf::TransformListener *tf_;
    ros::Time initialized_time_;
    string target_frame_, logfile_, base_frame_;
    ofstream output_;
    int scan_seq_;

    CarmenLogWriter carmen_writer_;
    bool checkDistance(const tf::StampedTransform& newTf)
    {
        if(initialized_)
        {
            tf::Transform odom_old_new  = last_transform_.inverse() * newTf;
            float tx, ty;
            tx = -odom_old_new.getOrigin().y();
            ty =  odom_old_new.getOrigin().x();
            float mov_dis = sqrtf(tx*tx + ty*ty);
            if(mov_dis > min_dist_)
            {
                last_transform_ = newTf;
                return true;
            }
            else return false;
        }
        else
        {
            initialized_time_ = ros::Time::now();
            last_transform_ = newTf;
            initialized_ = true;
            return true;
        }
    }

    void scanCallback(const sensor_msgs::LaserScanConstPtr scan)
    {
        tf::StampedTransform sensor_transform, base_transform;
        tf_->lookupTransform(target_frame_, scan->header.frame_id, scan->header.stamp, sensor_transform);
        tf_->lookupTransform(target_frame_, base_frame_, scan->header.stamp, base_transform);
        if(checkDistance(base_transform))
        {
            if(writefile_)
            {
                double base_yaw,sensor_yaw, p,r;
                btMatrix3x3(base_transform.getRotation().asBt()).getEulerYPR(base_yaw,p,r);
                btMatrix3x3(sensor_transform.getRotation().asBt()).getEulerYPR(sensor_yaw,p,r);
                tf::Vector3 base_pose = base_transform.getOrigin();
                tf::Vector3 sensor_pose = sensor_transform.getOrigin();
                //output_<<"VERTEX2 "<<scan_seq_++<<" "<<base_pose.getX()<<" "<<base_pose.getY()<<" "<<base_yaw<<endl;
                /*output_<<"ROBOTLASER1 0 "<<scan->angle_min<<" "<<scan->angle_max-scan->angle_min<<" "<<scan->angle_increment<<" "<<scan->range_max<<" 0.03 0 "<<scan->ranges.size()<<" ";
                for(size_t i=0; i<scan->ranges.size(); i++)
                {
                    output_<<scan->ranges[i]<<" ";
                }
                //what's the meaning of x in the log?
                //from the source code, apparently it is normally has 0 remission
                //followed by laserpose xyz, robotpose xyz
                output_<< "0  "<< sensor_pose.getX() <<" "<< sensor_pose.getY() <<" "<<sensor_yaw<<" "<<base_pose.getX()<<" "<<base_pose.getY()<<" "<<base_yaw<<" ";
                //tv,rv,forward_safety_dist,side_safety_dist,turn_axis
                output_<< "0.0 0.0 0.0 0.0 0.0 ";
                //laser timestamp, host and relative timestamp
                output_<< scan->header.stamp.toSec()<<" x "<< (ros::Time::now() - initialized_time_).toSec()<<endl;*/
                ;
                vector<double> phi, range;
                for(size_t i =0; i<scan->ranges.size(); i++)
                {
                    phi.push_back(scan->angle_min + (i*scan->angle_increment));
                    range.push_back(scan->ranges[i]);
                }
                //all hell break loose when the laser is having a 360 deg fov
                LaserReading carmen_laser(phi, range, scan->header.stamp.toSec());
                OrientedPoint2D carmen_pose(sensor_pose.getX(), sensor_pose.getY(), sensor_yaw);
                carmen_laser.setLaserPose(carmen_pose);
                carmen_laser.setMaxRange(scan->range_max);
                vector<AbstractReading*> carmen_lasers;
                carmen_lasers.push_back(carmen_laser.clone());
                carmen_writer_.writeLog(output_, carmen_lasers);
            }
            laser_pub_.publish(scan);
        }

    }


public:
    ~ConvertLaserCarmen()
    {
        output_.close();
    }

    ConvertLaserCarmen(): initialized_(false), writefile_(false), scan_seq_(0)
    {
        ros::NodeHandle nh;
        laser_pub_ = nh.advertise<sensor_msgs::LaserScan>("throttled_laser_out", 10);

        ros::NodeHandle private_nh("~");
        private_nh.param("min_dist", min_dist_, 0.5);
        private_nh.param("target_frame", target_frame_, std::string("/odom"));
        private_nh.param("target_frame", base_frame_, std::string("/base_link"));
        private_nh.param("logfile", logfile_, std::string(""));
        tf::TransformListener tf;
        tf_ = &tf;

        laser_scan_sub_.subscribe(nh, "laser_out", 10);
        laser_scan_filter_ = new tf::MessageFilter<sensor_msgs::LaserScan>(laser_scan_sub_, *tf_, target_frame_, 10);
        laser_scan_filter_->registerCallback(boost::bind(&ConvertLaserCarmen::scanCallback, this, _1));

        if(logfile_.size()>0)
        {
            output_.open (logfile_.c_str());
            writefile_ = true;
            cout<<"Saving carmen log file to "<<logfile_<<endl;
        }

        ros::spin();
    }

};

int main(int argc, char** argcv)
{
    ros::init(argc, argcv, "convertLaserCarmen");
    ConvertLaserCarmen clc;
    return 0;

}
#endif /* CARMEN_H_ */
