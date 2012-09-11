/*
 * AccumulateData.cpp
 *
 *  Created on: Sep 4, 2012
 *      Author: demian
 */

#ifndef ACCUMULATEDATA_CPP_
#define ACCUMULATEDATA_CPP_


#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <tf/transform_listener.h>
#include <tf/message_filter.h>
#include <message_filters/subscriber.h>
#include <laser_geometry/laser_geometry.h>
#include <fmutil/fm_math.h>
#include <fmutil/fm_stopwatch.h>

#include "pcl/point_cloud.h"
#include "pcl_ros/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl/ros/conversions.h"
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/voxel_grid.h>

using namespace std;

class AccumulateData
{
    const string target_frame_;
    double min_dist_;
    unsigned int scan_buffer_;
    laser_geometry::LaserProjection projector_;
    tf::StampedTransform last_transform_;
    tf::StampedTransform sensor_transform_;
    vector<sensor_msgs::PointCloud> data_;

    bool checkDistance(const tf::StampedTransform& newTf)
    {
        tf::Transform odom_old_new  = last_transform_.inverse() * newTf;
        float tx, ty;
        tx = -odom_old_new.getOrigin().y();
        ty =  odom_old_new.getOrigin().x();
        float mov_dis = sqrtf(tx*tx + ty*ty);

        if(mov_dis > min_dist_) return true;
        else return false;
    }

public:

    bool new_data_;

    AccumulateData(string target_frame, double min_dist, double scan_buffer): target_frame_(target_frame), new_data_(false)
    {
    	scan_buffer_ = scan_buffer;
    	min_dist_ = min_dist;
    }

    void updateParameter(double min_dist, int scan_buffer)
    {
    	scan_buffer_ = abs(scan_buffer);//data_dist/min_dist+15.0;
    	min_dist_ = min_dist;
    	cout<<"AccumuateData parameter updated. scan_buffer="<<scan_buffer_<<" min_dist="<<min_dist_<<endl;
    }
    //add latest observation to the sensor and erase the old buffer if necessary
    void addData(sensor_msgs::PointCloud2 &src, tf::TransformListener &tf)
    {
        sensor_msgs::PointCloud scan;
        sensor_msgs::convertPointCloud2ToPointCloud(src, scan);
        tf::StampedTransform latest_transform;

        tf.lookupTransform(target_frame_, scan.header.frame_id, scan.header.stamp, latest_transform);
        sensor_transform_ = latest_transform;
        tf.transformPointCloud(target_frame_, scan, scan);
        if(data_.size() == 0)
        {
            last_transform_ = latest_transform;
            insertPointCloud(scan);
            return;
        }

        if(checkDistance(latest_transform))
        {
            if(insertPointCloud(scan))
            {
                last_transform_ = latest_transform;
                new_data_ = true;
            }
        }
    }

    void addData(sensor_msgs::LaserScan &scan, tf::TransformListener &tf)
    {
        tf::StampedTransform latest_transform;
        tf.lookupTransform(target_frame_, scan.header.frame_id, scan.header.stamp, latest_transform);
        sensor_transform_ = latest_transform;
        if(data_.size() == 0 )
        {
            //only happens during initialization
            last_transform_ = latest_transform;
            insertData(scan, tf);
            return;
        }

        if(checkDistance(latest_transform))
        {
            if(insertData(scan, tf))
            {
                last_transform_ = latest_transform;
                new_data_ = true;
            }
        }
    }

    bool insertPointCloud(sensor_msgs::PointCloud &scan)
    {
        data_.insert(data_.begin(), scan);
        if(data_.size()>scan_buffer_) data_.resize(scan_buffer_);
        return true;
    }

    bool insertData(sensor_msgs::LaserScan &scan, tf::TransformListener &tf)
    {
        sensor_msgs::PointCloud laser_cloud;

        try{projector_.transformLaserScanToPointCloud(target_frame_, scan, laser_cloud, tf);}
        catch (tf::TransformException& e){ ROS_ERROR("%s",e.what());return false;}

        return insertPointCloud(laser_cloud);
    }

    //obtain accumulated points so far that is sorted from new to old transformed by target frame
    bool getLatestAccumulated(vector<sensor_msgs::PointCloud> &data)
    {
        data = data_;

        //A simple flag to say we have new accumulated data
        bool new_data = new_data_;
        new_data_ = false;
        return new_data;
    }


};

#endif /* ACCUMULATEDATA_CPP_ */
