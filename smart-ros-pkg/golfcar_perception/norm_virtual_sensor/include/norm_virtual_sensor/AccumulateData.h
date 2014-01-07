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

struct PoinCloudAndDistance{
  sensor_msgs::PointCloud pc;
  double dist;
};

class AccumulateData
{
    const string target_frame_;
    double min_dist_;
    unsigned int scan_buffer_;
    laser_geometry::LaserProjection projector_;
    tf::StampedTransform last_transform_;
    tf::StampedTransform sensor_transform_;
    vector<PoinCloudAndDistance> data_;
    double max_dist_;
    bool checkDistance(const tf::StampedTransform& newTf, double &mov_dis)
    {
        tf::Transform odom_old_new  = last_transform_.inverse() * newTf;
        double tx, ty;
        tx = -odom_old_new.getOrigin().y();
        ty =  odom_old_new.getOrigin().x();
        mov_dis = sqrtf(tx*tx + ty*ty);

        if(mov_dis > min_dist_) return true;
        else return false;
    }

public:

    bool new_data_;

    AccumulateData(string target_frame, double min_dist, double scan_buffer): target_frame_(target_frame), new_data_(false)
    {
    	scan_buffer_ = scan_buffer;
    	min_dist_ = min_dist;
	max_dist_ = min_dist_*scan_buffer_;
    }

    void updateParameter(double min_dist, int scan_buffer)
    {
    	scan_buffer_ = abs(scan_buffer);//data_dist/min_dist+15.0;
    	min_dist_ = min_dist;
	max_dist_ = min_dist_*scan_buffer_;
	cout<<"Window keep at "<<max_dist_<<" m."<<endl;
    	cout<<"AccumuateData parameter updated. scan_buffer="<<scan_buffer_<<" min_dist="<<min_dist_<<endl;
    }
    //add latest observation to the sensor and erase the old buffer if necessary
    void addData(sensor_msgs::PointCloud &src, tf::TransformListener &tf)
    {
    	tf::StampedTransform latest_transform;
    	tf.lookupTransform(target_frame_, src.header.frame_id, src.header.stamp, latest_transform);
    	sensor_transform_ = latest_transform;
    	tf.transformPointCloud(target_frame_, src, src);
    	if(data_.size() == 0)
    	{
    		last_transform_ = latest_transform;
    		insertPointCloud(src, 0);
    		return;
    	}
	double move_dist;
    	if(checkDistance(latest_transform,move_dist))
    	{
    		if(insertPointCloud(src, move_dist))
    		{
    			last_transform_ = latest_transform;
    			new_data_ = true;
    		}
    	}
    }
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
            insertPointCloud(scan, 0);
            return;
        }
	double move_dist;
    	if(checkDistance(latest_transform,move_dist))
        {
            if(insertPointCloud(scan, move_dist))
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
            insertData(scan, tf, 0);
            return;
        }
	double move_dist;
    	if(checkDistance(latest_transform,move_dist))
        {
            if(insertData(scan, tf, move_dist))
            {
                last_transform_ = latest_transform;
                new_data_ = true;
            }
        }
    }

    bool insertPointCloud(sensor_msgs::PointCloud &scan, double move_dist)
    {
	PoinCloudAndDistance pc_and_dist;
	pc_and_dist.pc = scan;
	pc_and_dist.dist = move_dist;
        data_.insert(data_.begin(), pc_and_dist);
        if(data_.size()>scan_buffer_ ) data_.resize(scan_buffer_);
	double total_dist = 0;
	size_t window_size = 0;
	for(; window_size<data_.size(); window_size++){
	  total_dist += data_[window_size++].dist;
	  if(total_dist > max_dist_) break;
	}
	cout<<window_size<<" "<<data_.size()<<" "<<total_dist<<endl;
	if(window_size<data_.size()) 
	  data_.resize(window_size);
	
        return true;
    }

    bool insertData(sensor_msgs::LaserScan &scan, tf::TransformListener &tf, double move_dist)
    {
        sensor_msgs::PointCloud laser_cloud;

        try{projector_.transformLaserScanToPointCloud(target_frame_, scan, laser_cloud, tf);}
        catch (tf::TransformException& e){ ROS_ERROR("%s",e.what());return false;}

        return insertPointCloud(laser_cloud, move_dist);
    }

    //obtain accumulated points so far that is sorted from new to old transformed by target frame
    bool getLatestAccumulated(vector<sensor_msgs::PointCloud> &data)
    {
      vector<sensor_msgs::PointCloud> pointclouds;
      for(size_t i=0; i<data_.size(); i++)
	pointclouds.push_back(data_[i].pc);
        data = pointclouds;

        //A simple flag to say we have new accumulated data
        bool new_data = new_data_;
        new_data_ = false;
        return new_data;
    }


};

namespace pcl_utils
{
//a copy from common/impl/transform.hpp
//complaint about void pcl::transformPointCloud(const pcl::PointCloud<PointT>&, pcl::PointCloud<PointT>&, const Affine3f&)’ should have been declared inside ‘pcl’
template <typename PointT> void
transformPointCloud (const pcl::PointCloud<PointT> &cloud_in,
                          pcl::PointCloud<PointT> &cloud_out,
                          const Eigen::Affine3f &transform)
{
  if (&cloud_in != &cloud_out)
  {
    // Note: could be replaced by cloud_out = cloud_in
    cloud_out.header   = cloud_in.header;
    cloud_out.is_dense = cloud_in.is_dense;
    cloud_out.width    = cloud_in.width;
    cloud_out.height   = cloud_in.height;
    cloud_out.points.reserve (cloud_out.points.size ());
    cloud_out.points.assign (cloud_in.points.begin (), cloud_in.points.end ());
  }

  if (cloud_in.is_dense)
  {
    // If the dataset is dense, simply transform it!
    for (size_t i = 0; i < cloud_out.points.size (); ++i)
      cloud_out.points[i].getVector3fMap () = transform *
                                              cloud_in.points[i].getVector3fMap ();
  }
  else
  {
    // Dataset might contain NaNs and Infs, so check for them first,
    // otherwise we get errors during the multiplication (?)
    for (size_t i = 0; i < cloud_out.points.size (); ++i)
    {
      if (!pcl_isfinite (cloud_in.points[i].x) ||
          !pcl_isfinite (cloud_in.points[i].y) ||
          !pcl_isfinite (cloud_in.points[i].z))
        continue;
      cloud_out.points[i].getVector3fMap () = transform *
                                              cloud_in.points[i].getVector3fMap ();
    }
  }
}

    template <typename PointT> inline void
    transformPointCloud (const pcl::PointCloud<PointT> &cloud_in,
                              pcl::PointCloud<PointT> &cloud_out,
                              const Eigen::Vector3f &offset,
                              const Eigen::Quaternionf &rotation)
    {
      Eigen::Translation3f translation (offset);
      // Assemble an Eigen Transform
      Eigen::Affine3f t;
      t = translation * rotation;
      transformPointCloud (cloud_in, cloud_out, t);
    }

template <typename PointT> void
transformPointCloudWithNormals (const pcl::PointCloud<PointT> &cloud_in, 
                                     pcl::PointCloud<PointT> &cloud_out,
                                     const Eigen::Affine3f &transform)
{
  if (&cloud_in != &cloud_out)
  {
    // Note: could be replaced by cloud_out = cloud_in
    cloud_out.header   = cloud_in.header;
    cloud_out.width    = cloud_in.width;
    cloud_out.height   = cloud_in.height;
    cloud_out.is_dense = cloud_in.is_dense;
    cloud_out.points.reserve (cloud_out.points.size ());
    cloud_out.points.assign (cloud_in.points.begin (), cloud_in.points.end ());
  }

  // If the data is dense, we don't need to check for NaN
  if (cloud_in.is_dense)
  {
    for (size_t i = 0; i < cloud_out.points.size (); ++i)
    {
      cloud_out.points[i].getVector3fMap() = transform * 
                                             cloud_in.points[i].getVector3fMap ();

      // Rotate normals
      cloud_out.points[i].getNormalVector3fMap() = transform.rotation () * 
                                                   cloud_in.points[i].getNormalVector3fMap ();
    }
  }
  // Dataset might contain NaNs and Infs, so check for them first.
  else
  {
    for (size_t i = 0; i < cloud_out.points.size (); ++i)
    {
      if (!pcl_isfinite (cloud_in.points[i].x) || 
          !pcl_isfinite (cloud_in.points[i].y) || 
          !pcl_isfinite (cloud_in.points[i].z))
        continue;
      cloud_out.points[i].getVector3fMap() = transform * 
                                             cloud_in.points[i].getVector3fMap ();

      // Rotate normals
      cloud_out.points[i].getNormalVector3fMap() = transform.rotation () * 
                                                   cloud_in.points[i].getNormalVector3fMap ();
    }
  }
}

template <typename PointT> inline void
transformPointCloudWithNormals (const pcl::PointCloud<PointT> &cloud_in, 
                                     pcl::PointCloud<PointT> &cloud_out,
                                     const Eigen::Vector3f &offset, 
                                     const Eigen::Quaternionf &rotation)
{
  Eigen::Translation3f translation (offset);
  // Assemble an Eigen Transform
  Eigen::Affine3f t;
  t = translation * rotation;
  transformPointCloudWithNormals (cloud_in, cloud_out, t);
}

}
#endif /* ACCUMULATEDATA_CPP_ */
