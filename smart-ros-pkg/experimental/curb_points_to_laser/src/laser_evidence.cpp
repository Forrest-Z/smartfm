/*
 * laser_dynamic_object_filter.cpp
 *
 *  Created on: Apr 25, 2012
 *      Author: golfcar
 */

#include <ros/ros.h>

#include <geometry_msgs/PoseArray.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>

#include <tf/transform_listener.h>
#include <tf/message_filter.h>

#include <message_filters/subscriber.h>

#include "pcl/point_cloud.h"
#include "pcl_ros/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl/ros/conversions.h"
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/features/normal_3d.h>
#include <pcl_visualization/cloud_viewer.h>
#include <laser_geometry/laser_geometry.h>

#include <fmutil/fm_math.h>

#include <visualization_msgs/MarkerArray.h>

using namespace std;

class laser_evidence
{
public:
    laser_evidence();

private:
    ros::NodeHandle n_;

    message_filters::Subscriber<sensor_msgs::LaserScan> laser_scan_sub_;
    ros::Publisher accumulated_pc2_pub_, filtered_pc2_pub_, filtered_laser_pub_,
    filtered_pc_pub_, normals_marker_array_publisher_, normals_poses_pub_;

    string target_frame_;

    tf::TransformListener tf_;
    tf::MessageFilter<sensor_msgs::LaserScan> *laser_scan_filter_;
    laser_geometry::LaserProjection projector_;

    typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

    void scanCallback(const sensor_msgs::LaserScanConstPtr scan_in);
    void pointcloudsToLaser(sensor_msgs::PointCloud& cloud, sensor_msgs::LaserScan& output);
    void convertToLaser(sensor_msgs::PointCloud2 pointcloud2_in, sensor_msgs::LaserScan& laser_out);
    void normalEstimation(PointCloud input);
    void publishNormal(pcl::PointCloud<pcl::PointNormal>& pcl_cloud);
    size_t sample_size_;
    double static_prob_;
    vector<sensor_msgs::PointCloud> sample_data_;
    float search_radius_;
    geometry_msgs::PointStamped laser_pose_pre_;
    visualization_msgs::MarkerArray normals_marker_array_msg_;
};

laser_evidence::laser_evidence() : tf_()
{
    target_frame_ = "/odom";
    sample_size_ = 200;
    //filter_prob_ = 0.8;
    search_radius_ = 0.3;
    laser_scan_sub_.subscribe(n_, "scan_in", 10);
    laser_scan_filter_ = new tf::MessageFilter<sensor_msgs::LaserScan>(laser_scan_sub_, tf_, target_frame_, 10);
    laser_scan_filter_->registerCallback(boost::bind(&laser_evidence::scanCallback, this, _1));

    accumulated_pc2_pub_ = n_.advertise<sensor_msgs::PointCloud2>("accumulated_pts", 10);
    filtered_pc2_pub_ = n_.advertise<sensor_msgs::PointCloud2>("filtered_pts2", 10);
    filtered_laser_pub_ = n_.advertise<sensor_msgs::LaserScan>("filtered_laser", 10);
    filtered_pc_pub_ = n_.advertise<sensor_msgs::PointCloud>("filtered_pts", 10);
    normals_marker_array_publisher_ = n_.advertise<visualization_msgs::MarkerArray>("normals_marker_array", 100);
    normals_poses_pub_ = n_.advertise<geometry_msgs::PoseArray>("normals_array", 100);
    ros::spin();
}


void laser_evidence::pointcloudsToLaser(sensor_msgs::PointCloud& cloud, sensor_msgs::LaserScan& output)
{
    //adapted from turtlebot's cloud_to_scan.cpp
    filtered_pc_pub_.publish(cloud);
    output.header = cloud.header;
    output.angle_min = -M_PI*0.7;
    output.angle_max = M_PI*0.7;
    output.angle_increment = M_PI/180.0/2.0;
    output.time_increment = 0.0;
    output.scan_time = 1.0/30.0;
    output.range_min = 0.1;
    output.range_max = 80.0;

    uint32_t ranges_size = std::ceil((output.angle_max - output.angle_min) / output.angle_increment);
    output.ranges.assign(ranges_size, output.range_max + 1.0);

    for (size_t it = 0; it < cloud.points.size(); ++it)
    {
        const float &x = cloud.points[it].x;
        const float &y = cloud.points[it].y;
        const float &z = cloud.points[it].z;

        if ( std::isnan(x) || std::isnan(y) || std::isnan(z) )
        {
            ROS_DEBUG("rejected for nan in point(%f, %f, %f)\n", x, y, z);
            continue;
        }

        double range_sq = y*y+x*x;
        /*if (range_sq < 0.) {
            ROS_DEBUG("rejected for range %f below minimum value %f. Point: (%f, %f, %f)", range_sq, range_min_sq_, x, y, z);
            continue;
        }*/

        double angle = -atan2(-y, x);
        if (angle < output.angle_min || angle > output.angle_max)
        {
            ROS_DEBUG("rejected for angle %f not in range (%f, %f)\n", angle, output.angle_min, output.angle_max);
            continue;
        }
        int index = (angle - output.angle_min) / output.angle_increment;


        if (output.ranges[index] * output.ranges[index] > range_sq)
            output.ranges[index] = sqrt(range_sq);
    }
}

void laser_evidence::scanCallback(const sensor_msgs::LaserScanConstPtr scan_in)
{
    sensor_msgs::LaserScan scan_copy = *scan_in;
    sensor_msgs::PointCloud laser_cloud;
    geometry_msgs::PointStamped laser_pose;
    laser_pose.header = scan_in->header;
    try{
        projector_.transformLaserScanToPointCloud(target_frame_, scan_copy,
                                                  laser_cloud, tf_);

        //compress all the data points into single plane
        //for(size_t i=0; i<laser_cloud.points.size();i++)
            //laser_cloud.points[i].z = 0.0;

        tf_.transformPoint("odom", laser_pose, laser_pose);
    }
    catch (tf::TransformException& e){
        ROS_DEBUG("%s",e.what());
        return;
    }
    if(fmutil::distance(laser_pose_pre_.point, laser_pose.point)<0.01) return;
    else laser_pose_pre_ = laser_pose;
    //only start filtering when there is enough samples
    if(sample_data_.size()>sample_size_)
    {
        //sample_data_ always lag one step behind the current laser data
        sample_data_.resize(sample_size_);
        sensor_msgs::PointCloud accumulated_pts;
        accumulated_pts.header = sample_data_[0].header;
        for(size_t i=0; i<sample_data_.size(); i++)
        {
            accumulated_pts.points.insert(accumulated_pts.points.begin(), sample_data_[i].points.begin(), sample_data_[i].points.end());
        }
        sensor_msgs::PointCloud2 accumulated_pts2;
        sensor_msgs::convertPointCloudToPointCloud2(accumulated_pts, accumulated_pts2);
        accumulated_pc2_pub_.publish(accumulated_pts2);
        PointCloud accumulated_pcl, accumulated_filtered;
        pcl::fromROSMsg(accumulated_pts2, accumulated_pcl);

        //estimate normal
        normalEstimation(accumulated_pcl);
       /* //perform density based filtering
        pcl::RadiusOutlierRemoval<pcl::PointXYZ> outrem;
        outrem.setInputCloud(accumulated_pcl.makeShared());
        outrem.setRadiusSearch(0.2);
        outrem.setMinNeighborsInRadius(10);
        outrem.filter(accumulated_filtered);
        sensor_msgs::PointCloud2 accumulated_filtered_pc2;
        pcl::toROSMsg(accumulated_filtered, accumulated_filtered_pc2);
        sensor_msgs::LaserScan filtered_laser;
        convertToLaser(accumulated_filtered_pc2, filtered_laser);
        //filtered_pc2_pub_.publish(accumulated_filtered_pc2);
        filtered_laser_pub_.publish(filtered_laser);*/
    }

    sample_data_.insert(sample_data_.begin(), laser_cloud);

}

void laser_evidence::normalEstimation(PointCloud input)
{
      // Create the normal estimation class, and pass the input dataset to it
      pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
      ne.setInputCloud (input.makeShared());

      // Create an empty kdtree representation, and pass it to the normal estimation object.
      // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
      pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
      ne.setSearchMethod (tree);

      // Output datasets
      pcl::PointCloud<pcl::Normal> cloud_normals;

      // Use all neighbors in a sphere of radius 10cm
      ne.setRadiusSearch (0.3);
      ne.setViewPoint(laser_pose_pre_.point.x,laser_pose_pre_.point.y,laser_pose_pre_.point.z);
      // Compute the features
      ne.compute (cloud_normals);

      // concatentate the fileds
      pcl::PointCloud<pcl::PointNormal> point_normals;
      pcl::concatenateFields(input, cloud_normals, point_normals);
      cout<<"Point normal "<<point_normals.points[0].normal_x<<' '<<point_normals.points[0].normal_y<<' '<<point_normals.points[0].normal_z<<endl;
      // publish normal using visualization marker
      publishNormal(point_normals);

}

void laser_evidence::publishNormal(pcl::PointCloud<pcl::PointNormal>& pcl_cloud)
{

    visualization_msgs::MarkerArray normals_marker_array_msg;
    //just checking...
    cout<<"Point normal "<<pcl_cloud.points[0].normal_x<<' '<<pcl_cloud.points[0].normal_y<<' '<<pcl_cloud.points[0].normal_z<<endl;
      unsigned int count_filtered=0, count_correct=0, count_raw=0;
      for(size_t i=0; i<pcl_cloud.points.size();)
      {

          if(pcl_cloud.points[i].normal_z>0.85)//, , fmutil::d2r(-355)))
          {
              pcl_cloud.points.erase(pcl_cloud.points.begin()+i);
              if(pcl_cloud.width>1) pcl_cloud.width --;
              else if(pcl_cloud.height>1)pcl_cloud.height--;
              else printf("more filtered points than the allowable size, this is strange!\n");
              count_filtered++;
          }
          else
          {
              //it's very slow here. Need to speed up
              //cout<<"Point normal "<<pcl_cloud.points[i].normal_x<<' '<<pcl_cloud.points[i].normal_y<<' '<<pcl_cloud.points[i].normal_z<<endl;
              count_correct++;
              i++;
          }
          count_raw++;
      }
      ROS_INFO("raw size: %i, passed size: %i, filtered size: %i", count_raw, count_correct, count_filtered);
      sensor_msgs::PointCloud2 filtered_pc2;
      pcl::toROSMsg(pcl_cloud, filtered_pc2);
      filtered_pc2_pub_.publish(filtered_pc2);
      //if (size >= lastsize) {
      //    normals_marker_array_msg.markers.resize(size);
      //}
      normals_marker_array_msg_.markers.clear();
      geometry_msgs::PoseArray normals_poses;
      normals_poses.header = pcl_cloud.header;
      for(unsigned int i=0; i<pcl_cloud.points.size(); i++)
      {

          geometry_msgs::Pose normals_pose;
          geometry_msgs::Point pos;
          pos.x = pcl_cloud.points[i].x; pos.y = pcl_cloud.points[i].y; pos.z = pcl_cloud.points[i].z;

          normals_pose.position = pos;
          btVector3 axis(pcl_cloud.points[i].normal[0],pcl_cloud.points[i].normal[1],pcl_cloud.points[i].normal[2]);
          if(isnan(pcl_cloud.points[i].normal[0])||isnan(pcl_cloud.points[i].normal[1])||isnan(pcl_cloud.points[i].normal[2])) continue;
          //cout<<axis.x()<<" "<<axis.y()<<" "<<axis.z()<<" "<<axis.w()<<endl;
          btVector3 marker_axis(1, 0, 0);
          btQuaternion qt(marker_axis.cross(axis.normalize()), marker_axis.angle(axis.normalize()));
          double yaw, pitch, roll;
          btMatrix3x3(qt).getEulerYPR(yaw, pitch, roll);
          geometry_msgs::Quaternion quat_msg;
          tf::quaternionTFToMsg(qt, quat_msg);
          if(isnan(qt.x())||isnan(qt.y())||isnan(qt.z())||isnan(qt.w())) continue;
          normals_pose.orientation.x = qt.x();// = quat_msg;
          normals_pose.orientation.y = qt.y();
          normals_pose.orientation.z = qt.z();
          normals_pose.orientation.w = qt.w();

          normals_poses.poses.push_back(normals_pose);
      }
      normals_poses_pub_.publish(normals_poses);
     /* for (unsigned int i = 0; i < pcl_cloud.points.size(); i++)
        {

          marker_array_msg.pose.position = pos;
          //axis-angle rotation
          btVector3 axis(pcl_cloud.points[i].normal[0],pcl_cloud.points[i].normal[1],pcl_cloud.points[i].normal[2]);
          btVector3 marker_axis(1, 0, 0);
          btQuaternion qt(marker_axis.cross(axis.normalize()), marker_axis.angle(axis.normalize()));
          double yaw, pitch, roll;
          btMatrix3x3(qt).getEulerYPR(yaw, pitch, roll);
          geometry_msgs::Quaternion quat_msg;
          tf::quaternionTFToMsg(qt, quat_msg);
          marker_array_msg.pose.orientation = quat_msg;

          marker_array_msg.header.frame_id = pcl_cloud.header.frame_id;
          marker_array_msg.header.stamp = pcl_cloud.header.stamp;
          marker_array_msg.id = i;
          marker_array_msg.ns = "Normals";
          marker_array_msg.color.r = 1.0f;
          marker_array_msg.color.g = 0.0f;
          marker_array_msg.color.b = 0.0f;
          marker_array_msg.color.a = 0.5f;
          marker_array_msg.lifetime = ros::Duration::Duration();
          marker_array_msg.type = visualization_msgs::Marker::ARROW;
          marker_array_msg.scale.x = 0.2;
          marker_array_msg.scale.y = 0.2;
          marker_array_msg.scale.z = 0.2;

          marker_array_msg.action = visualization_msgs::Marker::ADD;
          normals_marker_array_msg_.markers.push_back(marker_array_msg);
        }



      normals_marker_array_publisher_.publish(normals_marker_array_msg_);*/
}
void laser_evidence::convertToLaser(sensor_msgs::PointCloud2 pointcloud2_in, sensor_msgs::LaserScan& laser_out)
{
    //todo: Why laser keep shifting in the location.
    sensor_msgs::PointCloud pointcloud_in;
    sensor_msgs::convertPointCloud2ToPointCloud(pointcloud2_in, pointcloud_in);
    try
    {

        tf_.transformPointCloud("base_link", pointcloud_in, pointcloud_in);
        sensor_msgs::convertPointCloudToPointCloud2(pointcloud_in, pointcloud2_in);
    }
    catch (tf::TransformException &ex)
    {
        printf ("Failure %s\n", ex.what()); //Print exception which was caught
        return;
    }
    //filtered_pc2_pub_.publish(pointcloud2_in);
    //PointCloud pcl;
    //pcl::fromROSMsg(pointcloud2_in, pcl);
    pointcloudsToLaser(pointcloud_in, laser_out);
}

int main(int argc, char** argcv)
{
    ros::init(argc, argcv, "laser_dynamic_object_filter");
    laser_evidence ldf;
    return 0;
}
