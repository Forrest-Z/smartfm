/*
 * laser_dynamic_object_filter.cpp
 *
 *  Created on: Apr 25, 2012
 *      Author: golfcar
 *
 *      laser_evidence.cpp will move towards a more modular model
 *      by splitting rolling windows and other perception processes
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
#include <pcl/features/normal_3d_omp.h>
#include <pcl/filters/conditional_removal.h>

#include <laser_geometry/laser_geometry.h>

#include <fmutil/fm_math.h>

#include <visualization_msgs/MarkerArray.h>

#include <sys/time.h>
#include <ctime>

using namespace std;


uint64_t GetTimeMs64()
{
	struct timeval tv;

	 gettimeofday(&tv, NULL);

	 uint64_t ret = tv.tv_usec;
	 /* Convert from micro seconds (10^-6) to milliseconds (10^-3) */
	 ret /= 1000;

	 /* Adds the seconds (10^0) after converting them to milliseconds (10^-3) */
	 ret += (tv.tv_sec * 1000);

	 return ret;
}

class laser_evidence
{
public:
    laser_evidence();

private:
    ros::NodeHandle n_;

    message_filters::Subscriber<sensor_msgs::LaserScan> laser_scan_sub_;
    message_filters::Subscriber<sensor_msgs::PointCloud2> cloud_scan_sub_;
    ros::Publisher accumulated_pc2_pub_, filtered_pc2_pub_, filtered_laser_pub_,
    filtered_pc_pub_, normals_marker_array_publisher_, normals_poses_pub_;

    string target_frame_;

    tf::TransformListener tf_;
    tf::MessageFilter<sensor_msgs::LaserScan> *laser_scan_filter_;
    tf::MessageFilter<sensor_msgs::PointCloud2> *cloud_scan_filter_;
    laser_geometry::LaserProjection projector_;

    typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

    void scanCallback(const sensor_msgs::LaserScanConstPtr scan_in);
    void cloudCallback(const sensor_msgs::PointCloud2ConstPtr cloud_in);
    void pointcloudsToLaser(sensor_msgs::PointCloud& cloud, sensor_msgs::LaserScan& output);
    void convertToLaser(sensor_msgs::PointCloud2 pointcloud2_in, sensor_msgs::LaserScan& laser_out);
    void normalEstimation(PointCloud input);
    void publishNormal(pcl::PointCloud<pcl::PointNormal>& pcl_cloud);
    void mainLoop(geometry_msgs::PointStamped& laser_pose, sensor_msgs::PointCloud& laser_cloud);

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
    sample_size_ = 75;
    //filter_prob_ = 0.8;
    search_radius_ = 0.3;
    laser_scan_sub_.subscribe(n_, "scan_in", 10);
    laser_scan_filter_ = new tf::MessageFilter<sensor_msgs::LaserScan>(laser_scan_sub_, tf_, target_frame_, 10);
    laser_scan_filter_->registerCallback(boost::bind(&laser_evidence::scanCallback, this, _1));

    cloud_scan_sub_.subscribe(n_, "cloud_in", 10);
    cloud_scan_filter_ = new tf::MessageFilter<sensor_msgs::PointCloud2>(cloud_scan_sub_, tf_, target_frame_, 10);
    cloud_scan_filter_->registerCallback(boost::bind(&laser_evidence::cloudCallback, this, _1));

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
    output.angle_min = -M_PI;//*0.7;
    output.angle_max = M_PI;//*0.7;
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


        //added to output the maximum range available from the point cloud
        if (output.ranges[index] == output.range_max + 1.0) output.ranges[index] = sqrt(range_sq);
        if (output.ranges[index] * output.ranges[index] < range_sq)
            output.ranges[index] = sqrt(range_sq);
    }
}

void laser_evidence::cloudCallback(const sensor_msgs::PointCloud2ConstPtr cloud_in)
{
    sensor_msgs::PointCloud laser_cloud;
    geometry_msgs::PointStamped laser_pose;
    PointCloud laser_pcl;
    pcl::fromROSMsg(*cloud_in, laser_pcl);
    sensor_msgs::PointCloud2 laser_cloud2;
    pcl::toROSMsg(laser_pcl, laser_cloud2);

    sensor_msgs::convertPointCloud2ToPointCloud(laser_cloud2, laser_cloud);
    laser_pose.header = cloud_in->header;

    try{
        tf_.transformPointCloud(target_frame_, laser_cloud, laser_cloud);
        tf_.transformPoint("odom", laser_pose, laser_pose);
    }
    catch(tf::TransformException& e){
        ROS_DEBUG("%s",e.what());
        return;
    }
    mainLoop(laser_pose, laser_cloud);
//    sample_data_.insert(sample_data_.begin(), laser_cloud);
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

    mainLoop(laser_pose, laser_cloud);


}

void laser_evidence::mainLoop(geometry_msgs::PointStamped& laser_pose, sensor_msgs::PointCloud& laser_cloud)
{
	double laser_pose_dist = fmutil::distance(laser_pose_pre_.point, laser_pose.point);
	static int laserNo_received = 0, laserNo_filtered = 0;
	if(laser_pose_dist<0.03) laserNo_filtered++;
	laserNo_received++;
	cout<<laser_pose_dist<<" "<<(laser_pose_dist<0.03)<<" "<<laserNo_filtered<<"/"<<laserNo_received<<": "<<(double)laserNo_filtered/laserNo_received<<endl;
    if(fmutil::distance(laser_pose_pre_.point, laser_pose.point)<0.03) return;
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


    }
    sample_data_.insert(sample_data_.begin(), laser_cloud);
}

void laser_evidence::normalEstimation(PointCloud input)
{
      // Create the normal estimation class, and pass the input dataset to it
      pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> ne;
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
      uint64_t stopwatch = GetTimeMs64();
      ne.compute (cloud_normals);
      cout<<"Compute normals "<< (GetTimeMs64()-stopwatch)<<endl;
      // concatentate the fileds
      pcl::PointCloud<pcl::PointNormal> point_normals;
      stopwatch = GetTimeMs64();
      pcl::concatenateFields(input, cloud_normals, point_normals);
      cout<<"Concatenate fileds "<< (GetTimeMs64()-stopwatch)<<endl;
      //cout<<"Point normal "<<point_normals.points[0].normal_x<<' '<<point_normals.points[0].normal_y<<' '<<point_normals.points[0].normal_z<<endl;
      // publish normal using visualization marker
      stopwatch = GetTimeMs64();
      publishNormal(point_normals);
      cout<<"Normals calculation "<< (GetTimeMs64()-stopwatch)<<endl;

}

void laser_evidence::publishNormal(pcl::PointCloud<pcl::PointNormal>& pcl_cloud)
{

    visualization_msgs::MarkerArray normals_marker_array_msg;
    //just checking...
    //cout<<"Point normal "<<pcl_cloud.points[0].normal_x<<' '<<pcl_cloud.points[0].normal_y<<' '<<pcl_cloud.points[0].normal_z<<endl;
      unsigned int count_filtered=0, count_correct=0, count_raw=0;
      uint64_t stopwatch = GetTimeMs64();
      //absolutely stunningly quick!
      pcl::ConditionAnd<pcl::PointNormal>::Ptr range_cond (new pcl::ConditionAnd<pcl::PointNormal> ());
      range_cond->addComparison (pcl::FieldComparison<pcl::PointNormal>::ConstPtr (new
            pcl::FieldComparison<pcl::PointNormal> ("normal_z", pcl::ComparisonOps::LT, 0.90)));
      pcl::ConditionalRemoval<pcl::PointNormal> condrem (range_cond);
      condrem.setInputCloud (pcl_cloud.makeShared());
      condrem.filter (pcl_cloud);
      cout<<"Filter clouds "<< (GetTimeMs64()-stopwatch)<<endl;
      //ROS_INFO("raw size: %i, passed size: %i, filtered size: %i", count_raw, count_correct, count_filtered);
      //sensor_msgs::PointCloud2 filtered_pc2;
      //pcl::toROSMsg(pcl_cloud, filtered_pc2);
      //filtered_pc2_pub_.publish(filtered_pc2);
      //if (size >= lastsize) {
      //    normals_marker_array_msg.markers.resize(size);
      //}



       //perform density based filtering
      /*stopwatch = GetTimeMs64();
      pcl::PointCloud<pcl::PointNormal> radius_filtered_pcl2;
      pcl::RadiusOutlierRemoval<pcl::PointNormal> outrem;
      outrem.setInputCloud(pcl_cloud.makeShared());
      outrem.setRadiusSearch(0.4);
      outrem.setMinNeighborsInRadius(8);
      outrem.filter(radius_filtered_pcl2);
      cout<<"density filtering "<< (GetTimeMs64()-stopwatch)<<endl;*/
      sensor_msgs::PointCloud2 radius_filtered_pc2;
      pcl::toROSMsg(pcl_cloud, radius_filtered_pc2);
      sensor_msgs::LaserScan filtered_laser;
      stopwatch = GetTimeMs64();
      convertToLaser(radius_filtered_pc2, filtered_laser);
      cout<<"convert to laser "<< (GetTimeMs64()-stopwatch)<<endl;
      filtered_pc2_pub_.publish(radius_filtered_pc2);
      filtered_laser_pub_.publish(filtered_laser);

      // publish normal as posearray for visualization
      bool publish_normals = false;
      if(publish_normals)
      {
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
      }

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
