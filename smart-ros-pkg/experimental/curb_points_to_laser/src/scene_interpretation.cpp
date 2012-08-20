/*
 * scnene_interpretation.cpp
 *
 *  Created on: July 2, 2012
 *      Author: Demian
 */

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
#include <pcl/filters/voxel_grid.h>

#include <laser_geometry/laser_geometry.h>

#include <fmutil/fm_math.h>
#include <fmutil/fm_stopwatch.h>
#include <visualization_msgs/MarkerArray.h>


#include <dynamic_reconfigure/server.h>
#include <curb_points_to_laser/SceneInterpretationConfig.h>

#include <sys/time.h>
#include <ctime>

using namespace std;



class SceneInterpretation
{
public:
	SceneInterpretation();

private:
	ros::NodeHandle n_;
	ros::Timer timer_;
	ros::Subscriber raw_pc_sub_, curb_pc_sub_;
	ros::Publisher interpreted_pub_, downsampled_pub_, bef_downsampled_pub_, laser_pub_, normals_poses_pub_;
	ros::Publisher interpreted_pc_pub_;
	tf::TransformListener tf_;
	void rawPointCloudCallback(sensor_msgs::PointCloud2ConstPtr raw_pointcloud);
	void curbPointCallback(sensor_msgs::PointCloud2ConstPtr curb_pointcloud);

	//dynamic parameters
	bool min_pc2laser_, publish_normals_;
	double downsample_size_, normal_radius_search_, normal_thres_, density_radius_search_;
	int density_min_neighbors_;
	sensor_msgs::PointCloud2 curb_pc_;

	typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
	void pointcloudsToLaser(sensor_msgs::PointCloud& cloud, sensor_msgs::LaserScan& output);
	void normalEstimation(PointCloud& input);

	void publishNormal(pcl::PointCloud<pcl::PointNormal>& pcl_cloud);
	void convertToLaser(sensor_msgs::PointCloud2 pointcloud2_in, sensor_msgs::LaserScan& laser_out);

	dynamic_reconfigure::Server<curb_points_to_laser::SceneInterpretationConfig> dynamic_server_;
	dynamic_reconfigure::Server<curb_points_to_laser::SceneInterpretationConfig>::CallbackType dynamic_server_cb_;
	void dynamicCallback(curb_points_to_laser::SceneInterpretationConfig &config, uint32_t level);
};

void SceneInterpretation::dynamicCallback(curb_points_to_laser::SceneInterpretationConfig &config, uint32_t level) {
	min_pc2laser_ = config.min_pc2laser;
	publish_normals_ = config.publish_normals;
	downsample_size_ = config.downsample_size;
	normal_radius_search_ = config.normal_radius_search;
	normal_thres_ = config.normal_thres;
	density_radius_search_ = config.density_radius_search;
	density_min_neighbors_ = config.density_min_neighbors;
}

SceneInterpretation::SceneInterpretation()
{
	raw_pc_sub_ = n_.subscribe("pc_in", 1, &SceneInterpretation::rawPointCloudCallback, this);
	curb_pc_sub_ = n_.subscribe("rolling_window_curb", 10, &SceneInterpretation::curbPointCallback, this);
	interpreted_pub_ = n_.advertise<sensor_msgs::PointCloud2>("pc_out", 10);
	interpreted_pc_pub_ = n_.advertise<sensor_msgs::PointCloud>("pc_legacy_out", 10);
	downsampled_pub_ = n_.advertise<sensor_msgs::PointCloud2>("downsampled", 10);
	bef_downsampled_pub_ = n_.advertise<sensor_msgs::PointCloud2>("bef_downsampled", 10);
	laser_pub_ = n_.advertise<sensor_msgs::LaserScan>("laser_out", 10);
	normals_poses_pub_ = n_.advertise<geometry_msgs::PoseArray>("normals_array", 100);

	dynamic_server_cb_ = boost::bind(&SceneInterpretation::dynamicCallback, this, _1, _2);
	dynamic_server_.setCallback(dynamic_server_cb_);
	ros::spin();
}

void SceneInterpretation::curbPointCallback(sensor_msgs::PointCloud2ConstPtr curb_pointcloud)
{
	this->curb_pc_ = *curb_pointcloud;
}

void SceneInterpretation::rawPointCloudCallback(sensor_msgs::PointCloud2ConstPtr raw_pointcloud)
{
	cout<<"Raw pointcloud received"<<endl;
	PointCloud pc;
	pcl::fromROSMsg(*raw_pointcloud, pc);
	normalEstimation(pc);
}

void SceneInterpretation::pointcloudsToLaser(sensor_msgs::PointCloud& cloud, sensor_msgs::LaserScan& output)
{
    //adapted from turtlebot's cloud_to_scan.cpp
    output.header = cloud.header;
    output.angle_min = -M_PI;//*0.7;
    output.angle_max = M_PI;//*0.7;
    output.angle_increment = M_PI/180.0/5.0;
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


        //added to output the max range available from the point cloud
        //todo: more advanced processing of the points should use, right now just the min/max is sufficient

        if (output.ranges[index] == output.range_max + 1.0) output.ranges[index] = sqrt(range_sq);
        bool update_range;
        if(min_pc2laser_) update_range = output.ranges[index] * output.ranges[index] > range_sq;
        else update_range = output.ranges[index] * output.ranges[index] < range_sq;
        if (update_range)
            output.ranges[index] = sqrt(range_sq);
    }
}


void SceneInterpretation::normalEstimation(PointCloud& input)
{
	if(input.points.size() == 0) return;
	//down sampling moved to rolling windows
	fmutil::Stopwatch sw;

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
	ne.setRadiusSearch (normal_radius_search_);
	ne.setViewPoint(0, 0, 0);
	// Compute the features
	sw.start("Compute normals");
	ne.compute (cloud_normals);
	sw.end();
	// concatentate the fileds
	pcl::PointCloud<pcl::PointNormal> point_normals;
	sw.start("Concatenate fields");
	pcl::concatenateFields(input, cloud_normals, point_normals);
	sw.end();
	//cout<<"Point normal "<<point_normals.points[0].normal_x<<' '<<point_normals.points[0].normal_y<<' '<<point_normals.points[0].normal_z<<endl;
	// publish normal using visualization marker

	sw.start("Publish normal process:");
	publishNormal(point_normals);
	sw.end();
	cout<<"----------"<<endl;
	cout<<"Total: "<<sw.total_<<endl;
	cout<<"----------"<<endl;
	cout<<endl;
}

void SceneInterpretation::publishNormal(pcl::PointCloud<pcl::PointNormal>& pcl_cloud)
{
	fmutil::Stopwatch sw;
	visualization_msgs::MarkerArray normals_marker_array_msg;
	sw.start("Filter clouds");
	unsigned int count_filtered=0, count_correct=0, count_raw=0;

	//absolutely stunningly quick!
	pcl::ConditionAnd<pcl::PointNormal>::Ptr range_cond (new pcl::ConditionAnd<pcl::PointNormal> ());
	range_cond->addComparison (pcl::FieldComparison<pcl::PointNormal>::ConstPtr (new
			pcl::FieldComparison<pcl::PointNormal> ("normal_z", pcl::ComparisonOps::LT, normal_thres_)));
	//range_cond->addComparison (pcl::FieldComparison<pcl::PointNormal>::ConstPtr (new
	//		pcl::FieldComparison<pcl::PointNormal> ("z", pcl::ComparisonOps::GT, 0.0)));
	pcl::ConditionalRemoval<pcl::PointNormal> condrem (range_cond);
	condrem.setInputCloud (pcl_cloud.makeShared());
	condrem.filter (pcl_cloud);

	pcl::ConditionAnd<pcl::PointNormal>::Ptr range_cond2 (new pcl::ConditionAnd<pcl::PointNormal> ());
	range_cond2->addComparison (pcl::FieldComparison<pcl::PointNormal>::ConstPtr (new
			pcl::FieldComparison<pcl::PointNormal> ("normal_z", pcl::ComparisonOps::GT, -normal_thres_)));
	//range_cond2->addComparison (pcl::FieldComparison<pcl::PointNormal>::ConstPtr (new
	//		pcl::FieldComparison<pcl::PointNormal> ("z", pcl::ComparisonOps::GT, 0.0)));
	pcl::ConditionalRemoval<pcl::PointNormal> condrem2 (range_cond2);
	condrem2.setInputCloud (pcl_cloud.makeShared());
	condrem2.filter (pcl_cloud);
	sw.end();

	//perform density based filtering
	sw.start("Density filtering");
	pcl::PointCloud<pcl::PointNormal> radius_filtered_pcl2;// = pcl_cloud;
	pcl::RadiusOutlierRemoval<pcl::PointNormal> outrem;
	outrem.setInputCloud(pcl_cloud.makeShared());
	outrem.setRadiusSearch(density_radius_search_);
	outrem.setMinNeighborsInRadius(density_min_neighbors_);
	outrem.filter(radius_filtered_pcl2);
    sw.end();

    //insertion of curb points to make sure curb will correctly mark as occupied
    pcl::PointCloud<pcl::PointXYZ> curb_pcl, radius_filtered_pcl2_xyz;
    pcl::copyPointCloud(radius_filtered_pcl2, radius_filtered_pcl2_xyz);
    if(this->curb_pc_.data.size()>0)
    {
    	pcl::fromROSMsg(this->curb_pc_, curb_pcl);
    	radius_filtered_pcl2_xyz = radius_filtered_pcl2_xyz + curb_pcl;
    }

	sensor_msgs::PointCloud2 radius_filtered_pc2;
	pcl::toROSMsg(radius_filtered_pcl2_xyz, radius_filtered_pc2);


	sensor_msgs::LaserScan filtered_laser;
	sw.start("Convert to laser");
	convertToLaser(radius_filtered_pc2, filtered_laser);
	sw.end();
	interpreted_pub_.publish(radius_filtered_pc2);
	sensor_msgs::PointCloud filtered_pc;
	sensor_msgs::convertPointCloud2ToPointCloud(radius_filtered_pc2, filtered_pc);



	interpreted_pc_pub_.publish(filtered_pc);
	laser_pub_.publish(filtered_laser);

	// publish normal as posearray for visualization
	if(publish_normals_)
	{
		sw.start("Normal calculate");
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
		sw.end();
	}


}
void SceneInterpretation::convertToLaser(sensor_msgs::PointCloud2 pointcloud2_in, sensor_msgs::LaserScan& laser_out)
{
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
	ros::init(argc, argcv, "scene_interpretation");
	SceneInterpretation si;
	return 0;
}
