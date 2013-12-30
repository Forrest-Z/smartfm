#include <norm_virtual_sensor/AccumulateData.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <pcl_ros/transforms.h>
class MapMergingOptimize{
  laser_geometry::LaserProjection projector_;
  ros::Publisher transformed_template_pub_, transformed_matching_pub_;
  tf::TransformListener *tf_;
public:
  MapMergingOptimize(){
    ros::NodeHandle nh;
    tf_ = new tf::TransformListener();
    message_filters::Subscriber<geometry_msgs::PoseStamped>
      pose_sub(nh, "vehicle_pose", 1);
    message_filters::Subscriber<sensor_msgs::PointCloud2>
      observation_sub(nh, "accumulated_pts", 1);
    message_filters::Subscriber<sensor_msgs::LaserScan>
      scan_sub(nh, "scan_in", 1);
    typedef message_filters::sync_policies::ApproximateTime
    <geometry_msgs::PoseStamped, 
    sensor_msgs::PointCloud2,
    sensor_msgs::LaserScan> MySyncPolicy;
    transformed_template_pub_ = nh.advertise<sensor_msgs::PointCloud2>
				  ("accumulated_pts_tfed", 1);
    transformed_matching_pub_ = nh.advertise<sensor_msgs::PointCloud2>
				  ("matching_pts_tfed", 1);
    message_filters::Synchronizer<MySyncPolicy>
      sync(MySyncPolicy(10), pose_sub, observation_sub, scan_sub);
    sync.registerCallback(boost::bind(&MapMergingOptimize::callback,
				      this, _1, _2, _3));
    ros::spin();
  }
  
  void callback(const geometry_msgs::PoseStampedConstPtr& pose,
		const sensor_msgs::PointCloud2ConstPtr& observation_pc2,
		const sensor_msgs::LaserScanConstPtr& scan){
    sensor_msgs::PointCloud cloud;
    projector_.projectLaser(*scan, cloud);
    sensor_msgs::PointCloud2 pc2;
    sensor_msgs::convertPointCloudToPointCloud2(cloud, pc2);
    pcl::PointCloud<pcl::PointXYZ> pcl_match, pcl_template;
    pcl::fromROSMsg(pc2, pcl_match);
    pcl::fromROSMsg(*observation_pc2, pcl_template);
    cout<<pcl_match.size()<<" to match with "<<pcl_template.size()<<endl;
    geometry_msgs::Quaternion orientation = pose->pose.orientation;
    geometry_msgs::Point position = pose->pose.position;
    Eigen::Quaternionf bl_rotation(orientation.w, orientation.x,
				   orientation.y,orientation.z); 
    Eigen::Vector3f bl_trans(position.x,position.y,position.z);
    pcl::transformPointCloud<pcl::PointXYZ>(pcl_template, pcl_template, bl_trans, bl_rotation);
    pcl::toROSMsg(pcl_template, pc2);
    pc2.header = pose->header;
    transformed_template_pub_.publish(pc2);
    
    tf::StampedTransform cur_sensor_trans;
    tf_->waitForTransform(scan->header.frame_id, pose->header.frame_id, ros::Time::now(), ros::Duration(1.0));
    tf_->lookupTransform(pose->header.frame_id, scan->header.frame_id, scan->header.stamp, cur_sensor_trans);
    Eigen::Quaternionf bl_rotation_sensor(cur_sensor_trans.getRotation().w(), cur_sensor_trans.getRotation().x(),cur_sensor_trans.getRotation().y(),cur_sensor_trans.getRotation().z()); 
    Eigen::Vector3f bl_trans_sensor(cur_sensor_trans.getOrigin().x(),cur_sensor_trans.getOrigin().y(),cur_sensor_trans.getOrigin().z());
    pcl::transformPointCloud<pcl::PointXYZ>(pcl_match, pcl_match, bl_trans_sensor, bl_rotation_sensor);
    pcl::toROSMsg(pcl_match, pc2);
    pc2.header = pose->header;
    transformed_matching_pub_.publish(pc2);
  }
};

int main(int argc, char** argv){
  ros::init(argc, argv, "mapOptimize");
  MapMergingOptimize mmo;
  return 0;
}
