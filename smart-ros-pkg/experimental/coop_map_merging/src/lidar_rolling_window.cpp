#include <norm_virtual_sensor/AccumulateData.h>
#include <pcl_ros/transforms.h>
class LIDARRollingWindow{
  AccumulateData *laser_accumulate_;
  tf::TransformListener *tf_;
  message_filters::Subscriber<sensor_msgs::LaserScan> laser_scan_sub_;
  tf::MessageFilter<sensor_msgs::LaserScan>       *laser_scan_filter_;
  ros::Publisher accumulated_pts_pub_;
  string odom_frame_id_, baselink_frame_id_;
  void scanCallback(sensor_msgs::LaserScanConstPtr scan)
  {
    try{
      tf::StampedTransform cur_sensor_trans;
      tf_->waitForTransform(scan->header.frame_id, odom_frame_id_, ros::Time::now(), ros::Duration(1.0));
      tf_->lookupTransform(odom_frame_id_, scan->header.frame_id, scan->header.stamp, cur_sensor_trans);
	sensor_msgs::LaserScan ls = *scan;
	laser_accumulate_->addData(ls, *tf_);
	mainProcess(scan->header);
    }
    catch (tf::TransformException& e){ ROS_ERROR("%s",e.what());}
  }
  
  void mainProcess(std_msgs::Header header){
    vector<sensor_msgs::PointCloud> data;
    sensor_msgs::PointCloud all_data;
    all_data.header = header;
    all_data.header.frame_id = odom_frame_id_;
    if(laser_accumulate_->getLatestAccumulated(data)){
      for(size_t i=0; i<data.size(); i++)
	all_data.points.insert(all_data.points.begin(), data[i].points.begin(),
	  data[i].points.end()
	);
      tf::StampedTransform baselink_transform;
      tf_->lookupTransform(baselink_frame_id_, odom_frame_id_, header.stamp, baselink_transform);
      Eigen::Quaternionf bl_rotation(baselink_transform.getRotation().w(), baselink_transform.getRotation().x(),baselink_transform.getRotation().y(),baselink_transform.getRotation().z()); 
      Eigen::Vector3f bl_trans(baselink_transform.getOrigin().x(),baselink_transform.getOrigin().y(),baselink_transform.getOrigin().z());
      for(size_t i=0; i<all_data.points.size(); i++)
              all_data.points[i].z = 0.0;
      sensor_msgs::PointCloud2 temp;
      sensor_msgs::convertPointCloudToPointCloud2(all_data, temp);
      pcl::PointCloud<pcl::PointXYZ> pcl_temp;
      pcl::fromROSMsg(temp,  pcl_temp);
      pcl_downsample(pcl_temp, 0.05, 0.05, 0.05);
      pcl::transformPointCloud<pcl::PointXYZ>(pcl_temp, pcl_temp, bl_trans, bl_rotation);
      pcl::toROSMsg(pcl_temp, temp);
      temp.header.frame_id = baselink_frame_id_;
      accumulated_pts_pub_.publish(temp);
    }
  }
  
template<class T>
  void pcl_downsample(pcl::PointCloud<T> &point_cloud, double size_x, double size_y, double size_z)
  {
    if(point_cloud.size()<100) return;
      pcl::VoxelGrid<T> sor;
      // always good not to use in place filtering as stated in
      // http://www.pcl-users.org/strange-effect-of-the-downsampling-td3857829.html
      pcl::PointCloud<T> input_msg_filtered = *(new pcl::PointCloud<T> ());
      //float downsample_size_ = 0.05;
      sor.setInputCloud(point_cloud.makeShared());
      sor.setLeafSize (size_x, size_y, size_z);
      pcl::PointIndicesPtr pi;
      sor.filter (input_msg_filtered);
      point_cloud = input_msg_filtered;
  }
public:
  LIDARRollingWindow(){
    ros::NodeHandle n;
    ros::NodeHandle priv_nh("~");
    tf_ = new tf::TransformListener();
    priv_nh.param("odom_frame_id", odom_frame_id_, string("odom"));
    priv_nh.param("baselink_frame_id", baselink_frame_id_, string("base_link"));
    double min_move_dist, max_dist;
    priv_nh.param("max_dist", max_dist, 2.0);
    priv_nh.param("min_move_dist", min_move_dist, 0.02);
    
    laser_accumulate_ = new AccumulateData(odom_frame_id_, min_move_dist, max_dist/min_move_dist);
    
    laser_scan_sub_.subscribe(n, "scan_in", 1);
    laser_scan_filter_ = new tf::MessageFilter<sensor_msgs::LaserScan>(laser_scan_sub_, *tf_, baselink_frame_id_, 100);
    laser_scan_filter_->registerCallback(boost::bind(&LIDARRollingWindow::scanCallback, this, _1));
    
    accumulated_pts_pub_ = n.advertise<sensor_msgs::PointCloud2>("accumulated_pts", 10);
    
    ros::spin();
  }

};

int main(int argc, char** argv){
  ros::init(argc, argv, "lidar_rolling_window");
  LIDARRollingWindow lidarRollingWindow;
  return 0;
}