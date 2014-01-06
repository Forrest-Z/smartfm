#define  EIGEN_YES_I_KNOW_SPARSE_MODULE_IS_NOT_STABLE_YET 
#include <mrpt/slam.h>
#include <mrpt/gui.h>

#include <norm_virtual_sensor/AccumulateData.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <pcl_ros/transforms.h>
#include <tf/transform_broadcaster.h>
#include <mrpt/slam.h>
#include <mrpt/gui.h>

#include "csm_cuda/csm.h"

using namespace mrpt;
using namespace mrpt::utils;
using namespace mrpt::slam;

class MapMergingOptimize{
  laser_geometry::LaserProjection projector_;
  ros::Publisher transformed_template_pub_, transformed_matching_pub_;
  ros::Publisher matched_matching_ICP_pub_, matched_matching_CSM_pub_;
  ros::Publisher raw_matched_matching_ICP_pub_, raw_matched_matching_CSM_pub_;
  ros::Publisher csm_vehicle_pub_, icp_vehicle_pub_;
  tf::TransformListener *tf_;
  tf::TransformBroadcaster *tf_broadcaster_;
  string icp_base_frame_id_, csm_base_frame_id_, laser_base_frame_id_,
  icp_vehicle_frame_id_, csm_vehicle_frame_id_, laser_detect_vehicle_frame_id_;
public:
  MapMergingOptimize(){
    ros::NodeHandle nh;
    ros::NodeHandle priv_nh("~");
    priv_nh.param("icp_base_frame_id", icp_base_frame_id_, string("golfcart/base_link"));
    priv_nh.param("csm_base_frame_id", csm_base_frame_id_, string("golfcart/base_link"));
    priv_nh.param("laser_base_frame_id", laser_base_frame_id_, string("golfcart/base_link"));
    priv_nh.param("icp_vehicle_frame_id", icp_vehicle_frame_id_, string("icp_base_link"));
    priv_nh.param("csm_vehicle_frame_id", csm_vehicle_frame_id_, string("csm_base_link"));
    priv_nh.param("laser_detect_vehicle_frame_id", laser_detect_vehicle_frame_id_, string("laser_base_link"));
    tf_ = new tf::TransformListener();
    tf_broadcaster_ = new tf::TransformBroadcaster();
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
				  ("raw_matching_prior", 1);
    raw_matched_matching_ICP_pub_ = nh.advertise<sensor_msgs::PointCloud2>
				      ("raw_matched_matching_ICP", 1);
    raw_matched_matching_CSM_pub_ = nh.advertise<sensor_msgs::PointCloud2>
				      ("raw_matched_matching_CSM", 1);
    csm_vehicle_pub_ = nh.advertise<geometry_msgs::PoseStamped>("csm_vehicle",1);
    icp_vehicle_pub_ = nh.advertise<geometry_msgs::PoseStamped>("icp_vehicle",1);
    message_filters::Synchronizer<MySyncPolicy>
      sync(MySyncPolicy(10), pose_sub, observation_sub, scan_sub);
    sync.registerCallback(boost::bind(&MapMergingOptimize::callback,
				      this, _1, _2, _3));
    ros::spin();
  }
  template<typename T>
inline boost::tuples::tuple<T,T,T> matrixToYawPitchRoll(const Eigen::Matrix<T,3,3>& r)
{
  Eigen::Matrix<T,3,1> euler = r.eulerAngles(2, 1, 0);
  return boost::tuples::make_tuple(euler(0,0), euler(1,0), euler(2,0));
}
  poseResult eigenAffineTo2DPose(Eigen::Affine3f t, string msg){
    double yaw, pitch, roll;
    boost::tie(yaw,pitch,roll) = matrixToYawPitchRoll(t.rotation());
    poseResult offset;
    offset.x = t.translation()(0);
    offset.y = t.translation()(1);
    offset.r = yaw/M_PI*180;
    cout<<msg<<": "<<
    offset.x<<","<<offset.y<<","<<offset.r<<endl;
    return offset;
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
    pcl::toROSMsg(pcl_template, pc2);
    pc2.header = pose->header;
    transformed_template_pub_.publish(pc2);
    Eigen::Translation3f translation (bl_trans);
    Eigen::Affine3f t = translation * bl_rotation;
    publishVehicle(t, pose->header.stamp, pose->header.frame_id, laser_base_frame_id_,
		   laser_detect_vehicle_frame_id_);
    eigenAffineTo2DPose(t, string("given transform of leader vehicle based on ego vehicle"));
    tf::StampedTransform cur_sensor_trans;
    tf_->waitForTransform(scan->header.frame_id, pose->header.frame_id, ros::Time::now(), ros::Duration(1.0));
    tf_->lookupTransform(pose->header.frame_id, scan->header.frame_id, scan->header.stamp, cur_sensor_trans);
    Eigen::Quaternionf bl_rotation_sensor(cur_sensor_trans.getRotation().w(), cur_sensor_trans.getRotation().x(),cur_sensor_trans.getRotation().y(),cur_sensor_trans.getRotation().z()); 
    Eigen::Vector3f bl_trans_sensor(cur_sensor_trans.getOrigin().x(),cur_sensor_trans.getOrigin().y(),cur_sensor_trans.getOrigin().z());
    pcl::PointCloud<pcl::PointXYZ> pcl_match_prior;
    pcl::transformPointCloud<pcl::PointXYZ>(pcl_match, pcl_match, bl_trans_sensor, bl_rotation_sensor);
    pcl::transformPointCloud<pcl::PointXYZ>(pcl_match, pcl_match_prior, t.inverse().matrix());
    pcl::toROSMsg(pcl_match_prior, pc2);
    pc2.header = pose->header;
    transformed_matching_pub_.publish(pc2);
    pcl_match_prior.header = pose->header;
    pcl_match.header = pose->header;
    
    Eigen::Affine3f ICP_transpose_correction = MICPMatching(pcl_match_prior, pcl_template);
    Eigen::Affine3f CSM_transpose_correction = CSMMatching(pcl_match_prior, pcl_template);
    //subtract away the correction value given by scan matching to get true transformation
    //reminder, rigid transformation Ta * Tb != Tb * Ta
    Eigen::Affine3f ICP_corrected_tf = ICP_transpose_correction * t.inverse();
    Eigen::Affine3f CSM_corrected_tf = CSM_transpose_correction * t.inverse();
    icp_vehicle_pub_.publish(publishVehicle(ICP_corrected_tf.inverse(), pose->header.stamp, pose->header.frame_id, 
		   icp_base_frame_id_, icp_vehicle_frame_id_));
    csm_vehicle_pub_.publish(publishVehicle(CSM_corrected_tf.inverse(), pose->header.stamp, pose->header.frame_id, 
		   csm_base_frame_id_, csm_vehicle_frame_id_));
    
    eigenAffineTo2DPose(ICP_corrected_tf, string("ICP corrected"));
    eigenAffineTo2DPose(CSM_corrected_tf, string("CSM corrected"));
    
    pcl::PointCloud<pcl::PointXYZ> pcl_template_ICP_corrected,
				      pcl_template_CSM_corrected;
    
    
    pcl::transformPointCloud<pcl::PointXYZ>(pcl_match, pcl_template_ICP_corrected, ICP_corrected_tf);
    pcl::transformPointCloud<pcl::PointXYZ>(pcl_match, pcl_template_CSM_corrected, CSM_corrected_tf);
    
    pcl_template_CSM_corrected.header = pose->header;
    pcl_template_ICP_corrected.header = pose->header;
    pcl_template.header = pose->header;
    raw_matched_matching_ICP_pub_.publish(pcl_template_ICP_corrected);
    raw_matched_matching_CSM_pub_.publish(pcl_template_CSM_corrected);
  }
  
  geometry_msgs::PoseStamped publishVehicle(Eigen::Affine3f transformation, ros::Time time,
		      string pose_frame_id, string base_frame_id, string child_frame_id){
    poseResult tf_result = eigenAffineTo2DPose(transformation, "publish vehicle");
    geometry_msgs::PoseStamped pose;
    pose.header.frame_id = pose_frame_id;
    pose.header.stamp = time;
    pose.pose.position.x = tf_result.x;
    pose.pose.position.y = tf_result.y;
    pose.pose.orientation = tf::createQuaternionMsgFromYaw(tf_result.r/180*M_PI);
    tf::StampedTransform vehicle_tf;
    tf::StampedTransform trans(tf::Transform(), time, base_frame_id, child_frame_id);
    tf::poseMsgToTF(pose.pose, trans);
    tf_broadcaster_->sendTransform(trans);
    
    return pose;
  }
  
  Eigen::Affine3f CSMMatching(pcl::PointCloud<pcl::PointXYZ> matching_cloud, 
		    pcl::PointCloud<pcl::PointXYZ> template_cloud){
    fmutil::Stopwatch sw0("csm_ini");
    CsmGPU<pcl::PointXYZ> csmGPU(0.1, cv::Point2d(50.0, 75.0), template_cloud, false);
    sw0.end();
    poseResult result;
    result.x = result.y = result.r = 0.0;
    fmutil::Stopwatch sw1("csm1");
    result = csmGPU.getBestMatch(1.0, 1.0, 5, 5, 5, 90, matching_cloud, result);
    sw1.end();
    fmutil::Stopwatch sw2("csm2");
    result = csmGPU.getBestMatch(0.1, 0.1, 1.0, 1, 1, 10, matching_cloud, result);
    sw2.end();
    cout<<"Best score 2 found: "<< result.x<<", "<<result.y<<", "<<result.r<<": "<<result.score<<endl;
    pcl::PointCloud<pcl::PointXYZ> cloud_out;
    Eigen::Affine3f transposeAffine = PoseToAffine(result.x, result.y, result.r/180*M_PI);
    
    return transposeAffine;
  }
  
  Eigen::Affine3f PoseToAffine(double x, double y, double r){
    Eigen::Vector3f bl_trans(x, y, 0.);
    double yaw_rotate = -r;
    Eigen::Quaternionf bl_rotation(cos(yaw_rotate / 2.), 0, 0,
    -sin(yaw_rotate / 2.));
    Eigen::Translation3f translation (bl_trans);
    Eigen::Affine3f transposeAffine = translation * bl_rotation;
    return transposeAffine;
  }
  
  Eigen::Affine3f MICPMatching(pcl::PointCloud<pcl::PointXYZ> matching_cloud, 
		    pcl::PointCloud<pcl::PointXYZ> template_cloud){
    CSimplePointsMap matching_c, template_c;
    matching_c.setFromPCLPointCloud<pcl::PointCloud<pcl::PointXYZ> >
      (matching_cloud);
    template_c.setFromPCLPointCloud<pcl::PointCloud<pcl::PointXYZ> >
      (template_cloud);
    CICP ICP;
    CICP::TReturnInfo info;
    ICP.options.ICP_algorithm = icpLevenbergMarquardt;
    ICP.options.maxIterations			= 100;
    ICP.options.thresholdAng			= DEG2RAD(90.0f);
    ICP.options.thresholdDist			= 2.5f;
    ICP.options.ALFA				= 0.5f;
    ICP.options.smallestThresholdDist		= 0.05f;
    ICP.options.doRANSAC = false;
    CPose2D	initialPose(0.0f,0.0f,(float)DEG2RAD(0.0f));
    float runningTime;
    CPosePDFPtr pdf = ICP.Align(
		  &template_c,
		  &matching_c,
		  initialPose,
		  &runningTime,
		  (void*)&info);
    vector_double mean_pose;
    pdf->getMeanVal().getAsVector(mean_pose);
    cout << "Mean of estimation: " << pdf->getMeanVal() << endl;
    Eigen::Affine3f transposeAffine = PoseToAffine(mean_pose[0], mean_pose[1], mean_pose[2]);
    return transposeAffine;
  }
};

int main(int argc, char** argv){
  ros::init(argc, argv, "mapOptimize");
  MapMergingOptimize mmo;
  return 0;
}
