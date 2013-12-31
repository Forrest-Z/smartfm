#define  EIGEN_YES_I_KNOW_SPARSE_MODULE_IS_NOT_STABLE_YET 
#include <mrpt/slam.h>
#include <mrpt/gui.h>

#include <norm_virtual_sensor/AccumulateData.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <pcl_ros/transforms.h>

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
    matched_matching_ICP_pub_ = nh.advertise<sensor_msgs::PointCloud2>
				  ("matched_matching_ICP", 1);
    matched_matching_CSM_pub_ = nh.advertise<sensor_msgs::PointCloud2>
				  ("matched_matching_CSM", 1);
    raw_matched_matching_ICP_pub_ = nh.advertise<sensor_msgs::PointCloud2>
				      ("raw_matched_matching_ICP", 1);
    raw_matched_matching_CSM_pub_ = nh.advertise<sensor_msgs::PointCloud2>
				      ("raw_matched_matching_CSM", 1);
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
  void eigenAffineTo2DPose(Eigen::Affine3f t, string msg){
    double yaw, pitch, roll;
    boost::tie(yaw,pitch,roll) = matrixToYawPitchRoll(t.rotation());
    double x = t.translation()(0);
    double y = t.translation()(1);
    double rot = yaw/M_PI*180;
    cout<<msg<<": "<<
    x<<","<<y<<","<<rot<<endl;
  }
  void callback(const geometry_msgs::PoseStampedConstPtr& pose,
		const sensor_msgs::PointCloud2ConstPtr& observation_pc2,
		const sensor_msgs::LaserScanConstPtr& scan){
    sensor_msgs::PointCloud cloud;
    projector_.projectLaser(*scan, cloud);
    sensor_msgs::PointCloud2 pc2;
    sensor_msgs::convertPointCloudToPointCloud2(cloud, pc2);
    pcl::PointCloud<pcl::PointXYZ> pcl_match, pcl_template, pcl_template_prior;
    pcl::fromROSMsg(pc2, pcl_match);
    pcl::fromROSMsg(*observation_pc2, pcl_template);
    cout<<pcl_match.size()<<" to match with "<<pcl_template.size()<<endl;
    geometry_msgs::Quaternion orientation = pose->pose.orientation;
    geometry_msgs::Point position = pose->pose.position;
    Eigen::Quaternionf bl_rotation(orientation.w, orientation.x,
				   orientation.y,orientation.z); 
    Eigen::Vector3f bl_trans(position.x,position.y,position.z);
    pcl::transformPointCloud<pcl::PointXYZ>(pcl_template, pcl_template_prior, bl_trans, bl_rotation);
    pcl::toROSMsg(pcl_template, pc2);
    pc2.header = pose->header;
    transformed_template_pub_.publish(pc2);
    Eigen::Translation3f translation (bl_trans);
    Eigen::Affine3f t = translation * bl_rotation;
    eigenAffineTo2DPose(t, string("given transform of leader vehicle based on ego vehicle"));
    tf::StampedTransform cur_sensor_trans;
    tf_->waitForTransform(scan->header.frame_id, pose->header.frame_id, ros::Time::now(), ros::Duration(1.0));
    tf_->lookupTransform(pose->header.frame_id, scan->header.frame_id, scan->header.stamp, cur_sensor_trans);
    Eigen::Quaternionf bl_rotation_sensor(cur_sensor_trans.getRotation().w(), cur_sensor_trans.getRotation().x(),cur_sensor_trans.getRotation().y(),cur_sensor_trans.getRotation().z()); 
    Eigen::Vector3f bl_trans_sensor(cur_sensor_trans.getOrigin().x(),cur_sensor_trans.getOrigin().y(),cur_sensor_trans.getOrigin().z());
    pcl::PointCloud<pcl::PointXYZ> pcl_match_prior;
    pcl::transformPointCloud<pcl::PointXYZ>(pcl_match, pcl_match, bl_trans_sensor, bl_rotation_sensor);
    pcl::transformPointCloud<pcl::PointXYZ>(pcl_match, pcl_match_prior, t.inverse().matrix());
    pcl::toROSMsg(pcl_match, pc2);
    pc2.header = pose->header;
    transformed_matching_pub_.publish(pc2);
    pcl_match_prior.header = pose->header;
    pcl_match.header = pose->header;
    pcl_template_prior.header = pose->header;
    
    //recheck the calculation here. Doesn't seems to be correct.
    Eigen::Affine3f ICP_transpose_correction = MICPMatching(pcl_match_prior, pcl_template);
    Eigen::Affine3f CSM_transpose_correction = CSMMatching(pcl_match_prior, pcl_template);
    //subtract away the correction value given by scan matching to get true transformation
    Eigen::Affine3f ICP_corrected_tf = t.inverse() * ICP_transpose_correction;
    Eigen::Affine3f CSM_corrected_tf = t.inverse() * CSM_transpose_correction;
    eigenAffineTo2DPose(ICP_corrected_tf, string("ICP corrected"));
    eigenAffineTo2DPose(CSM_corrected_tf, string("CSM corrected"));
    
    pcl::PointCloud<pcl::PointXYZ> pcl_template_ICP_corrected,
				      pcl_template_CSM_corrected;
    pcl::transformPointCloud<pcl::PointXYZ>(pcl_match, pcl_template_ICP_corrected, t.inverse().matrix());
    pcl::transformPointCloud<pcl::PointXYZ>(pcl_template_ICP_corrected, pcl_template_ICP_corrected, ICP_transpose_correction.matrix());
    //the above means that t.inverse()*transpose_correction != transpose(t.inverse()) -> transpose(t.inverse())?
    pcl::transformPointCloud<pcl::PointXYZ>(pcl_match, pcl_template_CSM_corrected, t.inverse().matrix());
    pcl::transformPointCloud<pcl::PointXYZ>(pcl_template_CSM_corrected, pcl_template_CSM_corrected, CSM_transpose_correction.matrix());
    pcl_template_CSM_corrected.header = pose->header;
    pcl_template_ICP_corrected.header = pose->header;
    pcl_template.header = pose->header;
    raw_matched_matching_ICP_pub_.publish(pcl_template_ICP_corrected);
    raw_matched_matching_CSM_pub_.publish(pcl_template_CSM_corrected);
  }
  
  Eigen::Affine3f CSMMatching(pcl::PointCloud<pcl::PointXYZ> matching_cloud, 
		    pcl::PointCloud<pcl::PointXYZ> template_cloud){
    CsmGPU<pcl::PointXYZ> csmGPU(0.1, cv::Point2d(50.0, 75.0), template_cloud, false);
    poseResult result;
    result.x = result.y = result.r = 0.0;
    result = csmGPU.getBestMatch(1.0, 1.0, 2, 5, 5, 45, matching_cloud, result);
    result = csmGPU.getBestMatch(0.1, 0.1, 0.5, 1, 1, 5, matching_cloud, result);
    cout<<"Best score 2 found: "<< result.x<<", "<<result.y<<", "<<result.r<<": "<<result.score<<endl;
    pcl::PointCloud<pcl::PointXYZ> cloud_out;
    Eigen::Affine3f transposeAffine = PoseToAffine(result.x, result.y, result.r/180*M_PI);
    Eigen::Matrix4f transform = transposeAffine.matrix();
    pcl::transformPointCloud(matching_cloud, cloud_out, transform);
    sensor_msgs::PointCloud2 pc2;
    pcl::toROSMsg(cloud_out, pc2);
    matched_matching_CSM_pub_.publish(pc2);
    
    return transposeAffine;//PoseToAffine(-result.x, -result.y, -result.r/180*M_PI);
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
    ICP.options.thresholdAng			= DEG2RAD(10.0f);
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
    Eigen::Matrix4f tf = transposeAffine.matrix();
    pcl::transformPointCloud<pcl::PointXYZ>(matching_cloud, matching_cloud, tf);
    sensor_msgs::PointCloud2 pc2;
    pcl::toROSMsg(matching_cloud, pc2);
    matched_matching_ICP_pub_.publish(pc2); 
    return transposeAffine;//PoseToAffine(-mean_pose[0], -mean_pose[1], -mean_pose[2]);
  }
};

int main(int argc, char** argv){
  ros::init(argc, argv, "mapOptimize");
  MapMergingOptimize mmo;
  return 0;
}
