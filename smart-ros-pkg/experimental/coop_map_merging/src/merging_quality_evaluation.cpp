#include <norm_virtual_sensor/AccumulateData.h>

struct pose2D_final{
  double x,y,r;
};

tf::TransformListener *tf_listener_;

pose2D_final getTransform(string base_frame, string child_frame){
  pose2D_final pose;
  try{
      tf::StampedTransform transform;
      tf_listener_->waitForTransform(base_frame, child_frame, ros::Time::now(), ros::Duration(1.0));
      tf_listener_->lookupTransform(base_frame, child_frame, ros::Time(), transform);
      double roll, pitch, yaw;
      tf::Matrix3x3(transform.getRotation()).getRPY(roll, pitch, yaw);
      pose.x = transform.getOrigin().x();
      pose.y = transform.getOrigin().y();
      pose.r = yaw/180*M_PI;
    }
    catch (tf::LookupException ex)
    {
    }
    cout<<pose.x<<" "<<pose.y<<" "<<pose.r<<endl;
    return pose;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "merging_quality_evaluation");
  ros::NodeHandle n;
  ros::Rate loop_rate(10);
  tf_listener_ = new tf::TransformListener();
  while(ros::ok()){
    
    cout<<"Ground truth: "; getTransform("golfcart/base_link", "base_link");
    cout<<"laser pose: "; getTransform("golfcart/base_link", "laser_base_link");
    cout<<"icp pose: "; getTransform("golfcart/base_link", "icp_base_link");
    cout<<"csm pose: "; getTransform("golfcart/base_link", "csm_base_link");
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}