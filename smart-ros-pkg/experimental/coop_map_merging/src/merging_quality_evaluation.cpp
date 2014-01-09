#include <norm_virtual_sensor/AccumulateData.h>

struct pose2D_final{
  double x,y,r;
};

tf::TransformListener *tf_listener_;

tf::StampedTransform getTransform(string base_frame, string child_frame){
  tf::StampedTransform transform;
  pose2D_final pose;
  try{
      
      //tf_listener_->waitForTransform(base_frame, child_frame, ros::Time::now(), ros::Duration(1.0));
      tf_listener_->lookupTransform(base_frame, child_frame, ros::Time(0), transform);
      double roll, pitch, yaw;
      tf::Matrix3x3(transform.getRotation()).getRPY(roll, pitch, yaw);
      pose.x = transform.getOrigin().x();
      pose.y = transform.getOrigin().y();
      pose.r = yaw/M_PI*180;
    }
    catch (tf::LookupException ex)
    {
    }
    catch (tf::ExtrapolationException ex2){}
    //cout<<pose.x<<" "<<pose.y<<" "<<pose.r<<" ";
    return transform;
}

void outputError(tf::StampedTransform ground_truth, tf::StampedTransform measurement){
  tf::Transform transform = ground_truth.inverseTimes(measurement);
  double roll, pitch, yaw;
  tf::Matrix3x3(transform.getRotation()).getRPY(roll, pitch, yaw);
  pose2D_final pose;
  pose.x = transform.getOrigin().x();
  pose.y = transform.getOrigin().y();
  pose.r = yaw/M_PI*180;
  cout<<sqrtf(pose.x*pose.x+pose.y*pose.y)<<" "<<pose.r<<" ";
}
  
int main(int argc, char **argv)
{
  ros::init(argc, argv, "merging_quality_evaluation");
  ros::NodeHandle n;
  ros::Rate loop_rate(10);
  tf_listener_ = new tf::TransformListener();
  cout<<"ground ground_r laser laser_r icp icp_r csm csm_r ";
  cout<<"ground2 ground2_r laser2 laser2_r icp2 icp2_r csm2 csm2_r"<<endl;
  while(ros::ok()){
    
    tf::StampedTransform ground_truth1 = getTransform("golfcart/base_link", "base_link");
    tf::StampedTransform laser1 = getTransform("golfcart/base_link", "laser_base_link");
    tf::StampedTransform icp1 = getTransform("golfcart/base_link", "icp_base_link");
    tf::StampedTransform csm1 = getTransform("golfcart/base_link", "csm_base_link");
    tf::StampedTransform ground_truth2 = getTransform("golfcart/base_link", "golfcart2/base_link");
    tf::StampedTransform laser2 = getTransform("golfcart/base_link", "golfcart2/laser_base_link");
    tf::StampedTransform icp2 = getTransform("golfcart/base_link", "golfcart2/icp_base_link");
    tf::StampedTransform csm2 = getTransform("golfcart/base_link", "golfcart2/csm_base_link");
    outputError(ground_truth1, laser1);
    outputError(ground_truth1, icp1);
    outputError(ground_truth1, csm1);
    outputError(ground_truth2, laser2);
    outputError(ground_truth2, icp2);
    outputError(ground_truth2, csm2);
    
    cout<<endl;
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}