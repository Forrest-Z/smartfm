#include "ros/ros.h"
#include "std_msgs/Int32.h"
#include <cawcatcher/CalPotRead.h>
#include <tf/transform_broadcaster.h>

void chatterCallback(const cawcatcher::CalPotRead::ConstPtr& msg)
{
  //  msg->header.stamp = ros::Time::now();

  static tf::TransformBroadcaster br;
  
// left
  tf::Transform LPitchTransform;
  LPitchTransform.setOrigin( tf::Vector3(0, 0, 0) );
  LPitchTransform.setRotation( tf::Quaternion(M_PI/180*msg->Lpitch, 0, 0) );
  br.sendTransform(tf::StampedTransform(LPitchTransform, ros::Time::now(), "LStaticFrame", "LPitchTF"));
	
  tf::Transform LRollTransform;
  LRollTransform.setOrigin( tf::Vector3(0.11, 0, 0.0) );
  						//pitch	//roll	//yaw
  LRollTransform.setRotation( tf::Quaternion(0, M_PI/180*msg->Lroll, 0) );
  br.sendTransform(tf::StampedTransform(LRollTransform, ros::Time::now(), "LPitchTF", "LRollTF"));

//Right	
  tf::Transform RPitchTransform;
  RPitchTransform.setOrigin( tf::Vector3(0, 0, 0) );
  RPitchTransform.setRotation( tf::Quaternion(M_PI/180*msg->Rpitch, 0, 0) );
  br.sendTransform(tf::StampedTransform(RPitchTransform, ros::Time::now(), "RStaticFrame", "RPitchTF"));
	
  tf::Transform RRollTransform;
  RRollTransform.setOrigin( tf::Vector3(0.11, 0, 0.0) );
  						//pitch	//roll	//yaw
  RRollTransform.setRotation( tf::Quaternion(0, M_PI/180*msg->Rroll, 0) );
  br.sendTransform(tf::StampedTransform(RRollTransform, ros::Time::now(), "RPitchTF", "RRollTF"));
}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "listener");

  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("attitude", 1000, chatterCallback);

  ros::spin();

  return 0;
}


