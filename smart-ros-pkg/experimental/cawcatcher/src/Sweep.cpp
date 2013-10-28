#include <ros/ros.h>
// #include <sensor_msgs/Joy.h>
#include <cawcatcher/CalPotRead.h>
#include <cawcatcher/AngleComm.h>
ros::Publisher *angle_pub;

cawcatcher::AngleComm angle_msg;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "joyIN");

  ros::NodeHandle n;
  
//  ros::Subscriber sub = n.subscribe("/joy", 1, joy_servo_cb);

  angle_pub = new ros::Publisher(n.advertise<cawcatcher::AngleComm>("cawcatcherIN", 1));

	ros::Rate  r(30);
	
	while (ros::ok())
	{
	  for (int i = 0; i <= 30; i++)
	  {
	    angle_msg.LRoll_com = i;
	    angle_msg.LPitch_com = i;
	    angle_msg.RRoll_com = -i;
	    angle_msg.RPitch_com = i;
	    angle_pub->publish(angle_msg);
	    r.sleep();
	    }
	  
	  for (int i = 30; i >= -30; i--)
	  {
	    angle_msg.LRoll_com = i;    
	    angle_msg.LPitch_com = i;
	    angle_msg.RRoll_com = -i;
	    angle_msg.RPitch_com = i;
	    angle_pub->publish(angle_msg);
	    r.sleep();
	    }
	  
	  for (int i = -30; i <= 0; i++)
	  {
	    angle_msg.LRoll_com = i;    
	    angle_msg.LPitch_com = i;
	    angle_msg.RRoll_com = -i;
	    angle_msg.RPitch_com = i;
	    angle_pub->publish(angle_msg);
	    r.sleep();
	    }
	}
	  ros::spin();

  return 0;
}

