#include <ros/ros.h>
// #include <sensor_msgs/Joy.h>
#include <cawcatcher/CalPotRead.h>
#include <cawcatcher/AngleComm.h>
ros::Publisher *angle_pub;

cawcatcher::AngleComm angle_msg;

class cmd_gen
{
	int cur_angle_;
	int min_angle_;
	int max_angle_;
	bool increase_;
	int inc_;
	public:
	cmd_gen(int min_angle,int max_angle, int inc)
	{
		cur_angle_ = 0;
		increase_ = true;
		min_angle_ = min_angle;
		max_angle_ = max_angle;
		inc_ = inc;
	}
	int update(void)
	{
		if(increase_)
		{
			cur_angle_ += inc_;
			if(cur_angle_ > max_angle_)
			{
			  cur_angle_ = max_angle_;
			  increase_ = false;
			}
		}
		else
		{
			cur_angle_ -= inc_;
			if(cur_angle_  < min_angle_)
			{
			  cur_angle_ = min_angle_;
			  increase_ = true;
			}
		}
		return cur_angle_;
		
	}
	int getAngle(void)
	{
		return cur_angle_;
	}
};

cmd_gen roll_cg(-40,40,3);
cmd_gen pitch_cg(-30,30,1);


int main(int argc, char **argv)
{
  ros::init(argc, argv, "joyIN");

  ros::NodeHandle n;
  int roll_angle  = 0;
  int pitch_angle = 0; 
//  ros::Subscriber sub = n.subscribe("/joy", 1, joy_servo_cb);

  angle_pub = new ros::Publisher(n.advertise<cawcatcher::AngleComm>("cawcatcherIN_2", 1));

	ros::Rate  r(10);
	
	while (ros::ok())
	{
	    roll_cg.update();
	    pitch_cg.update();
	    
	    angle_msg.LRoll_com = roll_cg.getAngle();
	    angle_msg.LPitch_com = pitch_cg.getAngle();
	    angle_msg.RRoll_com = - roll_cg.getAngle();
	    angle_msg.RPitch_com = pitch_cg.getAngle();
	    angle_pub->publish(angle_msg);

		r.sleep();
	}
	    angle_msg.LRoll_com = 0;
	    angle_msg.LPitch_com = 0;
	    angle_msg.RRoll_com = 0;
	    angle_msg.RPitch_com = 0;
	    angle_pub->publish(angle_msg);
	 ros::spin();

  return 0;
}

