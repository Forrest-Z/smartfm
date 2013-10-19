#include <ros/ros.h>
#include <can_reader/CanMsg.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/Point32.h>

ros::Publisher * vel_pub_;
ros::Publisher * odom_pub_;
ros::Publisher * steer_pub_;
ros::Publisher * volt_pub_;
ros::Publisher * current_pub_;
ros::Publisher * brake_onoff_pub_;
ros::Publisher * mile_left_pub_;
ros::Publisher * brake_pedal_pub_;
ros::Publisher * front_encoder_pub_;
ros::Publisher * encoder_pub_;


//gearshift 0x418(1048) park80, reverse82, neutral78, drive 68, Bgear131, Cgear50

//1060(0x424) is for some switches, car key in/out, left,right,left/right ...
//headlight Byte2 96(110000)high 64(100000)low  0off
//RL lamp  Byte2 0nosignal 1right 2left 3both
//automatic locks Byte3 13unlocked 77locked (sixth bit of byte3 is lock signal)
//windshield wiper Byte2 4thbit(0or8)for the front 5th(0 or 64) bit for the back one

//528(0x210) is for the throttle, Byte3 (0-250)

//645 648 is related with abs(suspected)

//0x325 (805) is always [1 0] seems ... when the key is inserted, it will detected it changed 2 times

//20ms 119, 149, 156, 200, 208, 210, 212, 215, 231, 300, 308, 325, 346, 418

//0x212(530) and 0x215(533) is related with driving, because the changing rate changes dramatically

//0x215 is more accurate odom, already integral, byte[0][1] is unknown
//0x200(512) (([2] - 192)*256 + [3] )  (([4] - 192)*256 + [5] ) seems to be the speed, this is the front wheel speed, confirmed
//0x208(520) similar to 0x200, the last 4bytes, the formula are the same. .., This is the back wheel speed, confirmed
//0x298(664) last two bytes related with motion, the curve is similar to back wheel speed curve ...

//0x149 (329) the 7th bit from 0 - 15 counting ... no idea what it is ...
//[xx, 127, 0, 255, xx, 128, 0-15, xx]
//0x156 (342) changing strangely, not encoder .
//0x119(281)  119[6]  from (0-15) then 119[5]++ (from 0 to 10), not encoder ..
// [0, 0, 0, 0, 0, 4, 14, 129]

//0x285(645), the first two bytes are changing , others unchanged ...,maybe correspond to one sensor ...
//[8, 231, 20, 0, 141, 254, 14, 16]
//0x288(648) similar, I think it related with motion, no sure what it is ...


//0x695(1685) is kind of a sensor ([0]*256+[1] , [3]*256+[4], [5]*256+[6])

#define NUM_MSG  14
int test_id[NUM_MSG] = {0x119, 0x149, 0x156, 0x200, 0x208, 0x210, 0x212, 0x215, 0x231, 0x300, 0x308, 0x325, 0x346, 0x418};
int changed_times[NUM_MSG] = {-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1};

//#define NUM_MSG  20
//int test_id[NUM_MSG] = {0x101, 0x286, 0x298, 0x29A, 0x2F2, 0x374, 0x375, 0x384, 0x385, 0x389 , 0x38A , 0x3A4, 0x408, 0x412, 0x695, 0x696, 0x697, 0x6FA, 0x75A, 0x75B};//
//int changed_times[NUM_MSG] = {-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1 ,-1,-1,-1,-1,-1,-1};


#define SQ(x) ((x)*(x))

can_reader::CanMsg msg[NUM_MSG];
unsigned long int last_time[NUM_MSG] = {0};
bool compareCanMsg(can_reader::CanMsg msg1, can_reader::CanMsg msg2)
{
	int dlc = 0;

	if(msg1.dlc < msg2.dlc)
	{
		dlc = msg1.dlc;
	}
	else
	{
		dlc = msg2.dlc;
	}

	for(int i = 0; i < dlc; i++)
	{
		if(msg1.data[i] != msg2.data[i])
		{
			return false;
		}
	}

	return true;
}


void canMsgCallback(can_reader::CanMsg::ConstPtr can_msg_ptr)
{
	std_msgs::Float32 float_msg;
	geometry_msgs::Point32 point_msg;
	geometry_msgs::Point32 last_point_msg;
	geometry_msgs::Point32 front_encoder_msg;
	switch(can_msg_ptr->id)
	{
	//1042
	case 0x412:
		//speed and odom
		float_msg.data = can_msg_ptr->data[1];
		vel_pub_->publish(float_msg);
		float_msg.data = (can_msg_ptr->data[2] * 256 + can_msg_ptr->data[3])*256 + can_msg_ptr->data[4];
		odom_pub_->publish(float_msg);
		break;
	//566
	case 0x236:
		//steering wheel
		float_msg.data = (can_msg_ptr->data[0]*256 + can_msg_ptr->data[1] - 4096)/2.0;
		steer_pub_->publish(float_msg);
		break;
	case 0x373:
		//volt and current
		float_msg.data =(can_msg_ptr->data[2] * 256 + can_msg_ptr->data[3] - 128*256)/100.0;
		current_pub_->publish(float_msg);
		float_msg.data = (can_msg_ptr->data[4] * 256 + can_msg_ptr->data[5])/10.0;
		volt_pub_->publish(float_msg);
		break;
	case 0x231:
		float_msg.data = can_msg_ptr->data[4];
		brake_onoff_pub_->publish(float_msg);
		break;
	case 0x346:
		float_msg.data = can_msg_ptr->data[7];
		mile_left_pub_->publish(float_msg);
		break;
	case 0x208:
		float_msg.data = (can_msg_ptr->data[2] - 96)*256 + can_msg_ptr->data[3];
		brake_pedal_pub_->publish(float_msg);
		break;
	case 0x215:
		//point_msg.x = can_msg_ptr->data[0] * 256 + can_msg_ptr->data[1];
		point_msg.y = can_msg_ptr->data[2] * 256 + can_msg_ptr->data[3]; //y and z seems for the encoder , x is the changing rate?? that is not very accurate!!!
		point_msg.z = can_msg_ptr->data[4] * 256 + can_msg_ptr->data[5];
		point_msg.x  = sqrt(SQ(point_msg.y - last_point_msg.y) + SQ(point_msg.x - last_point_msg.x));
		encoder_pub_->publish(point_msg);
		last_point_msg = point_msg;
		break;
	case 0x75a:
		// this message some how related with motion, not sure what it is, it seems the differences are related with speed
		front_encoder_msg.y = can_msg_ptr->data[0]*256 + can_msg_ptr->data[1];
		front_encoder_msg.z = can_msg_ptr->data[2]*256 + can_msg_ptr->data[3];
		front_encoder_pub_->publish(front_encoder_msg);
		break;
	default:
		for( int ind = 0; ind < NUM_MSG; ind++)
		{
			if(test_id[ind] == can_msg_ptr->id)
			{
				if(changed_times[ind] < 0)
				{
					changed_times[ind] = 0;
					msg[ind] = *can_msg_ptr;
					last_time[ind] = can_msg_ptr->time;
				}
				else
				{
					if(compareCanMsg(msg[ind],*can_msg_ptr) == false)
					{
						changed_times[ind]++;
					}
					
					if((can_msg_ptr->time - last_time[ind]) > 2000)
					{
						if(changed_times[ind]>0)
						{
							printf("can msg id: %x  changed %d times\n", can_msg_ptr->id,changed_times[ind]);
							changed_times[ind] = 0;
							last_time[ind] = can_msg_ptr->time;
						}
					}
					msg[ind] = *can_msg_ptr;

				}
			}
                }
		break;

	}
}

int main(int argc, char ** argv)
{
	ros::init(argc,argv,"test_can");
	ros::NodeHandle nh;
	ros::Subscriber can_sub = nh.subscribe("can_msg",100,&canMsgCallback);
	ros::Publisher vel_pub = nh.advertise<std_msgs::Float32>("can_velocity",10);
	ros::Publisher odom_pub = nh.advertise<std_msgs::Float32>("can_odom",10);
	ros::Publisher steer_pub = nh.advertise<std_msgs::Float32>("can_steer",10);
	ros::Publisher volt_pub = nh.advertise<std_msgs::Float32>("can_voltage",10);
	ros::Publisher current_pub = nh.advertise<std_msgs::Float32>("can_current",10);
	ros::Publisher brake_onoff_pub = nh.advertise<std_msgs::Float32>("can_brake_on_off",10);
	ros::Publisher mile_left_pub = nh.advertise<std_msgs::Float32>("can_mile_left",10);
	ros::Publisher brake_pedal_pub = nh.advertise<std_msgs::Float32>("can_brake_pedal",10);
    ros::Publisher encoder_pub = nh.advertise<geometry_msgs::Point32>("can_encoders",10);
    ros::Publisher front_encoder_pub = nh.advertise<geometry_msgs::Point32>("can_front_encoders",10);
	vel_pub_ = & vel_pub;
	odom_pub_ = & odom_pub;
	steer_pub_ = & steer_pub;
	volt_pub_ = & volt_pub;
	current_pub_ = & current_pub;
	brake_onoff_pub_ = & brake_onoff_pub;
	mile_left_pub_ = & mile_left_pub;
	brake_pedal_pub_ = & brake_pedal_pub;
	encoder_pub_ = & encoder_pub;
	front_encoder_pub_ = & front_encoder_pub;
	ros::spin();
	return 0;
}
