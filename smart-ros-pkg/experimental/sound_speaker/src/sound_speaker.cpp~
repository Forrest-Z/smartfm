/*
 * sound_speaker.cpp
 *
 *  Created on: Oct 14, 2013
 *      Author: Shen Xiaotong
 */

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/UInt16.h>
#include <sound_play/sound_play.h>
#include <unistd.h>
#include <string>

using namespace std;
sound_play::SoundClient * sc_;
string sound_path;

void voiceStrCB(const std_msgs::String::ConstPtr str_ptr)
{
	/*switch(str_ptr->data)
	{
	default:
		break;
	}
	*/
}

void voiceIdCB(const std_msgs::UInt16::ConstPtr id_ptr)
{
	string file_name;
	switch(id_ptr->data)
	{
	case 1:
		file_name = sound_path + string("DANGER.WAV");
		//sc_->playWave(file_name.c_str());
		sc_->repeat(file_name.c_str());
		break;
	case 2:

		file_name = sound_path + string("The Imperial March.mp3");
		sc_->playWave(file_name.c_str());
		break;
	case 3:

		file_name = sound_path + string("gameover.wav");
		sc_->playWave(file_name.c_str());
		break;
	default:
		break;
	}

}



int main(int argc, char ** argv)
{
	ros::init(argc,argv,"sound_speaker");
	ros::NodeHandle nh;
	nh.param("sound_path",sound_path,string("/home/sxt/smartfm/smartfm/smart-ros-pkg/experimental/sound_speaker/sounds/"));
	sound_play::SoundClient sc;
	sc_ = & sc;

	ros::Subscriber voice_str_sub = nh.subscribe("voice_str",10,voiceStrCB);
	ros::Subscriber voice_id_sub = nh.subscribe("voice_id",10,voiceIdCB);

	ros::spin();
	return 0;

}
