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
#include <std_msgs/Bool.h>
#include <unistd.h>
#include <string>

#define NUM_SONGS 4
#define BACKGROUND_ID (NUM_SONGS - 1)


using namespace std;
sound_play::SoundClient * sc_;
string sound_path;

double music_duration[NUM_SONGS] = {0.001,6.0,2.0,25.0};
//the smaller the id is, the higher priority ...
class voice_manager
{
	typedef enum{
			BUSY = 0,
			IDLE = 1,
		}STATUS;
public:
	unsigned int cur_id_;
	ros::Time last_update_time_;
	STATUS status_;
	ros::Duration cur_music_duration_;
	voice_manager(void)
	{
		status_ = IDLE;
		cur_id_ = 0;
	}

	unsigned int get_cur_id(void)
	{
		return cur_id_;
	}

	void checkStatus(unsigned int music_id)
	{
		if(status_ ==  BUSY)
		{
			cur_music_duration_.fromSec(music_duration[cur_id_]);
			if(ros::Duration(ros::Time::now() - last_update_time_) >= cur_music_duration_)
			{
				status_ = IDLE;
				return;
			}

			if(cur_id_  > music_id)
			{
				status_ = IDLE;
			}
		}
	}
	void update(unsigned int music_id)
	{
		if(music_id >= NUM_SONGS)
		{
			return;
		}

		checkStatus(music_id);
		if(status_ == IDLE)
		{
			play(music_id);
			last_update_time_ = ros::Time::now();
			cur_id_ = music_id;
			status_ = BUSY;
		}
	}
	void play(unsigned int play_id)
	{
		string file_name;
		switch(play_id)
		{
		case 0:
			sc_->stopAll();
			break;
		case 1:
			file_name = sound_path + string("DANGER.WAV");
			sc_->playWave(file_name.c_str());
			//sc_->say("Excuse me");
			break;
		case 2:
			file_name = sound_path + string("gameover.wav");
			sc_->playWave(file_name.c_str());
			break;
		case BACKGROUND_ID:
			file_name = sound_path + string("The Imperial March.mp3");
			sc_->startWave(file_name.c_str());
			break;
		default:
			break;
		}
	}



};

voice_manager vm;

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
	if(vm.get_cur_id() != id_ptr->data)
	{
		vm.update(id_ptr->data);
	}
}

void music_timer_CB(const ros::TimerEvent & event)
{
	cout<<"current playing "<<vm.get_cur_id()<<endl;

	if(vm.get_cur_id() != BACKGROUND_ID)
	{
		vm.update(BACKGROUND_ID);
	}
}

void CPvoiceCB(const std_msgs::Bool::ConstPtr id_ptr)
{   
	string file_name;
    if (id_ptr->data == 1){
		ros::Rate r(1.0);
        file_name = sound_path + string("two_times_warning.wav");
	    sc_->playWave(file_name.c_str());
	    r.sleep();
	}
}

int main(int argc, char ** argv)
{
	ros::init(argc,argv,"sound_speaker");
	ros::NodeHandle nh;
	ros::NodeHandle private_nh("~");
	private_nh.param("sound_path",sound_path,string("/home/sxt/smartfm/smartfm/smart-ros-pkg/experimental/sound_speaker/sounds/"));
	cout<<"Music is On now!"<<endl;
	cout<<"music path is "<<sound_path<<endl;
	sound_play::SoundClient sc;
	sc_ = & sc;

	//ros::Subscriber voice_str_sub = nh.subscribe("voice_str",20,voiceStrCB);
	//ros::Subscriber voice_id_sub = nh.subscribe("voice_id",20,voiceIdCB);
    ros::Subscriber waring_id_sub = nh.subscribe("/replan_trigger",1,CPvoiceCB);
	//ros::Timer music_timer = nh.createTimer(ros::Duration(4.0),music_timer_CB);

	ros::spin();
	return 0;

}
