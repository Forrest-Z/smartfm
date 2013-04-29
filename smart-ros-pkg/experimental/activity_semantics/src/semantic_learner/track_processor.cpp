#include "track_processor.h"

namespace golfcar_semantics{

	track_processor::track_processor(pd_track_container* pd_container, size_t track_size_thresh, double track_time_thresh, double track_length_thresh)
	{
		track_container_ = pd_container;
		track_size_thresh_ = track_size_thresh;
		track_time_thresh_ = track_time_thresh;
		track_length_thresh_ = track_length_thresh;
	}

	void track_processor::ped_track_classification()
	{
		size_t moving_track_num=0;
		size_t static_track_num=0;
		size_t noisy_track_num =0;


		for(size_t i=0; i<track_container_->tracks.size(); i++)
		{
			track_common &ped_track = track_container_->tracks[i];
			ROS_INFO("elements size %ld, track time %lf", ped_track.elements.size(), ped_track.elements.back().time-ped_track.elements.front().time);

			if(ped_track.elements.size() >= track_size_thresh_ && (ped_track.elements.back().time-ped_track.elements.front().time > track_time_thresh_))
			{
				double track_length = 0.0 ;
				track_length = fmutil::distance(ped_track.elements.front().x, ped_track.elements.front().y, ped_track.elements.back().x, ped_track.elements.back().y);

				if(track_length >= track_length_thresh_)
				{
					ped_track.ped_activity = MOVING;
					moving_track_num++;
				}
				else
				{
					ped_track.ped_activity = STATIC;
					static_track_num++;
				}
			}
			else
			{
				//tracks classified as noise;
				ped_track.ped_activity = NOISE;
				noisy_track_num++;
			}
		}
		ROS_INFO("total_tracks: %ld, moving_tracks: %ld, static_tracks: %ld", moving_track_num, static_track_num, noisy_track_num);
	}

	track_processor::~track_processor()
	{
	}
};

