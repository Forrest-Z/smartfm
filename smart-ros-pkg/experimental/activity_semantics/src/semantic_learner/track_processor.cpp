#include "track_processor.h"

namespace golfcar_semantics{

	track_processor::track_processor(pd_track_container* pd_container, size_t track_size_thresh, double track_time_thresh, double track_length_thresh)
	{
		track_container_ = pd_container;
		track_size_thresh_ = track_size_thresh;
		track_time_thresh_ = track_time_thresh;
		track_length_thresh_ = track_length_thresh;
		printf("%ld, %f, %f\n", track_size_thresh_, track_time_thresh, track_length_thresh);
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
		ROS_INFO("moving_tracks: %ld, : static_tracks: %ld, noisy_tracks %ld", moving_track_num, static_track_num, noisy_track_num);

		//to interpolate between the track points;
		bool interpolate_flag = true;
		if(interpolate_flag)
		{
			double interpolate_distance = 0.3;
			for(size_t i=0; i<track_container_->tracks.size(); i++)
			{
				track_common &ped_track = track_container_->tracks[i];
				if(ped_track.ped_activity == MOVING)
				{
					for(size_t j=1; j<ped_track.elements.size(); j++)
					{
						double length_between_elements = fmutil::distance(ped_track.elements[j].x, ped_track.elements[j].y, ped_track.elements[j-1].x, ped_track.elements[j-1].y);
						double time_between_elements = ped_track.elements[j].time - ped_track.elements[j-1].time;

						int interpolate_Num = floor((length_between_elements-0.001)/interpolate_distance);
						if(interpolate_Num<=1)continue;
						else
						{
							double dist_inverval_x = (ped_track.elements[j].x-ped_track.elements[j-1].x)/double(interpolate_Num);
							double dist_inverval_y = (ped_track.elements[j].y-ped_track.elements[j-1].y)/double(interpolate_Num);
							double time_inverval = time_between_elements/double(interpolate_Num);
							vector<track_element> interpolate_elements;
							for(int a=1; a<=interpolate_Num; a++)
							{
								track_element element_tmp;
								element_tmp.x = ped_track.elements[j-1].x + dist_inverval_x*a;
								element_tmp.y = ped_track.elements[j-1].y + dist_inverval_y*a;
								element_tmp.time = ped_track.elements[j-1].time + time_inverval*a;
								interpolate_elements.push_back(element_tmp);
							}
							std::vector<track_element>::iterator it;
							it = ped_track.elements.begin();
							ped_track.elements.insert (it+j-1, interpolate_elements.begin(),interpolate_elements.end());

							j=j+interpolate_Num;
						}
					}
				}
			}
		}
	}

	track_processor::~track_processor()
	{
	}
};

