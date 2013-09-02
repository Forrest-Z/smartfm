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

	void track_processor::process_tracks()
	{
		ped_track_classification();
		calc_speed_and_thetha();
		cluster_moving_tracks();
		interpolate_elements();
	}

	void track_processor::ped_track_classification()
	{
		size_t moving_track_num=0;
		size_t static_track_num=0;
		size_t noisy_track_num =0;
		size_t uturn_track_num =0;

		for(size_t i=0; i<track_container_->tracks.size(); i++)
		{
			track_common &ped_track = track_container_->tracks[i];
			ROS_INFO("track %ld, elements size %ld, track time %lf", i, ped_track.elements.size(), ped_track.elements.back().time-ped_track.elements.front().time);

			if(ped_track.elements.size() >= track_size_thresh_ && (ped_track.elements.back().time-ped_track.elements.front().time > track_time_thresh_))
			{
				double track_length = 0.0 ;
				track_length = fmutil::distance(ped_track.elements.front().x, ped_track.elements.front().y, ped_track.elements.back().x, ped_track.elements.back().y);

				if(track_length >= track_length_thresh_)
				{
					if(!check_Uturn_track(ped_track))
					{
						ped_track.ped_activity = MOVING;
						moving_track_num++;
					}
					else
					{
						//tracks classified as noise;
						ped_track.ped_activity = NOISE;
						uturn_track_num++;
						noisy_track_num++;
						ROS_INFO("detect U-turn track %ld, pay attention !!!", i);
					}
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
		ROS_INFO("moving_tracks: %ld, : static_tracks: %ld, noisy_tracks %ld, uturn_track_num %ld", moving_track_num, static_track_num, noisy_track_num, uturn_track_num);
	}

	//If this track contains tight U-turn, return true; else return false;
	//1st step: perform "approxPolyDP" to extract dominant line pieces;
	//2nd step: check the angle between long pieces; if the angle is over certain threshold, it is simply denoted as U-turn track;
	bool track_processor::check_Uturn_track(track_common &track)
	{
		vector<Point2f> track_raw, track_poly;
		for(size_t i=0; i<track.elements.size(); i++)
		{
			track_raw.push_back(Point2f(float(track.elements[i].x),float(track.elements[i].y)));
		}

		double epsilon = 0.5;
		approxPolyDP( Mat(track_raw), track_poly, epsilon, false );
		assert(track_poly.size()>1);

		for(size_t i=0; i<track_poly.size(); i++)
		{
			printf("(%3f, %3f)\t", track_poly[i].x, track_poly[i].y);
		}
		printf("\n");


		//find a little bug: According to OpenCV's implementation, the raw point may be outside approxPoly's end points;
		//add some rescue steps;
		double distance_threshold = 1.0;
		double delt_angle_threshold = M_PI/180.0*150.0;
		vector<double> angles;
		for(size_t i=1; i<track_poly.size(); i++)
		{
			double distance_tmp = std::sqrt(std::pow(track_poly[i].x-track_poly[i-1].x, 2.0)+std::pow(track_poly[i].y-track_poly[i-1].y, 2.0));
			double angle_tmp = atan2(track_poly[i].y-track_poly[i-1].y, track_poly[i].x-track_poly[i-1].x);
			if(distance_tmp > distance_threshold)angles.push_back(angle_tmp);
		}
		double max_delt_angle = 0;

		for(size_t i=0; i<angles.size(); i++)
		{
			for(size_t j=0; j<angles.size(); j++)
			{
				double delt_angle_tmp = fabs(angles[i]-angles[j]);
				if(delt_angle_tmp>M_PI) delt_angle_tmp = 2.0*M_PI-delt_angle_tmp;
				if(delt_angle_tmp>max_delt_angle) max_delt_angle = delt_angle_tmp;
			}
		}

		if(max_delt_angle > delt_angle_threshold) return true;
		else
		{
			//rescue step: check whether the approxPoly has such issue;
			for(size_t i=0; i<track_raw.size(); i++)
			{
				max_delt_angle = 0.0;
				size_t poly_point_serial = 0;

				//when the point itself is one of the polygon point;
				double MinDist_to_polygonPoint = DBL_MAX;
				for(size_t j=0; j<track_poly.size(); j++)
				{
					double side_tmp = std::sqrt(std::pow(track_raw[i].x-track_poly[j].x, 2.0)+std::pow(track_raw[i].y-track_poly[j].y, 2.0));
					if(side_tmp<MinDist_to_polygonPoint)MinDist_to_polygonPoint=side_tmp;
				}
				if(MinDist_to_polygonPoint < 0.001) continue;

				for(size_t j=1; j<track_poly.size(); j++)
				{
					double angle_tmp1 = atan2(track_raw[i].y-track_poly[j].y, track_raw[i].x-track_poly[j].x);
					double angle_tmp2 = atan2(track_raw[i].y-track_poly[j-1].y, track_raw[i].x-track_poly[j-1].x);
					double delt_angle_tmp = fabs(angle_tmp1-angle_tmp2);
					if(delt_angle_tmp>M_PI) delt_angle_tmp = 2.0*M_PI-delt_angle_tmp;
					if(delt_angle_tmp>max_delt_angle) {max_delt_angle = delt_angle_tmp; poly_point_serial = j;}
				}

				if(max_delt_angle < M_PI - delt_angle_threshold )
				{
					double side_tmp1, side_tmp2;
					side_tmp1 = std::sqrt(std::pow(track_raw[i].x-track_poly[poly_point_serial].x, 2.0)+std::pow(track_raw[i].y-track_poly[poly_point_serial].y, 2.0));
					side_tmp2 = std::sqrt(std::pow(track_raw[i].x-track_poly[poly_point_serial-1].x, 2.0)+std::pow(track_raw[i].y-track_poly[poly_point_serial-1].y, 2.0));
					if(side_tmp1 > distance_threshold && side_tmp2 > distance_threshold)
					{
						ROS_INFO("rescue filtering: %3f, (%3f, %3f), (%3f, %3f), (%3f, %3f)", max_delt_angle, track_raw[i].x, track_raw[i].y, track_poly[poly_point_serial].x, track_poly[poly_point_serial].y, track_poly[poly_point_serial-1].x, track_poly[poly_point_serial-1].y);
						return true;
					}
				}
			}
			return false;
		}
	}

	void track_processor::calc_speed_and_thetha()
	{
		for(size_t i=0; i<track_container_->tracks.size(); i++)
		{
			track_common &track = track_container_->tracks[i];
			if(track.ped_activity == MOVING)
			{
				for(size_t j=0; j<track.elements.size(); j++)
				{
					size_t element_serial = j;
					bool activity_calculated = false;

					if(element_serial>0)
					{
						for(size_t p=element_serial-1; p!=0; p--)
						{
							double distance = fmutil::distance(track.elements[element_serial], track.elements[p]);
							if(distance >= 1.0)
							{
								track.elements[element_serial].thetha = std::atan2(track.elements[element_serial].y-track.elements[p].y, track.elements[element_serial].x-track.elements[p].x);
								track.elements[element_serial].speed = distance/(track.elements[element_serial].time - track.elements[p].time);
								activity_calculated = true;
								break;
							}
						}
					}

					if(!activity_calculated)
					{
						for(size_t p=element_serial; p<track.elements.size(); p++)
						{
							double distance = fmutil::distance(track.elements[p], track.elements[element_serial]);
							if(distance >= 1.0)
							{
								track.elements[element_serial].thetha = std::atan2(track.elements[p].y-track.elements[element_serial].y, track.elements[p].x-track.elements[element_serial].x);
								track.elements[element_serial].speed  = distance/(track.elements[p].time - track.elements[element_serial].time);
								activity_calculated = true;
								break;
							}
						}
					}
				}
			}
		}

	}

	void track_processor::cluster_moving_tracks()
	{
		for(size_t i=0; i<track_container_->tracks.size(); i++)
		{
			if(track_container_->tracks[i].ped_activity != MOVING) continue;
			track_container_->tracks[i].cluster_label = 0;
		}

		vector<size_t> cluster_a, cluster_b;
		vector<track_element> elements_a, elements_b;

		//1st step: find the longest track as initial seed of cluster_a; and find its most dissimilar track as initial seed for cluster_b;

		//1.a: to get the longest track;
		size_t longest_serial = 0; double longest_length = 0;
		for(size_t i=0; i<track_container_->tracks.size(); i++)
		{
			if(track_container_->tracks[i].ped_activity != MOVING)continue;

			track_common &track = track_container_->tracks[i];
			double track_length_tmp = sqrt(pow(track.elements.front().x - track.elements.back().x, 2.0)+pow(track.elements.front().y - track.elements.back().y, 2.0));
			if(track_length_tmp > longest_length){longest_length = track_length_tmp; longest_serial = i;}
		}

		track_container_->tracks[longest_serial].cluster_label = 1;
		cluster_a.push_back(longest_serial);
		for(size_t i=0; i<track_container_->tracks[longest_serial].elements.size(); i++)
		{elements_a.push_back(track_container_->tracks[longest_serial].elements[i]);}

		//1.b: to get the most dissimilar track;
		size_t min_similar_serial = 0; double min_similar_score = DBL_MAX;
		for(size_t i=0; i<track_container_->tracks.size(); i++)
		{
			if(track_container_->tracks[i].ped_activity != MOVING)continue;

			if(track_container_->tracks[i].cluster_label!=0)continue;
			track_common &track = track_container_->tracks[i];
			double track_MaxSim, track_MinSim;
			calc_track_MaxMinSimScore(track, elements_a, track_MaxSim,  track_MinSim);
			if(track_MinSim < min_similar_score){min_similar_score = track_MinSim; min_similar_serial = i;}
		}
		track_container_->tracks[min_similar_serial].cluster_label = 2;
		cluster_b.push_back(min_similar_serial);
		for(size_t i=0; i<track_container_->tracks[min_similar_serial].elements.size(); i++)
		{elements_b.push_back(track_container_->tracks[min_similar_serial].elements[i]);}

		ROS_INFO("initial track for cluster_a: %ld; initial track for cluster_b: %ld", longest_serial, min_similar_serial);
		//2nd step: to concatenate the tracks with initial 2 seeds;
		bool track_remained_to_cluster = true;
		while(track_remained_to_cluster)
		{
			//if there is still track(s) to be clustered, cluster_a first find its most similar track, "grow from near to far";
			size_t MaxMscore_track_serial_a = 0; size_t MaxMscore_track_serial_b = 0;
			double MaxMscore_a = -DBL_MAX; double MaxMscore_b = -DBL_MAX;
			for(size_t i=0; i<track_container_->tracks.size(); i++)
			{
				if(track_container_->tracks[i].ped_activity != MOVING)continue;
				if(track_container_->tracks[i].cluster_label!=0)continue;

				track_common &track = track_container_->tracks[i];
				double track_MaxSim_a, track_MinSim_a, track_Mscore_a;
				calc_track_MaxMinSimScore(track, elements_a, track_MaxSim_a,  track_MinSim_a);
				track_Mscore_a = (track_MaxSim_a + track_MinSim_a)*0.5;
				if(track_Mscore_a > MaxMscore_a){MaxMscore_a = track_Mscore_a; MaxMscore_track_serial_a = i;}
				double track_MaxSim_b, track_MinSim_b, track_Mscore_b;
				calc_track_MaxMinSimScore(track, elements_b, track_MaxSim_b,  track_MinSim_b);
				track_Mscore_b = (track_MaxSim_b + track_MinSim_b)*0.5;
				if(track_Mscore_b > MaxMscore_b){MaxMscore_b = track_Mscore_b; MaxMscore_track_serial_b = i;}
			}

			//ROS_INFO("MaxMscore_a, MaxMscore_B %lf, %lf, %ld, %ld", MaxMscore_a, MaxMscore_b, MaxMscore_track_serial_a, MaxMscore_track_serial_b);

			if(MaxMscore_a > MaxMscore_b)
			{
				//ROS_INFO("track %ld into cluster_a", MaxMscore_track_serial_a);

				track_container_->tracks[MaxMscore_track_serial_a].cluster_label = 1;
				cluster_a.push_back(MaxMscore_track_serial_a);
				for(size_t i=0; i<track_container_->tracks[MaxMscore_track_serial_a].elements.size(); i++)
				{elements_a.push_back(track_container_->tracks[MaxMscore_track_serial_a].elements[i]);}

			}
			else
			{
				//ROS_INFO("track %ld into cluster_b", MaxMscore_track_serial_b);

				track_container_->tracks[MaxMscore_track_serial_b].cluster_label = 2;
				cluster_b.push_back(MaxMscore_track_serial_b);
				for(size_t i=0; i<track_container_->tracks[MaxMscore_track_serial_b].elements.size(); i++)
				{elements_b.push_back(track_container_->tracks[MaxMscore_track_serial_b].elements[i]);}
			}

			track_remained_to_cluster = false;
			size_t remained_track_number = 0;
			//printf("\n");
			for(size_t i=0; i<track_container_->tracks.size(); i++)
			{
				if(track_container_->tracks[i].ped_activity != MOVING)continue;
				if(track_container_->tracks[i].cluster_label == 0)
				{
					//printf("%ld\t", i);
					track_remained_to_cluster = true; remained_track_number++;
				}
			}
			//printf("\n");
			//ROS_INFO("tracks remained to be classified: %ld", remained_track_number);
		}
	}

	void track_processor::calc_track_MaxMinSimScore(track_common & track, vector<track_element> & cluster_elements, double & maxScore_track, double & minScore_track)
	{
		maxScore_track = 0.0;
		minScore_track = 0.0;
		for(size_t i=0; i<track.elements.size(); i++)
		{
			double max_score_element = -DBL_MAX;
			double min_score_element = DBL_MAX;
			track_element element_tmp = track.elements[i];
			for(size_t j=0; j<cluster_elements.size(); j++)
			{
				double similarity_score_tmp = similariry_between_elements(element_tmp, cluster_elements[j]);
				if(similarity_score_tmp > max_score_element) max_score_element = similarity_score_tmp;
				if(similarity_score_tmp < min_score_element) min_score_element = similarity_score_tmp;
			}
			maxScore_track = maxScore_track + max_score_element;
			minScore_track = minScore_track + min_score_element;
		}
	}

	double track_processor::similariry_between_elements(track_element &element_A, track_element &element_B)
	{
		double delt_angle = fabs(element_B.thetha - element_A.thetha);
		if(delt_angle>M_PI) delt_angle = M_PI*2.0-delt_angle;
		double angle_factor = (M_PI_2 - delt_angle)/M_PI_2;

		double dist2d = sqrt(pow(element_A.x-element_B.x, 2.0)+pow(element_A.y-element_B.y, 2.0));
		double param_c = 5.0;
		double dist_factor = param_c/(param_c+dist2d);

		double similarity_score = angle_factor * dist_factor;
		return(similarity_score);
	}

	//find some small bugs, the interpolated elements in the vector is not inserted in the correct places;
	//but it doesn't influence our intensity calculation at all;
	void track_processor::interpolate_elements()
	{
		//to interpolate between the track points;
		bool interpolate_flag = false;
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

