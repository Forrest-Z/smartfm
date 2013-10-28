#include "pedestrian_collector.h"

namespace golfcar_semantics{
pedestrian_collector::pedestrian_collector():
		private_nh_("~"),
		it_(nh_)
	{
		pedestrian_sub_ = nh_.subscribe("ped_data_assoc", 10, &pedestrian_collector::pedCallback, this);

		private_nh_.param("map_scale",    				map_scale_,   				0.1);
		private_nh_.param("map_pic_path", 				map_pic_path_, 				std::string("road_map.png"));
		private_nh_.param("file_path",  				file_path_,     			std::string("ped_track.data"));
		private_nh_.param("ped_belief_threshold",    	ped_belief_threshold_,   	0.05);

		global_viewer_ = new global_track_show(map_pic_path_.c_str(), map_scale_);

		local_view_size_ = cvSize(600, 300);
		local_show_scale_ = 0.1;
		local_viewer_ = new local_track_show(local_view_size_, local_show_scale_);
	}

	void pedestrian_collector::pedCallback(const sensing_on_road::pedestrian_vision_batch::ConstPtr& ped_batch_in)
	{
		ROS_INFO("pedCallback BEGIN");
		track_updating(ped_batch_in);
		track_visualization();
		ROS_INFO("pedCallback END");
	}

	void pedestrian_collector::track_updating(const sensing_on_road::pedestrian_vision_batch::ConstPtr& ped_batch_in)
	{
		sensing_on_road::pedestrian_vision_batch ped_batch_input = *ped_batch_in;
		for(size_t i=0; i<ped_batch_input.pd_vector.size(); i++)
		{
			if(ped_batch_input.pd_vector[i].decision_flag==false) continue;

			bool find_flag = false;
			for(size_t j=0; j<ped_tracks_.size(); j++)
			{
				if(ped_batch_input.pd_vector[i].object_label == ped_tracks_[j].object_label)
				{
					 if(ped_tracks_[j].ped_confidence < ped_batch_input.pd_vector[i].confidence)
						 ped_tracks_[j].ped_confidence = ped_batch_input.pd_vector[i].confidence;

					 ped_tracks_[j].ped_track.push_back(ped_batch_input.pd_vector[i]);
					 find_flag = true;
					 break;
				}
			}
			if(!find_flag)
			{
				pedestrian_track tmp_track;
				tmp_track.object_label = ped_batch_input.pd_vector[i].object_label;
				tmp_track.ped_confidence = ped_batch_input.pd_vector[i].confidence;
				tmp_track.ped_track.push_back(ped_batch_input.pd_vector[i]);
				ped_tracks_.push_back(tmp_track);
			}
		}
	}

	void pedestrian_collector::track_visualization()
	{
		for(size_t i=0; i<ped_tracks_.size(); i++)
		{
			//to adjust this threshold for different results;
			if(ped_tracks_[i].ped_confidence < ped_belief_threshold_) continue;

			CvPoint prev_point = cvPoint(-1, -1);
			for(size_t j=0; j<ped_tracks_[i].ped_track.size(); j++)
			{
				geometry_msgs::Point32 track_pt = ped_tracks_[i].ped_track[j].cluster.centroid;
				global_viewer_->show_update(ped_tracks_[i].ped_track[j].cluster.centroid.x, ped_tracks_[i].ped_track[j].cluster.centroid.y, CV_RGB(0,0,255), false);
				local_viewer_->show_update( ped_tracks_[i].ped_track[j].local_centroid.x, ped_tracks_[i].ped_track[j].local_centroid.y, prev_point, CV_RGB(0,0,255));
			}
		}
		cvWaitKey(1);
	}


	void pedestrian_collector::track_saving()
	{
		//pay attention that above tracks are tracks of all clusters;
		//erase the low-belief tracks that are probably noise;
		for(size_t i=0; i<ped_tracks_.size(); )
		{
			if(ped_tracks_[i].ped_confidence < ped_belief_threshold_) ped_tracks_.erase(ped_tracks_.begin()+i);
			else i++;
		}

		FILE *fp_output;
	    if((fp_output=fopen(file_path_.c_str(), "a"))==NULL){ROS_ERROR("cannot open output_file\n"); return;}
	    fprintf(fp_output, "%ld", ped_tracks_.size());

		for(size_t i=0; i<ped_tracks_.size(); i++)
		{
			//the following "ped_track" just to illustrates the datatype;
			track_common ped_track;
			ped_track.confidence = ped_tracks_[i].ped_confidence;
			ped_track.track_length =  ped_tracks_[i].ped_track.size();

			fprintf(fp_output, "\n%f\t", ped_tracks_[i].ped_confidence);
			fprintf(fp_output, "%ld\n", ped_tracks_[i].ped_track.size());

			for(size_t j=0; j<ped_tracks_[i].ped_track.size(); j++)
			{
				fprintf(fp_output, "%lf\t", ped_tracks_[i].ped_track[j].cluster.last_update.toSec());

				fprintf(fp_output, "%f\t", ped_tracks_[i].ped_track[j].cluster.centroid.x);
				fprintf(fp_output, "%f\t", ped_tracks_[i].ped_track[j].cluster.centroid.y);
				fprintf(fp_output, "%f\t", ped_tracks_[i].ped_track[j].cluster.width);
				fprintf(fp_output, "%f\t", ped_tracks_[i].ped_track[j].cluster.depth);
				fprintf(fp_output, "%f\t", ped_tracks_[i].ped_track[j].local_centroid.x);
				fprintf(fp_output, "%f\t", ped_tracks_[i].ped_track[j].local_centroid.y);
			}
		}
		fclose(fp_output);
	}

	pedestrian_collector::~pedestrian_collector()
	{
		track_saving();
		delete local_viewer_;
		delete global_viewer_;
	}
};

int main(int argc, char** argv)
{
	 ros::init(argc, argv, "pedestrian_collector");
	 ros::NodeHandle n;
	 golfcar_semantics::pedestrian_collector pedestrian_collect_node;
     ros::spin();
     return 0;
}
