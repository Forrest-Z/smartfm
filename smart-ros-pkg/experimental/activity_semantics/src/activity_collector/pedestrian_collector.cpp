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

		if((visual_image_ = cvLoadImage( map_pic_path_.c_str(), CV_LOAD_IMAGE_COLOR)) == 0){ROS_ERROR("unable to load map image");}
		else {cvNamedWindow("visualization");}
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

			for(size_t j=0; j<ped_tracks_[i].ped_track.size(); j++)
			{
				geometry_msgs::Point32 track_pt = ped_tracks_[i].ped_track[j].cluster.centroid;
				CvPoint pixel;
				pixel.x = PIC_GXWX(visual_image_, track_pt.x, map_scale_);
				pixel.y = PIC_GYWY(visual_image_, track_pt.y, map_scale_);
				//ROS_INFO("pixel %d, %d", pixel.x, pixel.y);

				if(!PIC_VALID(visual_image_, pixel.x, pixel.y)) continue;
				cvSet2D(visual_image_, pixel.y, pixel.x, CV_RGB(255, 0, 0));
			}
		}
		cvShowImage("visualization", visual_image_);
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
			}
		}
		fclose(fp_output);
	}

	pedestrian_collector::~pedestrian_collector()
	{
		track_saving();
		cvDestroyWindow("visualization");
		cvReleaseImage(&visual_image_);
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
