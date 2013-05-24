#include "ped_semantics.h"

namespace golfcar_semantics{

	ped_semantics::ped_semantics(char* parameter_file)
	{
		parameter_file_ = parameter_file;
		parameter_init();

		track_container_ = new pd_track_container();
		pedtrack_loading();

		roadmap_loading();
		global_viewer_ = new global_track_show(image_path_.c_str(), map_scale_);
		local_view_size_ = cvSize(600, 300);
		local_show_scale_ = 0.1;
		local_viewer_ = new local_track_show(local_view_size_, local_show_scale_);

		activity_track_processor_ = new track_processor(track_container_, track_size_thresh_, track_time_thresh_, track_length_thresh_);
		activity_map_learner_ = new AM_learner(image_path_.c_str(), map_scale_, track_container_);
	}

	void ped_semantics::parameter_init()
	{
		ROS_INFO("parameter initialization for pedestrian_semantics");
		FileStorage fs_read(parameter_file_, FileStorage::READ);
		if(!fs_read.isOpened())ROS_ERROR("ped_semantics cannot find parameter file");

		string image_path, track_file_path;
		fs_read["image_path"] >> image_path_;
		fs_read["track_file_path"] >> track_file_path_;
		map_scale_ = (double) fs_read["map_scale"];

		//FileNode only supports int, and no "size_t"; have to perform conversion;
		track_size_thresh_ = size_t((int) fs_read["track_size_thresh"]);

		track_time_thresh_ = (double) fs_read["track_time_thresh"];
		track_length_thresh_ = (double) fs_read["track_length_thresh"];
	}

	void ped_semantics::semantics_learning()
	{
		//1st function: extraction pedestrian Entrance-Exits;
		ped_EE_extraction();

		//to be developed;
	}

	void ped_semantics::roadmap_loading()
	{
		if((road_image_ = cvLoadImage( image_path_.c_str(), CV_LOAD_IMAGE_GRAYSCALE)) == 0){ROS_ERROR("unable to load map image");return;}

		distance_image_ = cvCreateImage( cvGetSize(road_image_), IPL_DEPTH_32F, 1 );
		visualize_image_ = cvCreateImage( cvGetSize(road_image_), 8, 3 );
		cvCvtColor(road_image_, visualize_image_, CV_GRAY2RGB);

		int binary_threshold_value = 120;
		cvThreshold(road_image_, road_image_, binary_threshold_value, 255, CV_THRESH_BINARY);

		//apply distance transform;
		cvDistTransform( road_image_, distance_image_, CV_DIST_L2, 3);
	}

	//initialize "track_container_->tracks";
	void ped_semantics::pedtrack_loading()
	{
		FILE *fp_input;
		if((fp_input=fopen(track_file_path_.c_str(), "r"))==NULL){ROS_ERROR("cannot open output_file\n"); return;}

		size_t track_number;
		fscanf(fp_input,  "%ld", &track_number);
		track_container_->tracks.resize(track_number);

		for(size_t i=0; i<track_container_->tracks.size(); i++)
		{
			track_common &ped_track = track_container_->tracks[i];
			fscanf(fp_input, "\n%lf\t", &ped_track.confidence);
			fscanf(fp_input, "%ld\n", &ped_track.track_length);
			ped_track.elements.resize(ped_track.track_length);
			for(size_t j=0; j<ped_track.elements.size(); j++)
			{
				fscanf(fp_input, "%lf\t", &ped_track.elements[j].time);
				fscanf(fp_input, "%lf\t", &ped_track.elements[j].x);
				fscanf(fp_input, "%lf\t", &ped_track.elements[j].y);
				fscanf(fp_input, "%lf\t", &ped_track.elements[j].width);
				fscanf(fp_input, "%lf\t", &ped_track.elements[j].depth);
				fscanf(fp_input, "%lf\t", &ped_track.elements[j].local_x);
				fscanf(fp_input, "%lf\t", &ped_track.elements[j].local_y);
			}
		}
		fclose(fp_input);
	}

	void ped_semantics::ped_EE_extraction()
	{
		ROS_INFO("ped_EE_extraction");

		//1st, visualize all the input tracks;
		raw_track_show();

		//2nd, perform track classification;
		activity_track_processor_->ped_track_classification();
		processed_track_show();

		//3nd, learn activity map given tracks;
		activity_map_learner_->GridMap_init();
		activity_map_learner_->learn_activity_map();
		activity_map_learner_->view_activity_map();



		//analyze moving tracks;
		//use road maps as prior knowledge;
		//apply GMM to extract entrances and exits;
	}

	void ped_semantics::raw_track_show()
	{
		for(size_t i=0; i<track_container_->tracks.size(); i++)
		{
			CvPoint prev_point = cvPoint(-1, -1);
			CvScalar ext_color = CV_RGB( rand()&255, rand()&255, rand()&255 );
			for(size_t j=0; j<track_container_->tracks[i].elements.size(); j++)
			{
				global_viewer_->show_update(track_container_->tracks[i].elements[j].x, track_container_->tracks[i].elements[j].y, ext_color, false);
				local_viewer_->show_update( track_container_->tracks[i].elements[j].local_x, track_container_->tracks[i].elements[j].local_y, prev_point, ext_color);
				cvWaitKey(10);
			}
		}
		global_viewer_->save_image("./data/raw_global.png");
		local_viewer_->save_image("./data/raw_local.png");
		cvWaitKey(0);

		global_viewer_->clear_image();
		local_viewer_->clear_image();
	}

	void ped_semantics::processed_track_show()
	{
		CvPoint prev_point = cvPoint(-1,-1);
		for(size_t i=0; i<track_container_->tracks.size(); i++)
		{
			if(track_container_->tracks[i].ped_activity == MOVING)
			{
				for(size_t j=0; j<track_container_->tracks[i].elements.size(); j++)
				{
					global_viewer_->show_update(track_container_->tracks[i].elements[j].x, track_container_->tracks[i].elements[j].y, CV_RGB(0,0,255), false);
					cvWaitKey(10);
				}
				global_viewer_->show_update(track_container_->tracks[i].elements.front().x, track_container_->tracks[i].elements.front().y, CV_RGB(255,0,0), true);
				global_viewer_->show_update(track_container_->tracks[i].elements.back().x, track_container_->tracks[i].elements.back().y, CV_RGB(255,0,0), true);

				prev_point = cvPoint(-1,-1);
				local_viewer_->show_update( track_container_->tracks[i].elements.front().local_x, track_container_->tracks[i].elements.front().local_y, prev_point, CV_RGB(0,255,0));
				prev_point = cvPoint(-1,-1);
				local_viewer_->show_update( track_container_->tracks[i].elements.back().local_x, track_container_->tracks[i].elements.back().local_y, prev_point, CV_RGB(255,0,0));

				int track_label = -1;
				//cvWaitKey(100);
				//printf("please key in the track label as training data");
				//scanf("%d", &track_label);
				track_container_->tracks[i].track_label = track_label;
			}
			else if(track_container_->tracks[i].ped_activity  == STATIC)
			{
				for(size_t j=0; j<track_container_->tracks[i].elements.size(); j++)
				{
					global_viewer_->show_update(track_container_->tracks[i].elements[j].x, track_container_->tracks[i].elements[j].y, CV_RGB(255,255,0), true);
				}
			}
			else if(track_container_->tracks[i].ped_activity == NOISE) continue;
		}

		global_viewer_->save_image("./data/processed_global.png");
		local_viewer_->save_image("./data/processed_local.png");
		cvWaitKey(0);
	}



	ped_semantics::~ped_semantics()
	{
		delete local_viewer_;
		delete global_viewer_;
	}
};

int main(int argc, char** argv)
{
	if (argc < 2)
	{
		cout << "Please run with -? for runtime options.\n";
		exit (0);
	}

	char *para_file_path;

	for (int i=1;i<argc;i++)
	{
	    if (0==strcmp(argv[i],"-?")) {
	      cout << "\nTo run : ./bin/ped_semantics -para-file (this file contains all the parameters to use)\n";
	      exit (0);
	    }
	    else if (i+1 < argc)
	    {
		  if (0==strcmp(argv[i],"-para-file"))
			  para_file_path=argv[++i];
		}
		else {
		  cout << "Incorrect parameter lists.\n";
		  cout << "Please run with -? for runtime options.\n";
		  exit(0);
		}
	  }

	golfcar_semantics::ped_semantics ped_semantics_node(para_file_path);
	ped_semantics_node.semantics_learning();
	return 0;
}
