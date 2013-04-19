#include "ped_semantics.h"

namespace golfcar_semantics{

	ped_semantics::ped_semantics(char* image_path, char* file_path, double map_scale, double track_length_threshold)
	{
		map_scale_ = map_scale;
		image_path_ = image_path;
		file_path_ 	= file_path;

		track_size_thresh_ = 20;
		track_time_thresh_ = 1.0;
		track_length_thresh_ = track_length_threshold;

		roadmap_loading();
		pedtrack_loading();
	}

	void ped_semantics::semantics_learning()
	{
		//1st function: extraction pedestrian Entrance-Exits;
		ped_EE_extraction();

		//to be developed;
	}

	void ped_semantics::roadmap_loading()
	{
		if((road_image_ = cvLoadImage( image_path_, CV_LOAD_IMAGE_GRAYSCALE)) == 0){ROS_ERROR("unable to load map image");return;}

		distance_image_ = cvCreateImage( cvGetSize(road_image_), IPL_DEPTH_32F, 1 );
		visualize_image_ = cvCreateImage( cvGetSize(road_image_), 8, 3 );
		cvCvtColor(road_image_, visualize_image_, CV_GRAY2RGB);

		int binary_threshold_value = 120;
		cvThreshold(road_image_, road_image_, binary_threshold_value, 255, CV_THRESH_BINARY);

		//apply distance transform;
		cvDistTransform( road_image_, distance_image_, CV_DIST_L2, 3);
	}

	//initialize "ped_tracks_";
	void ped_semantics::pedtrack_loading()
	{
		FILE *fp_input;
		if((fp_input=fopen(file_path_, "r"))==NULL){ROS_ERROR("cannot open output_file\n"); return;}

		size_t track_number;
		fscanf(fp_input,  "%ld", &track_number);
		ped_tracks_.resize(track_number);

		for(size_t i=0; i<ped_tracks_.size(); i++)
		{
			track_common &ped_track = ped_tracks_[i];
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
			}
		}
		fclose(fp_input);
	}

	void ped_semantics::ped_EE_extraction()
	{
		ROS_INFO("ped_EE_extraction");

		//classify tracks;
		ped_track_classification();

		track_visualization(ped_moving_tracks_, CV_RGB(255, 0, 0), true);
		cvSaveImage("./data/visualization.jpg", visualize_image_);

		track_visualization(ped_static_tracks_, CV_RGB(255, 255, 0), false);

		//analyze moving tracks;
		for(size_t i=0; i<ped_moving_tracks_.size(); i++)
		{
		}
		//use road maps as prior knowledge;
		//apply GMM to extract entrances and exits;
	}

	void ped_semantics::ped_track_classification()
	{
		for(size_t i=0; i<ped_tracks_.size(); i++)
		{
			track_common &ped_track = ped_tracks_[i];
			ROS_INFO("elements size %ld, track time %lf", ped_track.elements.size(), ped_track.elements.back().time-ped_track.elements.front().time);

			if(ped_track.elements.size() >= track_size_thresh_ && (ped_track.elements.back().time-ped_track.elements.front().time > track_time_thresh_))
			{
				double track_length = 0.0 ;
				/*
				for(size_t j=1; j<ped_track.elements.size(); j++)
				{
					double distance_between_elements = fmutil::distance(ped_track.elements[j-1].x, ped_track.elements[j-1].y, ped_track.elements[j].x, ped_track.elements[j].y);
					track_length = track_length + distance_between_elements;
				}
				*/
				track_length = fmutil::distance(ped_track.elements.front().x, ped_track.elements.front().y, ped_track.elements.back().x, ped_track.elements.back().y);

				if(track_length >= track_length_thresh_) ped_moving_tracks_.push_back(ped_track);
				else ped_static_tracks_.push_back(ped_track);
			}
			else
			{
				//tracks classified as noise;
			}
		}

		ROS_INFO("total_tracks: %ld, moving_tracks: %ld, static_tracks: %ld", ped_tracks_.size(), ped_moving_tracks_.size(), ped_static_tracks_.size());
	}

	void ped_semantics::track_visualization(vector<track_common>& ped_tracks, CvScalar color, bool plot_ends)
	{
		for(size_t i=0; i<ped_tracks.size(); i++)
		{
			for(size_t j=0; j<ped_tracks[i].elements.size(); j++)
			{
				CvPoint pixel;
				pixel.x = PIC_GXWX(visualize_image_, ped_tracks[i].elements[j].x, map_scale_);
				pixel.y = PIC_GYWY(visualize_image_, ped_tracks[i].elements[j].y, map_scale_);
				//ROS_INFO("pixel %d, %d", pixel.x, pixel.y);

				if(!PIC_VALID(visualize_image_, pixel.x, pixel.y)) continue;
				cvSet2D(visualize_image_, pixel.y, pixel.x, color);
			}

			if(plot_ends)
			{
				CvPoint pixel_begin, pixel_end;
				pixel_begin.x = PIC_GXWX(visualize_image_, ped_tracks[i].elements.front().x, map_scale_);
				pixel_begin.y = PIC_GYWY(visualize_image_, ped_tracks[i].elements.front().y, map_scale_);
				pixel_end.x = PIC_GXWX(visualize_image_, ped_tracks[i].elements.back().x, map_scale_);
				pixel_end.y = PIC_GYWY(visualize_image_, ped_tracks[i].elements.back().y, map_scale_);
				cvCircle( visualize_image_, pixel_begin, 1, CV_RGB(0,0,255), 1);
				cvCircle( visualize_image_, pixel_end, 1, CV_RGB(0,0,255), 1);
			}
		}
		cvShowImage("pedestrian_track", visualize_image_);
		ROS_INFO("track visualization");
		cvWaitKey(0);
	}

	ped_semantics::~ped_semantics()
	{

	}
};

int main(int argc, char** argv)
{
	if (argc < 2)
	{
		cout << "Please run with -? for runtime options.\n";
		exit (0);
	}

	double map_scale = 0.1;
	double track_length_threshold = 1.0;
	char *image_path, *file_path;

	for (int i=1;i<argc;i++)
	{
	    if (0==strcmp(argv[i],"-?")) {
	      cout << "\nTo run : ./bin/ped_semantics -image-file FILENAME1 <options>\n\n";
	      cout << "options (see README for details):\n";
	      cout << "-data-file FILENAME2\n";
	      cout << "-image-scale m/pixel \n";
	      cout << "-length-thresh \n";
	      exit (0);
	    }
	    else if (i+1 < argc)
	    {
		  if (0==strcmp(argv[i],"-image-file"))
			  image_path=argv[++i];
		  else if (0==strcmp(argv[i],"-data-file"))
			  file_path	=argv[++i];
		  else if (0==strcmp(argv[i],"-image-scale"))
			  map_scale =atof(argv[++i]);
		  else if (0==strcmp(argv[i],"-length-thresh"))
			  track_length_threshold=atof(argv[++i]);
		  else {
				cout << "Incorrect parameter list.\n";
				cout << "Please run with -? for runtime options.\n";
				exit(0);
		  	   }
		}
		else {
		  cout << "Incorrect parameter lists.\n";
		  cout << "Please run with -? for runtime options.\n";
		  exit(0);
		}
	  }

	golfcar_semantics::ped_semantics ped_semantics_node(image_path, file_path, map_scale, track_length_threshold);
	ped_semantics_node.semantics_learning();
	return 0;
}
