#include "track_processor.h"

namespace golfcar_semantics{

	AM_learner::AM_learner(char* image_path, double map_scale, pd_track_container* pd_container)
	{
		map_scale_ = map_scale;
		image_path_ = image_path;

		AM_ =  map_alloc();
		AM_->pd_container_pointer = pd_container;
	}


	void AM_learner::GridMap_init()
	{
		//1st: allocate memory for the grid map;

		ROS_INFO("GridMap_init");
		Mat img = imread( image_path_, CV_LOAD_IMAGE_GRAYSCALE );	//Future work: load these parameters from a YAML file using OpenCV API;
		AM_->road_image = &img;

		if( !AM_->road_image->data) {ROS_INFO("cannot load image");return;}

		ROS_INFO("loaded");

		AM_->size_x = AM_->road_image->cols;
		AM_->size_y = AM_->road_image->rows;
		AM_->scale	= map_scale_;

		AM_->origin_x = 0.0;			//currently these 3 parameters are not in use;
		AM_->origin_y = 0.0;
		AM_->yaw = 0.0;

		AM_->cells = (activity_grid*)malloc(sizeof(activity_grid)*AM_->size_x*AM_->size_y);
		ROS_ASSERT(AM_->cells);

		//2nd: perform grid accumulation;
		ROS_INFO("perform grid accumulation");
		std::vector<track_common> &tracks =	AM_->pd_container_pointer->tracks;
		for(size_t i=0; i<tracks.size(); i++)
		{
			for(size_t j=0; j<tracks[i].elements.size(); j++)
			{
				//associate grids to tracks, including noise-tracks;
				track_element element_tmp = tracks[i].elements[j];
				int grid_x = MAP_GXWX(AM_, element_tmp.x);
				int grid_y = MAP_GYWY(AM_, element_tmp.y);
				activity_grid &grid_tmp = AM_->cells[MAP_INDEX(AM_, grid_x, grid_y)];

				//pay attention to the case where multiple points in one track fall into the same grid;
				bool grid_updated_already = false;
				for(size_t a=0; a<grid_tmp.track_serials.size(); a++)
				{
					if(grid_tmp.track_serials[a]== i) grid_updated_already = true;
				}
				if(grid_updated_already) continue;
				grid_tmp.track_serials.push_back(i);
				accumulate_grid_activity(grid_tmp, tracks[i], j);
			}
		}
	}

	void AM_learner::accumulate_grid_activity(activity_grid &grid ,track_common track, size_t element_serial)
	{
		if(track.ped_activity == MOVING)
		{
			moving_activity activity_tmp;
			activity_tmp.width = track.elements[element_serial].width;
			activity_tmp.depth = track.elements[element_serial].depth;

			bool activity_calculated = false;
			if(element_serial>0)
			{
				for(size_t i=element_serial-1; i>=0; i--)
				{
					double distance = fmutil::distance(track.elements[element_serial], track.elements[i]);
					if(distance >= 0.3)
					{
						activity_tmp.thetha = std::atan2(track.elements[element_serial].y-track.elements[i].y, track.elements[element_serial].x-track.elements[i].x);
						if(activity_tmp.thetha<0) activity_tmp.thetha = activity_tmp.thetha + M_PI;

						activity_tmp.speed = distance/(track.elements[element_serial].time - track.elements[i].time);
						activity_calculated = true;
						break;
					}
				}
			}
			if(!activity_calculated)
			{
				for(size_t i=element_serial; i<track.elements.size(); i++)
				{
					double distance = fmutil::distance(track.elements[i], track.elements[element_serial]);
					if(distance >= 0.3)
					{
						activity_tmp.thetha = std::atan2(track.elements[i].y-track.elements[element_serial].y, track.elements[i].x-track.elements[element_serial].x);
						if(activity_tmp.thetha<0) activity_tmp.thetha = activity_tmp.thetha + M_PI;

						activity_tmp.speed = distance/(track.elements[i].time - track.elements[element_serial].time);
						break;
					}
				}
			}
			grid.moving_activities.push_back(activity_tmp);
		}
		else if(track.ped_activity == STATIC)
		{
			static_activity activity_tmp;
			activity_tmp.width = track.elements[element_serial].width;
			activity_tmp.depth = track.elements[element_serial].depth;
			activity_tmp.dwell_time =  track.elements.back().time - track.elements.front().time;
			grid.static_activities.push_back(activity_tmp);
		}
	}

	void AM_learner::learn_activity_map()
	{
		learn_moving_direction();
	}

	void AM_learner::learn_moving_direction()
	{
		unsigned int i, j;
		for(j = 0; j < (unsigned int) AM_->size_y; j++)
		{
			for (i = 0; i < (unsigned int) AM_->size_x; i++)
			{
				activity_grid &grid_tmp = AM_->cells[MAP_INDEX(AM_, i, j)];
				if(grid_tmp.moving_activities.size()==0)continue;

				Mat direction_mat( (int)grid_tmp.moving_activities.size(), 1, CV_64FC1 );

		    	for(size_t a=0; a<grid_tmp.moving_activities.size(); a++)
		    	{
		    		direction_mat.at<double>(a, 0) =  grid_tmp.moving_activities[a].thetha;
		    	}
		    	Mat direction_cov, direction_mean;
		    	calcCovarMatrix(direction_mat, direction_cov, direction_mean, CV_COVAR_NORMAL|CV_COVAR_ROWS|CV_COVAR_SCALE);
		    	grid_tmp.direction_gaussion.val[0] = direction_mean.at<double>(0,0);
		    	grid_tmp.direction_gaussion.val[1] = direction_cov.at<double>(0,0);
			}
		}
	}

	void AM_learner::view_activity_map()
	{
		show_moving_direction();
	}

	void AM_learner::show_moving_direction()
	{
		Mat direction_color(AM_->size_y,  AM_->size_x, CV_8UC1 );
		direction_color = Scalar(0);

		unsigned int i, j;
		for(j = 0; j < (unsigned int) AM_->size_y; j++)
		{
			for (i = 0; i < (unsigned int) AM_->size_x; i++)
			{
				activity_grid &grid_tmp = AM_->cells[MAP_INDEX(AM_, i, j)];
				if(grid_tmp.moving_activities.size()==0)continue;
				int x = i;
				int y = AM_->size_y-1-j;
				direction_color.at<uchar>(y,x) = floor(grid_tmp.direction_gaussion.val[0]/M_PI *255.0);
			}
		}
		Mat jetcolor;
		applyColorMap(direction_color, jetcolor, COLORMAP_JET);

		Mat mask;
		cvtColor(direction_color, mask, CV_GRAY2RGB);
		bitwise_and(jetcolor, mask, jetcolor);
		imshow("direction_map", jetcolor);

		waitKey(0);
	}



	AM_learner::~AM_learner()
	{
		if( AM_ != NULL )
		{
			map_free( AM_ );
			AM_ = NULL;
		}
	}
};

