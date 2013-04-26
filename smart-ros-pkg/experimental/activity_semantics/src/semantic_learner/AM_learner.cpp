#include "AM_learner.h"

namespace golfcar_semantics{

	AM_learner::AM_learner(char* image_path, double map_scale, pd_track_container* pd_container)
	{
		map_scale_ = map_scale;
		image_path_ = image_path;

		AM_ =  map_alloc();
		AM_->pd_container_pointer = pd_container;
		GridMap_init();
	}


	void AM_learner::GridMap_init()
	{
		//Future work: load these parameters from a YAML file using OpenCV API;
		AM_->road_image = imread( image_path_, CV_LOAD_IMAGE_COLOR );
		AM_->size_x = AM_->road_image.cols;
		AM_->size_y = AM_->road_image.rows;
		AM_->scale	= map_scale_;

		//currently these 3 parameters are not in use;
		AM_->origin_x = 0.0;
		AM_->origin_y = 0.0;
		AM_->yaw = 0.0;

		AM_->cells = (activity_grid*)malloc(sizeof(activity_grid)*AM_->size_x*AM_->size_y);
		ROS_ASSERT(AM_->cells);
	}

	void AM_learner::learn_activity_map()
	{
		learn_moving_direction();
	}

	void AM_learner::learn_moving_direction()
	{

	}

	void AM_learner::view_activity_map()
	{
		show_moving_direction();
	}

	void AM_learner::show_moving_direction()
	{

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

