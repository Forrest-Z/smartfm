#include "global_track_show.h"

namespace golfcar_semantics{

	global_track_show::global_track_show(const char* image_path, double show_scale)
	{
		image_path_ = image_path;
		show_scale_ = show_scale;
		if((global_trajectory_img_ = cvLoadImage( image_path_, CV_LOAD_IMAGE_COLOR)) == 0){ROS_ERROR("unable to load map image");return;}
		//cvNot(global_trajectory_img_ , global_trajectory_img_);
		cvNamedWindow("global_view: ped_tracks");
	}

	void global_track_show::show_update(double x, double y, CvScalar bgr_color, bool draw_circle)
	{
		CvPoint pixel;
		pixel.x = PIC_GXWX(global_trajectory_img_, x, show_scale_);
		pixel.y = PIC_GYWY(global_trajectory_img_, y, show_scale_);
		if(!PIC_VALID(global_trajectory_img_, pixel.x, pixel.y))
		{
			return;
		}
		cvSet2D(global_trajectory_img_, pixel.y, pixel.x, bgr_color);

		if(draw_circle) cvCircle( global_trajectory_img_, pixel, 1, bgr_color, 2);
		cvShowImage("global_view: ped_tracks", global_trajectory_img_);
	}

	void global_track_show::clear_image()
	{
		if(!global_trajectory_img_) cvReleaseImage(&global_trajectory_img_);
		if((global_trajectory_img_ = cvLoadImage( image_path_, CV_LOAD_IMAGE_COLOR)) == 0){ROS_ERROR("unable to load map image");return;}
	}

	void global_track_show::save_image(const char* image_name)
	{
		cvSaveImage(image_name, global_trajectory_img_);
	}

	global_track_show::~global_track_show()
	{
		cvReleaseImage(&global_trajectory_img_);
		cvDestroyWindow("global_view: ped_tracks");
	}
};
