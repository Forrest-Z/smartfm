#include "local_track_show.h"

namespace golfcar_semantics{

	local_track_show::local_track_show(CvSize img_size, double show_scale)
	{
		image_size_ = img_size;
		show_scale_ = show_scale;
		local_trajectory_img_ = cvCreateImage( image_size_, 8, 3 );
		cvZero(local_trajectory_img_);
		lidar_center_.x = local_trajectory_img_->width/2;
		lidar_center_.y = local_trajectory_img_->height;
		draw_backGND();
		cvNamedWindow("local_view: ped_tracks");
	}

	void local_track_show::show_update(double x, double y, CvPoint & prev_point, CvScalar bgr_color)
	{
		CvPoint current_point;
		traj_pixel(x, y, current_point);
		cvCircle( local_trajectory_img_, current_point, 2, bgr_color, 2);
		if(prev_point.x >= 0 && prev_point.y>=0){cvLine(local_trajectory_img_, prev_point, current_point, bgr_color);}
		prev_point = current_point;
		cvShowImage("local_view: ped_tracks", local_trajectory_img_);
	}

	void local_track_show::draw_backGND()
	{
		CvFont font;
		double hScale=1.0;
		double vScale=1.0;
		int lineWidth=1;
		cvInitFont(&font,CV_FONT_ITALIC, hScale, vScale, 0, lineWidth);

		CvScalar text_color = CV_RGB(0,255,0);
		cvPutText(local_trajectory_img_, "x", cvPoint(lidar_center_.x-30, 20), &font, text_color);
		cvPutText(local_trajectory_img_, "y", cvPoint(5, lidar_center_.y-20), &font, text_color);

		CvScalar axis_color = CV_RGB(0,255,0);
		drawArrow(local_trajectory_img_, lidar_center_, cvPoint(lidar_center_.x, 0), 20, 30, axis_color, 1);
		drawArrow(local_trajectory_img_, lidar_center_, cvPoint(0, lidar_center_.y), 20, 30, axis_color, 1);

		cvCircle( local_trajectory_img_, lidar_center_, 2, CV_RGB(255,0,0), 2);

		CvScalar circle_color = CV_RGB(0,255,0);
		cvCircle( local_trajectory_img_, lidar_center_, int(5.0/show_scale_), circle_color, 1);
		cvCircle( local_trajectory_img_, lidar_center_, int(10.0/show_scale_), circle_color, 1);
		cvCircle( local_trajectory_img_, lidar_center_, int(15.0/show_scale_), circle_color, 1);
		cvCircle( local_trajectory_img_, lidar_center_, int(20.0/show_scale_), circle_color, 1);

		/*
		hScale=0.5;
		vScale=0.5;
		cvInitFont(&font,CV_FONT_ITALIC, hScale, vScale, 0, lineWidth);
		cvPutText(local_trajectory_img_, "5", cvPoint(lidar_center_.x-20, lidar_center_.y- 5.0/show_scale_ -20), &font, CV_RGB(255,255,255));
		cvPutText(local_trajectory_img_, "10", cvPoint(lidar_center_.x-20, lidar_center_.y-10.0/show_scale_-20), &font, CV_RGB(255,255,255));
		cvPutText(local_trajectory_img_, "15", cvPoint(lidar_center_.x-20, lidar_center_.y-15.0/show_scale_-20), &font, CV_RGB(255,255,255));
		cvPutText(local_trajectory_img_, "20", cvPoint(lidar_center_.x-20, lidar_center_.y-20.0/show_scale_-20), &font, CV_RGB(255,255,255));
		 */



	}

	void local_track_show::drawArrow(IplImage* img, CvPoint pStart, CvPoint pEnd, int len, int alpha,  CvScalar color, int thickness, int lineType)
	{
		CvPoint arrow;
		double angle = atan2((double)(pStart.y - pEnd.y), (double)(pStart.x - pEnd.x));
		cvLine(img, pStart, pEnd, color, thickness, lineType);
		arrow.x = pEnd.x + len * cos(angle + M_PI * alpha / 180);
		arrow.y = pEnd.y + len * sin(angle + M_PI * alpha / 180);
		cvLine(img, pEnd, arrow, color, thickness, lineType);
		arrow.x = pEnd.x + len * cos(angle - M_PI * alpha / 180);
		arrow.y = pEnd.y + len * sin(angle - M_PI * alpha / 180);
		cvLine(img, pEnd, arrow, color, thickness, lineType);
	}

	void local_track_show::traj_pixel(double x, double y, CvPoint &img_pt)
	{
		img_pt.y = lidar_center_.x - int(x/show_scale_);
		img_pt.x = lidar_center_.y - int(y/show_scale_);
	}

	local_track_show::~local_track_show()
	{
		cvReleaseImage(&local_trajectory_img_);
		cvDestroyWindow("local_view: ped_tracks");
	}
};
