#include "AM_learner.h"

namespace golfcar_semantics{

	AM_learner::AM_learner(const char* image_path, double map_scale, pd_track_container* pd_container, road_semantics* road_semantics_analyzer)
	{
		map_scale_ = map_scale;
		image_path_ = image_path;

		AM_ =  map_alloc();
		AM_->pd_container_pointer = pd_container;

		gp_file_ = "./launch/gp_file.yaml";
		road_semantics_analyzer_ = road_semantics_analyzer;
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
			activity_tmp.track_label =  track.track_label;
			activity_tmp.width = track.elements[element_serial].width;
			activity_tmp.depth = track.elements[element_serial].depth;

			bool activity_calculated = false;
			if(element_serial>0)
			{
				for(size_t i=element_serial-1; i<element_serial; i--)
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
		    		//direction_mat.at<double>(a, 0) =  grid_tmp.moving_activities[a].thetha;
		    		direction_mat.at<double>(a, 0) =  sin(grid_tmp.moving_activities[a].thetha);
		    	}
		    	Mat direction_cov, direction_mean;

		    	//there remains a problem for angle variance calculation:
		    	//for example, 1 degree and 189 degree should be quite the same in term of direction, and their difference should be very small;
		    	calcCovarMatrix(direction_mat, direction_cov, direction_mean, CV_COVAR_NORMAL|CV_COVAR_ROWS|CV_COVAR_SCALE);
		    	grid_tmp.direction_gaussion.val[0] = direction_mean.at<double>(0,0);
		    	grid_tmp.direction_gaussion.val[1] = direction_cov.at<double>(0,0);
			}
		}

		//store the direction information into file, to be used for GP regression in Matlab (current version);
		record_map_into_file();

		GP_learning();
	}

	void AM_learner::GP_learning()
	{
		//1st: load GP results from Matlab, stored as pictures; to be replaced in the future by GP using C++;
		gp_file_ = "./launch/gp_file.yaml";
		FileStorage fs_read(gp_file_, FileStorage::READ);
		if(!fs_read.isOpened())ROS_ERROR("ped_semantics cannot find parameter file");

		gp_ROI_.x = (int)fs_read["gp_ROI_x"];
		gp_ROI_.y = (int)fs_read["gp_ROI_y"];
		gp_ROI_.width = (int)fs_read["gp_ROI_width"];
		gp_ROI_.height = (int)fs_read["gp_ROI_height"];
		double mean_min = (double)fs_read["mean_min"];
		double mean_max = (double)fs_read["mean_max"];
		double var_min = (double)fs_read["var_min"];
		double var_max = (double)fs_read["var_max"];

		string gpMean_path, gpVar_path;
		fs_read["gpMean_path"]>> gpMean_path;
		fs_read["gpVar_path"]>> gpVar_path;
		Mat gpMean = imread( gpMean_path, CV_LOAD_IMAGE_GRAYSCALE );
		Mat gpVar  = imread( gpVar_path,   CV_LOAD_IMAGE_GRAYSCALE );
		double mean_ratio = (mean_max-mean_min)/256.0;
		//pay attention here, may not cover the full range;
		double var_ratio  = (var_max-var_min)/256.0;

		//incorporate the information into "activity_map";
		int i, j;
		for(j = 0; j < gp_ROI_.height; j++)
		{
			for (i = 0; i < gp_ROI_.width; i++)
			{
				int roi_x = i+gp_ROI_.x;
				int roi_y = gp_ROI_.height-1-j+gp_ROI_.y;

				activity_grid &grid_tmp = AM_->cells[MAP_INDEX(AM_, roi_x, roi_y)];
		    	grid_tmp.gp_estimation.val[0] = double(gpMean.at<uchar>(j,i))*mean_ratio+mean_min;
		    	grid_tmp.gp_estimation.val[1] = double(gpVar.at<uchar>(j,i))*var_ratio+var_min;
			}
		}

		//2nd: use distance transform to calculate the nearest obstacle for free cells;
		string binary_img_path;
		fs_read["binary_img_path"]>> binary_img_path;
		Mat binary_img = imread(binary_img_path, 0);

		//Mat test_show_img(binary_img.rows,binary_img.cols,CV_8UC1);
		for(j = 0; j < AM_->size_y; j++)
		{
			for (i = 0; i < AM_->size_x; i++)
			{
				activity_grid &grid_tmp = AM_->cells[MAP_INDEX(AM_, i, j)];
				//when try to read information from image, or write information as image, pay attention to the upside down coordinate;
				if(binary_img.at<uchar>(AM_->size_y-1-j,i)> 127)
				{
					//test_show_img.at<uchar>(AM_->size_y-1-j,i)= 255;
					grid_tmp.road_flag = true;
				}
				else
				{
					//test_show_img.at<uchar>(AM_->size_y-1-j,i)= 0;
					grid_tmp.road_flag = false;
				}
			}
		}
		//imshow("testshow", test_show_img);
		//waitKey(0);

		Mat dist_img;
		distanceTransform(binary_img, dist_img, CV_DIST_L2, 3);
		for(j = 0; j < AM_->size_y; j++)
		{
			for (i = 0; i < AM_->size_x; i++)
			{
				activity_grid &grid_tmp = AM_->cells[MAP_INDEX(AM_, i, j)];
				if(!grid_tmp.road_flag) grid_tmp.obs_dist = 0.0;
				grid_tmp.obs_dist = dist_img.at<float>(AM_->size_y-1-j,i)*map_scale_;
			}
		}

		//3rd: calculate the direction of nearest edge;
		topo_graph &road_network = road_semantics_analyzer_-> topology_extractor_ ->road_graph_;

		for(j = 0; j < gp_ROI_.height; j++)
		{
			for (i = 0; i < gp_ROI_.width; i++)
			{
				int roi_x = i+gp_ROI_.x;
				int roi_y = j+gp_ROI_.y;
				activity_grid &grid_tmp = AM_->cells[MAP_INDEX(AM_, roi_x, roi_y)];

				//printf("grid_tmp.obs_dist %f\t", grid_tmp.obs_dist);
				if(grid_tmp.obs_dist < 0.01) continue;

				double grid_realx = double(roi_x)*map_scale_;
				double grid_realy = double(roi_y)*map_scale_;

				//find the nearest edge, and the corresponding point;
				double nearest_dist = DBL_MAX;
				int    nearest_edgeID = -1;
				int    nearest_ptID   = -1;

				for(size_t ie=0; ie<road_network.edges.size();ie++)
				{
					double current_edge_nearestDist = DBL_MAX;
					int  current_edge_nearestPt= -1;
					for(size_t ip=0; ip<road_network.edges[ie].points.size();ip++)
					{
						double real_x = (double)road_network.edges[ie].points[ip].x*map_scale_;
						double real_y = (double)(AM_->size_y-1-road_network.edges[ie].points[ip].y)*map_scale_;

						double dist_tmp = sqrt((grid_realy-real_y)*(grid_realy-real_y)+(grid_realx-real_x)*(grid_realx-real_x));
						if(dist_tmp<current_edge_nearestDist)
						{
							current_edge_nearestPt = ip;
							current_edge_nearestDist = dist_tmp;
						}
					}
					if(current_edge_nearestDist<nearest_dist)
					{
						nearest_edgeID = ie;
						nearest_ptID = current_edge_nearestPt;
						nearest_dist = current_edge_nearestDist;
					}
				}

				//to calculate the edge angle;
				double edge_angle = 0.0;
				int left_end = nearest_ptID;
				int right_end = nearest_ptID;
				double dist_between_ends = 0.0;
				double dist_threshold = 2.0;
				bool reach_two_ends = false;
				while(!(reach_two_ends || dist_between_ends>dist_threshold))
				{
					reach_two_ends = true;
					if(left_end>0){left_end--;reach_two_ends = false;}
					if(right_end+1<(int)road_network.edges[nearest_edgeID].points.size()){right_end++; reach_two_ends = false;}

					double real_x1 = (double)road_network.edges[nearest_edgeID].points[left_end].x*map_scale_;
					double real_y1 = (double)(AM_->size_y-1-road_network.edges[nearest_edgeID].points[left_end].y)*map_scale_;
					double real_x2 = (double)road_network.edges[nearest_edgeID].points[right_end].x*map_scale_;
					double real_y2 = (double)(AM_->size_y-1-road_network.edges[nearest_edgeID].points[right_end].y)*map_scale_;

					dist_between_ends = sqrt((real_y1-real_y2)*(real_y1-real_y2)+(real_x1-real_x2)*(real_x1-real_x2));
					edge_angle = atan2(real_y1-real_y2, real_x1-real_x2);
					if(edge_angle < 0.0) edge_angle = edge_angle + M_PI;
				}

				//grid_tmp.skel_angle = sin(edge_angle);
				grid_tmp.skel_angle = sin(edge_angle);
				//printf("dist %f skel_angle %f\t", dist_between_ends, grid_tmp.skel_angle);
			}
		}


		//4th: synthesize above 3 information sources;
		for(j = 0; j < gp_ROI_.height; j++)
		{
			for (i = 0; i < gp_ROI_.width; i++)
			{
				int roi_x = i+gp_ROI_.x;
				int roi_y = j+gp_ROI_.y;
				activity_grid &grid_tmp = AM_->cells[MAP_INDEX(AM_, roi_x, roi_y)];
				if(!grid_tmp.road_flag)
				{
					grid_tmp.EE_score = 0.0;
					continue;
				}

				double distance_factor;
				if(grid_tmp.obs_dist<2.0) distance_factor = 2.0-grid_tmp.obs_dist;
				else distance_factor = 0.0;
				//distance_factor = 1.0;

				double direction_factor;
				direction_factor = fabs(grid_tmp.skel_angle-grid_tmp.gp_estimation.val[0]);
				//direction_factor = 1.0;

				double directionVar_factor;
				directionVar_factor = 1.0 - 1.0/(var_max-var_min)*(grid_tmp.gp_estimation.val[1]-var_min);

				double intensity_factor;
				intensity_factor = 1.0+double(grid_tmp.moving_activities.size());
				//intensity_factor = 1.0;

				grid_tmp.EE_score = distance_factor*direction_factor*directionVar_factor*intensity_factor;
			}
		}

		Mat EE_color(AM_->size_y,  AM_->size_x, CV_32FC1 );
		for(j = 0; j < AM_->size_y; j++)
		{
			for (i = 0; i < AM_->size_x; i++)
			{
				activity_grid &grid_tmp = AM_->cells[MAP_INDEX(AM_, i, j)];
				if(grid_tmp.obs_dist < 0.01) continue;
				int x = i;
				int y = AM_->size_y-1-j;
				EE_color.at<float>(y,x) = (float)(grid_tmp.EE_score);
				//if (grid_tmp.obs_dist>0.1) skeldirection_color.at<float>(y,x) = 1.0;
			}
		}
		normalize(EE_color, EE_color, 0.0, 1.0, NORM_MINMAX);
		imshow("EE_color", EE_color);
		waitKey();
		imwrite( "./data/EE_color.jpg", EE_color );
	}


	void AM_learner::view_activity_map()
	{
		show_moving_direction();
	}

	void AM_learner::show_moving_direction()
	{
		//Mat direction_color(AM_->size_y,  AM_->size_x, CV_8UC1 );
		//Mat directionVar_color(AM_->size_y,  AM_->size_x, CV_8UC1 );

		Mat direction_color(AM_->size_y,  AM_->size_x, CV_32FC1 );
		Mat directionVar_color(AM_->size_y,  AM_->size_x, CV_32FC1 );

		direction_color = Scalar(0);
		directionVar_color =  Scalar(0);

		unsigned int i, j;
		for(j = 0; j < (unsigned int) AM_->size_y; j++)
		{
			for (i = 0; i < (unsigned int) AM_->size_x; i++)
			{
				activity_grid &grid_tmp = AM_->cells[MAP_INDEX(AM_, i, j)];
				//if(grid_tmp.moving_activities.size()==0)continue;
				int x = i;
				int y = AM_->size_y-1-j;
				//direction_color.at<float>(y,x) = (float)(grid_tmp.direction_gaussion.val[0]);
				//directionVar_color.at<float>(y,x)  = (float)sqrt(grid_tmp.direction_gaussion.val[1]);

				direction_color.at<float>(y,x) = (float)(grid_tmp.gp_estimation.val[0]);
				directionVar_color.at<float>(y,x)  = (float)sqrt(grid_tmp.gp_estimation.val[1]);
			}
		}

		double minVal, maxVal;
		minMaxLoc(direction_color, &minVal, &maxVal); //find minimum and maximum intensities
		Mat draw1;
		direction_color.convertTo(draw1, CV_8U, 255.0/(maxVal - minVal), -minVal * 255.0/(maxVal - minVal));
		minMaxLoc(directionVar_color, &minVal, &maxVal); //find minimum and maximum intensities
		Mat draw2;
		directionVar_color.convertTo(draw2, CV_8U, 255.0/(maxVal - minVal), -minVal * 255.0/(maxVal - minVal));

		Mat jetcolor, jetcolorVar;
		applyColorMap(draw1, jetcolor, COLORMAP_JET);
		applyColorMap(draw2, jetcolorVar, COLORMAP_JET);

		/*
		Mat mask;
		cvtColor(direction_color, mask, CV_GRAY2RGB);
		bitwise_and(jetcolor, mask, jetcolor);
		bitwise_and(jetcolorVar, mask, jetcolorVar);
		 */

		imshow("direction_map", jetcolor);
		imshow("directionVar_map", jetcolorVar);

		imwrite( "./data/direction_map.png", 	jetcolor );
		imwrite( "./data/directionVar_map.png", jetcolorVar );
		waitKey(0);
	}

	void AM_learner::record_map_into_file()
	{
		int cell_number=0;
		unsigned int i, j;
		for(j = 0; j < (unsigned int) AM_->size_y; j++)
		{
			for (i = 0; i < (unsigned int) AM_->size_x; i++)
			{
				activity_grid &grid_tmp = AM_->cells[MAP_INDEX(AM_, i, j)];
				if(grid_tmp.moving_activities.size()!=0) cell_number ++;
			}
		}

		FILE *fp_output;
	    if((fp_output=fopen("map_data.txt", "a"))==NULL){ROS_ERROR("cannot open output_file\n"); return;}
	    fprintf(fp_output, "%d", cell_number);

		for(j = 0; j < (unsigned int) AM_->size_y; j++)
		{
			for (i = 0; i < (unsigned int) AM_->size_x; i++)
			{
				activity_grid &grid_tmp = AM_->cells[MAP_INDEX(AM_, i, j)];
				if(grid_tmp.moving_activities.size()==0)continue;
				fprintf(fp_output, "\n%ld\t%ld\t%ld\t", i, j, grid_tmp.moving_activities.size());
				for(size_t k=0; k<grid_tmp.moving_activities.size(); k++)
				{
					fprintf(fp_output, "%lf\t", grid_tmp.moving_activities[k].speed);
					fprintf(fp_output, "%lf\t", grid_tmp.moving_activities[k].thetha);
				}
			}
		}
		fclose(fp_output);
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

