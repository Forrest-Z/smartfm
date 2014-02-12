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

		//roi for McD area;
		visual_ROI_ = Rect(424, 510, 338, 100);

		//roi for crossing area;
		//visual_ROI_ = Rect(100, 490, 90, 90);
	}

	void AM_learner::GridMap_init()
	{
		//1st: allocate memory for the grid map;

		ROS_INFO("GridMap_init");
		road_image_ = imread( image_path_, CV_LOAD_IMAGE_COLOR );	//Future work: load these parameters from a YAML file using OpenCV API;
		AM_->road_image = &road_image_;

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
		//visualize_arrow_ROI();
	}

	void AM_learner::accumulate_grid_activity(activity_grid &grid ,track_common track, size_t element_serial)
	{
		if(track.ped_activity == MOVING)
		{
			moving_activity activity_tmp;
			activity_tmp.track_label =  track.track_label;
			activity_tmp.width = track.elements[element_serial].width;
			activity_tmp.depth = track.elements[element_serial].depth;
			activity_tmp.speed = track.elements[element_serial].speed;

			//put two clusters of tracks together:
			//rotate the moving direction of cluster 2 by PI, which means 2 clusters of tracks choose the same direction;
			//assumption: a pedestrian walkway is bidirectional;
			if(track.cluster_label == 1)activity_tmp.thetha = track.elements[element_serial].thetha;
			else if(track.cluster_label == 2)activity_tmp.thetha = track.elements[element_serial].thetha-M_PI;
			if(activity_tmp.speed<3.0 && track.cluster_label == 1)grid.moving_activities.push_back(activity_tmp);
		}
		else if(track.ped_activity == STATIC)
		{
			static_activity activity_tmp;
			activity_tmp.width = track.elements[element_serial].width;
			activity_tmp.depth = track.elements[element_serial].depth;
			activity_tmp.dwell_time =  track.elements.back().time - track.elements.front().time;
			grid.static_activities.push_back(activity_tmp);
		}
		else if(track.ped_activity == NOISE)
		{
			noisy_activity activity_tmp;
			activity_tmp.width = track.elements[element_serial].width;
			activity_tmp.depth = track.elements[element_serial].depth;
			grid.noisy_activities.push_back(activity_tmp);
		}
		grid.activity_intensity = grid.activity_intensity+1.0;
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
		    		//direction_mat.at<double>(a, 0) =  sin(grid_tmp.moving_activities[a].thetha);
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

		map_incorporating_sources();

		semantic_learning();
	}

	void AM_learner::map_incorporating_sources()
	{
		//1st: moving_direction;
		moving_direction_GP();

		//2nd: use distance transform to calculate the nearest obstacle for free cells;
		obstacle_dist();

		//3rd: calculate the direction of nearest edge;
		skel_direction();
		//this one works not so well;
		//boundary_direction();
	}

	void AM_learner::moving_direction_GP()
	{
		//load GP results from Matlab, stored as pictures; to be replaced in the future by GP using C++;
		FileStorage fs_read(gp_file_, FileStorage::READ);
		if(!fs_read.isOpened())ROS_ERROR("ped_semantics cannot find parameter file");

		gp_ROI_.x = (int)fs_read["gp_ROI_x"];
		gp_ROI_.y = (int)fs_read["gp_ROI_y"];
		gp_ROI_.width = (int)fs_read["gp_ROI_width"];
		gp_ROI_.height = (int)fs_read["gp_ROI_height"];
		GPmean_min_ = (double)fs_read["mean_min"];
		GPmean_max_ = (double)fs_read["mean_max"];
		GPvar_min_ = (double)fs_read["var_min"];
		GPvar_max_ = (double)fs_read["var_max"];

		string gpMean_path, gpVar_path;
		fs_read["gpMean_path"]>> gpMean_path;
		fs_read["gpVar_path"]>> gpVar_path;
		Mat gpMean = imread( gpMean_path, CV_LOAD_IMAGE_GRAYSCALE );
		Mat gpVar  = imread( gpVar_path,   CV_LOAD_IMAGE_GRAYSCALE );
		double mean_ratio = (GPmean_max_-GPmean_min_)/256.0;
		//pay attention here, may not cover the full range;
		double var_ratio  = (GPvar_max_-GPvar_min_)/256.0;

		//incorporate the information into "activity_map";
		int i, j;
		for(j = 0; j < gp_ROI_.height; j++)
		{
			for (i = 0; i < gp_ROI_.width; i++)
			{
				int roi_x = i+gp_ROI_.x;
				int roi_y = gp_ROI_.height-1-j+gp_ROI_.y;

				activity_grid &grid_tmp = AM_->cells[MAP_INDEX(AM_, roi_x, roi_y)];

				if(j==549 && i==727)ROS_ERROR("Please tell me what's wrong, dude? %d", gpMean.at<uchar>(j,i));

				double Thetha = double(gpMean.at<uchar>(j,i))*mean_ratio+GPmean_min_;
				double ThethaVar = double(gpVar.at<uchar>(j,i))*var_ratio+GPvar_min_;
				if(Thetha>M_PI) Thetha = Thetha-2*M_PI;

				grid_tmp.probe_direction = Thetha;
				grid_tmp.gp_estimation.val[0] = Thetha;
				grid_tmp.gp_estimation.val[1] = ThethaVar;


				/*
				double sinThetha_deriv = cos(grid_tmp.gp_estimation.val[0]);
				double sin2Thetha_deriv =2.0*cos(grid_tmp.gp_estimation.val[0]*2.0);
				if(fabs(sinThetha_deriv)>=fabs(sin2Thetha_deriv))
				{
					grid_tmp.gp_estimation.val[1] = sinThethaVar/pow(sinThetha_deriv, 2.0);
				}
				else
				{
					grid_tmp.gp_estimation.val[1] = sin2ThethaVar/pow(sin2Thetha_deriv, 2.0);
				}
				*/

				/*
				grid_tmp.gp_estimation.val[0] = double(gpMean.at<uchar>(j,i))*mean_ratio+GPmean_min_;
				grid_tmp.gp_estimation.val[1] = double(gpVar.at<uchar>(j,i))*var_ratio+GPvar_min_;
				assert(grid_tmp.gp_estimation.val[0]>= 0.0 && grid_tmp.gp_estimation.val[0]<=1.0);
				double angle_gp1 = asin(grid_tmp.gp_estimation.val[0]);
				double angle_gp2 = M_PI - angle_gp1;
				double angle_dist1 = fabs(angle_gp1 - grid_tmp.direction_gaussion.val[0]);
				double angle_dist2 = fabs(angle_gp2 - grid_tmp.direction_gaussion.val[0]);
				grid_tmp.probe_direction = (angle_dist1<angle_dist2)?angle_gp1:angle_gp2;
				*/
			}
		}

		show_moving_direction();
	}

	void AM_learner::obstacle_dist()
	{
		FileStorage fs_read(gp_file_, FileStorage::READ);
		int i, j;
		string binary_img_path;
		fs_read["binary_img_path"]>> binary_img_path;
		Mat binary_img = imread(binary_img_path, 0);
		for(j = 0; j < AM_->size_y; j++)
		{
			for (i = 0; i < AM_->size_x; i++)
			{
				activity_grid &grid_tmp = AM_->cells[MAP_INDEX(AM_, i, j)];
				//when try to read information from image, or write information as image, pay attention to the upside down coordinate;
				if(binary_img.at<uchar>(AM_->size_y-1-j,i)> 127)
				{
					grid_tmp.road_flag = true;
				}
				else
				{
					grid_tmp.road_flag = false;
				}
			}
		}

		Mat dist_img, inverted_dist_img;
		distanceTransform(binary_img, dist_img, CV_DIST_L2, 3);

		Mat binary_image_inverted = binary_img.clone();
		threshold(binary_img, binary_image_inverted, 128, 255, 1);
		distanceTransform(binary_image_inverted, inverted_dist_img, CV_DIST_L2, 3);

		for(j = 0; j < AM_->size_y; j++)
		{
			for (i = 0; i < AM_->size_x; i++)
			{
				activity_grid &grid_tmp = AM_->cells[MAP_INDEX(AM_, i, j)];
				if(grid_tmp.road_flag)grid_tmp.obs_dist = dist_img.at<float>(AM_->size_y-1-j,i)*map_scale_;
				else grid_tmp.obs_dist = inverted_dist_img.at<float>(AM_->size_y-1-j,i)*map_scale_;
			}
		}
	}

	/*
	void AM_learner::boundary_direction()
	{
		FileStorage fs_read(gp_file_, FileStorage::READ);
		int i, j;
		string binary_img_path;
		fs_read["binary_img_path"]>> binary_img_path;
		Mat binary_img = imread(binary_img_path, 0);
		vector<vector<Point> > boundary_contours;
		findContours( binary_img, boundary_contours, CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE );

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

				for(size_t ie=0; ie<boundary_contours.size();ie++)
				{
					double current_edge_nearestDist = DBL_MAX;
					int  current_edge_nearestPt= -1;
					for(size_t ip=0; ip<boundary_contours[ie].size();ip++)
					{
						double real_x = (double)boundary_contours[ie][ip].x*map_scale_;
						double real_y = (double)(AM_->size_y-1-boundary_contours[ie][ip].y)*map_scale_;

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
					if(right_end+1<(int)boundary_contours[nearest_edgeID].size()){right_end++; reach_two_ends = false;}

					double real_x1 = (double)boundary_contours[nearest_edgeID][left_end].x*map_scale_;
					double real_y1 = (double)(AM_->size_y-1-boundary_contours[nearest_edgeID][left_end].y)*map_scale_;
					double real_x2 = (double)boundary_contours[nearest_edgeID][right_end].x*map_scale_;
					double real_y2 = (double)(AM_->size_y-1-boundary_contours[nearest_edgeID][right_end].y)*map_scale_;

					dist_between_ends = sqrt((real_y1-real_y2)*(real_y1-real_y2)+(real_x1-real_x2)*(real_x1-real_x2));
					edge_angle = atan2(real_y1-real_y2, real_x1-real_x2);
					if(edge_angle < 0.0) edge_angle = edge_angle + M_PI;
				}

				//grid_tmp.skel_angle = sin(edge_angle);
				grid_tmp.skel_angle = (edge_angle);
				//printf("dist %f skel_angle %f\t", dist_between_ends, grid_tmp.skel_angle);
			}
		}
	}
	*/

	void AM_learner::skel_direction()
	{
		int i, j;
		topo_graph &road_network = road_semantics_analyzer_-> topology_extractor_ ->road_graph_;
		for(j = 0; j < gp_ROI_.height; j++)
		{
			for (i = 0; i < gp_ROI_.width; i++)
			{
				int roi_x = i+gp_ROI_.x;
				int roi_y = j+gp_ROI_.y;
				activity_grid &grid_tmp = AM_->cells[MAP_INDEX(AM_, roi_x, roi_y)];

				//there used to be a bug here;
				if(!grid_tmp.road_flag && grid_tmp.obs_dist>20) continue;

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
					vector<CvPoint> &deputy_points = road_network.edges[ie].cubic_spline->output_points_;

					find_nearest_points(grid_realx, grid_realy, deputy_points, current_edge_nearestPt, current_edge_nearestDist);

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
				double dist_threshold = 5.0;
				bool reach_two_ends = false;
				vector<CvPoint> &deputy_points = road_network.edges[nearest_edgeID].cubic_spline->output_points_;

				while(!(reach_two_ends || dist_between_ends>dist_threshold))
				{
					reach_two_ends = true;
					if(left_end>0){left_end--;reach_two_ends = false;}
					if(right_end+1<(int)deputy_points.size()){right_end++; reach_two_ends = false;}
					double real_x1 = (double)deputy_points[left_end].x*map_scale_;
					double real_y1 = (double)(AM_->size_y-1-deputy_points[left_end].y)*map_scale_;
					double real_x2 = (double)deputy_points[right_end].x*map_scale_;
					double real_y2 = (double)(AM_->size_y-1-deputy_points[right_end].y)*map_scale_;

					dist_between_ends = sqrt((real_y1-real_y2)*(real_y1-real_y2)+(real_x1-real_x2)*(real_x1-real_x2));
					edge_angle = atan2(real_y1-real_y2, real_x1-real_x2);
					if(edge_angle < 0.0) edge_angle = edge_angle + M_PI;
				}

				grid_tmp.skel_dist = nearest_dist;
				grid_tmp.skel_angle = edge_angle;
				grid_tmp.nearest_edge_ID = nearest_edgeID;
				//printf("dist %f skel_angle %f\t", dist_between_ends, grid_tmp.skel_angle);
			}
		}

		//there used to be a bug: must separate this block from the previous one, or skel_angle may be used without calculation;
		for(j = 0; j < gp_ROI_.height; j++)
		{
			for (i = 0; i < gp_ROI_.width; i++)
			{
				int roi_x = i+gp_ROI_.x;
				int roi_y = j+gp_ROI_.y;

				activity_grid &grid_tmp = AM_->cells[MAP_INDEX(AM_, roi_x, roi_y)];
				if(!grid_tmp.road_flag && grid_tmp.obs_dist>20) continue;

				//probe distance to the edge skel;
				direction_probe(roi_x, roi_y);
			}
		}
	}

	void AM_learner::direction_probe(int x_grid, int y_grid)
	{
		activity_grid &grid_tmp = AM_->cells[MAP_INDEX(AM_, x_grid, y_grid)];

		double grid_realx = double(x_grid)*map_scale_;
		double grid_realy = double(y_grid)*map_scale_;

		double probe_length1 = 10.0;

		Point2d end_point1, end_point2;
		end_point1.x = grid_realx + probe_length1*cos(grid_tmp.probe_direction);
		end_point1.y = grid_realy + probe_length1*sin(grid_tmp.probe_direction);
		end_point2.x = grid_realx + probe_length1*cos(grid_tmp.probe_direction+M_PI);
		end_point2.y = grid_realy + probe_length1*sin(grid_tmp.probe_direction+M_PI);

		int edgeID = grid_tmp.nearest_edge_ID;
		vector<CvPoint> &deputy_points = road_semantics_analyzer_-> topology_extractor_ ->road_graph_.edges[edgeID].cubic_spline->output_points_;

		double dist1, dist2;
		int PtID1, PtID2;
		find_nearest_points(end_point1.x, end_point1.y, deputy_points, PtID1, dist1);
		find_nearest_points(end_point2.x, end_point2.y, deputy_points, PtID2, dist2);
		//grid_tmp.probe_distance1 = (dist1 - grid_tmp.skel_dist)>(dist2 - grid_tmp.skel_dist)?(dist1 - grid_tmp.skel_dist):(dist2 - grid_tmp.skel_dist);

		int roi_x1 = floor(end_point1.x/map_scale_);
		int roi_y1 = floor(end_point1.y/map_scale_);
		int roi_x2 = floor(end_point2.x/map_scale_);
		int roi_y2 = floor(end_point2.y/map_scale_);
		activity_grid &grid_tmp1 = AM_->cells[MAP_INDEX(AM_, roi_x1, roi_y1)];
		activity_grid &grid_tmp2 = AM_->cells[MAP_INDEX(AM_, roi_x2, roi_y2)];

		double delt_skel_angle1, delt_skel_angle2;
		//pay attention that skel_angle is in [0, PI]; but  gp_estimation angle is converted to [-PI, +PI] when loading;
		delt_skel_angle1 = fabs(grid_tmp1.skel_angle - grid_tmp.gp_estimation[0])<M_PI ? fabs(grid_tmp1.skel_angle - grid_tmp.gp_estimation[0]) : 2*M_PI-fabs(grid_tmp1.skel_angle - grid_tmp.gp_estimation[0]);
		delt_skel_angle1 = delt_skel_angle1<M_PI_2? delt_skel_angle1: M_PI - delt_skel_angle1;
		delt_skel_angle2 = fabs(grid_tmp2.skel_angle - grid_tmp.gp_estimation[0])<M_PI ? fabs(grid_tmp2.skel_angle - grid_tmp.gp_estimation[0]) : 2*M_PI-fabs(grid_tmp2.skel_angle - grid_tmp.gp_estimation[0]);
		delt_skel_angle2 = delt_skel_angle2<M_PI_2? delt_skel_angle2: M_PI - delt_skel_angle2;

		double max_delt_skel = delt_skel_angle1 >delt_skel_angle2?delt_skel_angle1:delt_skel_angle2;
		//grid_tmp.probe_distance1 = fabs(grid_tmp.skel_angle)<M_PI_2?fabs(grid_tmp.skel_angle):M_PI -fabs(grid_tmp.skel_angle);
		//it turns out that the accuracy of spline's skel_angle is quite important;
		grid_tmp.probe_distance1 = max_delt_skel;

		//probe:
		if((x_grid ==727 && y_grid == (973-549)) || (x_grid ==464 && y_grid == (973-563)))
		{
			ROS_ERROR("Could you please tell me your angles (%d, %d), dude? skel_angle: %3f; gp_angle: %3f; (%3f, %3f)", x_grid, (973-y_grid), grid_tmp.skel_angle, grid_tmp.gp_estimation[0], grid_tmp1.skel_angle, grid_tmp2.skel_angle);
			ROS_ERROR("roi1 (%d, %d); roi2 (%d, %d)", roi_x1, roi_y1, roi_x2, roi_y2);
		}

		//introduce into another measurement: probe_distance2;
		double probe_length2 = 1.0;
		end_point1.x = grid_realx + probe_length2*cos(grid_tmp.probe_direction);
		end_point1.y = grid_realy + probe_length2*sin(grid_tmp.probe_direction);
		end_point2.x = grid_realx + probe_length2*cos(grid_tmp.probe_direction+M_PI);
		end_point2.y = grid_realy + probe_length2*sin(grid_tmp.probe_direction+M_PI);
		find_nearest_points(end_point1.x, end_point1.y, deputy_points, PtID1, dist1);
		find_nearest_points(end_point2.x, end_point2.y, deputy_points, PtID2, dist2);
		//grid_tmp.probe_distance2 = dist1>dist2? dist1:dist2;
		grid_tmp.probe_distance2 = (dist1 - grid_tmp.skel_dist)>(dist2 - grid_tmp.skel_dist)?(dist1 - grid_tmp.skel_dist):(dist2 - grid_tmp.skel_dist);

		/*
		int endpoint1_x = (int)end_point1.x/map_scale_;
		int endpoint1_y = (int)end_point1.y/map_scale_;
		int endpoint2_x = (int)end_point2.x/map_scale_;
		int endpoint2_y = (int)end_point2.y/map_scale_;

		activity_grid &grid_tmp1 = AM_->cells[MAP_INDEX(AM_, endpoint1_x, endpoint1_y)];
		activity_grid &grid_tmp2 = AM_->cells[MAP_INDEX(AM_, endpoint2_x, endpoint2_y)];

		double delt_distance1, delt_distance2;
		if((grid_tmp.road_flag&&(!grid_tmp1.road_flag))||(grid_tmp1.road_flag&&(!grid_tmp.road_flag))) delt_distance1 =grid_tmp.obs_dist+grid_tmp1.obs_dist;
		else delt_distance1 = fabs(grid_tmp.obs_dist-grid_tmp1.obs_dist);
		if((grid_tmp.road_flag&&(!grid_tmp2.road_flag))||(grid_tmp2.road_flag&&(!grid_tmp.road_flag))) delt_distance2 =grid_tmp.obs_dist+grid_tmp2.obs_dist;
		else delt_distance2 = fabs(grid_tmp.obs_dist-grid_tmp2.obs_dist);
		grid_tmp.probe_distance2 = delt_distance1<delt_distance2?delt_distance1:delt_distance2;
		*/
	}

	void AM_learner::find_nearest_points(double gridx, double gridy, vector<CvPoint> &deputy_points, int & nearestPt_ID, double & nearestDist)
	{
		double current_edge_nearestDist = DBL_MAX;
		int  current_edge_nearestPt= -1;

		for(size_t ip=0; ip<deputy_points.size();ip++)
		{
			double real_x = (double)deputy_points[ip].x*map_scale_;
			double real_y = (double)(AM_->size_y-1-deputy_points[ip].y)*map_scale_;

			double dist_tmp = sqrt((gridy-real_y)*(gridy-real_y)+(gridx-real_x)*(gridx-real_x));
			if(dist_tmp<current_edge_nearestDist)
			{
				current_edge_nearestPt = ip;
				current_edge_nearestDist = dist_tmp;
			}
		}
		nearestPt_ID = current_edge_nearestPt;
		nearestDist = current_edge_nearestDist;
	}

	void AM_learner::semantic_learning()
	{
		pedestrian_path();
		ped_sourcesink();
		ped_sidewalk();
		ped_crossing();
	}

	//
	void AM_learner::pedestrian_path()
	{
		Mat intensity_image(AM_->size_y,  AM_->size_x, CV_32FC1 );
		intensity_image = Scalar(0);
		double road_half_width = 5.0;
		int neighbour_radius = (int)road_half_width/map_scale_;

		int i, j;
		for(j = 0; j < AM_->size_y; j++)
		{
			for (i = 0; i < AM_->size_x; i++)
			{
				int x = i;
				int y = AM_->size_y-1-j;
				activity_grid &grid_tmp = AM_->cells[MAP_INDEX(AM_, i, j)];
				int p, q;
				double biggest_activity_number = 0;
				for (p =-neighbour_radius; p<= neighbour_radius; p++)
				{
					for (q =-neighbour_radius; q<= neighbour_radius; q++)
					{
						int xp=x+p;
						int yq=y+q;
						if(MAP_VALID(AM_, xp, yq))
						{
							if(AM_->cells[MAP_INDEX(AM_, xp, yq)].activity_intensity>biggest_activity_number) biggest_activity_number = AM_->cells[MAP_INDEX(AM_, xp, yq)].activity_intensity;
						}
					}
				}

				double intensity_absolute_factor, intensity_local_ratio, intensity_local_factor, intensity_factor;

				//intensity_absolute_factor = 0.1+log2 (double(grid_tmp.moving_activities.size()+1.0));
				//use sigmoid function instead;
				double x_shift = -2.0;
				double y_shift = -1.0/(1+exp(-(x_shift)));
				intensity_absolute_factor = 1.0/(1+exp(-(double(grid_tmp.activity_intensity + x_shift))))+y_shift;

				intensity_local_ratio  = biggest_activity_number > 0 ? (double(grid_tmp.activity_intensity)/double(biggest_activity_number)) : 1.0;
				intensity_local_factor = intensity_local_ratio;
				intensity_factor = intensity_absolute_factor*intensity_local_factor;
				intensity_image.at<float>(y,x) = (float)(intensity_factor);
			}
		}

		GaussianBlur (intensity_image, intensity_image, cv::Size (5, 5), 0);
		double max_value, min_value;
		minMaxIdx(intensity_image, &min_value, &max_value, NULL, NULL);
		ROS_INFO("max_value, min_value %5f, %5f", max_value, min_value);
		normalize(intensity_image, intensity_image, 0.0, 1.0, NORM_MINMAX);

		Mat intensity_image_tmp;
		intensity_image.convertTo(intensity_image_tmp, CV_8U, 255.0, 0.0);
		imwrite( "./data/intensity_image.jpg", intensity_image_tmp );
		threshold(intensity_image_tmp, intensity_image_tmp, 40, 255, cv::THRESH_BINARY);
		int dilation_size = 1;
		Mat element = getStructuringElement( MORPH_RECT, Size( 2*dilation_size + 1, 2*dilation_size+1 ), Point( dilation_size, dilation_size ) );
		erode(intensity_image_tmp, intensity_image_tmp, element);
		dilate(intensity_image_tmp, intensity_image_tmp, element);

		for(j = 0; j < AM_->size_y; j++)
		{
			for (i = 0; i < AM_->size_x; i++)
			{
				activity_grid &grid_tmp = AM_->cells[MAP_INDEX(AM_, i, j)];
				//if(!grid_tmp.road_flag) continue;
				int x = i;
				int y = AM_->size_y-1-j;

				grid_tmp.path_score = intensity_image.at<float>(y,x);
				if(intensity_image_tmp.at<uchar>(y,x)>127) grid_tmp.pedPath_flag = true;
			}
		}
		imwrite( "./data/intensity_image_thresholded.jpg", intensity_image_tmp );
	}

	void AM_learner::ped_sourcesink()
	{
		int i, j;
		for(j = 0; j < gp_ROI_.height; j++)
		{
			for (i = 0; i < gp_ROI_.width; i++)
			{
				int roi_x = i+gp_ROI_.x;
				int roi_y = j+gp_ROI_.y;
				activity_grid &grid_tmp = AM_->cells[MAP_INDEX(AM_, roi_x, roi_y)];

				if(!grid_tmp.road_flag && grid_tmp.obs_dist > 5.0)
				{
					grid_tmp.EE_score = 0.001;
					continue;
				}

				/*
				double distance_factor;
				if(grid_tmp.obs_dist<2.0) distance_factor = 2.0-grid_tmp.obs_dist;
				else distance_factor = 0.001;
				//distance_factor = 1.0;

				double intensity_factor;
				intensity_factor = 0.1+log2 (double(grid_tmp.moving_activities.size()+1.0));
				//intensity_factor = 1.0;

				double directionVar_factor;
				directionVar_factor = 1.0 - sqrt(1.0/(GPvar_max_-GPvar_min_)*(grid_tmp.gp_estimation.val[1]-GPvar_min_));
				directionVar_factor = directionVar_factor*directionVar_factor*directionVar_factor;
				//directionVar_factor = 1.0;

				double direction_factor_crossing;
				direction_factor_crossing = 1.0;
				//direction_factor_crossing = (grid_tmp.probe_distance1-1.0)>0?(grid_tmp.probe_distance1-1.0):0;
				direction_factor_crossing = grid_tmp.probe_distance1;

				//if(grid_tmp.pedPath_flag) intensity_factor = intensity_factor;
				//else intensity_factor = intensity_factor * 0.1;

				float pedPath_factor;
				if(grid_tmp.pedPath_flag) pedPath_factor = 1.0;
				else pedPath_factor = 0.00001;

				grid_tmp.EE_score = distance_factor*cycle_factor*direction_factor_crossing*directionVar_factor*pedPath_factor*grid_tmp.path_score;
				//grid_tmp.EE_score = direction_factor_crossing;
				*/

				//1st feature, pedestrian path Fpp;
				double Fpp_EE[2];
				//epsilon is used to avoid degeneration case;
				//kee is the probability of pedestrian-path given non-EE, which is approximated to the ratio of pedestrian-path in all the road;
				double epsilon = 0.0000001, kee = 0.1;
				if(grid_tmp.pedPath_flag){Fpp_EE[0] = 1.0 - epsilon; Fpp_EE[1] = epsilon;}
				else {Fpp_EE[0] = kee; Fpp_EE[1] = 1-kee;}

				//2nd feature, moving direction Fd;
				double Fd_EE[2];
				Fd_EE[0]= grid_tmp.probe_distance1/M_PI_2;
				Fd_EE[1]= 1/M_PI;

				//3rd feature, moving direction variance Fdv;
				double Fdv_EE[2];
				Fdv_EE[0] = 2.0*(grid_tmp.gp_estimation.val[1]-GPvar_min_)/((GPvar_max_-GPvar_min_)*(GPvar_max_-GPvar_min_));
				Fdv_EE[1] = 1.0/(GPvar_max_-GPvar_min_);

				//4th feature, distance to road boundary;
				double Fp_EE[2];
				Fp_EE[0] = 1.0-grid_tmp.obs_dist/2.0 >0.0 ? 1.0-grid_tmp.obs_dist/2.0 : 0.00001;
				Fp_EE[1] = 2.0/8.0;

				double EE_score = Fpp_EE[0]*Fd_EE[0]*Fdv_EE[0]*Fp_EE[0];
				double non_EEscore = Fpp_EE[1]*Fd_EE[1]*Fdv_EE[1]*Fp_EE[1];

				grid_tmp.EE_score = EE_score/(EE_score+non_EEscore);

				//add one more criterion about "outside of cycles";
				double cycle_factor = 1.0;
				CvPoint2D32f tmp_point = cvPoint2D32f(roi_x, AM_->size_y-1-roi_y);
				bool inside_cycle = false;
				for(size_t c=0; c<road_semantics_analyzer_->topo_semantic_analyzer_->cycle_polygons_.size(); c++)
				{
					if(road_semantics_analyzer_->topo_semantic_analyzer_->pointInPolygon(tmp_point, road_semantics_analyzer_->topo_semantic_analyzer_->cycle_polygons_[c]))
					{
						inside_cycle=true;
						break;
					}
				}
				if(inside_cycle) cycle_factor = 0.0;

				grid_tmp.EE_score = grid_tmp.EE_score * cycle_factor;
			}
		}

		Mat EE_color(AM_->size_y,  AM_->size_x, CV_32FC1 );
		EE_color = Scalar(0);
		for(j = 0; j < AM_->size_y; j++)
		{
			for (i = 0; i < AM_->size_x; i++)
			{
				activity_grid &grid_tmp = AM_->cells[MAP_INDEX(AM_, i, j)];
				//if(!grid_tmp.road_flag) continue;
				int x = i;
				int y = AM_->size_y-1-j;
				EE_color.at<float>(y,x) = (float)(grid_tmp.EE_score);
			}
		}
		double minVal, maxVal;
		minMaxLoc(EE_color, &minVal, &maxVal); //find minimum and maximum intensities
		ROS_INFO("EE_color, minVAL, maxVal %3f, %3f", minVal, maxVal);

		normalize(EE_color, EE_color, 0.0, 1.0, NORM_MINMAX);

		for(j = 0; j < AM_->size_y; j++)
		{
			for (i = 0; i < AM_->size_x; i++)
			{
				activity_grid &grid_tmp = AM_->cells[MAP_INDEX(AM_, i, j)];
				//if(!grid_tmp.road_flag) continue;
				int x = i;
				int y = AM_->size_y-1-j;
				grid_tmp.EE_score = EE_color.at<float>(y,x);

				grid_tmp.EE_score = grid_tmp.EE_score * 255.0/70.0;
				if(grid_tmp.EE_score>1.0) grid_tmp.EE_score =0.98;
				EE_color.at<float>(y,x) = grid_tmp.EE_score;
			}
		}
		Mat EE_color_tmp;
		EE_color.convertTo(EE_color_tmp, CV_8U, 255.0, 0.0);
		//threshold(EE_color_tmp, EE_color_tmp, 30, 255, 0);
		imwrite( "./data/EE_color.jpg", EE_color_tmp );

		ped_SSEM();
	}

	void AM_learner::ped_SSEM()
	{
		Mat img_visual = Mat::zeros( AM_->size_y, AM_->size_x, CV_8UC3 );
	    const Scalar colors[] =
	    {
	        Scalar(0,0,255), Scalar(0,255,0),
	        Scalar(0,255,255),Scalar(255,255,0),
	        Scalar(255,0,255), Scalar(128,0,255), Scalar(0,128,255)
	    };

	    int i, j;

		//prepare for the grid samples;
	    vector<Vec2f> samples_vector;
		for(j = 0; j < AM_->size_y; j++)
		{
			for (i = 0; i < AM_->size_x; i++)
			{
				activity_grid &grid_tmp = AM_->cells[MAP_INDEX(AM_, i, j)];
				//if(grid_tmp.EE_score>=30.0/255)
				if(grid_tmp.EE_score>=0.5)
				{
					Vec2f sample_tmp(i, j);
					samples_vector.push_back(sample_tmp);
				}
			}
		}

		/*
		samples_vector.clear();
		for(i=0; i<100; i++)
		{
			float x_tmp = rand()%100+500;
			float y_tmp = 0.5*x_tmp + rand()%10;
			Vec2f sample_tmp(x_tmp, y_tmp);
			samples_vector.push_back(sample_tmp);
		}
		*/

		int nsamples = (int)samples_vector.size();
	    Mat all_samples( nsamples, 2, CV_32FC1 );
	    //all_samples = all_samples.reshape(2, 0);
	    for( i = 0; i < nsamples; i ++ )
	    {
	    	all_samples.at<float>(i, 0) = samples_vector[i].val[0];
	    	all_samples.at<float>(i, 1) = samples_vector[i].val[1];
	    }
	    //all_samples = all_samples.reshape(1, 0);

	    //How to determine cluster number "N";
	    //Method1: use Bayesian Information Criterion (BIC) for module selection, to determine N in this case;
	    //BIC(θ) = −2logp(X|θ) + k log(n) ∗ λ
	    //For post processing: may need to merge two over-segmented clusters according to their distances;

	    double bic_min = DBL_MAX;
	    int bic_N = 0;
	    int N;
	    for(N=1; N<15; N++)
	    {
	    	double loglikelihood_sum;
	    	self_EM(samples_vector, N, loglikelihood_sum);
			double k_tmp = N*6;
			double lamda_tmp = 1.0;
			double bic_tmp = -2*(loglikelihood_sum) + k_tmp*std::log(double(nsamples))* lamda_tmp;

			ROS_INFO("cluster N, prob factor, bic_tmp %d, %3f, %3f", N, loglikelihood_sum, bic_tmp);

			if(bic_tmp < bic_min)
			{
				bic_min = bic_tmp;
				bic_N = N;
			}
	    }

	    //Method2: use prior knowledge about distance between two SinkSources; the distance should be larger than certain value;

	    N = bic_N;
	    //http://docs.opencv.org/modules/ml/doc/expectation_maximization.html
	    Mat kmeans_labels, kmeans_centers;
		kmeans(all_samples, N, kmeans_labels, TermCriteria( CV_TERMCRIT_EPS+CV_TERMCRIT_ITER, 100, 1.0), 10, KMEANS_PP_CENTERS, kmeans_centers);
		EM em_model2( N, EM::COV_MAT_GENERIC, TermCriteria(CV_TERMCRIT_ITER|CV_TERMCRIT_EPS, 100,  0.1));
		Mat loglikelihood2, labels2, probability2;
		em_model2.trainE( all_samples, kmeans_centers, Mat(), Mat(), loglikelihood2, labels2, probability2);
		if(em_model2.isTrained()) printf("\n em_model2 trained\n");
		vector<Mat> covMats2= em_model2.get<vector<Mat> >("covs");
		Mat Means2 = em_model2.get<Mat>("means");

	    for( i = 0; i < nsamples; i++ )
	    {
	        Point pt(cvRound(all_samples.at<float>(i, 0)), cvRound(all_samples.at<float>(i, 1)));
	        int label_tmp = labels2.at<int>(i)%7;
	        circle( img_visual, pt, 1, colors[label_tmp], CV_FILLED );
	    }
	    for(i=0; i<N; i++)
		{
			float cx = Means2.at<double>(i,0)*map_scale_;
			float cy = Means2.at<double>(i,1)*map_scale_;
			draw_covMatrix_eclipse(img_visual, covMats2[i], Means2, i);
			SourceSinks_.push_back(Point2f(cx, cy));
		}

		for(j = 0; j < AM_->size_y/2; j++)
		{
			for (i = 0; i < AM_->size_x; i++)
			{
				int x = i;
				int y = AM_->size_y-1-j;
				Vec3b intensity = img_visual.at<Vec3b>(y, x);
				img_visual.at<Vec3b>(y, x) = img_visual.at<Vec3b>(j, i);
				img_visual.at<Vec3b>(j, i) = intensity;
			}
		}

	    imwrite( "./data/SourceSinks.jpg", img_visual );

	}

	void AM_learner::self_EM( vector<Vec2f>& samples_vector, int & cluster_N, double & loglikelihood_output)
	{
		int N = cluster_N;
		int nsamples = (int)samples_vector.size();
	    Mat all_samples( nsamples, 2, CV_32FC1 );
	    //all_samples = all_samples.reshape(2, 0);
	    int i, j;
	    for( i = 0; i < nsamples; i ++ )
	    {
	    	all_samples.at<float>(i, 0) = samples_vector[i].val[0];
	    	all_samples.at<float>(i, 1) = samples_vector[i].val[1];
	    }

	    int max_iteration = 100;
		int iteration_time = 0;
		double likelihood_EPS = 0.005;
		double last_likelihood = 0.0;
		double gaussian_fi[N];

		Mat gaussian_means[N];
		Mat gaussian_cov[N];
		Mat probsMat( nsamples, N, CV_64FC1 );
		Mat weightMat( nsamples, N, CV_64FC1 );
		Mat kmeans_labels, kmeans_centers;
		kmeans(all_samples, N, kmeans_labels, TermCriteria( CV_TERMCRIT_EPS+CV_TERMCRIT_ITER, 100, 1.0), 10, KMEANS_PP_CENTERS, kmeans_centers);

		typedef std::vector<int> label_vector;
		std::vector<label_vector> clusters_labels (N);
	    for( i = 0; i < nsamples; i++ )
	    {
	    	if(kmeans_labels.at<int>(i)<N) clusters_labels[kmeans_labels.at<int>(i)].push_back(i);
	    }
		Mat guassian_sample[N];
		for(i=0; i<N; i++)
		{
			//printf("\n cluster label: %d, number: %ld \n", i, clusters_labels[i].size());
			Mat samples_tmp( (int)clusters_labels[i].size(), 2, CV_32FC1 );
			//samples_tmp = samples_tmp.reshape(2, 0);
			for(j=0; j<clusters_labels[i].size(); j++)
			{
				samples_tmp.at<float>(j, 0) = all_samples.at<float>(clusters_labels[i][j], 0);
				samples_tmp.at<float>(j, 1) = all_samples.at<float>(clusters_labels[i][j], 1);
			}
			guassian_sample[i] = samples_tmp;
		}
		for(i=0; i<N; i++)
		{
			gaussian_fi[i]= double(clusters_labels[i].size())/double(nsamples);
			calcCovarMatrix(guassian_sample[i], gaussian_cov[i], gaussian_means[i], CV_COVAR_NORMAL|CV_COVAR_ROWS | CV_COVAR_SCALE);
			/*
			printf("\n == %d, %3f\n", clusters_labels[i].size(), determinant(gaussian_cov[i]) );
			if(determinant(gaussian_cov[i])  < 0.0000001) cout << guassian_sample[i]<<endl;
			cout<<gaussian_cov[i]<<endl;
			*/
		}

	    //printf("\n Enter EM-iteration \n");

		Mat em_labels(nsamples, 1, CV_32SC1 );

		bool first_time = true;
		for(iteration_time = 0; iteration_time < max_iteration; iteration_time++)
		{
			//printf("\niteration_time %d\n", iteration_time);

			//E-Step: calculate "probsMat";
			double current_likelihood = 0.0;
			//for(i=0; i<N; i++) printf("means (%lf, %lf);\t fi:%lf\n", gaussian_means[i].at<double>(0,0), gaussian_means[i].at<double>(0,1), gaussian_fi[i]);

			for(i=0; i<nsamples; i++)
			{
				Mat sample_i = all_samples.rowRange(i,i+1);
				//printf("sample_i (%f, %f)\t", sample_i.at<float>(0,0), sample_i.at<float>(0,1));

				double sample_i_likelihood = 0;
				double total_weights = 0.0;
				for(j=0; j<N; j++)
				{
					//printf("sample: %d, %d; means: %d, %d; cov: %d, %d\n", sample_i.rows, sample_i.cols, gaussian_means[j].rows, gaussian_means[j].cols, gaussian_cov[j].rows, gaussian_cov[j].cols);

					probsMat.at<double>(i,j) = determinant(gaussian_cov[j])>DBL_MIN ? prob_2DGaussian(sample_i, gaussian_means[j], gaussian_cov[j]) : 1.0/clusters_labels[j].size();

					//printf("probsMat %.12f\t", probsMat.at<double>(i,j));

					sample_i_likelihood = sample_i_likelihood + probsMat.at<double>(i,j)*gaussian_fi[j];
					total_weights = total_weights + probsMat.at<double>(i,j);
				}

				double largest_weight = 0.0;
				int largest_cluster = 0;
				for(j=0; j<N; j++)
				{
					if(probsMat.at<double>(i,j)>=largest_weight){largest_weight = probsMat.at<double>(i,j); largest_cluster=j;}
					//printf("sample: %d, %d; means: %d, %d; cov: %d, %d\n", sample_i.rows, sample_i.cols, gaussian_means[j].rows, gaussian_means[j].cols, gaussian_cov[j].rows, gaussian_cov[j].cols);
					weightMat.at<double>(i,j) =  probsMat.at<double>(i,j)/total_weights;
					//printf("weight %lf\t", weightMat.at<double>(i,j));
				}
				em_labels.at<int>(i,0) = largest_cluster;

				current_likelihood = current_likelihood+log(sample_i_likelihood);

				//printf("largest_cluster %d \n", largest_cluster);
			}

			double delta_likelihood = current_likelihood - last_likelihood;
			//printf("delta_likelihood %lf;\n", delta_likelihood);

			last_likelihood = current_likelihood;
			loglikelihood_output = current_likelihood;

			if(first_time) { first_time=false;}
			else if(delta_likelihood < likelihood_EPS) break;

			//----------------------------------------M-Step: calculate "means, weights, covar matrices"--------------------------------------;
			for(i=0; i<N; i++)
			{
				gaussian_fi[i] = 0.0;
				gaussian_means[i] = 0.0;
				gaussian_cov[i] = 0.0;

				for(j=0; j<(nsamples); j++)
				{
					Mat sample_j = all_samples.rowRange(j,j+1);
					Mat sample_j_tmp;
					sample_j.convertTo(sample_j_tmp, CV_64F);
					gaussian_fi[i] = gaussian_fi[i] + weightMat.at<double>(j,i);
					gaussian_means[i] = gaussian_means[i] + weightMat.at<double>(j,i)*sample_j_tmp;
				}
				gaussian_means[i] = gaussian_means[i]/gaussian_fi[i];
				gaussian_fi[i] = gaussian_fi[i]/nsamples;

				//printf("means (%lf, %lf);\t fi:%lf\n", gaussian_means[i].at<double>(0,0), gaussian_means[i].at<double>(0,1), gaussian_fi[i]);

				for(j=0; j<(nsamples); j++)
				{
					Mat sample_j = all_samples.rowRange(j,j+1);
					Mat sample_j_tmp;
					sample_j.convertTo(sample_j_tmp, CV_64F);
					gaussian_cov[i] = gaussian_cov[i] + weightMat.at<double>(j,i)*(sample_j_tmp-gaussian_means[i]).t()*(sample_j_tmp-gaussian_means[i]);
				}
				gaussian_cov[i] = gaussian_cov[i]/(gaussian_fi[i]*(nsamples));

			}
		}
	}

	void AM_learner::draw_covMatrix_eclipse( Mat img, Mat CurrCovMat, Mat Means, int i)
	{
		double cx = Means.at<double>(i,0);
		double cy = Means.at<double>(i,1);

		Mat eigenvalues;
		Mat eigenvectors;
		eigen( CurrCovMat, eigenvalues, eigenvectors );

		double eigenvec1_len = eigenvalues.at<double>(0,0);
		double len1          = sqrt(eigenvec1_len)*10;
		double eigenvec1_x   = eigenvectors.at<double>(0,0) * len1;
		double eigenvec1_y   = eigenvectors.at<double>(0,1) * len1;

		double eigenvec2_len = eigenvalues.at<double>(1,0);
		double len2          = sqrt(eigenvec2_len)*5;
		double eigenvec2_x   = eigenvectors.at<double>(1,0) * len2;
		double eigenvec2_y   = eigenvectors.at<double>(1,1) * len2;

		// Show axes of ellipse
		//line( img, cv::Point(cx,cy), cv::Point(cx+eigenvec1_x, cy+eigenvec1_y), CV_RGB(255,255,0) );
		//line( img, cv::Point(cx,cy), cv::Point(cx+eigenvec2_x, cy+eigenvec2_y), CV_RGB(0,255,0) );

		// Show ellipse rotated into direction of eigenvec1
		double dx = eigenvec1_x;
		double dy = eigenvec1_y;
		double angle_rad = atan2( dy, dx );
		double angle_deg = angle_rad * (180.0/M_PI); // convert radians (0,2PI) to degree (0°,360°)

		cv::RotatedRect* myRotatedRect = new cv::RotatedRect( cvPoint(cx,cy), cvSize(len1, len2), angle_deg );
		if (myRotatedRect != NULL)
		{
			int g = 1*255.0;
			cv::Scalar s = CV_RGB(g,g,g);
			ellipse( img, *myRotatedRect, s );
			delete myRotatedRect;
		}
	}

	double AM_learner::prob_2DGaussian(Mat sample, Mat Mean, Mat CovMat)
	{
		//"sample" and "Mean" are row vectors (1x2); CovMat 2x2;
		Mat sample_tmp;
		sample.convertTo(sample_tmp, CV_64F);

		/*
		printf("sample_tmp (%f, %f)\t", sample_tmp.at<double>(0,0), sample_tmp.at<double>(0,1));
		printf("Mean (%f, %f`)\t", Mean.at<double>(0,0), Mean.at<double>(0,1));
		printf("CovMat (%f, %f, %f, %f)\t", CovMat.at<double>(0,0), CovMat.at<double>(0,1), CovMat.at<double>(1,0), CovMat.at<double>(1,1));
		*/

		//Pay attention to the special case where "2D Gaussian degenerates to 1D Gaussian"; lazy solution below;
		if(determinant(CovMat)<DBL_MIN) return 0.0;

		double p1 = 1/(2*M_PI*sqrt(determinant(CovMat)));
		double p_tmp = (cv::sum((-0.5)*(sample_tmp-Mean)*CovMat.inv()*(sample_tmp-Mean).t())).val[0];
		double p2 = std::exp(p_tmp);

		//printf("p1 %f\t p_tmp %f\t p2 %f\t", p1, p_tmp, p2);
		return p1*p2;
	}

	void AM_learner::ped_sidewalk()
	{
		int i, j;
		for(j = 0; j < gp_ROI_.height; j++)
		{
			for (i = 0; i < gp_ROI_.width; i++)
			{
				int roi_x = i+gp_ROI_.x;
				int roi_y = j+gp_ROI_.y;
				activity_grid &grid_tmp = AM_->cells[MAP_INDEX(AM_, roi_x, roi_y)];

				if(!grid_tmp.road_flag && grid_tmp.obs_dist > 5.0)
				{
					grid_tmp.sidewalk_score = 0.0;
					continue;
				}

				double distance_factor;
				//if(grid_tmp.road_flag) distance_factor = (1.0-0.3*grid_tmp.obs_dist) > 0 ? (1.0-0.3*grid_tmp.obs_dist): 0.0;
				//use "skel_dist" instead for on-road pedestrian sidewalk;
				double half_road_width = 5.0;
				if(grid_tmp.road_flag)
				{
					if(grid_tmp.skel_dist>2.0)
					distance_factor = (0.01+ 0.9*grid_tmp.skel_dist/half_road_width) > 0.0 ? (0.01+ 0.9*grid_tmp.skel_dist/half_road_width): 0.0;
					else
					{
						distance_factor = 0.01 + 2.0*grid_tmp.skel_dist/(grid_tmp.skel_dist+grid_tmp.obs_dist);
					}
				}
				else distance_factor = (1.0-0.3*grid_tmp.obs_dist) > 0 ? (1.0-0.3*grid_tmp.obs_dist): 0.0;
				//distance_factor = 1.0;

				float pedPath_factor;
				if(grid_tmp.pedPath_flag) pedPath_factor = 1.0;
				else pedPath_factor = 0.00001;

				//add one more criterion about "outside of cycles";
				double cycle_factor = 1.0;
				CvPoint2D32f tmp_point = cvPoint2D32f(roi_x, AM_->size_y-1-roi_y);
				bool inside_cycle = false;
				for(size_t c=0; c<road_semantics_analyzer_->topo_semantic_analyzer_->cycle_polygons_.size(); c++)
				{
					if(road_semantics_analyzer_->topo_semantic_analyzer_->pointInPolygon(tmp_point, road_semantics_analyzer_->topo_semantic_analyzer_->cycle_polygons_[c]))
					{
						inside_cycle=true;
						break;
					}
				}
				if(inside_cycle) cycle_factor = 0.001;

				double directionVar_factor;
				directionVar_factor = 1.0 - sqrt(1.0/(GPvar_max_-GPvar_min_)*(grid_tmp.gp_estimation.val[1]-GPvar_min_));
				directionVar_factor = directionVar_factor*directionVar_factor*directionVar_factor;
				//directionVar_factor = 1.0;

				double direction_factor_sidewalk;
				direction_factor_sidewalk = 1.0;
				direction_factor_sidewalk = 1.0/(0.1+grid_tmp.probe_distance2*grid_tmp.probe_distance2);

				grid_tmp.sidewalk_score = distance_factor*direction_factor_sidewalk * directionVar_factor*cycle_factor*pedPath_factor*grid_tmp.path_score;
				//grid_tmp.sidewalk_score = direction_factor_sidewalk;
			}
		}

		Mat sidewalk_color(AM_->size_y,  AM_->size_x, CV_32FC1 );
		sidewalk_color = Scalar(0);

		for(j = 0; j < AM_->size_y; j++)
		{
			for (i = 0; i < AM_->size_x; i++)
			{
				activity_grid &grid_tmp = AM_->cells[MAP_INDEX(AM_, i, j)];
				//if(!grid_tmp.road_flag) continue;
				int x = i;
				int y = AM_->size_y-1-j;
				sidewalk_color.at<float>(y,x) = (float)(grid_tmp.sidewalk_score);
			}
		}

		normalize(sidewalk_color, sidewalk_color, 0.0, 1.0, NORM_MINMAX);

		for(j = 0; j < AM_->size_y; j++)
		{
			for (i = 0; i < AM_->size_x; i++)
			{
				activity_grid &grid_tmp = AM_->cells[MAP_INDEX(AM_, i, j)];
				//if(!grid_tmp.road_flag) continue;
				int x = i;
				int y = AM_->size_y-1-j;

				sidewalk_color.at<float>(y,x) = sidewalk_color.at<float>(y,x) * 3.0;
				if(sidewalk_color.at<float>(y,x) > 1.0) sidewalk_color.at<float>(y,x) = 1.0;
				grid_tmp.sidewalk_score = sidewalk_color.at<float>(y,x);
			}
		}

		Mat sidewalk_color_tmp;
		sidewalk_color.convertTo(sidewalk_color_tmp, CV_8U, 255.0, 0.0);
		imwrite( "./data/sidewalk_color.jpg", sidewalk_color_tmp );
	}

	void AM_learner::ped_crossing()
	{
		int i, j;
		for(j = 0; j < gp_ROI_.height; j++)
		{
			for (i = 0; i < gp_ROI_.width; i++)
			{
				int roi_x = i+gp_ROI_.x;
				int roi_y = j+gp_ROI_.y;
				activity_grid &grid_tmp = AM_->cells[MAP_INDEX(AM_, roi_x, roi_y)];

				//this setting is just to avoid unnecessary computation cost for a lot of unknow areas;
				if(!grid_tmp.road_flag && grid_tmp.obs_dist > 5.0)
				{
					grid_tmp.crossing_score = 0.001;
					continue;
				}

				double intensity_factor;
				intensity_factor = 0.1+log2 (double(grid_tmp.moving_activities.size()+1.0));
				//intensity_factor = 1.0;

				//add one more criterion about "outside of cycles";
				double cycle_factor = 1.0;

				CvPoint2D32f tmp_point = cvPoint2D32f(roi_x, AM_->size_y-1-roi_y);
				bool inside_cycle = false;
				for(size_t c=0; c<road_semantics_analyzer_->topo_semantic_analyzer_->cycle_polygons_.size(); c++)
				{
					if(road_semantics_analyzer_->topo_semantic_analyzer_->pointInPolygon(tmp_point, road_semantics_analyzer_->topo_semantic_analyzer_->cycle_polygons_[c]))
					{
						inside_cycle=true;
						break;
					}
				}
				if(inside_cycle) cycle_factor = 0.001;

				double directionVar_factor;
				directionVar_factor = 1.0 - sqrt(1.0/(GPvar_max_-GPvar_min_)*(grid_tmp.gp_estimation.val[1]-GPvar_min_));
				directionVar_factor = directionVar_factor*directionVar_factor*directionVar_factor;
				//directionVar_factor = 1.0;

				double direction_factor_crossing;
				direction_factor_crossing = 1.0;
				if(SourceSinks_.size()<2) direction_factor_crossing = 0.001;
				else
				{
					int SS_serial = 0; int SS2nd_serial = 0;
					double SS_dist = 10000.0; double SS2nd_dist=10000.0;
					double grid_realx = double(roi_x)*map_scale_;
					double grid_realy = double(roi_y)*map_scale_;

					for(int s=0; s<SourceSinks_.size(); s++)
					{
						double dist_tmp = sqrt((grid_realx-SourceSinks_[s].x)*(grid_realx-SourceSinks_[s].x)+(grid_realy-SourceSinks_[s].y)*(grid_realy-SourceSinks_[s].y));
						if(dist_tmp<SS_dist)
						{
							SS2nd_serial = SS_serial;
							SS2nd_dist = SS_dist;
							SS_serial = s;
							SS_dist = dist_tmp;
						}
						else if(dist_tmp<SS2nd_dist)
						{
							SS2nd_serial = s;
							SS2nd_dist = dist_tmp;
						}
					}
					double road_width = 8.0;
					double delt_distance = fabs(SS_dist+SS2nd_dist-road_width);
					//double sigma = 4.0;
					//double direction_factor1 = exp(-(delt_distance * delt_distance) / (2 * sigma * sigma));
					double direction_factor1 = 1.0/(delt_distance+0.1);
					//direction_factor1 = 1.0;
					//ROS_INFO("distance %3f", delt_distance);

					int roi_x1 = floor(SourceSinks_[SS_serial].x/map_scale_);
					int roi_y1 = floor(SourceSinks_[SS_serial].y/map_scale_);
					int roi_x2 = floor(SourceSinks_[SS2nd_serial].x/map_scale_);
					int roi_y2 = floor(SourceSinks_[SS2nd_serial].y/map_scale_);

					activity_grid &grid_tmp1 = AM_->cells[MAP_INDEX(AM_, roi_x1, roi_y1)];
					activity_grid &grid_tmp2 = AM_->cells[MAP_INDEX(AM_, roi_x2, roi_y2)];

					double delt_angle1 = fabs(grid_tmp1.gp_estimation.val[0]-grid_tmp.gp_estimation.val[0]) < M_PI_2 ? fabs(grid_tmp1.gp_estimation.val[0]-grid_tmp.gp_estimation.val[0]): (M_PI - fabs(grid_tmp1.gp_estimation.val[0]-grid_tmp.gp_estimation.val[0]));
					double delt_angle2 = fabs(grid_tmp2.gp_estimation.val[0]-grid_tmp.gp_estimation.val[0]) < M_PI_2 ? fabs(grid_tmp2.gp_estimation.val[0]-grid_tmp.gp_estimation.val[0]): (M_PI - fabs(grid_tmp2.gp_estimation.val[0]-grid_tmp.gp_estimation.val[0]));

					double direction_factor2 = (M_PI_2-delt_angle1)*(M_PI_2-delt_angle2);

					//ROS_INFO("direction_factor2 %3f", direction_factor2);
					if(direction_factor2 < DBL_MIN) direction_factor2 = 0.0;
					else direction_factor2 =direction_factor2 /(M_PI_2*M_PI_2);

					//direction_factor1 = 1.0;
					direction_factor_crossing = direction_factor1*direction_factor2;
				}

				float pedPath_factor;
				if(grid_tmp.pedPath_flag) pedPath_factor = 1.0;
				else pedPath_factor = 0.00001;

				grid_tmp.crossing_score = directionVar_factor*direction_factor_crossing*pedPath_factor*grid_tmp.path_score;
			}
		}

		Mat crossing_color(AM_->size_y,  AM_->size_x, CV_32FC1 );
		crossing_color = Scalar(0);
		for(j = 0; j < AM_->size_y; j++)
		{
			for (i = 0; i < AM_->size_x; i++)
			{
				activity_grid &grid_tmp = AM_->cells[MAP_INDEX(AM_, i, j)];
				//if(!grid_tmp.road_flag) continue;
				int x = i;
				int y = AM_->size_y-1-j;
				crossing_color.at<float>(y,x) = (float)(grid_tmp.crossing_score);
			}
		}
		normalize(crossing_color, crossing_color, 0.0, 1.0, NORM_MINMAX);


		for(j = 0; j < AM_->size_y; j++)
		{
			for (i = 0; i < AM_->size_x; i++)
			{
				activity_grid &grid_tmp = AM_->cells[MAP_INDEX(AM_, i, j)];
				//if(!grid_tmp.road_flag) continue;
				int x = i;
				int y = AM_->size_y-1-j;

				crossing_color.at<float>(y,x) = crossing_color.at<float>(y,x) * 3.0;
				if(crossing_color.at<float>(y,x) > 1.0) crossing_color.at<float>(y,x) = 1.0;
				grid_tmp.crossing_score = crossing_color.at<float>(y,x);
			}
		}

		Mat crossing_color_tmp;
		crossing_color.convertTo(crossing_color_tmp, CV_8U, 255.0, 0.0);
		imwrite( "./data/crossing_color_probability.jpg", crossing_color_tmp );
		threshold(crossing_color_tmp, crossing_color_tmp, 127, 255, 0);
		imwrite( "./data/crossing_color.jpg", crossing_color_tmp );
	}


	void AM_learner::view_activity_map()
	{
		show_moving_direction();
		visualize_arrow_ROI();
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

				direction_color.at<float>(y,x) = M_PI - fabs((float)(grid_tmp.gp_estimation.val[0]));
				directionVar_color.at<float>(y,x)  = (float)(grid_tmp.gp_estimation.val[1]);
			}
		}

		double minVal, maxVal;
		minMaxLoc(direction_color, &minVal, &maxVal); //find minimum and maximum intensities

		ROS_INFO("direction_color (minVal, maxVal) (%3f, %3f)", minVal, maxVal);

		Mat draw1;
		direction_color.convertTo(draw1, CV_8U, 255.0/(maxVal - minVal), -minVal * 255.0/(maxVal - minVal));
		//direction_color.convertTo(draw1, CV_8U, 255.0/(M_PI), 0.0);

		Mat draw2;
		minMaxLoc(directionVar_color, &minVal, &maxVal); //find minimum and maximum intensities
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

		//imshow("direction_map", jetcolor);
		//imshow("directionVar_map", jetcolorVar);

		imwrite( "./data/direction_map.png", 	draw1 );
		imwrite( "./data/directionVar_map.png", jetcolorVar );
		//waitKey(0);
	}

	void AM_learner::visualize_arrow_ROI()
	{
		int arrow_square_side = 18;
		int origin_image_neighbour = 3;
		double image_resize_factor = double(arrow_square_side)/(double)origin_image_neighbour;

		Mat road_img_ROI(road_image_, visual_ROI_);
		Mat road_img_ROI_clone = road_img_ROI.clone();
		imwrite("./data/MCDROI_satellite.png", road_img_ROI_clone);

		Mat road_img_ROIresized;
		resize(road_img_ROI_clone, road_img_ROIresized, Size(), image_resize_factor, image_resize_factor);

		Mat arrow_clean(road_img_ROIresized.rows,  road_img_ROIresized.cols, CV_8UC3);
		arrow_clean = Scalar(0);
		Mat directionVar_tmp(road_img_ROIresized.rows,  road_img_ROIresized.cols, CV_32FC1 );
		directionVar_tmp = Scalar(0.0);

		Rect map_ROI_tmp = Rect(visual_ROI_.x, (AM_->size_y - visual_ROI_.y)-visual_ROI_.height, visual_ROI_.width, visual_ROI_.height);
		int arrow_x=0; int arrow_y = 0;

		int i, j;
		for(j = map_ROI_tmp.y; j < map_ROI_tmp.y+map_ROI_tmp.height; j=j+origin_image_neighbour)
		{
			arrow_x=0;
			for(i = map_ROI_tmp.x; i < map_ROI_tmp.x+map_ROI_tmp.width; i=i+origin_image_neighbour)
			{
				//printf("\n");
				//printf("%d, %d\t",j,i);
				double angle_total=0.0; double var_total = 0.0;
				int neightbour_count = 0;
				for(int a=-origin_image_neighbour/2; a <= origin_image_neighbour/2; a++)
				{
					for(int b=-origin_image_neighbour/2; b <= origin_image_neighbour/2; b++)
					{
						angle_total = angle_total + AM_->cells[MAP_INDEX(AM_, i+a, j+b)].gp_estimation.val[0];
						var_total = var_total + AM_->cells[MAP_INDEX(AM_, i+a, j+b)].gp_estimation.val[1];
						//printf("%3f, %3f\t", angle_total, var_total);
						neightbour_count++;
					}
				}

				//due to that the range of angle is [-pi, +pi], straight average may generate strange values;
				//double angle_avg = angle_total/double(neightbour_count);
				double angle_avg = AM_->cells[MAP_INDEX(AM_, i, j)].gp_estimation.val[0];
				double var_avg = var_total/double(neightbour_count);
				//printf("avg %3f\n", angle_avg);

				Point arrow_center;
				arrow_center.x = arrow_square_side*(arrow_x) + arrow_square_side/2;
				arrow_center.y = arrow_square_side*(arrow_y) + arrow_square_side/2;
				arrow_center.y = road_img_ROIresized.rows - arrow_center.y;
				//drawArrow(road_img_ROIresized, arrow_center, arrow_square_side-2, angle_avg, 6, M_PI/180.0*30.0, Scalar(0,0,255), 1);
				drawArrow(arrow_clean, arrow_center, arrow_square_side-2, angle_avg, 6, M_PI/180.0*30.0, Scalar(0,0,255), 1);
				plotVar(directionVar_tmp, arrow_center, arrow_square_side, float(var_avg));
				arrow_x++;
			}
			arrow_y++;
		}

		imwrite("./data/MCDROI_satellite_resized.png", road_img_ROIresized);
		imwrite("./data/MCDROI_arrow_clean.png", arrow_clean);

		double minVal, maxVal;
		minMaxLoc(directionVar_tmp, &minVal, &maxVal);
		ROS_INFO("minVal, maxVal %3f, %3f", minVal, maxVal);

		maxVal = 0.13;
		Mat directionVar8U;
		directionVar_tmp.convertTo(directionVar8U, CV_8U, 255.0/(maxVal - minVal), -minVal * 255.0/(maxVal - minVal));
		directionVar8U = Scalar(255)-directionVar8U;
		imwrite("./data/MCDROI_directionVar_tmp.png", directionVar8U);

		Mat var_color_tmp;
		cvtColor(directionVar8U, var_color_tmp, CV_GRAY2RGB);
		Mat blended_img;
		double alpha = 0.5;
		addWeighted( road_img_ROIresized, alpha, var_color_tmp, 1-alpha, 0.0, blended_img);

		assert(arrow_clean.cols==road_img_ROIresized.cols && arrow_clean.rows==road_img_ROIresized.rows);
		for(j=0; j<arrow_clean.rows; j++)
		{
			for(i=0; i<arrow_clean.cols; i++)
			{
				Vec3b arrow_scalar = arrow_clean.at<Vec3b>(j, i);
				if(arrow_scalar.val[0]+arrow_scalar.val[1]+arrow_scalar.val[2]!=0)
				{
					if(directionVar8U.at<uchar>(j,i)>127)blended_img.at<Vec3b>(j,i)=arrow_scalar;
					else blended_img.at<Vec3b>(j,i)=blended_img.at<Vec3b>(j,i)*0.5 + arrow_scalar*0.5;
				}
			}
		}
		imwrite("./data/MCDROI_blended_img.png", blended_img);
	}

	void AM_learner::drawArrow(Mat img, Point arrow_center, int trunk_lenth, double arrow_angle, int side_length, double alpha,  Scalar color, int thickness)
	{
		Point arrow_head, arrow_tail;
		arrow_head.x = arrow_center.x + int(double(trunk_lenth/2)*cos(arrow_angle));
		//arrow_head.y = arrow_center.y + int(double(trunk_lenth/2)*sin(arrow_angle));

		//due to that the image angle definition is different from the one in ros-map;
		arrow_head.y = arrow_center.y - int(double(trunk_lenth/2)*sin(arrow_angle));
		arrow_tail.x = arrow_center.x + int(double(trunk_lenth/2)*cos(arrow_angle+M_PI));
		arrow_tail.y = arrow_center.y - int(double(trunk_lenth/2)*sin(arrow_angle+M_PI));
		line(img, arrow_head, arrow_tail, color, thickness);

		Point arrow_side1, arrow_side2;
		arrow_side1.x = arrow_head.x + side_length * cos(M_PI+arrow_angle + alpha);
		arrow_side1.y = arrow_head.y - side_length * sin(M_PI+arrow_angle + alpha);
		line(img, arrow_head, arrow_side1, color, thickness);

		arrow_side2.x = arrow_head.x + side_length * cos(M_PI+arrow_angle - alpha);
		arrow_side2.y = arrow_head.y - side_length * sin(M_PI+arrow_angle - alpha);
		line(img, arrow_head, arrow_side2, color, thickness);
	}

	void AM_learner::plotVar(Mat img, Point arrow_center, int side_length, float var_value)
	{
		for(int i=-side_length/2; i<=side_length/2;i++)
		{
			for(int j=-side_length/2; j<=side_length/2;j++)
			{
				if((j+arrow_center.y>=0)&&(j+arrow_center.y<img.rows)&&(i+arrow_center.x>=0)&&(i+arrow_center.x<img.cols))
				img.at<float>(j+arrow_center.y, i+arrow_center.x) = (var_value);
			}
		}
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
	    if((fp_output=fopen("map_data.txt", "w"))==NULL){ROS_ERROR("cannot open output_file\n"); return;}
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

