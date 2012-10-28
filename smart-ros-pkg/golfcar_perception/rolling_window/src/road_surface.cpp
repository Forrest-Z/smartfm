 #include "road_surface.h"
 
namespace golfcar_pcl{
	
	road_surface::road_surface():
	private_nh_("~")
   {
		odom_frame_ = "odom";
		base_frame_ = "base_link";
		tf_ = new tf::TransformListener();
		input_update_flag_ = false;
		
		batchNum_limit_ = 20;
		
		road_surface_odom_.clear();
		road_surface_odom_.height  				= 	1;
		road_surface_odom_.header.frame_id		=	odom_frame_;
		surface_index_batches_.clear();
		
		road_boundary_odom_.clear();
		road_boundary_odom_.height  				=  1;
		road_boundary_odom_.header.frame_id		=	odom_frame_;
		boundary_index_batches_.clear();
		
		pcl::PointXYZ rectangle_p1, rectangle_p2, rectangle_p3, rectangle_p4;
		rectangle_p1.x = 10.0; rectangle_p1.y = 3.0; 
		rectangle_p2.x = 4.0;  rectangle_p2.y = 3.0; 
		rectangle_p3.x = 4.0;  rectangle_p3.y = -3.0; 
		rectangle_p4.x = 10.0; rectangle_p4.y = -3.0; 
		poly_ROI_.push_back(rectangle_p1);
		poly_ROI_.push_back(rectangle_p2);
		poly_ROI_.push_back(rectangle_p3);
		poly_ROI_.push_back(rectangle_p4);

		viewpoint_td_sick_.x = 1.70; viewpoint_td_sick_.y = 0.00; viewpoint_td_sick_.z = 1.53;
		seedPoint_.x = 9.0; seedPoint_.y = 0.0; seedPoint_.z = 0.0;
		
		private_nh_.param("search_radius", search_radius_, 0.2);
		private_nh_.param("curvature_thresh", curvature_thresh_, 0.005);
		private_nh_.param("curvature_visual_limit", curvature_visual_limit_, 0.1);
		private_nh_.param("normalZ_visual_limit", normalZ_visual_limit_, 0.2);
		private_nh_.param("curvature_visualization", curvature_visualization_, true);
		private_nh_.param("normalZ_visualization", normalZ_visualization_, false);
		private_nh_.param("extract_training_data", extract_training_data_, true);

		rolling_pcl_sub_ = new message_filters::Subscriber<RollingPointCloud> (nh_, "rolling_window_pcl", 1);
		pcl_indices_sub_ = new message_filters::Subscriber<rolling_window::pcl_indices> (nh_, "process_fraction_indices", 1);
		sync_	= new message_filters::TimeSynchronizer<RollingPointCloud, rolling_window::pcl_indices>(*rolling_pcl_sub_, *pcl_indices_sub_, 5);
		sync_->registerCallback(boost::bind(&road_surface::pclCallback, this, _1, _2));
		
		odom_sub_.subscribe(nh_, "odom", 10);
		odom_filter_ = new tf::MessageFilter<nav_msgs::Odometry>(odom_sub_, *tf_, base_frame_, 10);
		odom_filter_ ->registerCallback(boost::bind(&road_surface::odomCallback, this, _1));
		odom_filter_->setTolerance(ros::Duration(0.05));
		
		process_fraction_pub_   = 	nh_.advertise<RollingPointCloud>("process_fraction", 10);
		road_surface_pub_   	= 	nh_.advertise<RollingPointCloud>("road_surface_pts", 10);
		road_boundary_pub_   	= 	nh_.advertise<RollingPointCloud>("road_boundary_pts", 10);
		fitting_plane_pub_   	=	nh_.advertise<PointCloud>("fitting_plane", 10);
		normals_poses_pub_ 		=	nh_.advertise<geometry_msgs::PoseArray>("normals_array", 10);
		plane_coef_pub_ 		= 	nh_.advertise<rolling_window::plane_coef>("plane_coef", 10);
		
		surface_all_pub_   		= 	nh_.advertise<RollingPointCloud>("surface_all", 10);
		boundary_all_pub_   	= 	nh_.advertise<RollingPointCloud>("boundary_all", 10);

		planefitting_init_ = false;
		clustering_init_   = false;
		planefitting_disThresh_ = 0.03;
		clustering_disThresh_   = 15.0;

		clusters_pub_ = nh_.advertise<PointCloudRGB>("clusters_RGBD", 10);
		normal_visual_pub_ = nh_.advertise<PointCloudRGB>("normal_visual_RGB", 10);
		surface_slope_pub_ = nh_.advertise<PointCloudRGB>("surface_slope_visual_RGB", 10);
		variance_visual_pub_ = nh_.advertise<PointCloudRGB>("variance_visual_RGB", 10);

		//planes_pub_ = nh_.advertise<PointCloudRGB>("planes_RGBD", 10);
		//pcl_cloud_restPub_ = nh_.advertise<PointCloudNormal>("pcl_cloud_restVisual", 10);

		//table for Jet colormap, 256 indices;
		float r[] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0.00588235294117645,0.02156862745098032,0.03725490196078418,0.05294117647058827,0.06862745098039214,0.084313725490196,0.1000000000000001,0.115686274509804,0.1313725490196078,0.1470588235294117,0.1627450980392156,0.1784313725490196,0.1941176470588235,0.2098039215686274,0.2254901960784315,0.2411764705882353,0.2568627450980392,0.2725490196078431,0.2882352941176469,0.303921568627451,0.3196078431372549,0.3352941176470587,0.3509803921568628,0.3666666666666667,0.3823529411764706,0.3980392156862744,0.4137254901960783,0.4294117647058824,0.4450980392156862,0.4607843137254901,0.4764705882352942,0.4921568627450981,0.5078431372549019,0.5235294117647058,0.5392156862745097,0.5549019607843135,0.5705882352941174,0.5862745098039217,0.6019607843137256,0.6176470588235294,0.6333333333333333,0.6490196078431372,0.664705882352941,0.6803921568627449,0.6960784313725492,0.7117647058823531,0.7274509803921569,0.7431372549019608,0.7588235294117647,0.7745098039215685,0.7901960784313724,0.8058823529411763,0.8215686274509801,0.8372549019607844,0.8529411764705883,0.8686274509803922,0.884313725490196,0.8999999999999999,0.9156862745098038,0.9313725490196076,0.947058823529412,0.9627450980392158,0.9784313725490197,0.9941176470588236,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,0.9862745098039216,0.9705882352941178,0.9549019607843139,0.93921568627451,0.9235294117647062,0.9078431372549018,0.892156862745098,0.8764705882352941,0.8607843137254902,0.8450980392156864,0.8294117647058825,0.8137254901960786,0.7980392156862743,0.7823529411764705,0.7666666666666666,0.7509803921568627,0.7352941176470589,0.719607843137255,0.7039215686274511,0.6882352941176473,0.6725490196078434,0.6568627450980391,0.6411764705882352,0.6254901960784314,0.6098039215686275,0.5941176470588236,0.5784313725490198,0.5627450980392159,0.5470588235294116,0.5313725490196077,0.5156862745098039,0.5};
		float g[] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0.001960784313725483,0.01764705882352935,0.03333333333333333,0.0490196078431373,0.06470588235294117,0.08039215686274503,0.09607843137254901,0.111764705882353,0.1274509803921569,0.1431372549019607,0.1588235294117647,0.1745098039215687,0.1901960784313725,0.2058823529411764,0.2215686274509804,0.2372549019607844,0.2529411764705882,0.2686274509803921,0.2843137254901961,0.3,0.3156862745098039,0.3313725490196078,0.3470588235294118,0.3627450980392157,0.3784313725490196,0.3941176470588235,0.4098039215686274,0.4254901960784314,0.4411764705882353,0.4568627450980391,0.4725490196078431,0.4882352941176471,0.503921568627451,0.5196078431372548,0.5352941176470587,0.5509803921568628,0.5666666666666667,0.5823529411764705,0.5980392156862746,0.6137254901960785,0.6294117647058823,0.6450980392156862,0.6607843137254901,0.6764705882352942,0.692156862745098,0.7078431372549019,0.723529411764706,0.7392156862745098,0.7549019607843137,0.7705882352941176,0.7862745098039214,0.8019607843137255,0.8176470588235294,0.8333333333333333,0.8490196078431373,0.8647058823529412,0.8803921568627451,0.8960784313725489,0.9117647058823528,0.9274509803921569,0.9431372549019608,0.9588235294117646,0.9745098039215687,0.9901960784313726,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,0.9901960784313726,0.9745098039215687,0.9588235294117649,0.943137254901961,0.9274509803921571,0.9117647058823528,0.8960784313725489,0.8803921568627451,0.8647058823529412,0.8490196078431373,0.8333333333333335,0.8176470588235296,0.8019607843137253,0.7862745098039214,0.7705882352941176,0.7549019607843137,0.7392156862745098,0.723529411764706,0.7078431372549021,0.6921568627450982,0.6764705882352944,0.6607843137254901,0.6450980392156862,0.6294117647058823,0.6137254901960785,0.5980392156862746,0.5823529411764707,0.5666666666666669,0.5509803921568626,0.5352941176470587,0.5196078431372548,0.503921568627451,0.4882352941176471,0.4725490196078432,0.4568627450980394,0.4411764705882355,0.4254901960784316,0.4098039215686273,0.3941176470588235,0.3784313725490196,0.3627450980392157,0.3470588235294119,0.331372549019608,0.3156862745098041,0.2999999999999998,0.284313725490196,0.2686274509803921,0.2529411764705882,0.2372549019607844,0.2215686274509805,0.2058823529411766,0.1901960784313728,0.1745098039215689,0.1588235294117646,0.1431372549019607,0.1274509803921569,0.111764705882353,0.09607843137254912,0.08039215686274526,0.06470588235294139,0.04901960784313708,0.03333333333333321,0.01764705882352935,0.001960784313725483,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
		float b[] = {0.5,0.5156862745098039,0.5313725490196078,0.5470588235294118,0.5627450980392157,0.5784313725490196,0.5941176470588235,0.6098039215686275,0.6254901960784314,0.6411764705882352,0.6568627450980392,0.6725490196078432,0.6882352941176471,0.7039215686274509,0.7196078431372549,0.7352941176470589,0.7509803921568627,0.7666666666666666,0.7823529411764706,0.7980392156862746,0.8137254901960784,0.8294117647058823,0.8450980392156863,0.8607843137254902,0.8764705882352941,0.892156862745098,0.907843137254902,0.9235294117647059,0.9392156862745098,0.9549019607843137,0.9705882352941176,0.9862745098039216,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,0.9941176470588236,0.9784313725490197,0.9627450980392158,0.9470588235294117,0.9313725490196079,0.915686274509804,0.8999999999999999,0.884313725490196,0.8686274509803922,0.8529411764705883,0.8372549019607844,0.8215686274509804,0.8058823529411765,0.7901960784313726,0.7745098039215685,0.7588235294117647,0.7431372549019608,0.7274509803921569,0.7117647058823531,0.696078431372549,0.6803921568627451,0.6647058823529413,0.6490196078431372,0.6333333333333333,0.6176470588235294,0.6019607843137256,0.5862745098039217,0.5705882352941176,0.5549019607843138,0.5392156862745099,0.5235294117647058,0.5078431372549019,0.4921568627450981,0.4764705882352942,0.4607843137254903,0.4450980392156865,0.4294117647058826,0.4137254901960783,0.3980392156862744,0.3823529411764706,0.3666666666666667,0.3509803921568628,0.335294117647059,0.3196078431372551,0.3039215686274508,0.2882352941176469,0.2725490196078431,0.2568627450980392,0.2411764705882353,0.2254901960784315,0.2098039215686276,0.1941176470588237,0.1784313725490199,0.1627450980392156,0.1470588235294117,0.1313725490196078,0.115686274509804,0.1000000000000001,0.08431372549019622,0.06862745098039236,0.05294117647058805,0.03725490196078418,0.02156862745098032,0.00588235294117645,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
		for(size_t i=0; i<256; i++)
		{
			jet_r_.push_back(r[i]);
			jet_g_.push_back(g[i]);
			jet_b_.push_back(b[i]);
		}
		record_batch_serial_ = 0;
	}
	
	road_surface::~road_surface()
	{	
	}
	
	inline void road_surface::colormap_jet(float plot_value, float upper_limit_, pcl::RGB &point_out)
	{
		float value_tmp = plot_value < upper_limit_ ? plot_value : upper_limit_;
		int color_serial = floor(256.0 * value_tmp /(upper_limit_+0.0000001));
		point_out.r = (u_int8_t( jet_r_[color_serial] * 255.0));
		point_out.g = (u_int8_t( jet_g_[color_serial] * 255.0));
		point_out.b = (u_int8_t( jet_b_[color_serial] * 255.0));
	}

	//to process input pcl batch with interesting indices: 
	//1st extract normals;
	//2nd region-growing segmentation to extract road surface;
	void road_surface::pclCallback(const RollingPointCloud::ConstPtr& pcl_in, const rolling_window::pcl_indices::ConstPtr &indices_in)
	{
		if(input_update_flag_) ROS_WARN("last road_surface and boundary pcl batch are not accumulated, please check odomCallback");
		
		surface_pts_.clear();
		surface_pts_.height = 1;
		surface_pts_.header =	pcl_in->header;

		boundary_pts_.clear();
		boundary_pts_.height = 1;
		boundary_pts_.header = pcl_in->header;
		
		road_surface::surface_extraction(*pcl_in,  *indices_in, surface_pts_, boundary_pts_);
		
		road_surface_pub_.publish(surface_pts_);
		road_boundary_pub_.publish(boundary_pts_);
	}
	
	//odomCallback triggers several processing functions;
	//1st: extract plane by RANSAC based on the maintained "road_surface" pcl;
	//2nd: do surface subtraction and clustering for other objects;
	void road_surface::odomCallback(const OdomConstPtr& odom)
	{
		road_surface_odom_.header = odom->header;
		road_boundary_odom_.header = odom->header;
		surface_all_pub_.publish(road_surface_odom_);
		boundary_all_pub_.publish(road_boundary_odom_);

		tf::StampedTransform OdomTemp;
		try
		{
			tf_->lookupTransform(odom_frame_, base_frame_, odom->header.stamp, OdomTemp);
		}
		catch(tf::TransformException e)
		{
			ROS_WARN("odom Failed to get fresh tf between odom and baselink, (%s)", e.what()); return;
		}

		//process1: some high-level processing triggered by odom;
		if(planefitting_init_)
		{
			bool planefitting_flag = road_surface::checkDistance(planefitting_OdomMeas_, OdomTemp, planefitting_disThresh_);
			if(planefitting_flag)
			{
				planefitting_OdomMeas_ = OdomTemp;
				RollingPointCloud surface_baselink_tmp;
				road_surface_odom_.header = odom->header;
				pcl_ros::transformPointCloud(base_frame_, road_surface_odom_, surface_baselink_tmp, *tf_ );

				PointCloud fitting_plane;
				fitting_plane.clear();
				fitting_plane.height = 1;
				fitting_plane.header = surface_baselink_tmp.header;
				rolling_window::plane_coef plane_coef;
				planefitting_ROI(surface_baselink_tmp, poly_ROI_, fitting_plane, plane_coef);
				fitting_plane_pub_.publish(fitting_plane);
			    plane_coef_pub_.publish(plane_coef);
			}
		}
		else
		{
			planefitting_init_ = true;
			planefitting_OdomMeas_ = OdomTemp;
		}

		//process1: some high-level processing triggered by odom;
		if(clustering_init_)
		{
			bool clustering_flag = road_surface::checkDistance(clustering_OdomMeas_, OdomTemp, clustering_disThresh_);
			if(clustering_flag && raw_pcl_batches_.size() == batchNum_limit_)
			{
			    clustering_OdomMeas_ = OdomTemp;
			    //add clustering function here, refer to "single_window.cpp" line 158-192;
			    //publish as RGBD pointcloud;

			    //1st step: prepare accumulated raw pcl, and combine surface indices;
			    RollingPointCloud accumulated_raw_pcl;
			    accumulated_raw_pcl.clear();
			    accumulated_raw_pcl.height = 1;
			    //pay attention to the serial calculation here;
			    size_t serial_base = 0;
			    pcl::PointIndices::Ptr surface_indices (new pcl::PointIndices);
			    pcl::PointIndices::Ptr outlier_indices (new pcl::PointIndices);
			    for(size_t batch =0; batch < raw_pcl_batches_.size()-1; batch++)
			    {
			    	accumulated_raw_pcl = accumulated_raw_pcl+raw_pcl_batches_[batch];
			    	for(size_t serial=0; serial < surface_index_batches_[batch].size(); serial++)
			    	{
			    		int serial_tmp = surface_index_batches_[batch][serial] + serial_base;
			    		surface_indices->indices.push_back(serial_tmp);
			    	}

			    	for(size_t serial=0; serial < outlier_index_batches_[batch].size(); serial++)
					{
						int serial_tmp = outlier_index_batches_[batch][serial] + serial_base;
						outlier_indices->indices.push_back(serial_tmp);
					}
			    	serial_base = serial_base + raw_pcl_batches_[batch].size();
			    }

			    pcl::PointIndices::Ptr total_indices (new pcl::PointIndices);
			    for(size_t serial=0; serial < surface_indices->indices.size(); serial++)
				{
					int serial_tmp = surface_indices->indices[serial];
					total_indices->indices.push_back(serial_tmp);
				}
			    for(size_t serial=0; serial < outlier_indices->indices.size(); serial++)
				{
					int serial_tmp = outlier_indices->indices[serial];
					total_indices->indices.push_back(serial_tmp);
				}

			    //2nd step: extract surface, and then do clustering for the rest;
			    pcl::ExtractIndices<RollingPointXYZ> extract;
				pcl::PointCloud<RollingPointXYZ>::Ptr surface_plane (new pcl::PointCloud<RollingPointXYZ> ());
				extract.setInputCloud (accumulated_raw_pcl.makeShared());
				extract.setIndices (surface_indices);
				extract.setNegative (false);
				extract.filter (*surface_plane);

				PointCloudRGB clusters_tmp;
				clusters_tmp.clear();
				clusters_tmp.header = odom->header;
				clusters_tmp.height = 1;

				pcl::PointXYZRGB xyzRGB_pt;

				xyzRGB_pt.r = 0;
				xyzRGB_pt.g = 0;
				xyzRGB_pt.b = 255;
				for(size_t sp=0; sp <surface_plane->points.size(); sp++)
				{
					xyzRGB_pt.x = surface_plane->points[sp].x;
					xyzRGB_pt.y = surface_plane->points[sp].y;
					xyzRGB_pt.z = surface_plane->points[sp].z;
					clusters_tmp.push_back(xyzRGB_pt);
				}

				pcl::PointCloud<RollingPointXYZ>::Ptr other_points (new pcl::PointCloud<RollingPointXYZ>);

				extract.setIndices (total_indices);
				extract.setNegative (true);
				extract.filter (*other_points);
				std::cout << "other_points size "<<other_points->points.size()<<endl;

				pcl::search::KdTree<RollingPointXYZ>::Ptr tree (new pcl::search::KdTree<RollingPointXYZ>);
				tree->setInputCloud (other_points);

				std::vector<pcl::PointIndices> cluster_indices;
				pcl::EuclideanClusterExtraction<RollingPointXYZ> ec;
				ec.setClusterTolerance (0.1); // 2cm
				ec.setMinClusterSize (50);
				ec.setMaxClusterSize (250000);
				ec.setSearchMethod (tree);
				ec.setInputCloud (other_points);
				ec.extract (cluster_indices);

				srand ( time(NULL) );
				std::cout << "cluster_indices size "<<cluster_indices.size()<<endl;
				for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
				{
					xyzRGB_pt.r = rand() % 255;
					xyzRGB_pt.g = rand() % 255;
					xyzRGB_pt.b = rand() % 255;

					for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); pit++)
					{
						pcl::PointXYZ point_tmp;
						xyzRGB_pt.x = other_points->points[*pit].x;
						xyzRGB_pt.y = other_points->points[*pit].y;
						xyzRGB_pt.z = other_points->points[*pit].z;
						clusters_tmp.push_back(xyzRGB_pt);
					}
				}
				clusters_pub_.publish(clusters_tmp);

				//to visualize normal and curvature in color;
				PointCloudRGBNormal normal_visual_tmp;
				normal_visual_tmp.clear();
				normal_visual_tmp.header = odom->header;
				normal_visual_tmp.height = 1;

				for(size_t batch =0; batch < pcl_normal_batches_.size()-1; batch++)
				{
					for(size_t serial=0; serial < pcl_normal_batches_[batch].size(); serial++)
					{
						pcl::RGB RGB_tmp;
						RollingPointXYZNormal pointNormal_tmp;
						pcl::PointXYZRGBNormal pointRBGNormal_tmp;
						pointNormal_tmp = pcl_normal_batches_[batch][serial];

						if(curvature_visualization_)
						{
							road_surface::colormap_jet(pointNormal_tmp.curvature,curvature_visual_limit_, RGB_tmp);
						}
						else if(normalZ_visualization_)
						{
							road_surface::colormap_jet(pointNormal_tmp.normal_z, normalZ_visual_limit_, RGB_tmp);
						}
						pointRBGNormal_tmp.r = RGB_tmp.r;
						pointRBGNormal_tmp.g = RGB_tmp.g;
						pointRBGNormal_tmp.b = RGB_tmp.b;

						pointRBGNormal_tmp.x = pointNormal_tmp.x;
						pointRBGNormal_tmp.y = pointNormal_tmp.y;
						pointRBGNormal_tmp.z = pointNormal_tmp.z;
						pointRBGNormal_tmp.curvature = pointNormal_tmp.curvature;
						pointRBGNormal_tmp.normal_x = pointNormal_tmp.normal_x;
						pointRBGNormal_tmp.normal_y = pointNormal_tmp.normal_y;
						pointRBGNormal_tmp.normal_z = pointNormal_tmp.normal_z;
						normal_visual_tmp.push_back(pointRBGNormal_tmp);
					}
				}
				normal_visual_pub_.publish(normal_visual_tmp);

				PointCloudRGB variance_visualization;
				variance_visualization.clear();
				variance_visualization.header = odom->header;
				variance_visualization.height = 1;

				for(size_t batch =0; batch < raw_pcl_batches_.size()-1; batch++)
				{
					for(size_t serial=0; serial < raw_pcl_batches_[batch].size(); serial++)
					{
						RollingPointXYZ point_tmp;
						point_tmp = raw_pcl_batches_[batch][serial];

						pcl::RGB RGB_tmp;
						pcl::PointXYZRGB pointRBG_tmp;
						road_surface::colormap_jet(point_tmp.z_var, 0.1, RGB_tmp);
						pointRBG_tmp.r = RGB_tmp.r;
						pointRBG_tmp.g = RGB_tmp.g;
						pointRBG_tmp.b = RGB_tmp.b;
						pointRBG_tmp.x = point_tmp.x;
						pointRBG_tmp.y = point_tmp.y;
						pointRBG_tmp.z = point_tmp.z;
						variance_visualization.push_back(pointRBG_tmp);
					}
				}
				variance_visual_pub_.publish(variance_visualization);

				//newly added function "plane-fitting" for patches;
				//it needs more work;
				/*
				pcl::ExtractIndices<pcl::PointNormal> extract2;
				pcl::PointCloud<pcl::PointNormal> rest_pcl;
				extract2.setInputCloud (normal_pcl_tmp.makeShared());
				extract2.setIndices (total_indices);
				extract2.setNegative (true);
				extract2.filter (rest_pcl);

				pcl::ConditionAnd<pcl::PointNormal>::Ptr range_cond (new pcl::ConditionAnd<pcl::PointNormal> ());
				range_cond->addComparison (pcl::FieldComparison<pcl::PointNormal>::ConstPtr (new
				pcl::FieldComparison<pcl::PointNormal> ("curvature", pcl::ComparisonOps::LT, curvature_thresh_)));
		 	    pcl::ConditionalRemoval<pcl::PointNormal> condrem (range_cond);
			    condrem.setInputCloud (rest_pcl.makeShared());

			    PointCloudNormal::Ptr pcl_cloud_filtered (new pcl::PointCloud<pcl::PointNormal>);
			    condrem.filter(*pcl_cloud_filtered);
			    pcl_cloud_filtered->header = odom->header;
			    pcl_cloud_restPub_.publish(*pcl_cloud_filtered);

			    pcl::search::KdTree<pcl::PointNormal>::Ptr tree2 (new pcl::search::KdTree<pcl::PointNormal>);
				tree2->setInputCloud (pcl_cloud_filtered);
			    std::vector<pcl::PointIndices> cluster_indices2;
				pcl::EuclideanClusterExtraction<pcl::PointNormal> ec2;
				ec2.setClusterTolerance (0.1);
				ec2.setMinClusterSize (100);
				ec2.setMaxClusterSize (250000);
				ec2.setSearchMethod (tree2);
				ec2.setInputCloud (pcl_cloud_filtered);
				ec2.extract (cluster_indices2);

				PointCloudRGB planes_tmp;
				planes_tmp.clear();
				planes_tmp.header = odom->header;
				planes_tmp.height = 1;

				for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices2.begin (); it != cluster_indices2.end (); ++it)
				{
					PointCloudNormal cluster_pcl_tmp;
					extract2.setInputCloud (pcl_cloud_filtered);
					pcl::PointIndices::Ptr cluster_indices_tmp (new pcl::PointIndices);
					cluster_indices_tmp->indices = it->indices;
					extract2.setIndices (cluster_indices_tmp);
					extract2.setNegative (false);
					extract2.filter (cluster_pcl_tmp);

					pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
					pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
					pcl::SACSegmentation<pcl::PointNormal> seg;
					seg.setOptimizeCoefficients (true);
					seg.setModelType (pcl::SACMODEL_PLANE);
					seg.setMethodType (pcl::SAC_RANSAC);
					seg.setDistanceThreshold (0.05);
					seg.setInputCloud (cluster_pcl_tmp.makeShared ());
					seg.segment (*inliers, *coefficients);

					if ((coefficients->values.size()==4))
					{
						double inlier_num = inliers->indices.size();
						double total_num  = cluster_pcl_tmp.points.size();
						if(inlier_num/total_num > 0.7)
						{
							pcl::PointXYZRGB xyzRGB_pt;
							xyzRGB_pt.r = rand() % 255;
							xyzRGB_pt.g = rand() % 255;
							xyzRGB_pt.b = rand() % 255;

							for  (size_t pit = 0; pit < inliers->indices.size (); pit++)
							{
								xyzRGB_pt.x = cluster_pcl_tmp.points[inliers->indices[pit]].x;
								xyzRGB_pt.y = cluster_pcl_tmp.points[inliers->indices[pit]].y;
								xyzRGB_pt.z = cluster_pcl_tmp.points[inliers->indices[pit]].z;
								planes_tmp.push_back(xyzRGB_pt);
							}
						}
					}
				}
				planes_pub_.publish(planes_tmp);
				*/
			}
		}
		else
		{
			clustering_init_ = true;
			clustering_OdomMeas_ = OdomTemp;
		}

	}
	
	void road_surface::surface_extraction (const RollingPointCloud &cloud_in, const rolling_window::pcl_indices& proc_indices,
											RollingPointCloud & surface_pts, RollingPointCloud & boundary_pts)
	{
		record_batch_serial_++;

		RollingPointCloud process_fraction_pcl;
		process_fraction_pcl.header = cloud_in.header;
		process_fraction_pcl.clear();
		process_fraction_pcl.height = 1;
		for(size_t i=0; i<proc_indices.indices.size(); i++)
		{
			size_t point_index = proc_indices.indices[i];
			process_fraction_pcl.points.push_back(cloud_in.points[point_index]);
			process_fraction_pcl.width ++;
		}
		process_fraction_pub_.publish(process_fraction_pcl);
		
		fmutil::Stopwatch sw;

		//process the cloud_in: a. calculate norms; b. extract road_boundary;
		//http://pointclouds.org/documentation/tutorials/kdtree_search.php
		//http://pointclouds.org/documentation/tutorials/how_features_work.php#how-3d-features-work
		sw.start("1. calculate pcl normals");
		//1st calculate the normals;
		pcl::NormalEstimationOMP<RollingPointXYZ, pcl::Normal> ne;
		ne.setInputCloud (cloud_in.makeShared());
		boost::shared_ptr<std::vector<int> > indicesptr (new std::vector<int> (proc_indices.indices));
		ne.setIndices (indicesptr);
		pcl::search::KdTree<RollingPointXYZ>::Ptr tree (new pcl::search::KdTree<RollingPointXYZ> ());
		ne.setSearchMethod (tree);
		pcl::PointCloud<pcl::Normal> cloud_normals;
		ne.setRadiusSearch (search_radius_);
		ne.setViewPoint(viewpoint_td_sick_.x,viewpoint_td_sick_.y,viewpoint_td_sick_.z);
		// Compute the features
		ne.compute (cloud_normals);
		// concatentate the fileds
		pcl::PointCloud<RollingPointXYZNormal> point_normals;
		pcl::concatenateFields(process_fraction_pcl, cloud_normals, point_normals);
		publishNormal(point_normals);
		sw.end();
		
		//2nd region-growing method for surface extraction;
		sw.start("2. region-growing to extract surface");
		pcl::KdTreeFLANN<RollingPointXYZ> kdtree;
		if(process_fraction_pcl.points.size()==0) return;
		kdtree.setInputCloud (process_fraction_pcl.makeShared());
		//just to find the initial seed of road surface;
		int K = 1;
		std::vector<int> pointIdxNKNSearch(K);
		std::vector<float> pointNKNSquaredDistance(K);
		
		int num = kdtree.nearestKSearch (seedPoint_, K, pointIdxNKNSearch, pointNKNSquaredDistance);
		
		if(num!=1){std::cout<<"ERROR When to find surface seed"<<endl; return;}
	
		pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
		pcl::PointIndices::Ptr boundary_inliers (new pcl::PointIndices);
		
		RollingPointXYZ searchPt_tmp;
		searchPt_tmp = process_fraction_pcl.points[pointIdxNKNSearch[0]];
		inliers->indices.push_back(pointIdxNKNSearch[0]);
		
		for(unsigned int pt=0; pt<inliers->indices.size(); pt++)
		{
			unsigned int serial = inliers->indices[pt];
			//std::cout <<"serial  "<< serial<< endl;	
			int neighbor_num = 8;
			std::vector<int> nbPts(neighbor_num);
			std::vector<float> nbPtDis(neighbor_num); 
			
			searchPt_tmp = process_fraction_pcl.points[serial];
			int num = kdtree.nearestKSearch (searchPt_tmp, neighbor_num, nbPts, nbPtDis);
			if(num!=neighbor_num) std::cout<<"neighbour points not enought, error"<<endl;
			for(unsigned int neighbour_pt=0; neighbour_pt <nbPts.size(); neighbour_pt++)
			{
				 int nb_serial = nbPts[neighbour_pt];
				 bool curvature_flat = point_normals.points[nb_serial].curvature <= curvature_thresh_;
				 if(curvature_flat)
				 {
					 bool no_copy = true;
					 for(unsigned int all_pt=0; all_pt<inliers->indices.size(); all_pt++)
					 {
						  if(inliers->indices[all_pt] == nb_serial) no_copy = false;
					 }
					 if(no_copy) inliers->indices.push_back(nb_serial);	 
				 }
				 else
				 {
					 bool no_copy = true;
					 for(unsigned int bd_pt=0; bd_pt< boundary_inliers->indices.size(); bd_pt++)
					 {
						  if(boundary_inliers->indices[bd_pt] == nb_serial) no_copy = false;
					 }
					 if(no_copy){boundary_inliers->indices.push_back(nb_serial);}
					 
				 }
			}
		}
		sw.end();
		
		sw.start("3. filter the noisy boundary");
		if(inliers->indices.size()==0) {ROS_WARN("no surface extracted!!!"); return;}

		/*
		pcl::ExtractIndices<RollingPointXYZ> extract;
		extract.setInputCloud (process_fraction_pcl.makeShared());
		extract.setIndices (inliers);
		extract.setNegative (false);
		extract.filter (surface_pts);

		pcl::ExtractIndices<RollingPointXYZ> extract_bd;
		extract_bd.setInputCloud (process_fraction_pcl.makeShared());
		extract_bd.setIndices (boundary_inliers);
		extract_bd.setNegative (false);
		extract_bd.filter (boundary_pts);
		*/

		input_update_flag_ = true;
		//to process the last but one PCLs;

		//1st maintain the "road_surface" and "boundary" in the odom frame, by erasing obsolete pcl and add new incoming pcl;
		//a. maintain raw rolling window data by a vector of pcl batches, which is delayed by one step than that in "rolling_window_pcl";
		//b. keep extracted surface and boundary data by indices corresponding to each pcl batch in the vector;
		RollingPointCloud pcl_odom_tmp;
		pcl_ros::transformPointCloud(odom_frame_, process_fraction_pcl, pcl_odom_tmp, *tf_ );
		raw_pcl_batches_.push_back(pcl_odom_tmp);

		RollingPointNormalCloud pcl_normal_odom_tmp;
		pcl_ros::transformPointCloud(odom_frame_, point_normals, pcl_normal_odom_tmp, *tf_ );
		pcl_normal_batches_.push_back(pcl_normal_odom_tmp);

		surface_index_batches_.push_back(inliers->indices);
		boundary_index_batches_.push_back(boundary_inliers->indices);

		//erase old history of road surface points;
		assert( pcl_normal_batches_.size() == raw_pcl_batches_.size() && raw_pcl_batches_.size() == surface_index_batches_.size()&& surface_index_batches_.size() == boundary_index_batches_.size());
		if(raw_pcl_batches_.size() > batchNum_limit_)
		{
			raw_pcl_batches_.erase (raw_pcl_batches_.begin());
			pcl_normal_batches_.erase (pcl_normal_batches_.begin());
			surface_index_batches_.erase (surface_index_batches_.begin());
			boundary_index_batches_.erase (boundary_index_batches_.begin());
		}


		//2nd filter, update and process road_surface and road_boundary point; connect differ surface batches;
		//a. filter road boundary noise, and repair holes in the surface;
		//b. update "road_surface_odom_" and "road_boundary_odom_";
		ROS_INFO("raw_pcl_batches_ size() %ld", raw_pcl_batches_.size());

		if(raw_pcl_batches_.size()>=3)
		{
			//a---1st step: to prepare the boundary to be filtered, and its supporting surface and raw pcl---
			RollingPointCloud supporting_raw_pcl, supporting_road_surface;
			supporting_raw_pcl.clear();
			supporting_raw_pcl.height = 1;
			supporting_road_surface.clear();
			supporting_road_surface.height = 1;

			RollingPointNormalCloud supporting_normal_pcl;
			supporting_normal_pcl.clear();
			supporting_normal_pcl.height = 1;

			for(vector<RollingPointCloud>::iterator it=raw_pcl_batches_.end()-3; it< raw_pcl_batches_.end(); it++)
			{
				supporting_raw_pcl = supporting_raw_pcl + *it;

			}
			supporting_raw_pcl.width = supporting_raw_pcl.points.size();
			supporting_raw_pcl.header.stamp = cloud_in.header.stamp;
			supporting_raw_pcl.header.frame_id = odom_frame_;
			pcl_ros::transformPointCloud(base_frame_, supporting_raw_pcl, supporting_raw_pcl, *tf_ );

			for(vector<RollingPointNormalCloud>::iterator it=pcl_normal_batches_.end()-3; it< pcl_normal_batches_.end(); it++)
			{
				supporting_normal_pcl = supporting_normal_pcl + *it;
			}
			supporting_normal_pcl.width = supporting_normal_pcl.points.size();
			supporting_normal_pcl.header.stamp = cloud_in.header.stamp;
			supporting_normal_pcl.header.frame_id = odom_frame_;
			pcl_ros::transformPointCloud(base_frame_, supporting_normal_pcl, supporting_normal_pcl, *tf_ );

			for(size_t i=surface_index_batches_.size()-3; i< surface_index_batches_.size(); i++)
			{
				RollingPointCloud surface_pts_tmp;

				pcl::ExtractIndices<RollingPointXYZ> extract_surface;
				extract_surface.setInputCloud (raw_pcl_batches_[i].makeShared());
				pcl::PointIndices::Ptr surface_indices_tmp (new pcl::PointIndices);
				surface_indices_tmp->indices = surface_index_batches_[i];
				extract_surface.setIndices (surface_indices_tmp);
				extract_surface.setNegative (false);
				extract_surface.filter (surface_pts_tmp);
				supporting_road_surface = supporting_road_surface + surface_pts_tmp;
			}
			supporting_road_surface.width = supporting_road_surface.points.size();
			supporting_road_surface.header.stamp = cloud_in.header.stamp;
			supporting_road_surface.header.frame_id = odom_frame_;
			pcl_ros::transformPointCloud(base_frame_, supporting_road_surface, supporting_road_surface, *tf_ );

			surface_pts.points = supporting_road_surface.points;
			surface_pts.width = supporting_road_surface.width;

			//only process the last but one "lb2" boundary batch;
			RollingPointCloud raw_pcl_lb2;
			RollingPointNormalCloud normal_pcl_lb2;
			pcl::PointIndices::Ptr boundary_inliers_lb2 (new pcl::PointIndices);
			raw_pcl_lb2 = raw_pcl_batches_[raw_pcl_batches_.size()-2];
			boundary_inliers_lb2 -> indices = boundary_index_batches_[boundary_index_batches_.size()-2];
			raw_pcl_lb2.header.stamp = cloud_in.header.stamp;
			raw_pcl_lb2.header.frame_id = odom_frame_;
			pcl_ros::transformPointCloud(base_frame_, raw_pcl_lb2, raw_pcl_lb2, *tf_ );

			normal_pcl_lb2 = pcl_normal_batches_[pcl_normal_batches_.size()-2];
			normal_pcl_lb2.header.stamp = cloud_in.header.stamp;
			normal_pcl_lb2.header.frame_id = odom_frame_;
			pcl_ros::transformPointCloud(base_frame_, normal_pcl_lb2, normal_pcl_lb2, *tf_ );

			pcl::PointIndices::Ptr boundary_inliers_tmp (new pcl::PointIndices);

			//a---2nd step: filter the boundary points using several strategies---

			/*
			pcl::KdTreeFLANN<RollingPointXYZ> kdtree_filter;
			kdtree_filter.setInputCloud (supporting_road_surface.makeShared());
			for(unsigned int bd_pt=0; bd_pt< boundary_inliers_lb2->indices.size(); bd_pt++)
			{
				RollingPointXYZ searchPt_tmp = raw_pcl_lb2.points[boundary_inliers_lb2->indices[bd_pt]];

				std::vector<int> pointIdxRadiusSearch;
				std::vector<float> pointRadiusSquaredDistance;

				//-----------------check the point density in boundary point's neighborhood;--------------------
				//std::vector<int> pointIdxRadiusSearch_PF;
				//std::vector<float> pointRadiusSquaredDistance_PF;
				//kdtree_PF.radiusSearch (searchPt_tmp, 0.3, pointIdxRadiusSearch_PF, pointRadiusSquaredDistance_PF);
				//size_t bdPt_neighborNum = pointIdxRadiusSearch_PF.size();
				//ROS_INFO("boundary point neighbors %ld", bdPt_neighborNum);

				std::vector <float> angles_tmp;
				//at least 4 support points from road surface pts;
				if(kdtree_filter.radiusSearch (searchPt_tmp, 0.3, pointIdxRadiusSearch, pointRadiusSquaredDistance)> 3 )
				{
					//ROS_INFO("searchPt_tmp (%3f, %3f)", searchPt_tmp.x, searchPt_tmp.y);
					for(size_t sp_pt=0; sp_pt< pointIdxRadiusSearch.size(); sp_pt++)
					{
						RollingPointXYZ supportPt_tmp = supporting_road_surface.points[pointIdxRadiusSearch[sp_pt]];
						float angle_tmp = atan2f(supportPt_tmp.y-searchPt_tmp.y, supportPt_tmp.x-searchPt_tmp.x);
						angles_tmp.push_back(angle_tmp);
						//printf("supportPt_tmp (%3f, %3f), angle %3f\t", supportPt_tmp.x, supportPt_tmp.y, angle_tmp);
					}

					//bubble sorting for small-to-big;
					size_t i,j;
					for(i=0; i<angles_tmp.size(); i++)
					{
						for(j=0;j<i;j++)
						{
							if(angles_tmp[i]<angles_tmp[j])
							{
								 float temp=angles_tmp[i]; //swap
								 angles_tmp[i]=angles_tmp[j];
								 angles_tmp[j]=temp;
							}
						}
					}

					//here is tricky;
					float max_delt_angle = 0;
					float delt_angle_tmp = 2*M_PI - fabsf(angles_tmp.back()-angles_tmp.front());
					if(delt_angle_tmp> max_delt_angle) max_delt_angle = delt_angle_tmp;
					for(i=1; i<angles_tmp.size(); i++)
					{
						delt_angle_tmp = fabsf(angles_tmp[i]-angles_tmp[i-1]);
						if(delt_angle_tmp> max_delt_angle) max_delt_angle = delt_angle_tmp;
					}

					//ROS_INFO("max_delt_angle %3f", max_delt_angle);
					if(max_delt_angle < M_PI_4)
					{
						ROS_INFO("boundary points filtered: max_delt_angle %3f", max_delt_angle);
						//this boundary point is a fake point;
						//incoporate it into the surface pcl;
					}
					else
					{
						boundary_inliers_tmp->indices.push_back(boundary_inliers_lb2->indices[bd_pt]);
						//ROS_INFO("boundary points laser serial %ld", raw_pcl_lb2.points[boundary_inliers_lb2->indices[bd_pt]].laser_serial);
					}
				}
				else
				{
					boundary_inliers_tmp->indices.push_back(boundary_inliers_lb2->indices[bd_pt]);
				}
			}
			*/

			//1st: validate laser scan integrity;
			typedef std::map < float, RollingPointXYZNormal> angle_pt_map;
			std::map< size_t , angle_pt_map > boundary_batches;

			for(unsigned int bd_pt=0; bd_pt< boundary_inliers_lb2->indices.size(); bd_pt++)
			{
				RollingPointXYZNormal boundaryPt_tmp = normal_pcl_lb2.points[boundary_inliers_lb2->indices[bd_pt]];
				size_t laser_serail = boundaryPt_tmp.laser_serial;
				if(boundary_batches.find(laser_serail) == boundary_batches.end())
				{
					angle_pt_map batch_tmp;
					batch_tmp[boundaryPt_tmp.beam_angle] = boundaryPt_tmp;
					boundary_batches[laser_serail] = batch_tmp;
				}
				else
				{
					boundary_batches[laser_serail][boundaryPt_tmp.beam_angle] = boundaryPt_tmp;
				}
			}
			std::map< size_t , boundary_scan_info > boundary_noiseNum_batches;


			for(std::map <size_t,angle_pt_map>::const_iterator batch_it = boundary_batches.begin(); batch_it != boundary_batches.end(); batch_it++ )
			{
				size_t laser_serial = (*batch_it).first;
				angle_pt_map batch_tmp = (*batch_it).second;
				boundary_scan_info info_tmp;

				std::map <float,RollingPointXYZNormal>::const_iterator pt_it = batch_tmp.begin();
				info_tmp.BDptNum = batch_tmp.size();
				info_tmp.pitch_speed = (*pt_it).second.pitch_speed;
				info_tmp.pitch = (*pt_it).second.pitch;
				info_tmp.roll = (*pt_it).second.roll;

				std::vector <float> beam_angles;
				for(pt_it = batch_tmp.begin(); pt_it != batch_tmp.end(); pt_it++ )
				{
					beam_angles.push_back((*pt_it).first);
				}
				//bubble sorting for small-to-big;
				size_t i,j;
				for(i=0; i<beam_angles.size(); i++)
				{
					for(j=0;j<i;j++)
					{
						if(beam_angles[i]<beam_angles[j])
						{
							 float temp=beam_angles[i]; //swap
							 beam_angles[i]=beam_angles[j];
							 beam_angles[j]=temp;
						}
					}
				}

				double max_distance = 0.0;
				for(i =1; i< beam_angles.size(); i++)
				{
					RollingPointXYZNormal pt1, pt2;
					pt1 = batch_tmp[beam_angles[i-1]];
					pt2 = batch_tmp[beam_angles[i]];
					double dist_tmp = fmutil::distance(pt1, pt2);
					if(dist_tmp > max_distance) max_distance = dist_tmp;
				}

				//give it a nominal distance in case that only one boundary point exists;
				if(max_distance == 0.0) max_distance = 4.0;
				info_tmp.longest_horizontal_dist = max_distance;

				boundary_noiseNum_batches[laser_serial]= info_tmp;
			}

			//try to record training data for laser scans;
			if(extract_training_data_)
			{
				stringstream  name_string;
				name_string<<"/home/baoxing/training_data/scan_data";
				const char *input_name = name_string.str().c_str();
				if((fp_=fopen(input_name, "a"))==NULL){ROS_ERROR("cannot open file\n");return;}
				else{ROS_INFO("file opened successfully!");}
			}

			std::map< size_t , bool> boundary_flag_batches;
			for(std::map <size_t,boundary_scan_info>::const_iterator batch_it = boundary_noiseNum_batches.begin(); batch_it != boundary_noiseNum_batches.end(); batch_it++ )
			{
				size_t laser_serial = (*batch_it).first;
				boundary_scan_info batch_tmp = (*batch_it).second;
				bool noisy_or_not =  false;
				ROS_INFO("batch_tmp informaiton: %d, %3f, %3f", batch_tmp.BDptNum, batch_tmp.longest_horizontal_dist, batch_tmp.pitch_speed);

				//classification1 step for one single laser scan, to be replaced by some learning methods;
				if(batch_tmp.BDptNum >= 10 && batch_tmp.longest_horizontal_dist < 2.0 && fabs(batch_tmp.pitch_speed) >=0.02)
				{
					noisy_or_not = true; ROS_WARN("-----------------filter this batch-----------");
				}
				//boundary_flag_batches[laser_serial] = noisy_or_not;
				boundary_flag_batches[laser_serial] = false;

				if(extract_training_data_)
				{
					ROS_INFO("write training data");
					fprintf(fp_, "%ld\t%ld\t", record_batch_serial_, laser_serial);
					fprintf(fp_, "%d\t%5f\t%5f\t%5f\t%5f\t", batch_tmp.BDptNum, batch_tmp.longest_horizontal_dist, batch_tmp.pitch_speed, batch_tmp.pitch, batch_tmp.roll);
					int tentative_value;
					if(noisy_or_not) {tentative_value = 1;}
					else{tentative_value = 0;}
					fprintf(fp_, "%d\n",tentative_value);
				}
			}

			if(extract_training_data_){fclose(fp_); ROS_INFO("file closed");}


			pcl::PointIndices::Ptr outlier_tmp (new pcl::PointIndices);
			for(size_t ip=0; ip<raw_pcl_lb2.points.size(); ip++)
			{
				RollingPointXYZ rawPttmp = raw_pcl_lb2.points[ip];
				size_t laser_serail_tmp = rawPttmp.laser_serial;
				bool point_noisy = false;;
				if(boundary_flag_batches.find(laser_serail_tmp) == boundary_flag_batches.end())
				{
					point_noisy = true;
				}
				else if(boundary_flag_batches[laser_serail_tmp])
				{
					point_noisy = true;
				}

				if(point_noisy) outlier_tmp->indices.push_back(ip);
			}

			//2nd: fine-tune the boundary points;
			pcl::KdTreeFLANN<RollingPointXYZNormal> kdtree_filter_tuneBD;
			kdtree_filter_tuneBD.setInputCloud (supporting_normal_pcl.makeShared());
			for(unsigned int bd_pt=0; bd_pt< boundary_inliers_lb2->indices.size(); bd_pt++)
			{
				RollingPointXYZNormal searchPt_tmp = normal_pcl_lb2.points[boundary_inliers_lb2->indices[bd_pt]];
				//float max_local_curvature = searchPt_tmp.curvature;
				float local_y = searchPt_tmp.y;
				int searchPt_serial_tmp = boundary_inliers_lb2->indices[bd_pt];

				std::vector<int> pointIdxRadiusSearch;
				std::vector<float> pointRadiusSquaredDistance;
				std::vector <float> delt_Zs;
				//at least 4 support points from road surface pts;
				if(kdtree_filter_tuneBD.radiusSearch (searchPt_tmp, 0.3, pointIdxRadiusSearch, pointRadiusSquaredDistance)> 3 )
				{
					for(size_t sp_pt=0; sp_pt< pointIdxRadiusSearch.size(); sp_pt++)
					{
						RollingPointXYZNormal supportPt_tmp = normal_pcl_lb2.points[pointIdxRadiusSearch[sp_pt]];

						//supportPt_tmp is from the same scan, and it is not in the boundary point set;
						if(supportPt_tmp.laser_serial == searchPt_tmp.laser_serial)
						{
							if(boundary_batches[searchPt_tmp.laser_serial].find(supportPt_tmp.beam_angle) == boundary_batches[searchPt_tmp.laser_serial].end())
							{
								if(supportPt_tmp.curvature >= searchPt_tmp.curvature && (local_y>0 && supportPt_tmp.y > local_y) && fabsf(supportPt_tmp.z -searchPt_tmp.z) < 0.1)
								{
									local_y = supportPt_tmp.y;
									searchPt_serial_tmp =pointIdxRadiusSearch[sp_pt];
								}
								else if(supportPt_tmp.curvature >= searchPt_tmp.curvature && (local_y <0.0 && supportPt_tmp.y < local_y) && fabsf(supportPt_tmp.z -searchPt_tmp.z) < 0.1)
								{
									local_y = supportPt_tmp.y;
									searchPt_serial_tmp =pointIdxRadiusSearch[sp_pt];
								}
							}
						}
					}
				}
				boundary_inliers_tmp->indices.push_back(searchPt_serial_tmp);
			}
			boundary_index_batches_[boundary_index_batches_.size()-2] = boundary_inliers_tmp->indices;

			pcl::PointIndices::Ptr boundary_inliers_tmp2 (new pcl::PointIndices);
			boundary_inliers_lb2 -> indices = boundary_index_batches_[boundary_index_batches_.size()-2];
			pcl::KdTreeFLANN<RollingPointXYZ> kdtree_filter;
			kdtree_filter.setInputCloud (supporting_raw_pcl.makeShared());
			for(unsigned int bd_pt=0; bd_pt< boundary_inliers_lb2->indices.size(); bd_pt++)
			{
				RollingPointXYZ searchPt_tmp = raw_pcl_lb2.points[boundary_inliers_lb2->indices[bd_pt]];
				size_t laser_serial_tmp = searchPt_tmp.laser_serial;
				bool noisy_flag = boundary_flag_batches[laser_serial_tmp];
				if(noisy_flag)
				{
					//outlier_tmp ->indices.push_back(boundary_inliers_lb2->indices[bd_pt]);
					continue;
				}

				std::vector<int> pointIdxRadiusSearch;
				std::vector<float> pointRadiusSquaredDistance;

				std::vector <float> delt_Zs;
				//at least 4 support points from road surface pts;
				if(kdtree_filter.radiusSearch (searchPt_tmp, search_radius_, pointIdxRadiusSearch, pointRadiusSquaredDistance)> 3 )
				{
					for(size_t sp_pt=0; sp_pt< pointIdxRadiusSearch.size(); sp_pt++)
					{
						RollingPointXYZ supportPt_tmp = supporting_raw_pcl.points[pointIdxRadiusSearch[sp_pt]];
						float delt_z = fabs(supportPt_tmp.z - searchPt_tmp.z);
						delt_Zs.push_back(delt_z);
					}

					//bubble sorting for small-to-big;
					size_t i,j;
					for(i=0; i<delt_Zs.size(); i++)
					{
						for(j=0;j<i;j++)
						{
							if(delt_Zs[i]<delt_Zs[j])
							{
								 float temp=delt_Zs[i]; //swap
								 delt_Zs[i]=delt_Zs[j];
								 delt_Zs[j]=temp;
							}
						}
					}

					//classification2 for individual boundary point;

					//if(delt_Zs.back() < 0.1) //for visualization when collection data;
					if(delt_Zs.back() < -0.1)
					{
						ROS_INFO("boundary points filtered: max_delt %3f", delt_Zs.back());
						outlier_tmp->indices.push_back(boundary_inliers_lb2->indices[bd_pt]);
					}
					else
					{
						//printf("Zvar:%3f\t", searchPt_tmp.z_var);
						boundary_inliers_tmp2->indices.push_back(boundary_inliers_lb2->indices[bd_pt]);
						//ROS_INFO("boundary points laser serial %ld", raw_pcl_lb2.points[boundary_inliers_lb2->indices[bd_pt]].laser_serial);
					}
				}
				else
				{
					boundary_inliers_tmp2->indices.push_back(boundary_inliers_lb2->indices[bd_pt]);
				}
			}
			boundary_index_batches_[boundary_index_batches_.size()-2] = boundary_inliers_tmp2->indices;

			//to be continued...
			//to-do list: to remove noisy scans that no points exist in surface & boundary set;
			//because region-growing will not put drastically untouched scans as road surface boundary;

			//think over it in a systematic way as noisy scans removing;
			//already done: remove noisy scans that has points in boundary set;

			//use set theory when write papers;
			outlier_index_batches_.push_back(outlier_tmp->indices);
			if(outlier_index_batches_.size() > batchNum_limit_-1 )
			{outlier_index_batches_.erase(outlier_index_batches_.begin());}

			pcl::ExtractIndices<RollingPointXYZ> extract_bd;
			extract_bd.setInputCloud (raw_pcl_lb2.makeShared());
			extract_bd.setIndices (boundary_inliers_tmp2);
			//"boundary_inliers" is the most recent boundary;
			//extract_bd.setIndices (boundary_inliers);
			extract_bd.setNegative (false);
			extract_bd.filter (boundary_pts);
			sw.end();

			//b. update "road_surface_odom_" and "road_boundary_odom_";
			//just incorporated filtered PCLs, which is one batch delayed than the most recent batch, and compose of (batchNum_limit_-1) batches;
			sw.start("4. update road_surface_odom and road_boundary_odom");
			road_surface_odom_.clear();
			road_surface_odom_.height=1;
			road_boundary_odom_.clear();
			road_boundary_odom_.height=1;
			for(size_t i=0; i< surface_index_batches_.size()-1; i++)
			{
				RollingPointCloud surface_pts_tmp;
				pcl::ExtractIndices<RollingPointXYZ> extract_surface;
				extract_surface.setInputCloud (raw_pcl_batches_[i].makeShared());
				pcl::PointIndices::Ptr surface_indices_tmp (new pcl::PointIndices);
				surface_indices_tmp->indices = surface_index_batches_[i];
				extract_surface.setIndices (surface_indices_tmp);
				extract_surface.setNegative (false);
				extract_surface.filter (surface_pts_tmp);
				road_surface_odom_ = road_surface_odom_ + surface_pts_tmp;
			}
			for(size_t i=0; i< boundary_index_batches_.size()-1; i++)
			{
				RollingPointCloud bd_pts_tmp;
				pcl::ExtractIndices<RollingPointXYZ> extract_bds;
				extract_bds.setInputCloud (raw_pcl_batches_[i].makeShared());
				pcl::PointIndices::Ptr bd_indices_tmp (new pcl::PointIndices);
				bd_indices_tmp->indices = boundary_index_batches_[i];
				extract_bds.setIndices (bd_indices_tmp);
				extract_bds.setNegative (false);
				extract_bds.filter (bd_pts_tmp);
				road_boundary_odom_ = road_boundary_odom_ + bd_pts_tmp;
			}
			sw.end();
			input_update_flag_ = false;

			road_slope_visualization(surface_pts, boundary_pts);
		}
	}
	
	void road_surface::road_slope_visualization(RollingPointCloud & surface_pts, RollingPointCloud & boundary_pts)
	{
		PointCloud patch_ROI;
		patch_ROI.clear();
		patch_ROI.height = 1;
		patch_ROI.header 	=	surface_pts.header;

		vector<pcl::PointXYZ> poly_ROI_tmp;
		pcl::PointXYZ rectangle_p1, rectangle_p2, rectangle_p3, rectangle_p4;
		rectangle_p1.x = 20.0;  rectangle_p1.y = 20.0;
		rectangle_p2.x = -20.0; rectangle_p2.y = 20.0;
		rectangle_p3.x = -20.0; rectangle_p3.y = -20.0;
		rectangle_p4.x = 20.0;  rectangle_p4.y = -20.0;


		//construct 2 clusters for the boundary_pts, representing two sides separately;
		RollingPointCloud boundary_pts_copy = boundary_pts;
		//bubble sorting for small-to-big;
		size_t i,j;
		for(i=0; i<boundary_pts_copy.points.size(); i++)
		{
			for(j=0;j<i;j++)
			{
				if(boundary_pts_copy.points[i].y<boundary_pts_copy.points[j].y)
				{
					 float temp=boundary_pts_copy.points[i].y; //swap
					 boundary_pts_copy.points[i].y=boundary_pts_copy.points[j].y;
					 boundary_pts_copy.points[j].y=temp;
				}
			}
		}

		double max_dist = 0.0;
		size_t break_serial = 0;
		for(i=1; i<boundary_pts_copy.points.size(); i++)
		{
			RollingPointXYZ pt1, pt2;
			pt1 = boundary_pts_copy.points[i-1];
			pt2 = boundary_pts_copy.points[i];
			double dist_tmp = fmutil::distance(pt1, pt2);
			if(dist_tmp>max_dist)
			{
				max_dist = dist_tmp;
				break_serial = i;
			}
		}

		if(max_dist >= 2.0)
		{
			float left_side_y, right_side_y, central_y;
			float total_y =0.0; int   ptNum_tmp=0;
			for(i=0; i<break_serial; i++)
			{
				total_y=total_y+boundary_pts_copy[i].y;
				ptNum_tmp++;
			}
			right_side_y = total_y/ptNum_tmp;

			total_y =0.0; ptNum_tmp=0;
			for(i=break_serial; i<boundary_pts_copy.points.size(); i++)
			{
				total_y=total_y+boundary_pts_copy[i].y;
				ptNum_tmp++;
			}
			left_side_y = total_y/ptNum_tmp;
			central_y = (right_side_y + left_side_y)/2.0;
			rectangle_p1.y = central_y + 2.0;
			rectangle_p2.y = central_y + 2.0;
			rectangle_p3.y = central_y - 2.0;
			rectangle_p4.y = central_y - 2.0;
		}

		poly_ROI_tmp.push_back(rectangle_p1);
		poly_ROI_tmp.push_back(rectangle_p2);
		poly_ROI_tmp.push_back(rectangle_p3);
		poly_ROI_tmp.push_back(rectangle_p4);

		for(size_t i=0; i<surface_pts.points.size();i++)
		{
			pcl::PointXYZ deputy_pt;
			deputy_pt.x = surface_pts.points[i].x;
			deputy_pt.y = surface_pts.points[i].y;
			deputy_pt.z = surface_pts.points[i].z;
			if(pointInPolygon(deputy_pt, poly_ROI_tmp))
			{
				patch_ROI.points.push_back(deputy_pt);
				patch_ROI.width ++;
			}
		}

		pcl_ros::transformPointCloud(odom_frame_, patch_ROI, patch_ROI, *tf_ );

		pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
		pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
		// Create the segmentation object
		pcl::SACSegmentation<pcl::PointXYZ> seg;
		// Optional
		seg.setOptimizeCoefficients (true);
		// Mandatory
		seg.setModelType (pcl::SACMODEL_PLANE);
		seg.setMethodType (pcl::SAC_RANSAC);
		seg.setDistanceThreshold (0.05);
		seg.setInputCloud (patch_ROI.makeShared ());
		seg.segment (*inliers, *coefficients);
		if(coefficients->values.size()!=4) return;
		float a =coefficients->values[0];
		float b =coefficients->values[1];
		float c =coefficients->values[2];

		float slope_angle = acos(fabs(c/sqrtf(a*a+b*b+c*c)));
		ROS_INFO("current slope angle %3f", slope_angle/3.141526*180.0);

		//to visualize normal and curvature in color, in base_link frame;
		PointCloudRGB slope_visual_tmp;
		slope_visual_tmp.clear();
		slope_visual_tmp.header = surface_pts.header;
		slope_visual_tmp.height = 1;

		int color_serial = floor(256.0 * slope_angle /(M_PI * 10.0/180.0 + 0.0000001));

		pcl::PointXYZRGB point_tmp;
		point_tmp.r = (u_int8_t( jet_r_[color_serial] * 255.0));
		point_tmp.g = (u_int8_t( jet_g_[color_serial] * 255.0));
		point_tmp.b = (u_int8_t( jet_b_[color_serial] * 255.0));

		for(size_t p =0; p < surface_pts.points.size(); p++)
		{
			point_tmp.x = surface_pts.points[p].x;
			point_tmp.y = surface_pts.points[p].y;
			point_tmp.z = surface_pts.points[p].z;
			slope_visual_tmp.push_back(point_tmp);
		}
		surface_slope_pub_.publish(slope_visual_tmp);
	}

	//input:		surface_pts;	poly_ROI;
	//output: 	fitting_plane; plane_coef;
	void road_surface::planefitting_ROI(RollingPointCloud & surface_pts, vector<pcl::PointXYZ> & poly_ROI,
										PointCloud & fitting_plane, rolling_window::plane_coef & plane_coef)
	{
		PointCloud patch_ROI;
		patch_ROI.clear();
		patch_ROI.height = 1;
		patch_ROI.header 	=	surface_pts.header;
		
		for(size_t i=0; i<surface_pts.points.size();i++)
		{
			pcl::PointXYZ deputy_pt;
			deputy_pt.x = surface_pts.points[i].x;
			deputy_pt.y = surface_pts.points[i].y;
			deputy_pt.z = surface_pts.points[i].z;
			if(pointInPolygon(deputy_pt, poly_ROI))
			{
				patch_ROI.points.push_back(deputy_pt);
				patch_ROI.width ++;
			}
		}
		pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
		pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
		// Create the segmentation object
		pcl::SACSegmentation<pcl::PointXYZ> seg;
		// Optional
		seg.setOptimizeCoefficients (true);
		// Mandatory
		seg.setModelType (pcl::SACMODEL_PLANE);
		seg.setMethodType (pcl::SAC_RANSAC);
		seg.setDistanceThreshold (0.05);
		seg.setInputCloud (patch_ROI.makeShared ());
		seg.segment (*inliers, *coefficients);
		
		if(coefficients->values.size()!=4) return;
		
		plane_coef.header = surface_pts.header;
		for(int i=0; i < 4; i++) plane_coef.coefs.push_back(coefficients->values[i]);

		fitting_plane.header = surface_pts.header;
		// Fill in the cloud data
		fitting_plane.width  = inliers->indices.size ();
		fitting_plane.height = 1;
		fitting_plane.points.resize (fitting_plane.width * fitting_plane.height);
		size_t j=0;
		
		for (size_t i = 0; i < inliers->indices.size (); ++i)
		{
			fitting_plane.points[j]=patch_ROI.points[inliers->indices[i]];
			j++;
		}
		
	}
	
	//to be continued...
	void road_surface::publishNormal(RollingPointNormalCloud & pcl_cloud)
	{
		fmutil::Stopwatch sw;
		
		//http://www.pointclouds.org/blog/gsoc/goleary/tutorials/conditional_removal.php
		sw.start("filter pcl");
		pcl::ConditionAnd<RollingPointXYZNormal>::Ptr range_cond (new pcl::ConditionAnd<RollingPointXYZNormal> ());
		range_cond->addComparison (pcl::FieldComparison<RollingPointXYZNormal>::ConstPtr (new
		pcl::FieldComparison<RollingPointXYZNormal> ("curvature", pcl::ComparisonOps::GT, curvature_thresh_)));

 	    pcl::ConditionalRemoval<RollingPointXYZNormal> condrem (range_cond);
	    condrem.setInputCloud (pcl_cloud.makeShared());
	   
	    //without this, the raw input pcl_cloud will be changed, which is not desired;
	    RollingPointNormalCloud pcl_cloud_filtered;
	    condrem.filter(pcl_cloud_filtered);
	    sw.end();

		// publish normal as posearray for visualization
		bool publish_normals = true;
		if(publish_normals)
		{
			geometry_msgs::PoseArray normals_poses;
			normals_poses.header = pcl_cloud.header;
			for(unsigned int i=0; i<pcl_cloud_filtered.points.size(); i++)
			{
				geometry_msgs::Pose normals_pose;
				geometry_msgs::Point pos;
				pos.x = pcl_cloud_filtered.points[i].x; pos.y = pcl_cloud_filtered.points[i].y; pos.z = pcl_cloud_filtered.points[i].z;
				normals_pose.position = pos;
				btVector3 axis(pcl_cloud_filtered.points[i].normal[0],pcl_cloud_filtered.points[i].normal[1],pcl_cloud_filtered.points[i].normal[2]);
				if(isnan(pcl_cloud_filtered.points[i].normal[0])||isnan(pcl_cloud_filtered.points[i].normal[1])||isnan(pcl_cloud_filtered.points[i].normal[2])) continue;
				btVector3 marker_axis(1, 0, 0);
				btQuaternion qt(marker_axis.cross(axis.normalize()), marker_axis.angle(axis.normalize()));
				double yaw, pitch, roll;
				btMatrix3x3(qt).getEulerYPR(yaw, pitch, roll);
				geometry_msgs::Quaternion quat_msg;
				tf::quaternionTFToMsg(qt, quat_msg);
				if(isnan(qt.x())||isnan(qt.y())||isnan(qt.z())||isnan(qt.w())) continue;
				normals_pose.orientation.x = qt.x();// = quat_msg;
				normals_pose.orientation.y = qt.y();
				normals_pose.orientation.z = qt.z();
				normals_pose.orientation.w = qt.w();

				normals_poses.poses.push_back(normals_pose);
			}
			normals_poses_pub_.publish(normals_poses);
		}
	}

	bool road_surface::checkDistance(const tf::StampedTransform& oldTf, const tf::StampedTransform& newTf, float Dis_thresh)
	{
		//"odom_old_new" denotes the tf of  "baselink_new" coordinate inside the "baselink_old" coordinate;
		tf::Transform odom_old_new  = oldTf.inverse() * newTf;
		float tx, ty;
		tx = -odom_old_new.getOrigin().y();
		ty =  odom_old_new.getOrigin().x();
		float mov_dis = sqrtf(tx*tx + ty*ty);
		//double yaw_dis, ttemp;
		//odom_old_new.getBasis().getEulerYPR(yaw_dis, ttemp, ttemp);
		if(mov_dis > Dis_thresh) return true;
		else return false;
	}

};

int main(int argc, char** argv)
{
	 ros::init(argc, argv, "road_surface_node");
	 ros::NodeHandle n;
	 golfcar_pcl::road_surface road_surface_node;
     ros::spin();
     return 0;
}


