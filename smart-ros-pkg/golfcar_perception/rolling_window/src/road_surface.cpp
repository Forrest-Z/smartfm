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
		private_nh_.param("extract_training_data_scan", extract_training_data_scan_, false);
		private_nh_.param("extract_training_data_point", extract_training_data_point_, false);

		rolling_pcl_sub_ = new message_filters::Subscriber<RollingPointCloud> (nh_, "rolling_window_pcl", 1);
		pcl_indices_sub_ = new message_filters::Subscriber<rolling_window::pcl_indices> (nh_, "process_fraction_indices", 1);
		sync_	= new message_filters::TimeSynchronizer<RollingPointCloud, rolling_window::pcl_indices>(*rolling_pcl_sub_, *pcl_indices_sub_, 5);
		sync_->registerCallback(boost::bind(&road_surface::pclCallback, this, _1, _2));
		
		odom_sub_.subscribe(nh_, "odom", 10);
		odom_filter_ = new tf::MessageFilter<nav_msgs::Odometry>(odom_sub_, *tf_, base_frame_, 10);
		odom_filter_ ->registerCallback(boost::bind(&road_surface::odomCallback, this, _1));
		odom_filter_->setTolerance(ros::Duration(0.05));
		
		process_fraction_pub_   = 	nh_.advertise<RollingPointCloud>("process_fraction", 10);

		//modification: to downsampling it, publish the raw PointCloud type;
		road_surface_pub_   	= 	nh_.advertise<PointCloud>("road_surface_pts", 10);
		road_boundary_pub_   	= 	nh_.advertise<PointCloud>("road_boundary_pts", 10);
		road_surface_pub2_   	= 	nh_.advertise<PointCloud>("road_surface_pts2", 10);
		road_boundary_pub2_   	= 	nh_.advertise<PointCloud>("road_boundary_pts2", 10);
		offroad_pub_   	= 	nh_.advertise<PointCloud>("offroad_pts", 10);

		fitting_plane_pub_   	=	nh_.advertise<PointCloud>("fitting_plane", 10);
		normals_poses_pub_ 		=	nh_.advertise<geometry_msgs::PoseArray>("normals_array", 10);
		plane_coef_pub_ 		= 	nh_.advertise<rolling_window::plane_coef>("plane_coef", 10);
		
		surface_all_pub_   		= 	nh_.advertise<RollingPointCloud>("surface_all", 10);
		boundary_all_pub_   	= 	nh_.advertise<RollingPointCloud>("boundary_all", 10);
		large_curvature_pub_   	= 	nh_.advertise<PointCloud>("large_curvature_pts", 10);
		scan_outlier_pub_		= 	nh_.advertise<PointCloud>("scan_outlier_pts", 10);
		scan_inlier_pub_		= 	nh_.advertise<PointCloud>("scan_inlier_pts", 10);

		//the pcl batch to extract latest road boundary and surface;
		pcl_to_process_pub_  	= 	nh_.advertise<RollingPointCloud>("pcl_to_process", 10);

		planefitting_init_ = false;
		clustering_init_   = false;
		planefitting_disThresh_ = 0.03;
		clustering_disThresh_   = 15.0;

		clusters_pub_ = nh_.advertise<PointCloudRGB>("clusters_RGBD", 10);
		normal_visual_pub_ = nh_.advertise<PointCloudRGB>("normal_visual_RGB", 10);
		normal_visual_pub2_ = nh_.advertise<PointCloudRGB>("normal_visual_RGB2", 10);
		variance_visual_pub_ = nh_.advertise<PointCloudRGB>("variance_visual_RGB", 10);
		variance_visual_pub2_ = nh_.advertise<PointCloudRGB>("variance_visual_RGB2", 10);

		surface_slope_pub_ = nh_.advertise<PointCloudRGB>("surface_slope_visual_RGB", 10);



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

		scan_angle_incremental_ = 0.0174532923847;

		string scan_model_path, scan_scale_path;
		string point_model_path, point_scale_path;
		private_nh_.param("scan_model_path", scan_model_path, std::string("/home/baoxing/workspace/data_and_model/rolling_window/scan.model"));
		private_nh_.param("scan_scale_path", scan_scale_path, std::string("/home/baoxing/workspace/data_and_model/rolling_window/scan.range"));
		private_nh_.param("point_model_path", point_model_path, std::string("/home/baoxing/workspace/data_and_model/rolling_window/point.model"));
		private_nh_.param("point_scale_path", point_scale_path, std::string("/home/baoxing/workspace/data_and_model/rolling_window/point.range"));

		private_nh_.param("record_path", record_path_, std::string("/home/baoxing/training_data"));

		scan_classifier_ = new golfcar_ml::svm_classifier(scan_model_path, scan_scale_path);
		point_classifier_ = new golfcar_ml::svm_classifier(point_model_path, point_scale_path);
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
		boost::recursive_mutex::scoped_lock pcll(configuration_mutex_);
		if(input_update_flag_) ROS_WARN("last road_surface and boundary pcl batch are not accumulated, please check odomCallback");
		
		surface_pts_.clear();
		surface_pts_.height = 1;
		surface_pts_.header =	pcl_in->header;

		boundary_pts_.clear();
		boundary_pts_.height = 1;
		boundary_pts_.header = pcl_in->header;
		
		road_surface::surface_extraction(*pcl_in,  *indices_in, surface_pts_, boundary_pts_);
		
		road_surface::pcl_downsample(surface_pts_);
		road_surface::pcl_downsample(boundary_pts_);

		//20121104 modifications;
		PointCloud surface_pcl, boundary_pcl;
		surface_pcl.clear();
		surface_pcl.height = 1;
		surface_pcl.header =	pcl_in->header;
		boundary_pcl.clear();
		boundary_pcl.height = 1;
		boundary_pcl.header = pcl_in->header;

		for(size_t i=0; i<surface_pts_.points.size(); i++)
		{
			pcl::PointXYZ point_tmp;
			point_tmp.x = surface_pts_.points[i].x;
			point_tmp.y = surface_pts_.points[i].y;
			point_tmp.z = surface_pts_.points[i].z;
			surface_pcl.push_back(point_tmp);
		}

		for(size_t i=0; i<boundary_pts_.points.size(); i++)
		{
			pcl::PointXYZ point_tmp;
			point_tmp.x = boundary_pts_.points[i].x;
			point_tmp.y = boundary_pts_.points[i].y;
			point_tmp.z = boundary_pts_.points[i].z;
			boundary_pcl.push_back(point_tmp);
		}

		road_surface_pub_.publish(surface_pcl);
		road_boundary_pub_.publish(boundary_pcl);
	}
	
	//odomCallback triggers several processing functions;
	//1st: extract plane by RANSAC based on the maintained "road_surface" pcl;
	//2nd: do surface subtraction and clustering for other objects;
	void road_surface::odomCallback(const OdomConstPtr& odom)
	{
		boost::recursive_mutex::scoped_lock ocl(configuration_mutex_);

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
			    for(size_t batch =0; batch < raw_pcl_batches_.size()-1; batch++)
			    {
			    	accumulated_raw_pcl = accumulated_raw_pcl+raw_pcl_batches_[batch];
			    	for(size_t serial=0; serial < surface_index_batches_[batch].size(); serial++)
			    	{
			    		int serial_tmp = surface_index_batches_[batch][serial] + serial_base;
			    		surface_indices->indices.push_back(serial_tmp);
			    	}
			    	serial_base = serial_base + raw_pcl_batches_[batch].size();
			    }

			    pcl::PointIndices::Ptr total_indices (new pcl::PointIndices);
			    for(size_t serial=0; serial < surface_indices->indices.size(); serial++)
				{
					int serial_tmp = surface_indices->indices[serial];
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

				/*
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
				*/

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
		boost::recursive_mutex::scoped_lock sel(configuration_mutex_);

		record_batch_serial_++;

		//step 1:Extract the point-cloud fraction to be processed;

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
		
		//process the cloud_in: calculate normals;
		//http://pointclouds.org/documentation/tutorials/kdtree_search.php
		//http://pointclouds.org/documentation/tutorials/how_features_work.php#how-3d-features-work
		fmutil::Stopwatch sw;
		sw.start("calculate pcl normals");
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

		//instantly show colored pointcloud according to variance;
		PointCloudRGBNormal normal_visual_tmp;
		normal_visual_tmp.clear();
		normal_visual_tmp.header = cloud_in.header;
		normal_visual_tmp.height = 1;
		for(size_t ip =0; ip < point_normals.points.size(); ip++)
		{
			pcl::RGB RGB_tmp;
			RollingPointXYZNormal pointNormal_tmp;
			pcl::PointXYZRGBNormal pointRBGNormal_tmp;
			pointNormal_tmp = point_normals.points[ip];

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
		normal_visual_pub_.publish(normal_visual_tmp);

		PointCloudRGB variance_visualization;
		variance_visualization.clear();
		variance_visualization.header = cloud_in.header;
		variance_visualization.height = 1;
		for(size_t ip =0; ip < point_normals.points.size(); ip++)
		{
			RollingPointXYZNormal point_tmp;
			point_tmp = point_normals.points[ip];

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
		variance_visual_pub_.publish(variance_visualization);

		//step 2: filter noisy point cloud according to their scan batch;
		//output: scan_inlier_pcl, and scan_inlier_pclNormal;


		//one tricky bug: use "float" or "double" as key will lead to unexpected "inequality";
		//typedef std::map < float, RollingPointXYZNormal> angle_pt_map;
		//use deputy_angle (multiplied by 1000 and converted to int);
		typedef std::map < int, RollingPointXYZNormal> angle_pt_map;
		std::map< size_t , angle_pt_map > laser_batches;
		std::map< size_t , std::vector<float> > angle_batches;
		std::map< size_t , scan_info > batch_infos;
		std::map< size_t , bool> batch_flags;

		for(unsigned int pt=0; pt< point_normals.points.size(); pt++)
		{
			RollingPointXYZNormal Pt_tmp = point_normals.points[pt];
			size_t laser_serail = Pt_tmp.laser_serial;
			if(laser_batches.find(laser_serail) == laser_batches.end())
			{
				angle_pt_map batch_tmp;
				batch_tmp[deputy_key(Pt_tmp.beam_angle)] = Pt_tmp;
				laser_batches[laser_serail] = batch_tmp;
			}
			else
			{
				(laser_batches[laser_serail])[deputy_key(Pt_tmp.beam_angle)] = Pt_tmp;
			}

			if(angle_batches.find(laser_serail) == angle_batches.end())
			{
				std::vector<float> vector_tmp;
				vector_tmp.push_back(Pt_tmp.beam_angle);
				angle_batches[laser_serail] = vector_tmp;
			}
			else
			{
				angle_batches[laser_serail].push_back(Pt_tmp.beam_angle);
			}
		}

		//to visualize points with exceedingly large curvature;
		PointCloud large_curvature_pts;
		large_curvature_pts.header = cloud_in.header;
		large_curvature_pts.clear();
		large_curvature_pts.height = 1;

		//to construct batch_infos for further scan validation;
		for(std::map <size_t,angle_pt_map>::const_iterator batch_it = laser_batches.begin(); batch_it != laser_batches.end(); batch_it++ )
		{
			size_t laser_serial = (*batch_it).first;
			angle_pt_map batch_tmp = (*batch_it).second;
			scan_info info_tmp;

			std::map <int,RollingPointXYZNormal>::const_iterator pt_it = batch_tmp.begin();
			info_tmp.pitch_speed = (*pt_it).second.pitch_speed;
			info_tmp.pitch = (*pt_it).second.pitch;
			info_tmp.roll = (*pt_it).second.roll;

			std::vector <float> beam_angles = angle_batches[laser_serial];

			//here we assume that points are arranged according to angle from small to big;
			float min_angle = beam_angles.front();
			bool scan_complete = true;
			if(beam_angles.size() < 120){scan_complete = false;}
			if( min_angle > - M_PI/3.0){scan_complete = false;}
			else
			{
				pcl::PointXYZ tmppoint;
				RollingPointXYZNormal tmpnormalpoint;

				int large_curvature_number_tmp =0;
				std::vector <float> point_curvatures;
				float near_zeroPlus_angle = fabs(floor(min_angle/scan_angle_incremental_))*scan_angle_incremental_ + min_angle;
				//printf("%lf\t", near_zeroPlus_angle);

				if(batch_tmp.find(deputy_key(near_zeroPlus_angle))==batch_tmp.end())
				{
					tmpnormalpoint.curvature = 0.0;
				}
				else
				{
					tmpnormalpoint = batch_tmp[deputy_key(near_zeroPlus_angle)];
					if(tmpnormalpoint.curvature > curvature_thresh_)
					{
						large_curvature_number_tmp++;
						tmppoint.x = tmpnormalpoint.x;
						tmppoint.y = tmpnormalpoint.y;
						tmppoint.z = tmpnormalpoint.z;
						large_curvature_pts.push_back(tmppoint);
					}
				}
				point_curvatures.push_back(tmpnormalpoint.curvature);

				for(float ia=1; ia<60; ia++)
				{
					float angle_tmpp =near_zeroPlus_angle+ float(ia * scan_angle_incremental_);

					if(batch_tmp.find(deputy_key(angle_tmpp))==batch_tmp.end())
					{
						tmpnormalpoint.curvature = batch_tmp[deputy_key(angle_tmpp -scan_angle_incremental_)].curvature;
					}
					else
					{
						tmpnormalpoint = batch_tmp[deputy_key(angle_tmpp)];
						if(tmpnormalpoint.curvature > curvature_thresh_)
						{
							large_curvature_number_tmp++;
							tmppoint.x = tmpnormalpoint.x;
							tmppoint.y = tmpnormalpoint.y;
							tmppoint.z = tmpnormalpoint.z;
							large_curvature_pts.push_back(tmppoint);
						}
					}
					point_curvatures.push_back(tmpnormalpoint.curvature);
				}
				for(float ia=1; ia<=60; ia++)
				{
					float angle_tmpp =near_zeroPlus_angle - ia*scan_angle_incremental_;

					if(batch_tmp.find(deputy_key(angle_tmpp))==batch_tmp.end())
					{
						tmpnormalpoint.curvature = batch_tmp[deputy_key(angle_tmpp + scan_angle_incremental_)].curvature;
					}
					else
					{
						tmpnormalpoint = batch_tmp[deputy_key(angle_tmpp)];
						if(tmpnormalpoint.curvature > curvature_thresh_)
						{
							large_curvature_number_tmp++;
							tmppoint.x = tmpnormalpoint.x;
							tmppoint.y = tmpnormalpoint.y;
							tmppoint.z = tmpnormalpoint.z;
							large_curvature_pts.push_back(tmppoint);
						}
					}
					point_curvatures.insert( point_curvatures.begin(), tmpnormalpoint.curvature);
				}

				info_tmp.curvature = point_curvatures;
				info_tmp.large_curvature_number = large_curvature_number_tmp;
			}

			if(!scan_complete){batch_flags[laser_serial] = false;}
			else
			{
				batch_infos[laser_serial] = info_tmp;
			}
		}
		large_curvature_pub_.publish(large_curvature_pts);

		//try to record training data for svm;
		if(extract_training_data_scan_)
		{
			stringstream  name_string;
			name_string<<record_path_.c_str()<<"/scan_data";
			const char *input_name = name_string.str().c_str();
			if((fp_=fopen(input_name, "a"))==NULL){ROS_ERROR("cannot open file\n");return;}
			else{ROS_INFO("file opened successfully!");}

			ROS_INFO("write training data");
			for(std::map <size_t, scan_info>::const_iterator batch_it = batch_infos.begin(); batch_it != batch_infos.end(); batch_it++ )
			{
				size_t laser_serial = (*batch_it).first;
				scan_info batch_tmp = (*batch_it).second;

				fprintf(fp_, "%ld\t%ld\t", (record_batch_serial_+1), laser_serial);
				fprintf(fp_, "%5f\t%5f\t%5f\t", batch_tmp.pitch_speed, batch_tmp.pitch, batch_tmp.roll);
				for(size_t i=0; i<120; i++) {fprintf(fp_, "%5f\t", batch_tmp.curvature[i]);}
				fprintf(fp_, "%ld\t", batch_tmp.large_curvature_number);
				int tentative_value = 0;
				fprintf(fp_, "%d\n",tentative_value);

				printf("\n laser_serial: %ld,\tbigCurve_number %ld\t \n", laser_serial, batch_tmp.large_curvature_number);
			}
			fclose(fp_); ROS_INFO("file closed");
		}
		//to record a simplified data for labeling process;
		if(extract_training_data_scan_)
		{
			stringstream  name_string;
			name_string<<record_path_.c_str()<<"/scan_label_data";
			const char *input_name = name_string.str().c_str();
			if((fp_=fopen(input_name, "a"))==NULL){ROS_ERROR("cannot open file\n");return;}
			else{ROS_INFO("file opened successfully!");}

			for(std::map <size_t, scan_info>::const_iterator batch_it = batch_infos.begin(); batch_it != batch_infos.end(); batch_it++ )
			{
				size_t laser_serial = (*batch_it).first;
				scan_info batch_tmp = (*batch_it).second;
				fprintf(fp_, "%ld\t%ld\t", (record_batch_serial_+1), laser_serial);
				fprintf(fp_, "%ld\t", batch_tmp.large_curvature_number);
				int tentative_value = 0;
				fprintf(fp_, "%d\t\n", tentative_value);
			}
			fclose(fp_); ROS_INFO("file closed");
		}

		for(std::map <size_t, scan_info>::const_iterator batch_it = batch_infos.begin(); batch_it != batch_infos.end(); batch_it++ )
		{
			size_t laser_serial = (*batch_it).first;
			scan_info batch_tmp = (*batch_it).second;
			int vector_length = 3+120;
			double scan_feature_vector[vector_length];
			scan_feature_vector[0]= batch_tmp.pitch_speed;
			scan_feature_vector[1]= batch_tmp.pitch;
			scan_feature_vector[2]= batch_tmp.roll;
			for(size_t i=0; i<120; i++) {scan_feature_vector[i+3] = batch_tmp.curvature[i];}

			int scan_type = scan_classifier_->classify_objects(scan_feature_vector, vector_length);

			if(scan_type == 1 || scan_type == -1)
			{
				batch_flags[laser_serial] = false;
				ROS_INFO("filter noisy scans");
			}
			else
			{
				batch_flags[laser_serial] = true;
			}
		}

		pcl::PointIndices::Ptr scanfilter_outlier (new pcl::PointIndices);
		for(unsigned int pt=0; pt< point_normals.points.size(); pt++)
		{
			size_t laser_serial = point_normals.points[pt].laser_serial;
			assert(batch_flags.find(laser_serial)!=batch_flags.end());
			if(!batch_flags[laser_serial]) scanfilter_outlier->indices.push_back(pt);
		}

		RollingPointCloud scan_inlier_pcl;
		scan_inlier_pcl.header = cloud_in.header;
		scan_inlier_pcl.clear();
		scan_inlier_pcl.height = 1;
		RollingPointCloud scan_outlier_pcl;
		scan_outlier_pcl.header = cloud_in.header;
		scan_outlier_pcl.clear();
		scan_outlier_pcl.height = 1;
		RollingPointNormalCloud scan_inlier_pclNormal;
		scan_inlier_pclNormal.header = cloud_in.header;
		scan_inlier_pclNormal.clear();
		scan_inlier_pclNormal.height = 1;

		pcl::ExtractIndices<RollingPointXYZ> extract_scanfilter;
		extract_scanfilter.setInputCloud (process_fraction_pcl.makeShared());
		extract_scanfilter.setIndices (scanfilter_outlier);
		extract_scanfilter.setNegative (false);
		extract_scanfilter.filter (scan_outlier_pcl);
		extract_scanfilter.setNegative (true);
		extract_scanfilter.filter (scan_inlier_pcl);
		//to visualize the noisy scan batches filtered;

		scan_outlier_pub_.publish(scan_outlier_pcl);
		scan_inlier_pub_.publish(scan_inlier_pcl);

		pcl::ExtractIndices<RollingPointXYZNormal> extract_scanfilter_normal;
		extract_scanfilter_normal.setInputCloud (point_normals.makeShared());
		extract_scanfilter_normal.setIndices (scanfilter_outlier);
		extract_scanfilter_normal.setNegative (true);
		extract_scanfilter_normal.filter (scan_inlier_pclNormal);

		RollingPointCloud pcl_odom_tmp;
		pcl_ros::transformPointCloud(odom_frame_, scan_inlier_pcl, pcl_odom_tmp, *tf_ );
		RollingPointNormalCloud pcl_normal_odom_tmp;
		pcl_ros::transformPointCloud(odom_frame_, scan_inlier_pclNormal, pcl_normal_odom_tmp, *tf_ );

		raw_pcl_buffers_.push_back(pcl_odom_tmp);
		pcl_normal_buffers_.push_back(pcl_normal_odom_tmp);

		//step 3: region-growing based on SVM classification;

		if(raw_pcl_buffers_.size() > 3)
		{
			raw_pcl_buffers_.erase (raw_pcl_buffers_.begin());
			pcl_normal_buffers_.erase (pcl_normal_buffers_.begin());
		}
		assert(raw_pcl_buffers_.size()<=3);

		if(raw_pcl_buffers_.size()==3)
		{
			RollingPointNormalCloud supporting_normal_pcl;
			supporting_normal_pcl.clear();
			supporting_normal_pcl.height = 1;

			for(vector<RollingPointNormalCloud>::iterator it=pcl_normal_buffers_.end()-3; it< pcl_normal_buffers_.end(); it++)
			{
				supporting_normal_pcl = supporting_normal_pcl + *it;
			}
			supporting_normal_pcl.width = supporting_normal_pcl.points.size();
			supporting_normal_pcl.header.stamp = cloud_in.header.stamp;
			supporting_normal_pcl.header.frame_id = odom_frame_;
			pcl_ros::transformPointCloud(base_frame_, supporting_normal_pcl, supporting_normal_pcl, *tf_ );

			RollingPointNormalCloud pcl_to_process;
			pcl_to_process = pcl_normal_buffers_[pcl_normal_buffers_.size()-2];
			pcl_to_process.header.stamp = cloud_in.header.stamp;
			pcl_to_process.header.frame_id = odom_frame_;
			pcl_ros::transformPointCloud(base_frame_, pcl_to_process, pcl_to_process, *tf_ );

			//to publish pcl_to_process for debugging;
			pcl_to_process_pub_.publish(pcl_to_process);

			//to visualize the curvature of "pcl_to_process";
			PointCloudRGBNormal normal_visual_tmp2;
			normal_visual_tmp2.clear();
			normal_visual_tmp2.header = pcl_to_process.header;
			normal_visual_tmp2.height = 1;
			for(size_t ip =0; ip < pcl_to_process.points.size(); ip++)
			{
				pcl::RGB RGB_tmp;
				RollingPointXYZNormal pointNormal_tmp;
				pcl::PointXYZRGBNormal pointRBGNormal_tmp;
				pointNormal_tmp = pcl_to_process.points[ip];

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
				normal_visual_tmp2.push_back(pointRBGNormal_tmp);
			}
			normal_visual_pub2_.publish(normal_visual_tmp2);


			//PointCloudRGB variance_visualization;
			variance_visualization.clear();
			variance_visualization.header = cloud_in.header;
			variance_visualization.height = 1;
			for(size_t ip =0; ip < pcl_to_process.points.size(); ip++)
			{
				RollingPointXYZNormal point_tmp;
				point_tmp = pcl_to_process.points[ip];

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
			variance_visual_pub2_.publish(variance_visualization);


			sw.start("region-growing to extract surface");
			pcl::KdTreeFLANN<RollingPointXYZNormal> kdtree;
			if(pcl_to_process.points.size()==0) return;
			kdtree.setInputCloud (pcl_to_process.makeShared());

			pcl::KdTreeFLANN<RollingPointXYZNormal> kdtree_filter;
			kdtree_filter.setInputCloud (supporting_normal_pcl.makeShared());

			//just to find the initial seed of road surface;
			int K = 1;
			std::vector<int> pointIdxNKNSearch(K);
			std::vector<float> pointNKNSquaredDistance(K);

			int num = kdtree.nearestKSearch (seedPoint_, K, pointIdxNKNSearch, pointNKNSquaredDistance);

			//pay attention to the "return" here, ignoring which may lead to bugs;
			if(num!=1)
			{
				ROS_WARN("ERROR When to find surface seed");
				raw_pcl_buffers_.erase(raw_pcl_buffers_.end());
				pcl_normal_buffers_.erase(pcl_normal_buffers_.end());
				return;
			}

			pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
			pcl::PointIndices::Ptr boundary_inliers (new pcl::PointIndices);
			RollingPointXYZNormal searchPt_tmp;
			searchPt_tmp = pcl_to_process.points[pointIdxNKNSearch[0]];
			inliers->indices.push_back(pointIdxNKNSearch[0]);

			for(unsigned int pt=0; pt<inliers->indices.size(); pt++)
			{
				unsigned int serial = inliers->indices[pt];
				//std::cout <<"serial  "<< serial<< endl;
				int neighbor_num = 8;
				std::vector<int> nbPts(neighbor_num);
				std::vector<float> nbPtDis(neighbor_num);

				searchPt_tmp = pcl_to_process.points[serial];
				int num = kdtree.nearestKSearch (searchPt_tmp, neighbor_num, nbPts, nbPtDis);
				if(num!=neighbor_num) std::cout<<"neighbour points not enought, error"<<endl;

				for(unsigned int neighbour_pt=0; neighbour_pt <nbPts.size(); neighbour_pt++)
				{
					 int nb_serial = nbPts[neighbour_pt];
					 RollingPointXYZNormal nbPoint_tmp = pcl_to_process.points[nb_serial];

					 //to first classify out most surface points;
					 bool curvature_flat =nbPoint_tmp.curvature <= curvature_thresh_;

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

						 if(no_copy)
						 {
							std::vector<int> pointIdxRadiusSearch_tmp;
							std::vector<float> pointRadiusSquaredDistance_tmp;
							//at least 4 support points from road surface pts;
							if(kdtree_filter.radiusSearch (nbPoint_tmp, search_radius_, pointIdxRadiusSearch_tmp, pointRadiusSquaredDistance_tmp)> 3 )
							{
								std::vector <float> delt_Zs;
								for(size_t sp_pt=0; sp_pt< pointIdxRadiusSearch_tmp.size(); sp_pt++)
								{
									RollingPointXYZNormal supportPt_tmp = supporting_normal_pcl.points[pointIdxRadiusSearch_tmp[sp_pt]];
									float delt_z = fabs(supportPt_tmp.z - nbPoint_tmp.z);
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

								float curvature_tmp = nbPoint_tmp.curvature;
								float pitch_speed_tmp = nbPoint_tmp.pitch_speed;
								float pitch_tmp = nbPoint_tmp.pitch;
								float roll_tmp = nbPoint_tmp.roll;
								float angle_tmp = nbPoint_tmp.beam_angle;
								float z_var_tmp = nbPoint_tmp.z_var;
								float max_deltZ_tmp = delt_Zs.back();
								float density_tmp = float(pointIdxRadiusSearch_tmp.size());

								//try to record training data for svm;
								if(extract_training_data_point_)
								{
									stringstream  name_string;
									name_string<<record_path_.c_str()<<"/point_data";
									const char *input_name = name_string.str().c_str();
									if((fp_=fopen(input_name, "a"))==NULL){ROS_ERROR("cannot open file\n");return;}
									else{ROS_INFO("file opened successfully!");}
									fprintf(fp_, "%ld\t", (record_batch_serial_+1));
									fprintf(fp_, "%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\n", curvature_tmp, pitch_speed_tmp, pitch_tmp, roll_tmp, angle_tmp, z_var_tmp, max_deltZ_tmp, density_tmp);
									fclose(fp_); ROS_INFO("point raw file closed");
								}
								//to record a simplified data for labeling process;
								if(extract_training_data_point_)
								{
									stringstream  name_string;
									name_string<<record_path_.c_str()<<"/point_label_data";
									const char *input_name = name_string.str().c_str();
									if((fp_=fopen(input_name, "a"))==NULL){ROS_ERROR("cannot open file\n");return;}
									else{ROS_INFO("file opened successfully!");}
									fprintf(fp_, "%ld\t", (record_batch_serial_+1));
									fprintf(fp_, "%f\t%f\t", nbPoint_tmp.x, nbPoint_tmp.y);
									int tentative_value = 0;
									fprintf(fp_, "%d\n", tentative_value);
									fclose(fp_); ROS_INFO("point label file closed");
								}


								int vector_length = 8;
								double point_feature_vector[vector_length];
								point_feature_vector[0]= curvature_tmp;
								point_feature_vector[1]= pitch_speed_tmp;
								point_feature_vector[2]= pitch_tmp;
								point_feature_vector[3]= roll_tmp;
								point_feature_vector[4]= angle_tmp;
								point_feature_vector[5]= z_var_tmp;
								point_feature_vector[6]= max_deltZ_tmp;
								point_feature_vector[7]= density_tmp;
								int point_type = point_classifier_->classify_objects(point_feature_vector, vector_length);

								//when extract points data, we need to visualize all the potential points in "boundary_pts";
								if(extract_training_data_point_) point_type = 0;

								//0 represents "true", "1" represents "noisy" (false) for boundary points;
								//vice versa for surface points;
								if(point_type == 0)
								{
									 boundary_inliers->indices.push_back(nb_serial);
								}
								else if( point_type == 1)
								{
									//just now only checked where there is already a copy in boundary points;
									//still need to check where this point already exists in surface points, when trying to add it to the surface points;
									 for(unsigned int all_pt=0; all_pt<inliers->indices.size(); all_pt++)
									 {
										  if(inliers->indices[all_pt] == nb_serial) no_copy = false;
									 }
									 if(no_copy) inliers->indices.push_back(nb_serial);
								}
								else
								{
									//do nothing when classification result is -1, meaning that "cannot classify";
								}
							}
							else
							{
								//do nothing when there are no enough local supporting points;
							}

						 }
					 }
				}
			}
			sw.end();

			//to generate classification result with only curvature;
			pcl::PointIndices::Ptr inliers2 (new pcl::PointIndices);
			pcl::PointIndices::Ptr boundary_inliers2 (new pcl::PointIndices);
			inliers2->indices.push_back(pointIdxNKNSearch[0]);
			for(unsigned int pt=0; pt<inliers2->indices.size(); pt++)
			{
				unsigned int serial = inliers2->indices[pt];
				//std::cout <<"serial  "<< serial<< endl;
				int neighbor_num = 8;
				std::vector<int> nbPts(neighbor_num);
				std::vector<float> nbPtDis(neighbor_num);

				searchPt_tmp = pcl_to_process.points[serial];
				int num = kdtree.nearestKSearch (searchPt_tmp, neighbor_num, nbPts, nbPtDis);
				if(num!=neighbor_num) std::cout<<"neighbour points not enought, error"<<endl;

				for(unsigned int neighbour_pt=0; neighbour_pt <nbPts.size(); neighbour_pt++)
				{
					 int nb_serial = nbPts[neighbour_pt];
					 RollingPointXYZNormal nbPoint_tmp = pcl_to_process.points[nb_serial];

					 //to first classify out most surface points;
					 bool curvature_flat =nbPoint_tmp.curvature <= curvature_thresh_;

					 if(curvature_flat)
					 {
						 bool no_copy = true;
						 for(unsigned int all_pt=0; all_pt<inliers2->indices.size(); all_pt++)
						 {
							  if(inliers2->indices[all_pt] == nb_serial) no_copy = false;
						 }
						 if(no_copy) inliers2->indices.push_back(nb_serial);
					 }
					 else
					 {
						 bool no_copy = true;
						 for(unsigned int bd_pt=0; bd_pt< boundary_inliers2->indices.size(); bd_pt++)
						 {
							  if(boundary_inliers2->indices[bd_pt] == nb_serial) no_copy = false;
						 }
						 if(no_copy)
						 {
							 boundary_inliers2->indices.push_back(nb_serial);
						 }
					 }
				}
			}

			RollingPointCloud raw_pcl_processed;
			raw_pcl_processed = raw_pcl_buffers_[raw_pcl_buffers_.size()-2];
			raw_pcl_processed.header.stamp = cloud_in.header.stamp;
			raw_pcl_processed.header.frame_id = odom_frame_;

			//distinguish between "raw_pcl_buffers_" and "raw_pcl_batches_";
			raw_pcl_batches_.push_back(raw_pcl_processed);

			pcl_ros::transformPointCloud(base_frame_, raw_pcl_processed, raw_pcl_processed, *tf_ );

			pcl::ExtractIndices<RollingPointXYZ> extract_tmp;
			extract_tmp.setInputCloud (raw_pcl_processed.makeShared());
			extract_tmp.setIndices (inliers);
			extract_tmp.setNegative (false);
			extract_tmp.filter (surface_pts);
			extract_tmp.setIndices (boundary_inliers);
			extract_tmp.setNegative (false);
			extract_tmp.filter (boundary_pts);

			RollingPointCloud offroad_pts;
			offroad_pts.header = cloud_in.header;
			offroad_pts.clear();
			pcl::PointIndices::Ptr road_and_boundary_inliers (new pcl::PointIndices);
			for(size_t is=0; is<inliers->indices.size(); is++) road_and_boundary_inliers->indices.push_back(inliers->indices[is]);
			for(size_t ib=0; ib<boundary_inliers->indices.size(); ib++) road_and_boundary_inliers->indices.push_back(boundary_inliers->indices[ib]);
			extract_tmp.setIndices (road_and_boundary_inliers);
			extract_tmp.setNegative (true);
			extract_tmp.filter (offroad_pts);
			offroad_pub_.publish(offroad_pts);

			RollingPointCloud surface_pts2, boundary_pts2;
			surface_pts2.header = cloud_in.header;
			boundary_pts2.header = cloud_in.header;
			surface_pts2.clear();
			boundary_pts2.clear();
			extract_tmp.setIndices (inliers2);
			extract_tmp.setNegative (false);
			extract_tmp.filter (surface_pts2);
			extract_tmp.setIndices (boundary_inliers2);
			extract_tmp.setNegative (false);
			extract_tmp.filter (boundary_pts2);
			road_surface_pub2_.publish(surface_pts2);
			road_boundary_pub2_.publish(boundary_pts2);

			surface_index_batches_.push_back(inliers->indices);
			boundary_index_batches_.push_back(boundary_inliers->indices);


			ROS_INFO("raw pcl batch size() %ld", batchNum_limit_);
			assert( raw_pcl_batches_.size() == surface_index_batches_.size() &&
					surface_index_batches_.size() == boundary_index_batches_.size());

			if(raw_pcl_batches_.size() > batchNum_limit_)
			{
				raw_pcl_batches_.erase (raw_pcl_batches_.begin());
				surface_index_batches_.erase (surface_index_batches_.begin());
				boundary_index_batches_.erase (boundary_index_batches_.begin());
			}


			//b. update "road_surface_odom_" and "road_boundary_odom_";
			//just incorporated filtered PCLs, which is one batch delayed than the most recent batch, and compose of (batchNum_limit_-1) batches;
			sw.start("update road_surface_odom and road_boundary_odom");
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

		ROS_INFO("extraction loop finished");
	}
	
	void road_surface::road_slope_visualization(RollingPointCloud & surface_pts, RollingPointCloud & boundary_pts)
	{
		boost::recursive_mutex::scoped_lock rsl(configuration_mutex_);

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
		boost::recursive_mutex::scoped_lock pfl(configuration_mutex_);
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

		ROS_INFO("surface_pts %ld, patch_ROI %ld", surface_pts.points.size(), patch_ROI.points.size());

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

	inline int road_surface::deputy_key(float angle_tmp)
	{
		int muliplying_times = 1000;
		return (int(angle_tmp * muliplying_times ));
	}

	//pay attention to this down-sampling process: it may lead to unexpected sparsity;
	void road_surface::pcl_downsample(RollingPointCloud &point_cloud)
	{
		pcl::VoxelGrid<RollingPointXYZ> sor;

		// always good not to use in place filtering as stated in
		// http://www.pcl-users.org/strange-effect-of-the-downsampling-td3857829.html
		RollingPointCloud::Ptr input_msg_filtered (new RollingPointCloud ());
		float downsample_size_ = 0.05;
		sor.setInputCloud(point_cloud.makeShared());
		sor.setLeafSize (downsample_size_, downsample_size_, downsample_size_);
		sor.filter (*input_msg_filtered);
		point_cloud = * input_msg_filtered;
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


