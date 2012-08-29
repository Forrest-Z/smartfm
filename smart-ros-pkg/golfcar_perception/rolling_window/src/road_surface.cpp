 #include "road_surface.h"
 
namespace golfcar_pcl{
	
	road_surface::road_surface():
	private_nh_("~")
   {
		pcl::PointXYZ rectangle_p1, rectangle_p2, rectangle_p3, rectangle_p4;
		rectangle_p1.x = 10.0; rectangle_p1.y = 3.0; 
		rectangle_p2.x = 4.0;  rectangle_p2.y = 3.0; 
		rectangle_p3.x = 4.0;  rectangle_p3.y = -3.0; 
		rectangle_p4.x = 10.0; rectangle_p4.y = -3.0; 
		poly_ROI_.push_back(rectangle_p1);
		poly_ROI_.push_back(rectangle_p2);
		poly_ROI_.push_back(rectangle_p3);
		poly_ROI_.push_back(rectangle_p4);
		
		geometry_msgs::Point32	viewpoint_td_sick_;
		viewpoint_td_sick_.x = 1.70; viewpoint_td_sick_.y = 0.00; viewpoint_td_sick_.z = 1.53;

		seedPoint_.x = 7.0; seedPoint_.y = 0.0; seedPoint_.z = 0.0;
		
		private_nh_.param("search_radius", search_radius_, 0.10);
		private_nh_.param("curvature_thresh", curvature_thresh_, 0.2);

		rolling_pcl_sub_ = new message_filters::Subscriber<PointCloud> (nh_, "rolling_window_pcl", 1);
		pcl_indices_sub_ = new message_filters::Subscriber<rolling_window::pcl_indices> (nh_, "process_fraction_indices", 1);
		sync_	= new message_filters::TimeSynchronizer<PointCloud, rolling_window::pcl_indices>(*rolling_pcl_sub_, *pcl_indices_sub_, 5);
		sync_->registerCallback(boost::bind(&road_surface::pclCallback, this, _1, _2));
		
		process_fraction_pub_   = nh_.advertise<PointCloud>("process_fraction", 10);
		road_surface_pub_   = nh_.advertise<PointCloud>("road_surface_pts", 10);
		road_boundary_pub_   = nh_.advertise<PointCloud>("road_boundary_pts", 10);
		fitting_plane_pub_   = nh_.advertise<PointCloud>("fitting_plane", 10);
		normals_poses_pub_ = nh_.advertise<geometry_msgs::PoseArray>("normals_array", 10);
		plane_coef_pub_ = nh_.advertise<rolling_window::plane_coef>("plane_coef", 10);
	}
	
	road_surface::~road_surface()
	{	
	}
	
	void road_surface::pclCallback(const PointCloud::ConstPtr& pcl_in, const rolling_window::pcl_indices::ConstPtr &indices_in)
	{
		PointCloud surface_pts;
		surface_pts.clear();
		surface_pts.height = 1;
		surface_pts.header =	pcl_in->header;
		
		PointCloud boundary_pts;
		boundary_pts.clear();
		boundary_pts.height = 1;
		boundary_pts.header = pcl_in->header;
		
		PointCloud fitting_plane;
		fitting_plane.clear();
		fitting_plane.height = 1;
		fitting_plane.header = pcl_in->header;
		
		pcl::PointCloud<pcl::PointNormal> point_normals;
		
		rolling_window::plane_coef plane_coefs;
      road_surface::surface_extraction(*pcl_in,  *indices_in, poly_ROI_, point_normals, surface_pts, 
													 boundary_pts, fitting_plane, plane_coefs);
		
		
		road_surface_pub_.publish(surface_pts);
		road_boundary_pub_.publish(boundary_pts);
		fitting_plane_pub_.publish(fitting_plane);
		plane_coef_pub_.publish(plane_coefs);
		publishNormal(point_normals);
	}
	
	void road_surface::surface_extraction (const PointCloud &cloud_in, const rolling_window::pcl_indices& proc_indices, 
														vector<pcl::PointXYZ> & poly_ROI,
														pcl::PointCloud<pcl::PointNormal> & point_normals,
														PointCloud & surface_pts, PointCloud & boundary_pts, 
														PointCloud & fitting_plane, rolling_window::plane_coef & plane_coef)
	{
		PointCloud process_fraction_pcl;
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
		pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> ne;
		ne.setInputCloud (cloud_in.makeShared());
		boost::shared_ptr<std::vector<int> > indicesptr (new std::vector<int> (proc_indices.indices));
		ne.setIndices (indicesptr);
		pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
		ne.setSearchMethod (tree);
		pcl::PointCloud<pcl::Normal> cloud_normals;
		ne.setRadiusSearch (search_radius_);
		ne.setViewPoint(viewpoint_td_sick_.x,viewpoint_td_sick_.y,viewpoint_td_sick_.z);
		// Compute the features
		ne.compute (cloud_normals);
		// concatentate the fileds
		//pcl::PointCloud<pcl::PointNormal> point_normals;
		pcl::concatenateFields(process_fraction_pcl, cloud_normals, point_normals);
		sw.end();
		
		//2nd region-growing method for surface extraction;
		sw.start("2. region-growing to extract surface");
		pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
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
		
		pcl::PointXYZ searchPt_tmp;
		searchPt_tmp = process_fraction_pcl.points[pointIdxNKNSearch[0]];
		std::cout <<"searching point (x,y,z)"<< searchPt_tmp.x << " " << searchPt_tmp.y<<" "<< searchPt_tmp.z<< endl;
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
		
		pcl::ExtractIndices<pcl::PointXYZ> extract;
		extract.setInputCloud (process_fraction_pcl.makeShared());
		extract.setIndices (inliers);
		extract.setNegative (false);
		extract.filter (surface_pts);
		
		ROS_INFO("boundary_inliers - size() %ld", boundary_inliers->indices.size());
		pcl::ExtractIndices<pcl::PointXYZ> extract_bd;
		extract_bd.setInputCloud (process_fraction_pcl.makeShared());
		extract_bd.setIndices (boundary_inliers);
		extract_bd.setNegative (false);
		extract_bd.filter (boundary_pts);
		
		//planefitting_ROI(surface_pts, poly_ROI, fitting_plane, plane_coef);
		
	}
	
	//input:		surface_pts;	poly_ROI;
	//output: 	fitting_plane; plane_coef;
	void road_surface::planefitting_ROI(PointCloud & surface_pts, vector<pcl::PointXYZ> & poly_ROI,
													PointCloud & fitting_plane, rolling_window::plane_coef & plane_coef)
	{
		PointCloud patch_ROI;
		patch_ROI.clear();
		patch_ROI.height = 1;
		patch_ROI.header 	=	surface_pts.header;
		
		for(size_t i=0; i<surface_pts.points.size();i++)
		{
          if(pointInPolygon(surface_pts.points[i], poly_ROI))
          {
              patch_ROI.points.push_back(surface_pts.points[i]);
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
	
	
	
	void road_surface::publishNormal(pcl::PointCloud<pcl::PointNormal>& pcl_cloud)
	{
		fmutil::Stopwatch sw;
		
		//http://www.pointclouds.org/blog/gsoc/goleary/tutorials/conditional_removal.php
		sw.start("filter pcl");
		pcl::ConditionAnd<pcl::PointNormal>::Ptr range_cond (new pcl::ConditionAnd<pcl::PointNormal> ());
		range_cond->addComparison (pcl::FieldComparison<pcl::PointNormal>::ConstPtr (new
		pcl::FieldComparison<pcl::PointNormal> ("curvature", pcl::ComparisonOps::GT, curvature_thresh_)));

 	   pcl::ConditionalRemoval<pcl::PointNormal> condrem (range_cond);
	   condrem.setInputCloud (pcl_cloud.makeShared());
	   condrem.filter (pcl_cloud);
		sw.end();

		// publish normal as posearray for visualization
		bool publish_normals = true;
		if(publish_normals)
		{
			geometry_msgs::PoseArray normals_poses;
			normals_poses.header = pcl_cloud.header;
			for(unsigned int i=0; i<pcl_cloud.points.size(); i++)
			{
				geometry_msgs::Pose normals_pose;
				geometry_msgs::Point pos;
				pos.x = pcl_cloud.points[i].x; pos.y = pcl_cloud.points[i].y; pos.z = pcl_cloud.points[i].z;
				normals_pose.position = pos;
				btVector3 axis(pcl_cloud.points[i].normal[0],pcl_cloud.points[i].normal[1],pcl_cloud.points[i].normal[2]);
				if(isnan(pcl_cloud.points[i].normal[0])||isnan(pcl_cloud.points[i].normal[1])||isnan(pcl_cloud.points[i].normal[2])) continue;
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
	
};

int main(int argc, char** argv)
{
	 ros::init(argc, argv, "road_surface_node");
	 ros::NodeHandle n;
	 golfcar_pcl::road_surface road_surface_node;
     ros::spin();
     return 0;
}

