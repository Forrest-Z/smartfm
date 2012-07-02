#include "single_window.h"

namespace golfcar_pcl{
	
	single_window::single_window():
	private_nh_("~")
    {
		tf_ = new tf::TransformListener();

		cloud_scan_sub_.subscribe(nh_, "cloud_pcd", 10);
		cloud_scan_filter_ = new tf::MessageFilter<sensor_msgs::PointCloud2>(cloud_scan_sub_, *tf_, "base_link", 10);
		cloud_scan_filter_->registerCallback(boost::bind(&single_window::cloudCallback, this, _1));
		cloud_scan_filter_->setTolerance(ros::Duration(0.05));

		normals_poses_pub_ = nh_.advertise<geometry_msgs::PoseArray>("normals_array", 100);		
		
		viewpoint_td_sick_.x = 1.70;
		viewpoint_td_sick_.y = 0.00;
		viewpoint_td_sick_.z = 1.53;
		
		private_nh_.param("search_radius", search_radius_, 0.10);
		private_nh_.param("curvature_thresh", curvature_thresh_, 0.2);
		
		road_surface_pub_ = nh_.advertise<PointCloud>("road_surface", 10);
		
		pub_surface_and_other_ = true;
	}
	
	single_window::~single_window(){}
	
    void single_window::cloudCallback(const sensor_msgs::PointCloud2ConstPtr cloud_in)
    {
		pcl::fromROSMsg(*cloud_in, rolling_window_baselink_);
		// Create the normal estimation class, and pass the input dataset to it
		pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
		ne.setInputCloud (rolling_window_baselink_.makeShared());

		// Create an empty kdtree representation, and pass it to the normal estimation object.
		// Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
		pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
		ne.setSearchMethod (tree);

		// Output datasets
		pcl::PointCloud<pcl::Normal> cloud_normals;

		// Use all neighbors in a sphere of radius 3cm
		ne.setRadiusSearch (search_radius_);
		ne.setViewPoint(viewpoint_td_sick_.x,viewpoint_td_sick_.y,viewpoint_td_sick_.z);
		// Compute the features
		ne.compute (cloud_normals);
		
		// concatentate the fileds
		pcl::PointCloud<pcl::PointNormal> point_normals;
		pcl::concatenateFields(rolling_window_baselink_, cloud_normals, point_normals);
		//cout<<"Point normal "<<point_normals.points[0].normal_x<<' '<<point_normals.points[0].normal_y<<' '<<point_normals.points[0].normal_z<<endl;
		
		// publish normal using visualization marker
		publishNormal(point_normals);
		
		
		extractSurface(rolling_window_baselink_, point_normals);
	}
	
	void single_window::extractSurface(pcl::PointCloud<pcl::PointXYZ>& pcl_cloud, pcl::PointCloud<pcl::PointNormal>& norm_pcl)
	{
		pcl::PCDWriter writer;
		
		
		
		std::cout << "extract surface"<< endl;
		pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
		
		//to improve here in the future;
		//kdtree.setInputCloud (&pcl_cloud);
		kdtree.setInputCloud (pcl_cloud.makeShared());
		pcl::PointXYZ searchPoint;
		searchPoint.x = 2.0;
		searchPoint.y = 0.0;
		searchPoint.z = 0.0;
		
		//just to find the initial seed of road surface;
		int K = 1;
		std::vector<int> pointIdxNKNSearch(K);
		std::vector<float> pointNKNSquaredDistance(K);
		
		int num = kdtree.nearestKSearch (searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance);
		
		if(num!=1){std::cout<<"ERROR When to find surface seed"<<endl;return;}
		std::cout << "    "  <<   pcl_cloud.points[ pointIdxNKNSearch[0] ].x 
                << " " << pcl_cloud.points[ pointIdxNKNSearch[0] ].y 
                << " " << pcl_cloud.points[ pointIdxNKNSearch[0] ].z 
                << " (squared distance: " << pointNKNSquaredDistance[0] << ")" << std::endl;
                
        //float max_radius = 0.1;
        
        pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
        inliers->indices.push_back(pointIdxNKNSearch[0]);
        for(unsigned int pt=0; pt<inliers->indices.size(); pt++)
        {
			unsigned int serial = inliers->indices[pt];
			//std::cout <<"serial  "<< serial<< endl;	
			int neighbor_num = 4;
			std::vector<int> nbPts(neighbor_num);
			std::vector<float> nbPtDis(neighbor_num); 
			
			//do not re-difine "searchPt_tmp", or some bug will happen; 
			pcl::PointXYZ searchPt_tmp;
			searchPt_tmp = pcl_cloud.points[serial];
			//std::cout <<"searching point (x,y,z)"<< searchPt_tmp.x << searchPt_tmp.y<< searchPt_tmp.z<< endl;
			
			int num = kdtree.nearestKSearch (searchPt_tmp, neighbor_num, nbPts, nbPtDis);
			if(num!=neighbor_num)std::cout<<"neighbour points not enought 4, error"<<endl;
			for(unsigned int neighbour_pt=0; neighbour_pt <nbPts.size(); neighbour_pt++)
			{
				 int nb_serial = nbPts[neighbour_pt];
				 bool no_copy = true;
				 bool curvature_flat = norm_pcl.points[nb_serial].curvature < curvature_thresh_;
				 
				 //std::cout <<"neightbour serial"<< nb_serial<< endl;
				 //std::cout <<"neightbour curvature"<< norm_pcl.points[nb_serial].curvature<< endl;
				 //std::cout <<"neightbour distance"<< nbPtDis[neighbour_pt]<< endl;
				 
				 for(unsigned int all_pt=0; all_pt<inliers->indices.size(); all_pt++)
				 {
					  if(inliers->indices[all_pt] == nb_serial) no_copy = false;
				 }
				 //if(!no_copy)std::cout <<"there is copy"<< endl;
				 //if(!curvature_flat)std::cout <<"curvature not flat"<< endl;
				 if(no_copy&&curvature_flat) inliers->indices.push_back(nb_serial);	 
			}
			//std::cout <<"inlier number for road surface  "<< inliers->indices.size()<<endl;	
		}
		
		pcl::ExtractIndices<pcl::PointXYZ> extract;
		pcl::PointCloud<pcl::PointXYZ>::Ptr surface_plane (new pcl::PointCloud<pcl::PointXYZ> ());
		
		extract.setInputCloud (pcl_cloud.makeShared());
		extract.setIndices (inliers);
		extract.setNegative (false);
		extract.filter (*surface_plane);
		road_surface_pub_.publish(*surface_plane);
		
		if(pub_surface_and_other_)
		{
			pub_surface_and_other_ = false;
			
			pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud_shift = pcl_cloud.makeShared();
			
			for(unsigned int ppp = 0; ppp<pcl_cloud_shift->points.size(); ppp++)
			{
				pcl_cloud_shift->points[ppp].x = pcl_cloud_shift->points[ppp].x + 15.0;
			}
			std::stringstream raw_shift;
			raw_shift << "/home/baoxing/workspace/PCD/raw_shift.pcd";
			writer.write<pcl::PointXYZ> (raw_shift.str (), *pcl_cloud_shift, false); 
			
			std::stringstream plane_surface;
			std::cout << "save surface plane"<<endl;
			plane_surface << "/home/baoxing/workspace/PCD/plane_surface.pcd";
			writer.write<pcl::PointXYZ> (plane_surface.str (), *surface_plane, false); 
			
			
			
			
			
			pcl::PointCloud<pcl::PointXYZ>::Ptr other_points (new pcl::PointCloud<pcl::PointXYZ>);
			extract.setNegative (true);
			extract.filter (*other_points);
			std::cout << "other_points size "<<other_points->points.size()<<endl;
			
			pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
			tree->setInputCloud (other_points);
			
			std::vector<pcl::PointIndices> cluster_indices;
			pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
			ec.setClusterTolerance (0.1); // 2cm
			ec.setMinClusterSize (50);
			ec.setMaxClusterSize (250000);
			ec.setSearchMethod (tree);
			ec.setInputCloud (other_points);
			ec.extract (cluster_indices);
			
			std::cout << "cluster_indices size "<<cluster_indices.size()<<endl;
			
			int j = 0;
			for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
			{
				pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
				for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); pit++)
				cloud_cluster->points.push_back (other_points->points[*pit]); 
				cloud_cluster->width = cloud_cluster->points.size ();
				cloud_cluster->height = 1;
				cloud_cluster->is_dense = true;

				std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size () << " data points." << std::endl;
				std::stringstream ss;
				ss << "/home/baoxing/workspace/PCD/cloud_cluster_" << j << ".pcd";
				writer.write<pcl::PointXYZ> (ss.str (), *cloud_cluster, false); 
				j++;
			}
		}
	}
	
	void single_window::publishNormal(pcl::PointCloud<pcl::PointNormal>& pcl_cloud)
	{
		pcl::PointCloud<pcl::PointNormal> edge_points;
		edge_points.header = pcl_cloud.header;
		edge_points.width = 1;

		for(size_t i=0; i<pcl_cloud.points.size();i++)
		{	
			if(fabs(pcl_cloud.points[i].curvature)>curvature_thresh_)
			{
				edge_points.points.push_back(pcl_cloud.points[i]);
			}
			//cout<<"Point normal "<<pcl_cloud.points[i].curvature<<endl;
		}
		edge_points.height = edge_points.points.size();

		// publish normal as posearray for visualization
		bool publish_normals = true;
		if(publish_normals)
		{
			geometry_msgs::PoseArray normals_poses;
			normals_poses.header = edge_points.header;
			for(unsigned int i=0; i<edge_points.points.size(); i++)
			{
				geometry_msgs::Pose normals_pose;
				geometry_msgs::Point pos;
				pos.x = edge_points.points[i].x; pos.y = edge_points.points[i].y; pos.z = edge_points.points[i].z;
				normals_pose.position = pos;
				btVector3 axis(edge_points.points[i].normal[0],edge_points.points[i].normal[1],edge_points.points[i].normal[2]);
				if(isnan(edge_points.points[i].normal[0])||isnan(edge_points.points[i].normal[1])||isnan(edge_points.points[i].normal[2])) continue;
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
	 ros::init(argc, argv, "single_window");
	 ros::NodeHandle n;
	 golfcar_pcl::single_window single_window_node;
     ros::spin();
     return 0;
}
