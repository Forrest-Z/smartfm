/*
 * pcl_clustering.cpp
 *
 *  Created on: Jan 15, 2012
 *      Author: golfcar
 */
#include "ped_clustering.h"

ped_clustering::ped_clustering()
{
    ros::NodeHandle private_nh("~");

    private_nh.param("global_frame", global_frame_, string("map"));
    private_nh.param("laser_frame", laser_frame_id_, string("ldmrs0"));
    private_nh.param("prior_distance_filter", prior_distance_filter_, 0.4);
    private_nh.param("use_octomap", use_octomap_, false);
    cloud_sub_ .subscribe(nh, "sickldmrs/cloud", 10);
    tf_pc2_filter_ = new tf::MessageFilter<sensor_msgs::PointCloud2>(cloud_sub_, tf_, global_frame_, 10);
    tf_pc2_filter_->registerCallback(boost::bind(&ped_clustering::scanCallback, this, _1));
    tf_pc2_filter_->setTolerance(ros::Duration(0.05));

    cloud_pub_ = nh.advertise<sensor_msgs::PointCloud>("pedestrian_cluster", 1);
    poi_pub_= nh.advertise<sensor_msgs::PointCloud>("pedestrian_poi",1);
    line_filtered_pub_ = nh.advertise<sensor_msgs::PointCloud2>("line_filtered",1);
    after_line_filter_pub_ = nh.advertise<sensor_msgs::PointCloud2>("after_line_filtered",1);
    boundary_pub_ = nh.advertise<geometry_msgs::PolygonStamped>("ped_boundary",1, true);
    filter_pub_ = nh.advertise<sensor_msgs::PointCloud2>("cloud_filtered",1);
    clusters_pub_ = nh.advertise<feature_detection::clusters>("pedestrian_clusters",1);
    after_prior_filter_pub_ = nh.advertise<sensor_msgs::PointCloud>("prior_filtered", 1);

    laser_sub_.subscribe(nh, "scanN", 10);
    tf_filter_ = new tf::MessageFilter<sensor_msgs::LaserScan>(laser_sub_, tf_, global_frame_, 10);
    tf_filter_->registerCallback(boost::bind(&ped_clustering::laserCallback, this, _1));
    tf_filter_->setTolerance(ros::Duration(0.05));

    stringstream ss;
    ss << 12345;
    cout << ss.str().c_str() <<endl;

	if(use_octomap_)
	{
		//global_octMap = new octomap::OcTree(0.1);
	    //getOctomap();
	}

	string svg_file;
    bool use_boundary;
	private_nh.param("svg_file", svg_file, string(""));
    private_nh.param("use_boundary", use_boundary_, false);
	//double polygon[][2] = {{662.155,3136.776},{602.0509,3019.346},{574.7768,3021.24},{567.4532,3005.204},{576.1658,2988.158},{584.6258,2983.612},{588.1613,2974.773},{566.1905,2920.351},{533.108,2843.074},{605.0814,2815.8},{653.8212,2933.736},{649.7806,2937.271},{660.1347,2961.767},{679.3276,2953.939},{694.4799,2983.738},{672.7616,2996.87},{677.8124,3007.729},{680.3377,3006.214},{733.6233,3109.25},{662.155,3136.776}};
	//int npts = NPTS(polygon);
	/*for (int i = 0; i < npts; i++)
		{
			Point l; l.x = polygon[i][0]/10.0; l.y = polygon[i][1]/10.0;
			Point32 l32; l32.x = l.x; l32.y = l.y;
			boundary_.push_back(l);
			boundary_msg.polygon.points.push_back(l32);
		}*/
	geometry_msgs::PolygonStamped boundary_msg;
	boundary_msg.header.stamp = ros::Time::now();
	boundary_msg.header.frame_id = "/map";
	boundary_msg.header.seq = 1;

	if(use_boundary_){
	    svg_boundary svg(svg_file.c_str(), 0.1);
	    boundary_ = svg.getPath("crossing_boundary");
	    boundary_msg.polygon.points = boundary_;
	}

	boundary_pub_.publish(boundary_msg);
	sequential_clustering_ = false;


    ros::spin();
}

void ped_clustering::getOctomap()
{
    /*const static std::string servname = "octomap_binary";
    ROS_INFO("Requesting the map from %s...", nh.resolveName(servname).c_str());
    octomap_msgs::GetOctomap::Request req;
    octomap_msgs::GetOctomap::Response resp;
    while(nh.ok() && !ros::service::call(servname, req, resp))
    {
        ROS_WARN("Request to %s failed; trying again...", nh.resolveName(servname).c_str());
        usleep(1000000);
    }
    octomap::octomapMsgToMap(resp.map, *global_octMap);//->octree);
    double x, y, z;
    global_octMap->getMetricMax(x, y, z);
    //global_octMap->octree.getMetricMax(x,y,z);
    ROS_INFO("Map received. Size of map = %lf, %lf, %lf with resolution %lf", x, y, z, global_octMap->getResolution());

    octomapTreeToPCLoctree(resp.map, global_pclOctree_);*/
}/*
void ped_clustering::octomapTreeToPCLoctree(octomap_msgs::OctomapBinary& octomap, pcl16::octree::OctreePointCloudSearch<pcl16::PointXYZ>* pcl_octree)
{
    std::list<octomap::point3d> all_cells;
    int level=15;
    octomap::OcTree octree(0.1);
    octomap::octomapMsgToMap(octomap, octree);
    octree.getOccupied(all_cells, level);
    std::list<octomap::point3d>::iterator it;
    pcl16::PointCloud<pcl16::PointXYZ> pcl_out;
    pcl_out.header.frame_id = "/map";
    pcl_out.header.stamp = ros::Time::now();

    for (it = all_cells.begin(); it != all_cells.end(); ++it)
    {
        pcl16::PointXYZ cube_center;
        cube_center.x = it->x();
        cube_center.y = it->y();
        cube_center.z = it->z();
        pcl_out.points.push_back(cube_center);
    }

    double resolution = octree.getResolution();
    global_pclOctree_ = new pcl16::octree::OctreePointCloudSearch<pcl16::PointXYZ>(resolution);

    global_pclOctree_->setInputCloud(pcl_out.makeShared());
    global_pclOctree_->addPointsFromInputCloud();

    ROS_INFO("pcl_octree ready with leaf size %d and depth %d", global_pclOctree_->getLeafCount(), global_pclOctree_->getTreeDepth());

    srand ((unsigned int) time (NULL));

    int K = 10;

    pcl16::PointXYZ searchPoint;

    searchPoint.x = 100;
    searchPoint.y = 100;
    searchPoint.z = 1.0;
    std::vector<int> pointIdxNKNSearch;
    std::vector<float> pointNKNSquaredDistance;

    std::cout << "K nearest neighbor search at (" << searchPoint.x
            << " " << searchPoint.y
            << " " << searchPoint.z
            << ") with K=" << K << std::endl;

    if (global_pclOctree_->nearestKSearch (searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0)
    {
        for (size_t i = 0; i < pointIdxNKNSearch.size (); ++i)
            std::cout
            << " (distance: " << sqrt(pointNKNSquaredDistance[i]) << ")" << std::endl;
    }
}

void ped_clustering::filterPCLOctreeNN(pcl16::octree::OctreePointCloudSearch<pcl16::PointXYZ> &priorMap, sensor_msgs::PointCloud& pc_in, sensor_msgs::PointCloud& pc_out, ros::Publisher& pub_, double threshold)
{
    cout<<"Filtering with points "<< pc_in.points.size()<<endl;
    int missed = 0;
    for(size_t i=0; i<pc_in.points.size();)
    {

        pcl16::PointXYZ searchPoint;
        searchPoint.x = pc_in.points[i].x;
        searchPoint.y = pc_in.points[i].y;
        searchPoint.z = pc_in.points[i].z;

        int K = 1;

        vector<int> pointIdxNKNSearch;
        vector<float> pointNKNSquaredDistance;

        if (global_pclOctree_->nearestKSearch(searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0)
        {
            if(sqrt(pointNKNSquaredDistance[0])<threshold)
            {
                pc_in.points.erase(pc_in.points.begin()+i);
                continue;
            }

        }
        else
        {
            missed++;
        }
        i++;
    }

    cout<<"After filtering points = "<<pc_in.points.size()<<" with missed point = "<< missed<<endl;
    pub_.publish(pc_in);
    pc_out = pc_in;
}

*/
//core function of ped_clustering, use PCL for pointcloud clustering;
void ped_clustering::clustering(const sensor_msgs::PointCloud2 &pc, sensor_msgs::PointCloud &ped_poi,
                                double tolerance, int minSize, int maxSize, bool publish)
{
    sensor_msgs::PointCloud2 pc_temp;
    pcl16::PointCloud<pcl16::PointXYZ> cloud_outremoved, cloud_without_line;
    pcl16::PointCloud<pcl16::PointXYZ> cloud_temp;
    pcl16::fromROSMsg(pc, cloud_temp);

    for(size_t i=0; i<cloud_temp.points.size(); i++) cloud_temp.points[i].z = 0;

    //perform outlier filtering. Final output: cloud_outremoved
    if(cloud_temp.points.size()>0)
    {
        pcl16::RadiusOutlierRemoval<pcl16::PointXYZ> outrem;
        outrem.setInputCloud(cloud_temp.makeShared());
		outrem.setRadiusSearch(1.0);
		ros::NodeHandle private_nh("~");
		int radius = 8;
		private_nh.param("radius", radius, 8);
        outrem.setMinNeighborsInRadius(radius);
        outrem.filter(cloud_outremoved);
        pcl16::toROSMsg(cloud_outremoved, pc_temp);
        ROS_DEBUG("Radius filter %d", cloud_outremoved.points.size());

    }

    //Then segmentation

    sensor_msgs::PointCloud total_clusters;
    feature_detection::clusters clusters;
    pcl16::PointCloud<pcl16::PointXYZ>::Ptr cloud_cluster(new pcl16::PointCloud<pcl16::PointXYZ>);
    clusters.header = pc.header;
    if(cloud_outremoved.points.size()>0)
    {
        pcl16::PointCloud<pcl16::PointXYZ>::Ptr cloud_filtered = cloud_outremoved.makeShared();

        int cluster_number = 0;

        std::vector<pcl16::PointIndices> cluster_indices;
        extractCluster(cloud_filtered, cluster_indices, tolerance, minSize, maxSize);

        ped_poi.points.clear();

        for (std::vector<pcl16::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
        {
            feature_detection::cluster cluster;
            Eigen::Vector4f min_pt, max_pt,mid_pt, abs_distance;

            pcl:getMinMax3D(*cloud_filtered, it->indices, min_pt, max_pt);

            abs_distance = max_pt - min_pt;
            mid_pt = (max_pt + min_pt)/2.0;
            geometry_msgs::Point32 p;
            p.x = mid_pt[0];
            p.y = mid_pt[1];
            p.z = mid_pt[2];

            cluster.centroid = p;
            cluster.width = abs_distance[1];
            cluster.height = abs_distance[2];
            cluster.depth = abs_distance[0];

            //bool use_boundary_ = false;
            //obtain only points that fall into the specified boundary
            if(use_boundary_)
            {
            	PointStamped global_pt, local_pt;
            	local_pt.header = pc.header;

            	local_pt.point.x = p.x;
            	local_pt.point.y = p.y;
            	local_pt.point.z = p.z;
            	try{tf_.transformPoint(global_frame_, local_pt, global_pt);}
            	catch(tf::TransformException& e){ROS_INFO_STREAM(e.what());continue;}

            	//log shows the wn_PnPoly falls into the boundary only if output is 1
            	if(!pointInPolygon<Point>(global_pt.point, boundary_))
            		continue;
            }
            if(cluster.width < 1.5 && cluster.depth < 1.5)
            	ped_poi.points.push_back(p);
            std::vector<geometry_msgs::Point32> cluster_points;
            for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); pit++)
            {
            	pcl16::PointXYZ pclpt_temp;
            	pclpt_temp = cloud_filtered->points[*pit];
            	geometry_msgs::Point32 p_temp;
            	p_temp.x = pclpt_temp.x;
            	p_temp.y = pclpt_temp.y;
            	p_temp.z = pclpt_temp.z;
            	cluster_points.push_back(p_temp);
            	total_clusters.points.push_back(p_temp);
            }
            cluster.points = cluster_points;
            /*pcl16::PointCloud<pcl16::PointXYZ> pca_input = cloud_temp;
            pca_input.points.clear();
            ROS_DEBUG("Start of cluster %d\n", cluster_number);
            for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); pit++)
            {
                pcl16::PointXYZ pclpt_temp;
                pclpt_temp = cloud_filtered->points[*pit];
                geometry_msgs::Point32 p_temp;
                p_temp.x = pclpt_temp.x;
                p_temp.y = pclpt_temp.y;
                p_temp.z = pclpt_temp.z;

                ROS_DEBUG("%.4lf %.4lf %.4lf\n", p_temp.x, p_temp.y, p_temp.z);
                cluster_points.push_back(p_temp);

                //only the horizontal plane pca info is useful to us
                pclpt_temp.z = 0;
                pca_input.points.push_back(pclpt_temp);

                pclpt_temp.z = cluster_number;
                cloud_cluster->points.push_back (pclpt_temp);
            }
            cluster.points = cluster_points;

            pcl16::PCA<pcl16::PointXYZ> pca(pca_input);
            Eigen::VectorXf eigen_values = pca.getEigenValues();
            cluster.eigen1 = eigen_values[0];
            cluster.eigen2 = eigen_values[1];
            cluster.eigen3 = eigen_values[2];
            std::vector<geometry_msgs::Point32> projected_points;
            pcl16::PointCloud<pcl16::PointXYZ> minmax_pcl;
            ROS_DEBUG("-----------------");
            for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); pit++)
            {
                pcl16::PointXYZ pclpt_in, pclpt_out;
                pclpt_in = cloud_filtered->points[*pit];
                pca.project(pclpt_in,pclpt_out);
                geometry_msgs::Point32 p_temp;
                p_temp.x = pclpt_out.x;
                p_temp.y = pclpt_out.y;
                p_temp.z = pclpt_out.z;
                ROS_DEBUG("%.4lf %.4lf %.4lf\n", p_temp.x, p_temp.y, p_temp.z);
                projected_points.push_back(p_temp);
                minmax_pcl.points.push_back(pclpt_out);
            }
            pcl16::getMinMax3D(minmax_pcl,min_pt, max_pt);

            abs_distance = max_pt - min_pt;
            // since this is already a projected point onto eigen space,
            // need to find out which projected axis is the most prominent
            // i.e. maximum eigen value
            Eigen::Vector4f sorted_eigen_values;
            int sorting[] = {0,1,2};
            for(int i=0; i<3; i++)
            {
                int min = i;
                for(int j=i+1; j<3; j++)
                {
                    if(eigen_values[j]<eigen_values[min]) min = j;
                }
                double t1 = eigen_values[min];
                eigen_values[min] = eigen_values[i];
                eigen_values[i] = t1;
                int t2 = sorting[min];
                sorting[min] = sorting[i];
                sorting[i] = t2;
            }

            cluster.projected_l1 = abs_distance[sorting[2]];
            cluster.projected_l2 = abs_distance[sorting[1]];
            cluster.projected_l3 = abs_distance[sorting[0]];
            cluster.projected_points = projected_points;*/

            clusters.clusters.push_back(cluster);
            cluster_number++;
        }
        ROS_INFO("End of cluster %d\n", cluster_number);
    }



    total_clusters.header = pc.header;
    ped_poi.header = pc.header;
    if(publish)
    {
        cloud_pub_.publish(total_clusters);
        poi_pub_.publish(ped_poi);
        clusters_pub_.publish(clusters);
        filter_pub_.publish(pc_temp);
    }
}

void ped_clustering::scanCallback(const sensor_msgs::PointCloud2ConstPtr& pc2)
{
    double clusterTolerance = 0.7;
    int minClusterSize = 1;
    int maxClusterSize = 1000;
    sensor_msgs::PointCloud2 final_pc2 = *pc2;
    sensor_msgs::PointCloud temp;
    if(use_octomap_)
    {
    	sensor_msgs::PointCloud pc, global_pc, octreeFiltered, pcloctreeFiltered;
    	sensor_msgs::convertPointCloud2ToPointCloud(*pc2, pc);
    	try{tf_.transformPointCloud(global_frame_, pc, global_pc);}
    	catch(tf::TransformException& e){ROS_INFO_STREAM(e.what());return;}
    	//filterPriorMap(*global_octMap, global_pc, octreeFiltered, after_prior_filter_pub_, prior_distance_filter_);

    	//when using "new" keyword, it has to be directly used with the variable
    	//if not seg fault occur
    	//filterPCLOctreeNN(*global_pclOctree_, global_pc, pcloctreeFiltered, after_prior_filter_pub_, prior_distance_filter_);

    	//tranfrom back to the original frame. This is because the cluster filter need to be at the sensor frame
    	try{tf_.transformPointCloud(pc2->header.frame_id, pcloctreeFiltered, pcloctreeFiltered);}
    	catch(tf::TransformException& e){ROS_INFO_STREAM(e.what());return;}

    	sensor_msgs::convertPointCloudToPointCloud2(pcloctreeFiltered, final_pc2);

    }

    clustering(final_pc2, temp, clusterTolerance, minClusterSize, maxClusterSize, true);
}
/*
void ped_clustering::filterPriorMap(octomap::OcTree& priorMap, sensor_msgs::PointCloud& pc_in, sensor_msgs::PointCloud& pc_out, ros::Publisher& pub, double threshold)
{
    pc_out = pc_in;
    for(size_t i=0; i<pc_out.points.size(); )
    {
        octomap::point3d pt(pc_out.points[i].x, pc_out.points[i].y, pc_out.points[i].z);
        octomap::OcTreeROS::NodeType* treeNode = priorMap.search(pt);
        if(treeNode)
        {
            ROS_INFO("Occupancy at %lf, %lf, %lf is %lf", pt.x(),pt.y(), pt.z(),  treeNode->getOccupancy());
            if(treeNode->getOccupancy()>threshold)
            {
                pc_out.points.erase(pc_out.points.begin()+i);
                continue;
            }
        }
        i++;
    }
    pub.publish(pc_out);
}*/
inline geometry_msgs::Point32 getCenterPoint(geometry_msgs::Point32 start_pt, geometry_msgs::Point32 end_pt)
{
    geometry_msgs::Point32 center;
    center.x = (start_pt.x + end_pt.x)/2;
    center.y = (start_pt.y + end_pt.y)/2;
    center.z = (start_pt.z + end_pt.z)/2;
    return center;
}

void ped_clustering::laserCallback(const sensor_msgs::LaserScanConstPtr& scan_in)
{
    sensor_msgs::PointCloud pc, global_pc;

    try{projector_.transformLaserScanToPointCloud(laser_frame_id_, *scan_in, pc, tf_);}
    catch (tf::TransformException& e){ROS_INFO_STREAM(e.what());return;}

    //interleave based on map frame
    //based on initial impression, it is quite good. Points appears to be more stable

    if(scan_in->header.seq%2)
    {
    	try{tf_.transformPointCloud(global_frame_, pc, global_pc);}
        catch(tf::TransformException& e){ROS_INFO_STREAM(e.what());return;}
        laser_global_pc_.points.clear();
        laser_global_pc_ = global_pc;
        return;
    }
    else
    {
    	laser_global_pc_.header.stamp = scan_in->header.stamp;
    	sensor_msgs::PointCloud pc_temp;
    	try{tf_.transformPointCloud(scan_in->header.frame_id, laser_global_pc_, pc_temp);}
    	catch(tf::TransformException& e){ROS_INFO_STREAM(e.what());return;}
    	pc.points.insert(pc.points.end(), pc_temp.points.begin(), pc_temp.points.end());
    }
    //cout<<"laser_scan seq "<<scan_in->header.seq<<endl;

    if(sequential_clustering_)
    {
        /*todo: publish pointcloud_vector msg*/
        double diff_t = 1.0;
        double seconddiff_t = 1.5;
        double z = 0;
        pc.points[0].z = 0;
        sensor_msgs::PointCloud centroid;
        centroid.header = pc.header;
        geometry_msgs::Point32 start_pt, center_pt;
        start_pt = pc.points[0];
        for(int i=1; i<pc.points.size(); i++)
        {
            if(fmutil::distance(pc.points[i-1].x, pc.points[i-1].y, pc.points[i].x, pc.points[i].y)>diff_t)
            {
                centroid.points.push_back(getCenterPoint(pc.points[i-1],start_pt));

                start_pt = pc.points[i];
                z++;
            }
            pc.points[i].z=z;
        }
        centroid.points.push_back(getCenterPoint(pc.points[pc.points.size()-1],start_pt));

        //compare the centroids, join them if there are near together
        //for now it will just look through exhaustively, more efficient method e.g. kNN search can be implemented
        sensor_msgs::PointCloud centroid2;
        centroid2.header = centroid.header;
        start_pt = centroid.points[0];
        //ROS_INFO("1st pass: %d", centroid.points.size());
        cloud_pub_.publish(centroid);
        vector<geometry_msgs::Point32>::iterator it;
        int first_loop = 0;
        for( it=centroid.points.begin(); it<centroid.points.end()-1; it++)
        {
            int second_loop = 0;
            cout<<"1L: "<<first_loop++<<endl;
            vector<geometry_msgs::Point32>::iterator it2;
            for(it2=it+1; it2<centroid.points.end(); it2++)
            {
                cout<<"2L: ";
                cout<<second_loop++<<endl;
                if(fmutil::distance(it->x, it->y, it2->x, it2->y)<seconddiff_t)
                {
                    cout<<"Erasing "<<second_loop<<endl;
                    geometry_msgs::Point32 cp = getCenterPoint(*it,*it2);
                    it->x = cp.x; it->y = cp.y; it->z = cp.z;
                    centroid.points.erase(it2);
                }
            }

        }
        poi_pub_.publish(centroid);
    }
    else
    {
        //A workaround with an apparent bug where sometimes It shows a different
        //cluster each with single point although there are separated by less than the threshold value
    	//perform interleave action by default
        //pc.points.insert(pc.points.end(),pc.points.begin(),pc.points.end());
        sensor_msgs::PointCloud2 pc2;
        sensor_msgs::PointCloud output;
        sensor_msgs::convertPointCloudToPointCloud2(pc,pc2);
        clustering(pc2, output, 0.7, 2, 100, true);
    }

}

void ped_clustering::extractCluster(pcl16::PointCloud<pcl16::PointXYZ>::Ptr cloud_filtered, std::vector<pcl16::PointIndices>& cluster_indices,
                                    double clusterTolerance, int minSize,int maxSize)
{
	ROS_INFO("cloud_filter size(); cloud_filtered->points.size() %d, %d", (int)cloud_filtered->size(), (int)cloud_filtered->points.size());
    if(cloud_filtered->points.size()<1) return;
    // Creating the KdTree object for the search method of the extraction
    pcl16::search::KdTree<pcl16::PointXYZ>::Ptr tree(new pcl16::search::KdTree<pcl16::PointXYZ>);
    tree->setInputCloud (cloud_filtered);


    pcl16::EuclideanClusterExtraction<pcl16::PointXYZ> ec;
    ec.setClusterTolerance (clusterTolerance);//0.5);
    ec.setMinClusterSize (minSize);//3);
    ec.setMaxClusterSize (maxSize);//100);
    ec.setSearchMethod (tree);
    ec.setInputCloud( cloud_filtered);

    cluster_indices.clear();
    ec.extract (cluster_indices);
}

void ped_clustering::filterLines(pcl16::PointCloud<pcl16::PointXYZ>::Ptr cloud_in, pcl16::PointCloud<pcl16::PointXYZ>& cloud_out)
{
    // Create the segmentation object for the planar model and set all the parameters
    pcl16::SACSegmentation<pcl16::PointXYZ> seg;
    pcl16::PointIndices::Ptr inliers (new pcl16::PointIndices);
    pcl16::ModelCoefficients::Ptr coefficients (new pcl16::ModelCoefficients);
    pcl16::PointCloud<pcl16::PointXYZ>::Ptr cloud_plane (new pcl16::PointCloud<pcl16::PointXYZ> ());

    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl16::SACMODEL_LINE);
    seg.setMethodType (pcl16::SAC_RANSAC);
    seg.setMaxIterations (100);
    seg.setDistanceThreshold (0.1);
    pcl16::PointCloud<pcl16::PointXYZ>::Ptr cloud_f (new pcl16::PointCloud<pcl16::PointXYZ>);

    // Segment the largest planar component from the remaining cloud
    int i=0, nr_points = (int) cloud_in->points.size ();
    for(int i=0; i<1; i++)
    {
        seg.setInputCloud(cloud_in);
        seg.segment (*inliers, *coefficients); //*
        if (inliers->indices.size () == 0)
        {
            std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
            return;
        }

        // Extract the planar inliers from the input cloud
        pcl16::ExtractIndices<pcl16::PointXYZ> extract;
        extract.setInputCloud (cloud_in);
        extract.setIndices (inliers);
        extract.setNegative (false);

        // Write the planar inliers to disk
        extract.filter (*cloud_plane); //*
        sensor_msgs::PointCloud2 pc_temp;
        pcl16::toROSMsg(*cloud_plane, pc_temp);
        line_filtered_pub_.publish(pc_temp);
        //std::cout << "PointCloud representing the planar component: " << cloud_plane->points.size () << " data points." << std::endl;

        // Remove the planar inliers, extract the rest
        extract.setNegative (true);
        extract.filter (*cloud_f);
        cloud_in = cloud_f;
    }
    cloud_out = *cloud_in;
}
int
main (int argc, char** argv)
{
    ros::init(argc, argv, "ped_clustering");
    ped_clustering ped_cluster;
    return (0);
}
