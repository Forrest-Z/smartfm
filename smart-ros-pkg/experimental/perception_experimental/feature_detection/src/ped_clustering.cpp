/*
 * pcl_clustering.cpp
 *
 *  Created on: Jan 15, 2012
 *      Author: golfcar
 */
#include <ros/ros.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/common.h>
#include <pcl/common/pca.h>
#include <pcl/octree/octree.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <fmutil/fm_math.h>
#include <std_msgs/Int64.h>

#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <laser_geometry/laser_geometry.h>
#include <tf/message_filter.h>
#include <message_filters/subscriber.h>
#include <feature_detection/clusters.h>
#include <feature_detection/cluster.h>

#include <octomap_ros/OctomapROS.h>
#include <octomap_ros/GetOctomap.h>
#include <octomap/octomap.h>
#include <pcl/filters/radius_outlier_removal.h>
using namespace std;

class ped_clustering
{
public:
    ped_clustering();
private:
    void scanCallback(const sensor_msgs::PointCloud2ConstPtr& pc2);
    void clustering(const sensor_msgs::PointCloud2& pc2, sensor_msgs::PointCloud &ped_poi,
                    double tolerance, int minSize, int maxSize, bool publish);
    void filterLines(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in, pcl::PointCloud<pcl::PointXYZ>& cloud_out);
    void extractCluster(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered, std::vector<pcl::PointIndices>& cluster_indices,
                        double clusterTolerance, int minSize,int maxSize);
    void laserCallback(const sensor_msgs::LaserScanConstPtr& scan_in);
    void filterPriorMap(octomap::OcTree& priorMap, sensor_msgs::PointCloud& pc_in, sensor_msgs::PointCloud& pc_out, ros::Publisher& pub_, double threshold);
    void octomapTreeToPCLoctree(octomap_ros::OctomapBinary& octomap, pcl::octree::OctreePointCloud<pcl::PointXYZ>* pcl_octree);
    void filterPCLOctreeNN(pcl::octree::OctreePointCloud<pcl::PointXYZ> &priorMap, sensor_msgs::PointCloud& pc_in, sensor_msgs::PointCloud& pc_out, ros::Publisher& pub_, double threshold);
    message_filters::Subscriber<sensor_msgs::PointCloud2> cloud_sub_;
    message_filters::Subscriber<sensor_msgs::LaserScan> laser_sub_;
    ros::Publisher cloud_pub_, filter_pub_, after_line_filter_pub_, after_prior_filter_pub_;
    ros::Publisher poi_pub_, line_filtered_pub_;
    ros::Publisher clusters_pub_;

    tf::TransformListener tf_;
    tf::MessageFilter<sensor_msgs::LaserScan> * tf_filter_;
    tf::MessageFilter<sensor_msgs::PointCloud2>* tf_pc2_filter_;
    double prior_distance_filter_;
    laser_geometry::LaserProjection projector_;
    string laser_frame_id_, global_frame_;
    bool bounding_box_filter_, line_filter_;
    bool sequential_clustering_;
    octomap::OcTree* global_octMap;
    pcl::octree::OctreePointCloud<pcl::PointXYZ>* global_pclOctree_;
    void getOctomap();
    ros::NodeHandle nh;
};

ped_clustering::ped_clustering()
{
    ros::NodeHandle private_nh("~");

    private_nh.param("gloabl_frame", global_frame_, string("map"));
    private_nh.param("laser_frame", laser_frame_id_, string("ldmrs0"));
    private_nh.param("prior_distance_filter", prior_distance_filter_, 0.4);
    cloud_sub_ .subscribe(nh, "sickldmrs/cloud", 10);
    tf_pc2_filter_ = new tf::MessageFilter<sensor_msgs::PointCloud2>(cloud_sub_, tf_, global_frame_, 10);
    tf_pc2_filter_->registerCallback(boost::bind(&ped_clustering::scanCallback, this, _1));
    tf_pc2_filter_->setTolerance(ros::Duration(0.05));

    cloud_pub_ = nh.advertise<sensor_msgs::PointCloud>("pedestrian_cluster", 1);
    poi_pub_= nh.advertise<sensor_msgs::PointCloud>("pedestrian_poi",1);
    line_filtered_pub_ = nh.advertise<sensor_msgs::PointCloud2>("line_filtered",1);
    after_line_filter_pub_ = nh.advertise<sensor_msgs::PointCloud2>("after_line_filtered",1);
    filter_pub_ = nh.advertise<sensor_msgs::PointCloud2>("cloud_filtered",1);
    clusters_pub_ = nh.advertise<feature_detection::clusters>("pedestrian_clusters",1);
    after_prior_filter_pub_ = nh.advertise<sensor_msgs::PointCloud>("prior_filtered", 1);

    laser_sub_.subscribe(nh, "sickldmrs/scan0", 10);
    tf_filter_ = new tf::MessageFilter<sensor_msgs::LaserScan>(laser_sub_, tf_, laser_frame_id_, 10);
    tf_filter_->registerCallback(boost::bind(&ped_clustering::laserCallback, this, _1));
    tf_filter_->setTolerance(ros::Duration(0.05));


    global_octMap = new octomap::OcTree(0.1);
    getOctomap();
    ros::spin();
}

void ped_clustering::getOctomap()
{
    const static std::string servname = "octomap_binary";
    ROS_INFO("Requesting the map from %s...", nh.resolveName(servname).c_str());
    octomap_ros::GetOctomap::Request req;
    octomap_ros::GetOctomap::Response resp;
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
    sequential_clustering_ = false;
    octomapTreeToPCLoctree(resp.map, global_pclOctree_);
}
void ped_clustering::octomapTreeToPCLoctree(octomap_ros::OctomapBinary& octomap, pcl::octree::OctreePointCloud<pcl::PointXYZ>* pcl_octree)
{
    std::list<octomap::point3d> all_cells;
    int level=15;
    octomap::OcTree octree(0.1);
    octomap::octomapMsgToMap(octomap, octree);
    octree.getOccupied(all_cells, level);
    std::list<octomap::point3d>::iterator it;
    pcl::PointCloud<pcl::PointXYZ> pcl_out;
    pcl_out.header.frame_id = "/map";
    pcl_out.header.stamp = ros::Time::now();

    for (it = all_cells.begin(); it != all_cells.end(); ++it)
    {
        pcl::PointXYZ cube_center;
        cube_center.x = it->x();
        cube_center.y = it->y();
        cube_center.z = it->z();
        pcl_out.points.push_back(cube_center);
    }

    double resolution = octree.getResolution();
    global_pclOctree_ = new pcl::octree::OctreePointCloud<pcl::PointXYZ>(resolution);

    global_pclOctree_->setInputCloud(pcl_out.makeShared());
    global_pclOctree_->addPointsFromInputCloud();

    ROS_INFO("pcl_octree ready with leaf size %d and depth %d", global_pclOctree_->getLeafCount(), global_pclOctree_->getTreeDepth());

    srand ((unsigned int) time (NULL));

    int K = 10;

    pcl::PointXYZ searchPoint;

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

void ped_clustering::filterPCLOctreeNN(pcl::octree::OctreePointCloud<pcl::PointXYZ> &priorMap, sensor_msgs::PointCloud& pc_in, sensor_msgs::PointCloud& pc_out, ros::Publisher& pub_, double threshold)
{
    cout<<"Filtering with points "<< pc_in.points.size()<<endl;
    int missed = 0;
    for(size_t i=0; i<pc_in.points.size();)
    {

        pcl::PointXYZ searchPoint;
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
void ped_clustering::clustering(const sensor_msgs::PointCloud2 &pc, sensor_msgs::PointCloud &ped_poi,
                                double tolerance, int minSize, int maxSize, bool publish)
{
    sensor_msgs::PointCloud2 pc_temp;
    pcl::PointCloud<pcl::PointXYZ> cloud_outremoved, cloud_without_line;
    pcl::PointCloud<pcl::PointXYZ> cloud_temp;
    pcl::fromROSMsg(pc, cloud_temp);

    for(size_t i=0; i<cloud_temp.points.size(); i++) cloud_temp.points[i].z = 0;

    //perform outlier filtering. Final output: cloud_outremoved
    if(cloud_temp.points.size()>0)
    {
        pcl::RadiusOutlierRemoval<pcl::PointXYZ> outrem;
        outrem.setInputCloud(cloud_temp.makeShared());
        outrem.setRadiusSearch(0.5);
        outrem.setMinNeighborsInRadius(4);
        outrem.filter(cloud_outremoved);
        pcl::toROSMsg(cloud_outremoved, pc_temp);
        ROS_DEBUG("Radius filter %d", cloud_outremoved.points.size());

    }

    //Then segmentation
    feature_detection::clusters clusters;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZ>);
    clusters.header = pc.header;
    if(cloud_outremoved.points.size()>0)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered = cloud_outremoved.makeShared();

        int cluster_number = 0;

        std::vector<pcl::PointIndices> cluster_indices;
        extractCluster(cloud_filtered, cluster_indices, tolerance, minSize, maxSize);

        ped_poi.points.clear();


        for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
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
            ped_poi.points.push_back(p);
            cluster.centroid = p;
            cluster.width = abs_distance[1];
            cluster.height = abs_distance[2];
            cluster.depth = abs_distance[0];
            std::vector<geometry_msgs::Point32> cluster_points;
            pcl::PointCloud<pcl::PointXYZ> pca_input = cloud_temp;
            pca_input.points.clear();
            ROS_DEBUG("Start of cluster %d\n", cluster_number);
            for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); pit++)
            {
                pcl::PointXYZ pclpt_temp;
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

            pcl::PCA<pcl::PointXYZ> pca(pca_input);
            Eigen::VectorXf eigen_values = pca.getEigenValues();
            cluster.eigen1 = eigen_values[0];
            cluster.eigen2 = eigen_values[1];
            cluster.eigen3 = eigen_values[2];
            std::vector<geometry_msgs::Point32> projected_points;
            pcl::PointCloud<pcl::PointXYZ> minmax_pcl;
            ROS_DEBUG("-----------------");
            for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); pit++)
            {
                pcl::PointXYZ pclpt_in, pclpt_out;
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
            pcl::getMinMax3D(minmax_pcl,min_pt, max_pt);

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
            cluster.projected_points = projected_points;


            clusters.clusters.push_back(cluster);
            ROS_DEBUG("End of cluster %d\n", cluster_number);
            cluster_number++;
        }
    }
    sensor_msgs::PointCloud2 total_clusters2;

    pcl::toROSMsg(*cloud_cluster, total_clusters2);
    sensor_msgs::PointCloud total_clusters;
    sensor_msgs::convertPointCloud2ToPointCloud(total_clusters2, total_clusters);
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
    int minClusterSize = 3;
    int maxClusterSize = 1000;
    sensor_msgs::PointCloud pc, temp, global_pc, octreeFiltered, pcloctreeFiltered;
    sensor_msgs::convertPointCloud2ToPointCloud(*pc2, pc);
    try{tf_.transformPointCloud(global_frame_, pc, global_pc);}
    catch(tf::TransformException& e){ROS_INFO_STREAM(e.what());return;}
    //filterPriorMap(*global_octMap, global_pc, octreeFiltered, after_prior_filter_pub_, prior_distance_filter_);

    //when using "new" keyword, it has to be directly used with the variable
    //if not seg fault occur
    filterPCLOctreeNN(*global_pclOctree_, global_pc, pcloctreeFiltered, after_prior_filter_pub_, prior_distance_filter_);

    //tranfrom back to the original frame. This is because the cluster filter need to be at the sensor frame
    try{tf_.transformPointCloud(pc2->header.frame_id, pcloctreeFiltered, pcloctreeFiltered);}
    catch(tf::TransformException& e){ROS_INFO_STREAM(e.what());return;}
    sensor_msgs::PointCloud2 filtered_pc2;
    sensor_msgs::convertPointCloudToPointCloud2(pcloctreeFiltered, filtered_pc2);

    clustering(filtered_pc2, temp, clusterTolerance, minClusterSize, maxClusterSize, true);
}

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
}
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
    sensor_msgs::PointCloud pc;

    try{projector_.transformLaserScanToPointCloud(laser_frame_id_, *scan_in, pc, tf_);}
    catch (tf::TransformException& e){ROS_INFO_STREAM(e.what());return;}

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
        pc.points.insert(pc.points.end(),pc.points.begin(),pc.points.end());
        sensor_msgs::PointCloud2 pc2;
        sensor_msgs::PointCloud output;
        sensor_msgs::convertPointCloudToPointCloud2(pc,pc2);
        clustering(pc2, output, 1.2, 2, 100, true);
    }

}

void ped_clustering::extractCluster(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered, std::vector<pcl::PointIndices>& cluster_indices,
                                    double clusterTolerance, int minSize,int maxSize)
{
    if(cloud_filtered->size()==0) return;
    // Creating the KdTree object for the search method of the extraction
    pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr tree (new pcl::KdTreeFLANN<pcl::PointXYZ>);
    tree->setInputCloud (cloud_filtered);


    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance (clusterTolerance);//0.5);
    ec.setMinClusterSize (minSize);//3);
    ec.setMaxClusterSize (maxSize);//100);
    ec.setSearchMethod (tree);
    ec.setInputCloud( cloud_filtered);

    ec.extract (cluster_indices);
}
void ped_clustering::filterLines(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in, pcl::PointCloud<pcl::PointXYZ>& cloud_out)
{
    // Create the segmentation object for the planar model and set all the parameters
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane (new pcl::PointCloud<pcl::PointXYZ> ());

    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_LINE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setMaxIterations (100);
    seg.setDistanceThreshold (0.1);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_f (new pcl::PointCloud<pcl::PointXYZ>);

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
        pcl::ExtractIndices<pcl::PointXYZ> extract;
        extract.setInputCloud (cloud_in);
        extract.setIndices (inliers);
        extract.setNegative (false);

        // Write the planar inliers to disk
        extract.filter (*cloud_plane); //*
        sensor_msgs::PointCloud2 pc_temp;
        pcl::toROSMsg(*cloud_plane, pc_temp);
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
