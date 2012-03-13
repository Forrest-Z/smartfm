
#include "data_assoc.h"

using namespace std;




data_assoc::data_assoc(int argc, char** argv) 
{
    ROS_DEBUG("Starting Pedestrian Avoidance ... ");

    /// Setting up subsciption
    ros::NodeHandle nh;

    pedClustSub_.subscribe(nh, "pedestrian_clusters", 10);
    pedVisionAngularSub_.subscribe(nh, "pedestrian_roi", 10);
    /// TBP : how to add multiple subscription to same call back ????

    ros::NodeHandle n("~");

    n.param("global_frame", global_frame_, string("map"));
    n.param("camera_frame", camera_frame_, string("usb_cam"));
    n.param("time_out", time_out_, 3.0);
    n.param("poll_increment", poll_inc_, 0.1);
    n.param("poll_decrement", poll_dec_, 0.05);

    /// Setting up publishing
    pedPub_ = nh.advertise<sensing_on_road::pedestrian_vision_batch>("ped_data_assoc",1); /// topic name
    visualizer_ = nh.advertise<sensor_msgs::PointCloud>("ped_data_assoc_visual",1);
    latest_id=0;

    listener_ = new tf::TransformListener(ros::Duration(10));

    /// Setting up callback with transform cache
    laser_tf_filter_ = new tf::MessageFilter<feature_detection::clusters>(pedClustSub_, *listener_, global_frame_, 10);
    laser_tf_filter_->registerCallback(boost::bind(&data_assoc::pedClustCallback, this, _1));

    vision_angular_tf_filter_ = new tf::MessageFilter<sensor_msgs::PointCloud>(pedVisionAngularSub_, *listener_, "usb_cam", 10);
    vision_angular_tf_filter_ -> registerCallback(boost::bind(&data_assoc::pedVisionAngularCallback, this, _1));

}

bool sort_clg(centroid_local_global const &a, centroid_local_global const &b)
{
    return a.local_centroid.x < b.local_centroid.x;
}

bool sort_point(local_lPedInView const &a, local_lPedInView const&b)
{
    return a.location.point.x < b.location.point.x;
}

void data_assoc::increaseConfidence(int id)
{
    for(size_t i=0; i < lPedInView.pd_vector.size(); i++)
    {
        if(lPedInView.pd_vector[i].object_label == id) lPedInView.pd_vector[i].confidence+=0.01;
    }
}

void data_assoc::pedVisionAngularCallback(sensor_msgs::PointCloudConstPtr pedestrian_vision_angular)
{
    ROS_INFO(" Entering vision call back with lPedInView %d", lPedInView.pd_vector.size());
    std::vector<geometry_msgs::Point32> vision_point = pedestrian_vision_angular->points;
    ROS_INFO("Getting new id if any. Vision roi points received=%d", vision_point.size());

    //get a copy lPedInView in local frame and sort it
    vector<local_lPedInView> lPedInView_local;
    for(size_t i=0; i < lPedInView.pd_vector.size(); i++)
    {
        local_lPedInView ped_temp;
        geometry_msgs::PointStamped global_point;
        global_point.header = lPedInView.header;
        global_point.point.x = lPedInView.pd_vector[i].cluster.centroid.x;
        global_point.point.y = lPedInView.pd_vector[i].cluster.centroid.y;
        global_point.point.z = lPedInView.pd_vector[i].cluster.centroid.z;
        ped_temp.location.header = lPedInView.header;
        ped_temp.id = lPedInView.pd_vector[i].object_label;
        //todo: frame_id at param
        ped_temp.location.header.frame_id = camera_frame_;
        transformGlobalToLocal(global_point, ped_temp.location);

        lPedInView_local.push_back(ped_temp);
    }
    sort(lPedInView_local.begin(), lPedInView_local.end(), sort_point);

    //whatever tracking cluster that is in view is erased from the vision points
    for(size_t i=0; i < lPedInView_local.size(); i++)
    {
        double laser_cluster_angular = fmutil::r2d(atan2(lPedInView_local[i].location.point.x, -lPedInView_local[i].location.point.y));

        for(size_t j=0; j < vision_point.size(); )
        {
            double vision_angular1 =fmutil::r2d(atan2(vision_point[j].x, -vision_point[j].y));
            double vision_angular2 = fmutil::r2d(atan2(vision_point[j+1].x, -vision_point[j+1].y));
            ROS_DEBUG("laser_angular %lf, vision_angular1 %lf, vision_angular2 %lf\n", laser_cluster_angular, vision_angular1, vision_angular2);
            if(fmutil::isWithin(laser_cluster_angular, vision_angular1, vision_angular2))
            {
                ROS_DEBUG("Vision at %lf %lf will be erased, size = %d\n", vision_angular1, vision_angular2, vision_point.size());
                ROS_DEBUG("Matched with tracking lPedInView at %lf %lf\n", lPedInView_local[i].location.point.x, lPedInView_local[i].location.point.y);
                increaseConfidence(lPedInView_local[i].id);
                vision_point.erase(vision_point.begin()+j, vision_point.begin()+j+2);
            }
            else j+=2;
        }
    }
/*
    vector<centroid_local_global> clg;
    getLatestLaserCluster(clg);
    if(clg.size()==0) return;
    ROS_INFO("Laser copy of %d obtained and remaining %d vision_points", clg.size(), vision_point.size());
    //sort the cluster according to the distance from local frame
    sort(clg.begin(), clg.end(), sort_clg);
    //add the possible vision recognised pedestrian into lPedInView
    for(size_t i=0; i<vision_point.size(); i+=2)
    {
        double vision_angular1 = fmutil::r2d(atan2(vision_point[i].x, -vision_point[i].y));
        double vision_angular2 = fmutil::r2d(atan2(vision_point[i+1].x, -vision_point[i+1].y));
        for(size_t j=0; j<clg.size(); j++)
        {
            double laser_angular = fmutil::r2d(atan2(clg[j].local_centroid.x, -clg[j].local_centroid.y));
            //printf("laser_angular %lf, vision_angular1 %lf, vision_angular2 %lf\n", laser_angular, vision_angular1, vision_angular2);
            if(fmutil::isWithin(laser_angular, vision_angular1, vision_angular2))
            {
                //new laser cluster that match with vision proposal found, proceed to assign a new id
                if(clg.size())
                {
                    //calibration is important, or may use ray tracing method
                    sensing_on_road::pedestrian_vision newPed;
                    newPed.object_label = latest_id++;
                    newPed.cluster.centroid = clg[j].global_centroid;
                    newPed.cluster.last_update = ros::Time::now();
                    ROS_INFO_STREAM( "Creating new pedestrian from vision with id #" << latest_id << " at x:" << newPed.cluster.centroid.x << " y:" << newPed.cluster.centroid.y);
                    lPedInView.pd_vector.push_back(newPed);
                    clg.erase(clg.begin()+j);
                }
            }
        }
    }*/

    publishPed();
}

data_assoc::~data_assoc()
{
    //cmd.angular.z = 0;
    //cmd.linear.x = 0;
    //cmdPub_.publish(cmd);
}

double dist(geometry_msgs::Point32 A, geometry_msgs::Point32 B)
{
    double distance = -1;
    //if(A.x || B.x || A.y || B.y)
    distance = sqrt( (A.x -B.x) *(A.x-B.x) + (A.y -B.y) *(A.y-B.y));

    return distance;
}

bool data_assoc::transformGlobalToLocal(geometry_msgs::PointStamped& global_point, geometry_msgs::PointStamped& local_point)
{
    try{

        listener_->transformPoint(local_point.header.frame_id, global_point, local_point);
    }
    catch(tf::TransformException& ex){
        ROS_ERROR("Received an exception trying to transform point: %s", ex.what());
        return false;
    }

    return true;
}

bool data_assoc::transformPointToGlobal(std_msgs::Header header, geometry_msgs::Point32 input_point, geometry_msgs::Point32& output_point)
{
    //why there is geometry_msgs::Point32 and geometry_msgs??
    try{
        geometry_msgs::PointStamped global_point, local_point;
        local_point.header = header;
        local_point.point.x = input_point.x;
        local_point.point.y = input_point.y;
        local_point.point.z = input_point.z;
        listener_->transformPoint(global_frame_, local_point, global_point);
        geometry_msgs::Point32 p32;
        p32.x = global_point.point.x; p32.y = global_point.point.y; p32.z = global_point.point.z;
        output_point = p32;
    }
    catch(tf::TransformException& ex){
        ROS_ERROR("Received an exception trying to transform point: %s", ex.what());
        return false;
    }

    return true;
}

void data_assoc::getLatestLaserCluster(vector<centroid_local_global> &clg_copy)
{
    for(size_t i=0;i<laser_latest_global_.size();i++)
    {
        centroid_local_global clg;
        clg.global_centroid = laser_latest_global_[i];
        clg.local_centroid = laser_latest_local_[i];
        clg_copy.push_back(clg);
    }
}

void data_assoc::pedClustCallback(feature_detection::clustersConstPtr cluster_vector)
{
    frame_id_ = cluster_vector->header.frame_id;
    ROS_INFO_STREAM( " Entering pedestrian call back with lPedInView " << lPedInView.pd_vector.size() << " With frame id "<< frame_id_ );
    /// loop over clusters to match with existing lPedInView
    std::vector<feature_detection::cluster> clusters = cluster_vector->clusters;

    //pedestrian_vision_vector.pd_vector[].cluster.centroid;
    laser_latest_local_.clear();
    laser_latest_global_.clear();
    //get a copy to be used for laser
    for(int i=0; i < clusters.size(); i++)
    {
        laser_latest_local_.push_back(clusters[i].centroid);
        geometry_msgs::Point32 global_point;
        bool transformed = transformPointToGlobal(cluster_vector->header, clusters[i].centroid, global_point);
        laser_latest_global_.push_back(global_point);
    }

    feature_detection::clusters clusters_visualize;
    clusters_visualize.header = cluster_vector->header;
    lPedInView.header = cluster_vector->header;
    lPedInView.header.frame_id = global_frame_;
    resetLPedInViewDecisionflag();
    for(int jj=0; jj< lPedInView.pd_vector.size(); jj++)
    {
        double minDist=10000;
        int minID=-1;
        lPedInView.pd_vector[jj].decision_flag = false;
        for(int ii=0; ii < clusters.size(); ii++)
        {
            geometry_msgs::Point32 global_point;
            bool transformed = transformPointToGlobal(cluster_vector->header, clusters[ii].centroid, global_point);
            if(!transformed) return;
            double currDist = dist(lPedInView.pd_vector[jj].cluster.centroid, global_point);
            if( (currDist < minDist) && currDist>-1)
            {
                minDist = currDist;
                minID = ii;
            }
        }

        if(minDist < NN_MATCH_THRESHOLD)
        {

            /// if cluster matched, remove from contention
            if(-1 != minID)
            {
                ROS_DEBUG_STREAM(" Cluster matched with ped id #" << lPedInView.pd_vector[jj].object_label );
                geometry_msgs::Point32 global_point;
                bool transformed = transformPointToGlobal(cluster_vector->header, clusters[minID].centroid,global_point);
                if(!transformed) return;
                //we are update everything except the id;

                lPedInView.pd_vector[jj].cluster = clusters[minID];
                lPedInView.pd_vector[jj].cluster.id = lPedInView.pd_vector[jj].object_label;
                lPedInView.pd_vector[jj].cluster.centroid = global_point;
                lPedInView.pd_vector[jj].cluster.last_update = ros::Time::now();
                /// remove minID element
                if(clusters.size())
                    clusters.erase(clusters.begin()+minID);
                lPedInView.pd_vector[jj].decision_flag = true;

                //update the merge list also
                for(size_t j=0; j<merge_lists.size();j++)
                {
                    //the first element is the active element
                    if(lPedInView.pd_vector[jj].cluster.id == merge_lists[j][0])
                    {
                        //the merge list should have at least 2 elements
                        assert(merge_lists[j].size()>1);
                        for(size_t k=1; k<merge_lists[j].size();k++) updatelPedInViewWithID(merge_lists[j][k], lPedInView.pd_vector[jj]);
                    }
                }
            }
        }

    }


    //check for possible merged clusters
    for(size_t i=0; i<lPedInView.pd_vector.size();i++)
    {
        if(lPedInView.pd_vector[i].decision_flag) continue;
        double minDist=10000;
        int minID=-1;
        for(int j=0; j < lPedInView.pd_vector.size(); j++)
        {
            if(!lPedInView.pd_vector[j].decision_flag) continue;

            double currDist = dist(lPedInView.pd_vector[j].cluster.centroid, lPedInView.pd_vector[i].cluster.centroid);
            if( (currDist < minDist) && currDist>-1)
            {
                minDist = currDist;
                minID = j;
            }
        }

        if(minDist < NN_MATCH_THRESHOLD)
        {
            //it will only add to the merge list and update the lPedInView
            //unlike erasing, although it is very effective, but a new id will be generated if the pedestrian split again
            //lPedInView.pd_vector.erase(lPedInView.pd_vector.begin()+i);

            //find if the cluster is inside merged list
            bool merged_cluster = false;
            for(size_t k=0;k<merge_lists.size();k++)
            {
                if(merge_lists[k][0] == lPedInView.pd_vector[minID].object_label)
                {
                    merge_lists[k].push_back(lPedInView.pd_vector[i].object_label);
                    updatelPedInView(lPedInView.pd_vector[minID], lPedInView.pd_vector[i]);
                    merged_cluster = true;
                    break;
                }
            }
            if(!merged_cluster)
            {
                //new element under merge list need to be created
                vector<int> new_merge_id;
                new_merge_id.push_back(lPedInView.pd_vector[minID].object_label);
                new_merge_id.push_back(lPedInView.pd_vector[i].object_label);
                merge_lists.push_back(new_merge_id);
                ROS_WARN("New merge id created with %d %d with lists %d", new_merge_id[0], new_merge_id[1], merge_lists.size());

            }
        }
    }
    ROS_INFO("Remaining clusters %d", clusters.size());
    for(size_t i=0; i<clusters.size(); i++)
    {
        geometry_msgs::Point32 global_point;
        bool transformed = transformPointToGlobal(cluster_vector->header, clusters[i].centroid,global_point);
        if(!transformed) continue;

        //check for possible split cluster

        double minDist=10000;
        int minID=-1;
        ROS_INFO("Check for possible split cluster from merge list");
        for(size_t j=0; j<merge_lists.size(); j++)
        {
            for(size_t k=1; k<merge_lists[j].size(); k++)
            {
                geometry_msgs::Point32 merge_centroid;
                if(getlPedInViewCentroid(merge_lists[j][k], merge_centroid))
                {
                    double currDist = dist(merge_centroid, global_point);
                    if( (currDist < minDist) && currDist>-1)
                    {
                        minDist = currDist;
                        minID = merge_lists[j][k];
                        ROS_INFO("minID %d minDist %lf", minID, minDist);
                    }
                }
            }
        }
        if(minDist < 1.5)
        {
            ROS_WARN("Giving the new cluster the previous id");
            erase_merge_lists(minID);
            sensing_on_road::pedestrian_vision oldPed;
            oldPed.cluster = clusters[i];
            oldPed.cluster.centroid = global_point;
            updatelPedInViewWithID(minID, oldPed);
        }
        else
        {
            sensing_on_road::pedestrian_vision newPed;
            newPed.object_label = latest_id++;
            newPed.cluster.centroid = global_point;
            newPed.cluster.last_update = ros::Time::now();
            ROS_INFO_STREAM( "Creating new pedestrian from vision with id #" << latest_id << " at x:" << newPed.cluster.centroid.x << " y:" << newPed.cluster.centroid.y);
            lPedInView.pd_vector.push_back(newPed);
        }
    }

    ///// Add remaining clusters as new pedestrians
    ///// check with caveat .... Or just ignore new clusters
    //for(int ii=0; ii< cluster_vector.clusters.size(); ii++)
    //{
    //If satisfies some criterion  or should we ignore and
    //let the HoG find proper pedestrians.

    //PED_DATA_ASSOC ped;
    //ped.id = assign some id;
    //ped.ped_pose = cluster_vector.clusters[ii].centroid;
    //}
    cleanUp();
    publishPed();
    ROS_DEBUG_STREAM("PedCluster callback end");
}

void data_assoc::erase_merge_lists(int id)
{
    for(size_t j=0; j<merge_lists.size(); )
    {
        for(size_t k=1; k<merge_lists[j].size(); k++)
        {
            if(merge_lists[j][k]==id) merge_lists[j].erase(merge_lists[j].begin()+k);
        }
        if(merge_lists[j].size() == 1) merge_lists.erase(merge_lists.begin()+j);
        else j++;
    }
}
bool data_assoc::getlPedInViewCentroid(int id, geometry_msgs::Point32& centroid)
{
    for(size_t i=0; i < lPedInView.pd_vector.size(); i++)
    {
        if(lPedInView.pd_vector[i].object_label == id)
        {
            centroid = lPedInView.pd_vector[i].cluster.centroid;
        }
    }
}
void data_assoc::resetLPedInViewDecisionflag()
{
    for(size_t i=0; i < lPedInView.pd_vector.size(); i++) lPedInView.pd_vector[i].decision_flag = false;
}
void data_assoc::updatelPedInViewWithID(int id, sensing_on_road::pedestrian_vision& pd_vector)
{
    for(size_t i=0; i < lPedInView.pd_vector.size(); i++)
    {
        if(lPedInView.pd_vector[i].object_label == id)
        {
            updatelPedInView(pd_vector, lPedInView.pd_vector[i]);
        }
    }
}

void data_assoc::updatelPedInView(sensing_on_road::pedestrian_vision& update_source, sensing_on_road::pedestrian_vision& update_dest)
{
    update_dest.cluster.centroid = update_source.cluster.centroid;
    update_dest.decision_flag = true;
    update_dest.cluster.last_update = ros::Time::now();
    update_dest.cluster.width = update_source.cluster.width;
}
void data_assoc::cleanUp()
{
    ROS_DEBUG_STREAM("cleanUp start");
    for(int jj=0; jj< lPedInView.pd_vector.size(); )
    {
        ros::Duration unseen_time = ros::Time::now() - lPedInView.pd_vector[jj].cluster.last_update;
        ROS_DEBUG("ped with ID %d unseen time = %lf", lPedInView.pd_vector[jj].cluster.id, unseen_time.toSec());
        if(unseen_time.toSec()>time_out_)
        {
            ROS_INFO("Erase ped with ID %d due to time out", lPedInView.pd_vector[jj].object_label);
            lPedInView.pd_vector.erase(lPedInView.pd_vector.begin()+jj);

        }
        else
            jj++;

    }
    ROS_DEBUG_STREAM("cleanup end");
}

void data_assoc::publishPed()
{
    dataAssoc_experimental::PedDataAssoc_vector lPed;
    sensor_msgs::PointCloud pc;
    ROS_DEBUG_STREAM("publishPed start");
    pc.header.frame_id = global_frame_;
    pc.header.stamp = ros::Time::now();
    for(int ii=0; ii <lPedInView.pd_vector.size(); ii++)
    {

        geometry_msgs::Point32 p;
        p = lPedInView.pd_vector[ii].cluster.centroid;
        p.z = lPedInView.pd_vector[ii].object_label;
        pc.points.push_back(p);
        ROS_DEBUG("lPenInView.pd_vector confidence = %lf ", lPedInView.pd_vector[ii].confidence);
        camera_project::CvRectangle rect;
        geometry_msgs::PointStamped pt_src, pt_dest;
        pt_src.header = lPedInView.header;
        pt_src.point.x = lPedInView.pd_vector[ii].cluster.centroid.x;
        pt_src.point.y = lPedInView.pd_vector[ii].cluster.centroid.y;
        pt_src.point.z = lPedInView.pd_vector[ii].cluster.centroid.z;
        try {
            rect = projector.project(pt_src, lPedInView.pd_vector[ii].cluster.width, 2);
        } catch( std::out_of_range & e ) {
            ROS_DEBUG("out of range: %s", e.what());
            lPedInView.pd_vector[ii].cvRect_x1 = 0;
            lPedInView.pd_vector[ii].cvRect_y1 = 0;
            lPedInView.pd_vector[ii].cvRect_x2 = 0;
            lPedInView.pd_vector[ii].cvRect_y2 = 0;
            continue;
        } catch( tf::TransformException & e ) {
            ROS_WARN("camera project tf error: %s", e.what());
            return;
        }
        lPedInView.pd_vector[ii].cvRect_x1 = rect.upper_left.x;
        lPedInView.pd_vector[ii].cvRect_y1 = rect.upper_left.y;
        lPedInView.pd_vector[ii].cvRect_x2 = rect.lower_right.x;
        lPedInView.pd_vector[ii].cvRect_y2 = rect.lower_right.y;

    }
    pedPub_.publish(lPedInView);
    visualizer_.publish(pc);
    ROS_DEBUG_STREAM("publishPed end");
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "data_assoc");

    data_assoc *data_assoc_node = new data_assoc(argc, argv);

    ros::spin();
}
