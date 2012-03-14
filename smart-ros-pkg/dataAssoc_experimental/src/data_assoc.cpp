
#include "data_assoc.h"

using namespace std;




data_assoc::data_assoc(int argc, char** argv) : merge_lists(nh_), it_(nh_)
{
    ROS_DEBUG("Starting Pedestrian Avoidance ... ");

    /// Setting up subsciption
    image_sub_.subscribe(it_, "/usb_cam/image_raw", 20);

    pedClustSub_.subscribe(nh_, "pedestrian_clusters", 10);
    pedVisionAngularSub_.subscribe(nh_, "pedestrian_roi", 10);
    /// TBP : how to add multiple subscription to same call back ????

    ros::NodeHandle n("~");

    n.param("global_frame", global_frame_, string("map"));
    n.param("camera_frame", camera_frame_, string("usb_cam"));
    n.param("time_out", time_out_, 3.0);
    n.param("poll_increment", poll_inc_, 0.1);
    n.param("poll_decrement", poll_dec_, 0.05);

    /// Setting up publishing
    pedPub_ = nh_.advertise<sensing_on_road::pedestrian_vision_batch>("ped_data_assoc",1); /// topic name
    visualizer_ = nh_.advertise<sensor_msgs::PointCloud>("ped_data_assoc_visual",1);
    latest_id=0;

    listener_ = new tf::TransformListener(ros::Duration(10));

    /// Setting up callback with transform cache
    //laser_tf_filter_ = new tf::MessageFilter<feature_detection::clusters>(pedClustSub_, *listener_, global_frame_, 10);
    //laser_tf_filter_->registerCallback(boost::bind(&data_assoc::pedClustCallback, this, _1));

    vision_angular_tf_filter_ = new tf::MessageFilter<sensor_msgs::PointCloud>(pedVisionAngularSub_, *listener_, "usb_cam", 10);
    vision_angular_tf_filter_ -> registerCallback(boost::bind(&data_assoc::pedVisionAngularCallback, this, _1));

    dynamic_cb = boost::bind(&data_assoc::dynamic_callback, this, _1, _2);
    dynamic_server.setCallback(dynamic_cb);

    typedef sync_policies::ApproximateTime<sensor_msgs::Image, feature_detection::clusters> MySyncPolicy;
    Synchronizer<MySyncPolicy> sync(MySyncPolicy(20), image_sub_, pedClustSub_);
    sync.registerCallback(boost::bind(&data_assoc::pedClustCallback,this, _1, _2));

    ros::spin();
}

void data_assoc::dynamic_callback(dataAssoc_experimental::CameraParamConfig &config, uint32_t level)
{
    laser_height_ = config.laser_height;
    pixel_padding_ = config.pixel_padding;
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

void data_assoc::getLocalLPedInView(vector<local_lPedInView>& lPedInView_local)
{
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
}

void data_assoc::pedVisionAngularCallback(sensor_msgs::PointCloudConstPtr pedestrian_vision_angular)
{
    ROS_DEBUG(" Entering vision call back with lPedInView %d", lPedInView.pd_vector.size());
    std::vector<geometry_msgs::Point32> vision_point = pedestrian_vision_angular->points;
    ROS_DEBUG("Getting new id if any. Vision roi points received=%d", vision_point.size());

    //get a copy lPedInView in local frame and sort it
    vector<local_lPedInView> lPedInView_local;
    getLocalLPedInView(lPedInView_local);

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
    //publishPed();
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

void data_assoc::pedClustCallback(sensor_msgs::ImageConstPtr image, feature_detection::clustersConstPtr cluster_vector)
{

    cv_bridge::CvImagePtr cv_image;
    try
    {
        cv_image = cv_bridge::toCvCopy(image, "bgra8");
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    Mat img(cv_image->image);
    //imshow("data_assoc",img);
    //waitKey(2);
    frame_id_ = cluster_vector->header.frame_id;
    ROS_DEBUG_STREAM( " Entering pedestrian call back with lPedInView " << lPedInView.pd_vector.size() << " With frame id "<< frame_id_ );
    /// loop over clusters to match with existing lPedInView
    std::vector<feature_detection::cluster> clusters = cluster_vector->clusters;

    feature_detection::clusters clusters_visualize;
    clusters_visualize.header = cluster_vector->header;
    lPedInView.header = cluster_vector->header;
    lPedInView.header.frame_id = global_frame_;
    resetLPedInViewDecisionflag();
    cout<<endl;
    for(int jj=0; jj< lPedInView.pd_vector.size(); jj++)
    {
        cout<<"Width: "<<lPedInView.pd_vector[jj].object_label<<" "<< lPedInView.pd_vector[jj].cluster.width<<endl;
        cout<<"ProjE: "<<lPedInView.pd_vector[jj].object_label<<" "<< lPedInView.pd_vector[jj].cluster.projected_l1<<endl;
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
                lPedInView.pd_vector[jj].decision_flag = true;

                //update the merge list
                bool merge_exist = false;
                for(size_t j=0; j<merge_lists.merged_ids.size();j++)
                {
                    //the first element is the active element
                    if(lPedInView.pd_vector[jj].cluster.id == merge_lists.merged_ids[j].peds[0].id)
                    {
                        //the merge list should have at least 2 elements
                        assert(merge_lists.merged_ids[j].peds.size()>1);
                        for(size_t k=1; k<merge_lists.merged_ids[j].peds.size();k++) updatelPedInViewWithID(merge_lists.merged_ids[j].peds[k].id, lPedInView.pd_vector[jj]);
                    }
                }
                //if not under the merge list update the image_hash

                /// remove minID element
                if(clusters.size()) clusters.erase(clusters.begin()+minID);
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
        bool update_image_hash = true;
        if(minDist < NN_MATCH_THRESHOLD)
        {
            PedImgHash src, dest;
            src.id = lPedInView.pd_vector[minID].object_label;
            dest.id = lPedInView.pd_vector[i].object_label;
            merge_lists.create_merge_lists(src, dest);
            update_image_hash = false;
        }
        if(!imageProjection(img, lPedInView.pd_vector[i], update_image_hash)) continue;
    }
    ROS_DEBUG("Remaining clusters %d", clusters.size());
    for(size_t i=0; i<clusters.size(); i++)
    {
        geometry_msgs::Point32 global_point;
        bool transformed = transformPointToGlobal(cluster_vector->header, clusters[i].centroid,global_point);
        if(!transformed) continue;

        //check for possible split cluster

        double minDist=10000;
        int minID=-1;
        ROS_DEBUG("Check for possible split cluster from merge list");
        for(size_t j=0; j<merge_lists.merged_ids.size(); j++)
        {
            for(size_t k=1; k<merge_lists.merged_ids[j].peds.size(); k++)
            {
                geometry_msgs::Point32 merge_centroid;
                if(getlPedInViewCentroid(merge_lists.merged_ids[j].peds[k].id, merge_centroid))
                {
                    double currDist = dist(merge_centroid, global_point);
                    if( (currDist < minDist) && currDist>-1)
                    {
                        minDist = currDist;
                        minID = merge_lists.merged_ids[j].peds[k].id;
                        ROS_DEBUG("minID %d minDist %lf", minID, minDist);
                    }
                }
            }
        }
        if(minDist < 1.5)
        {
            ROS_WARN("Giving the new cluster the previous id");
            merge_lists.erase_merge_lists(minID);
            //erase_merge_lists(minID);
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
            ROS_DEBUG_STREAM( "Creating new pedestrian from vision with id #" << latest_id << " at x:" << newPed.cluster.centroid.x << " y:" << newPed.cluster.centroid.y);
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
    publishPed(img);
    ROS_DEBUG_STREAM("PedCluster callback end");
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
            ROS_DEBUG("Erase ped with ID %d due to time out", lPedInView.pd_vector[jj].object_label);
            lPedInView.pd_vector.erase(lPedInView.pd_vector.begin()+jj);

        }
        else
            jj++;

    }
    ROS_DEBUG_STREAM("cleanup end");
}

void data_assoc::publishPed(Mat img)
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
        if(!imageProjection(img, lPedInView.pd_vector[ii], false)) continue;

    }

    pedPub_.publish(lPedInView);
    visualizer_.publish(pc);
    ROS_DEBUG_STREAM("publishPed end");
}

bool data_assoc::imageProjection(Mat& img, sensing_on_road::pedestrian_vision& ped, bool generate_image_hash)
{
    //Do all the projection for all the elements in lPenInView
    //Perform image hash if necessary

    camera_project::CvRectangle rect;
    geometry_msgs::PointStamped pt_src;
    pt_src.header = lPedInView.header;
    pt_src.point.x = ped.cluster.centroid.x;
    pt_src.point.y = ped.cluster.centroid.y;
    pt_src.point.z = ped.cluster.centroid.z;
    projector.setPixelAllowance(pixel_padding_);
    projector.setLaserMountingHeight(laser_height_);
    try {
        rect = projector.project(pt_src, ped.cluster.width, 2);
    } catch( std::out_of_range & e ) {
        ROS_DEBUG("out of range: %s", e.what());
        ped.cvRect_x1 = 0;
        ped.cvRect_y1 = 0;
        ped.cvRect_x2 = 0;
        ped.cvRect_y2 = 0;
        return true;
    } catch( tf::TransformException & e ) {
        ROS_WARN("camera project tf error: %s", e.what());
        return false;
    }
    ped.cvRect_x1 = rect.upper_left.x;
    ped.cvRect_y1 = rect.upper_left.y;
    ped.cvRect_x2 = rect.lower_right.x;
    ped.cvRect_y2 = rect.lower_right.y;
    Point UL = Point(rect.upper_left.x, rect.upper_left.y);
    Point BR = Point(rect.lower_right.x, rect.lower_right.y);

    vector<double> img_hash;
    Mat ped_img = Mat(img, Rect(UL,BR));
    if(generate_image_hash)
    {
        //cout<<ped.object_label<<": ";
        colorHist(ped_img, img_hash);
        ped.image_hash = img_hash;
    }
    return true;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "data_assoc");

    data_assoc *data_assoc_node = new data_assoc(argc, argv);

    ;
}
