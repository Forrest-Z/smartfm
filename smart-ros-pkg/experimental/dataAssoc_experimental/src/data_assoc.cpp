
#include "data_assoc.h"

using namespace std;




data_assoc::data_assoc(int argc, char** argv) : merge_lists(nh_), it_(nh_)
{
    ROS_DEBUG("Starting Pedestrian Avoidance ... ");

    /// Setting up subsciption
    image_sub_.subscribe(it_, "/camera_front/image_raw", 20);

    pedClustSub_.subscribe(nh_, "pedestrian_clusters", 10);
    pedVisionAngularSub_.subscribe(nh_, "pedestrian_roi", 10);
    /// TBP : how to add multiple subscription to same call back ????

    ros::NodeHandle n("~");

    n.param("global_frame", global_frame_, string("map"));
    n.param("camera_frame", camera_frame_, string("camera_front_base"));
    n.param("time_out", time_out_, 3.0);
    n.param("poll_increment", poll_inc_, 0.1);
    n.param("poll_decrement", poll_dec_, 0.05);

    /// Setting up publishing
    filtered_cluster_pub_ = nh_.advertise<sensor_msgs::PointCloud>("filtered_clusters",1);
    pedPub_ = nh_.advertise<sensing_on_road::pedestrian_vision_batch>("ped_data_assoc",1); /// topic name
    visualizer_ = nh_.advertise<sensor_msgs::PointCloud>("ped_data_assoc_visual",1);
    visualize_good_object_ = nh_.advertise<sensor_msgs::PointCloud>("confident_objects",1);

    latest_id=0;

    listener_ = new tf::TransformListener(ros::Duration(10));

    /// Setting up callback with transform cache
    //laser_tf_filter_ = new tf::MessageFilter<feature_detection::clusters>(pedClustSub_, *listener_, global_frame_, 10);
    //laser_tf_filter_->registerCallback(boost::bind(&data_assoc::pedClustCallback, this, _1));

    vision_angular_tf_filter_ = new tf::MessageFilter<sensor_msgs::PointCloud>(pedVisionAngularSub_, *listener_, "camera_front_base", 10);
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
    color_cost_ = config.color_cost;
    dist_cost_ = config.dist_cost;
    cost_threshold_ = config.cost_threhold;
    merge_dist_ = config.merge_dist;
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
    double distance = numeric_limits<double>::max();
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
unsigned char data_assoc::color_downsample(unsigned char color)
{

    int data_i = color;

    data_i += int(data_i / 255.0 * 7)*36;
    if(data_i>255) data_i = 255;
    unsigned char data_c = data_i;
    return data_c;
}

void data_assoc::updatelPedInViewWithNewCluster(feature_detection::clusters& cluster_vector, cv::Mat& img)
{
    cout<<"Current ped list: ";
    for(size_t i = 0; i < lPedInView.pd_vector.size(); i++)
    {
        cout<<lPedInView.pd_vector[i].object_label<<" "<<lPedInView.pd_vector[i].cluster.centroid.x<<" "<<lPedInView.pd_vector[i].cluster.centroid.y<<" ";
    }
    cout<<endl;
    for(size_t i = 0; i < cluster_vector.clusters.size(); )
    {

        // Get the cluster's centroid in global position
        geometry_msgs::Point32 global_point;
        bool transformed = transformPointToGlobal(cluster_vector.header, cluster_vector.clusters[i].centroid, global_point);
        if(!transformed) return;

        // Get the cluster's image hash
        sensing_on_road::pedestrian_vision cluster_vision;
        cluster_vision.cluster = cluster_vector.clusters[i];
        std_msgs::Header cluster_header = cluster_vector.header;
        imageProjection(img, cluster_header, cluster_vision, true);
        //cout<<i<<" cluster, size of image hashing = "<<cluster_vision.image_hash.size()<<endl;

        double minCost = numeric_limits<double>::max();
        int minID = -1;
        double img_score = 0, dist_score = 0;
        //cout << "img diff ";
        for( size_t j = 0; j < lPedInView.pd_vector.size(); j++)
        {
            double imgdiff, currDist;
            getColorDiff(cluster_vision.image_hash, lPedInView.pd_vector[j].image_hash, imgdiff);
            currDist = dist(lPedInView.pd_vector[j].cluster.centroid, global_point);

            // Get the total cost with a simple linear function
            double currCost = color_cost_ * imgdiff + dist_cost_ * currDist;
            //cout <<"ID: "<<lPedInView.pd_vector[j].object_label<<" "<< imgdiff << " "<<currDist<<" ";
            if( (currCost < minCost))
            {
                minCost = currCost;
                minID = j;
                img_score = color_cost_ * imgdiff;
                dist_score = dist_cost_ * currDist;
            }
            //cout << imgdiff << " ";
        }
        //cout<<endl;
        if(minID>-1)
        {
            cout<<"New cluster matched ped ID "<<lPedInView.pd_vector[minID].cluster.id <<endl;
            cout<<"Score details: "<<img_score <<" + "<<dist_score<<" = "<<minCost<<endl;
            cout<<"At location xy "<<global_point.x<<" "<<global_point.y<<endl;
        }
        if(minCost < cost_threshold_ && minID > -1)
        {
            // Found the matching pedestrian, update the lPedInView accordingly
            lPedInView.pd_vector[minID].cluster = cluster_vector.clusters[i];
            lPedInView.pd_vector[minID].cluster.id = lPedInView.pd_vector[minID].object_label;
            lPedInView.pd_vector[minID].cluster.centroid = global_point;
            lPedInView.pd_vector[minID].cluster.last_update = ros::Time::now();
            lPedInView.pd_vector[minID].decision_flag = true;
            cluster_vector.clusters.erase(cluster_vector.clusters.begin()+i);
        }
        else
        {
            // No match found, just increment the count
            i++;
        }


    }

}

void data_assoc::checkMergedlPedInView(Mat& img)
{
    //check for possible merged clusters
    for(size_t i=0; i<lPedInView.pd_vector.size();i++)
    {
        if(lPedInView.pd_vector[i].decision_flag) continue;
        double minDist=10000;
        int minID=-1;
        for(int j=0; j < lPedInView.pd_vector.size(); j++)
        {
            if(!lPedInView.pd_vector[j].decision_flag) continue;
            //get the measurement of the 2 points and use it to determine if we want to merge them together
            double currDist = dist(lPedInView.pd_vector[j].cluster.centroid, lPedInView.pd_vector[i].cluster.centroid);
            if( (currDist < minDist) && currDist>-1)
            {
                minDist = currDist;
                minID = j;
            }
        }
        bool update_image_hash = true;

        if(minDist < merge_dist_)
        {
            cout<<"Ped in view with id "<<lPedInView.pd_vector[minID].object_label<<" has min dist of "<<minDist <<" with id "<<lPedInView.pd_vector[i].object_label;
            cout<<" merging those 2 id"<<endl;
            PedImgHash src, dest;
            src.id = lPedInView.pd_vector[minID].object_label;
            dest.id = lPedInView.pd_vector[i].object_label;

            merge_lists.create_merge_lists(src, dest);
            update_image_hash = false;
        }

    }
}

void data_assoc::updateImageHash(Mat& img)
{
    //only update those lPedInView that is not in the merge list
    //this is to retain the color histogram before the merging occur for better discriminative features
    for(size_t i=0; i<lPedInView.pd_vector.size();i++)
    {
        bool merge_existed=false;
        for(size_t j = 0; j<merge_lists.merged_ids.size(); j++)
            for(size_t k = 0; k<merge_lists.merged_ids[j].peds.size();k++)
                if(lPedInView.pd_vector[i].object_label == merge_lists.merged_ids[j].peds[k].id) merge_existed = true;
        //update image hash here
        imageProjection(img,lPedInView.header, lPedInView.pd_vector[i], !merge_existed);
        if(merge_existed) cout<<"Ped id "<<lPedInView.pd_vector[i].object_label<<" is a merged ped, not going to update image hash"<<endl;
    }
}

void data_assoc::updateMergeList()//feature_detection::clusters cluster_vector)
{
    //check for split list
    for(size_t i=0; i<merge_lists.merged_ids.size();i++)
    {
        vector<int> id_updated;
        for(size_t j = 0; j<merge_lists.merged_ids[i].peds.size(); j++)
        {
            for(size_t k = 0; k<lPedInView.pd_vector.size();k++)
            {
                if(!lPedInView.pd_vector[k].decision_flag) continue;
                if(lPedInView.pd_vector[k].object_label == merge_lists.merged_ids[i].peds[j].id)
                    id_updated.push_back(lPedInView.pd_vector[k].object_label);
            }
        }

        if(id_updated.size()>1)
        {
            for(size_t j = 0; j<id_updated.size(); j++)
            {
                merge_lists.erase_merge_lists(id_updated[j]);
                cout<<"Merged ped id "<<id_updated[j]<<" split, erased from merge list"<<endl;
            }
        }
    }

    //update merge list
    for(size_t i=0; i<merge_lists.merged_ids.size();i++)
    {
        for(size_t j=0; j<lPedInView.pd_vector.size(); j++)
        {
            //only update those lPedInView that's updated on this cycle
            if(!lPedInView.pd_vector[j].decision_flag) continue;

            bool match_merge = false;
            for(size_t k=0;k<merge_lists.merged_ids[i].peds.size();k++)
            {
                if(lPedInView.pd_vector[j].object_label == merge_lists.merged_ids[i].peds[k].id)
                {
                    match_merge = true;
                    break;
                }
            }
            if(match_merge)
            {
                for(size_t k=0;k<merge_lists.merged_ids[i].peds.size();k++)
                {
                    //the merge list should have at least 2 elements
                    assert(merge_lists.merged_ids[i].peds.size()>1);

                    //update the merged id. Note that the decision_flag will not be altered
                    updatelPedInViewWithID(merge_lists.merged_ids[i].peds[k].id, lPedInView.pd_vector[j]);
                    cout<<"Update merged list with size "<<merge_lists.merged_ids[i].peds.size()<<" of ped id "<<merge_lists.merged_ids[i].peds[k].id<<" triggered by lPedInView id"<<lPedInView.pd_vector[j].object_label<<endl;

                }
                break;
            }
        }
    }
}

void data_assoc::pedClustCallback(sensor_msgs::ImageConstPtr image, feature_detection::clustersConstPtr cluster_vector_ptr)
{
    ROS_DEBUG_STREAM( " Entering pedestrian call back with lPedInView " << lPedInView.pd_vector.size() << " With frame id "<< frame_id_ );

    cv_bridge::CvImagePtr cv_image;
    try{cv_image = cv_bridge::toCvCopy(image, "bgra8");}
    catch (cv_bridge::Exception& e){ROS_ERROR("cv_bridge exception: %s", e.what());return;}
    Mat img(cv_image->image);
    feature_detection::clusters cluster_vector = *cluster_vector_ptr;

    sensor_msgs::PointCloud filtered_clusters;
    filtered_clusters.header = cluster_vector.header;
    for(size_t i=0; i<cluster_vector.clusters.size(); i++)
	{
		for(size_t j=0; j<cluster_vector.clusters[i].points.size(); j++)
		{
			filtered_clusters.points.push_back(cluster_vector.clusters[i].points[j]);
		}
	}
    filtered_cluster_pub_.publish(filtered_clusters);

    /// loop over clusters to match with existing lPedInView
    //std::vector<feature_detection::cluster> clusters = cluster_vector.clusters;

    lPedInView.header = cluster_vector.header;
    lPedInView.header.frame_id = global_frame_;
    resetLPedInViewDecisionflag();
    cout<<"****Update lPedInView****"<<endl;
    //update the new cluster with the existing lPedInView
    updatelPedInViewWithNewCluster(cluster_vector, img);

    //then update the merge list, check if any pair in the merge list has already been updated (split cluster)
    //if just update the position of the clusters according to the merge list
    cout<<"****Update Merge List****"<<endl;
    updateMergeList();

    //check any remaining lPedInView for possible merged cluster
    cout<<"****Check for merged clusters****"<<endl;
    checkMergedlPedInView(img);

    // Add any remaining clusters as new lPedInView
    cout<<"****Add new lPedInView****"<<endl;
    for(size_t i=0; i<cluster_vector.clusters.size(); i++)
    {
        geometry_msgs::Point32 global_point;
        bool transformed = transformPointToGlobal(cluster_vector.header, cluster_vector.clusters[i].centroid, global_point);
        if(!transformed) continue;
        sensing_on_road::pedestrian_vision newPed;
        newPed.object_label = latest_id++;
        newPed.cluster.centroid = global_point;
        newPed.cluster.last_update = ros::Time::now();
        cout<< "Creating new pedestrian with id #" << latest_id << " at x:" << newPed.cluster.centroid.x << " y:" << newPed.cluster.centroid.y<<endl;
        //imageProjection(img, lPedInView.header, newPed, true);
        lPedInView.pd_vector.push_back(newPed);
    }
    updateImageHash(img);

    cleanUp();
    publishPed(img);
    ROS_DEBUG_STREAM("PedCluster callback end");
}

void data_assoc::getColorDiff(vector<double>& first, vector<double>& second, double& diff)
{

    diff = 0.0;

    // In this model, the cost will be effective when the pedestrian come in view
    // Since we are taking the average difference, it can be thought as percentage color matched
    if(first.size() == second.size() && first.size()>0)
    {
        for(size_t i=0; i<first.size(); i++)
        {
            diff += (first[i]-second[i])*(first[i]-second[i]);
        }

        diff = sqrt(diff/(int)first.size());
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
    return true;
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
    //update_dest.decision_flag = true;
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
            printf("Erase ped with ID %d due to time out", lPedInView.pd_vector[jj].object_label);
            merge_lists.erase_merge_lists(lPedInView.pd_vector[jj].object_label);
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
    pc.header.frame_id = global_frame_;
    pc.header.stamp = ros::Time::now();
    sensor_msgs::PointCloud ppc;
    ppc.header.frame_id = global_frame_;
    ppc.header.stamp = ros::Time::now();
    ROS_DEBUG_STREAM("publishPed start");
    for(int ii=0; ii <lPedInView.pd_vector.size(); ii++)
    {
        geometry_msgs::Point32 p;
        p = lPedInView.pd_vector[ii].cluster.centroid;
        //p.z = lPedInView.pd_vector[ii].object_label;
        p.z = 0.0;
        ROS_DEBUG("lPenInView.pd_vector confidence = %lf ", lPedInView.pd_vector[ii].confidence);
        pc.points.push_back(p);
        if(lPedInView.pd_vector[ii].confidence > 0.1) ppc.points.push_back(p);
        if(!imageProjection(img, lPedInView.header, lPedInView.pd_vector[ii], false)) continue;
    }
    pedPub_.publish(lPedInView);
    visualizer_.publish(pc);
    visualize_good_object_.publish(ppc);

    ROS_DEBUG_STREAM("publishPed end");
}

bool data_assoc::imageProjection(Mat& img, std_msgs::Header& source_header, sensing_on_road::pedestrian_vision& ped, bool generate_image_hash)
{
    //Do all the projection for all the elements in lPenInView
    //Perform image hash if necessary

    camera_project::CvRectangle rect;
    geometry_msgs::PointStamped pt_src;
    pt_src.header = source_header;
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


    if(generate_image_hash)
    {
        vector<double> img_hash;
        Mat ped_img = Mat(img, Rect(UL,BR));
        //imshow("before sending to colorHist", ped_img);
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
}
