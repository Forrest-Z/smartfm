
#include "data_assoc.h"

using namespace std;




data_assoc::data_assoc(int argc, char** argv) 
{
	ROS_DEBUG("Starting Pedestrian Avoidance ... ");
	
	/// Setting up subsciption 
    ros::NodeHandle nh;
    
    pedClustSub_.subscribe(nh, "ped_laser_cluster", 10);
    pedVisionSub_.subscribe(nh, "ped_vision", 10);
    pedVisionAngularSub_.subscribe(nh, "pedestrian_detect_visual", 10);
    /// TBP : how to add multiple subscription to same call back ????
    
    ros::NodeHandle n("~");
    
    n.param("global_frame", global_frame_, string("odom"));
    n.param("time_out", time_out_, 3.0);
    n.param("poll_increment", poll_inc_, 0.1);
    n.param("poll_decrement", poll_dec_, 0.05);
    n.param("confirm_threshold", threshold_, 0.3);
    /// Setting up publishing
    pedPub_ = nh.advertise<sensing_on_road::pedestrian_vision_batch>("ped_data_assoc",1); /// topic name
    visualizer_ = nh.advertise<sensor_msgs::PointCloud>("ped_data_assoc_visual",1);
	latest_id=0;

	listener_ = new tf::TransformListener(ros::Duration(10));

	/// Setting up callback with transform cache
	laser_tf_filter_ = new tf::MessageFilter<feature_detection::clusters>(pedClustSub_, *listener_, global_frame_, 10);
	laser_tf_filter_->registerCallback(boost::bind(&data_assoc::pedClustCallback, this, _1));

	vision_tf_filter_ = new tf::MessageFilter<sensing_on_road::pedestrian_vision_batch>(pedVisionSub_, *listener_, global_frame_, 10);
	vision_tf_filter_ ->registerCallback(boost::bind(&data_assoc::pedVisionCallback, this, _1));

	vision_angular_tf_filter_ = new tf::MessageFilter<geometry_msgs::PolygonStamped>(pedVisionAngularSub_, *listener_, global_frame_, 10);
	vision_angular_tf_filter_ -> registerCallback(boost::bind(&data_assoc::pedVisionAngularCallback, this, _1));

}

void data_assoc::pedVisionAngularCallback(geometry_msgs::PolygonStampedConstPtr pedestrian_vision_angular)
{


    for(int jj=0; jj< lPedInView.pd_vector.size(); jj++)
     {
         double minAng=10000;
         int minID=-1;
         geometry_msgs::PointStamped global_point, local_point;
         global_point.header = lPedInView.header;
         global_point.point = lPedInView.pd_vector[jj].cluster.centroid;
         local_point.header = lPedInView.header;
         //todo: frame_id at param
         local_point.header.frame_id = "usb_cam";
         transformGlobalToLocal(global_point, local_point);
         double laser_cluster_angular = atan2(local_point.y, local_point.x);
         std::vector<geometry_msgs::Point32> vision_point = pedestrian_vision_angular->polygon.points;
         for(size_t i=0; i<vision_point.size(); i+=2)
         {
             double vision_angluar = atan2(vision_point[i].y, vision_point[i].x);
             double currAng = fabs(vision_angular - laser_cluster_angular);
             if(currAng < minAng)
             {
                 minAng = currAng;
                 minID = i;
             }
         }
        if(minAng < NN_ANG_MATCH_THRESHOLD)
        {
            if(vision_point.size()) vision_point.erase(vision_point.begin()+minID);
        }
         //maybe we can add image features to check similarity


     }
    //todo: match it with the current list of clusters from laser, then create new ID if neccessary
/*
    for(int jj=0; jj< lPedInView.pd_vector.size(); jj++)
         {
             double minAng=10000;
             int minID=-1;
             geometry_msgs::PointStamped global_point, local_point;
             global_point.header = lPedInView.header;
             global_point.point = lPedInView.pd_vector[jj].cluster.centroid;
             local_point.header = lPedInView.header;
             //todo: frame_id at param
             local_point.header.frame_id = "usb_cam";
             transformGlobalToLocal(global_point, local_point);
             double laser_cluster_angular = atan2(local_point.y, local_point.x);
             std::vector<geometry_msgs::Point32> vision_point = pedestrian_vision_angular->polygon.points;
             for(size_t i=0; i<vision_point.size(); i+=2)
             {
                 double vision_angluar = atan2(vision_point[i].y, vision_point[i].x);
                 double currAng = fabs(vision_angular - laser_cluster_angular);
                 if(currAng < minAng)
                 {
                     minAng = currAng;
                     minID = i;
                 }
             }
            if(minAng < NN_ANG_MATCH_THRESHOLD)
            {
                if(vision_point.size()) vision_point.erase(vision_point.begin()+minID);
            }
             //maybe we can add image features to check similarity


         }*/
     for(int ii=0 ; ii<vision_point.size(); ii++)
     {


             sensing_on_road::pedestrian_vision newPed;
             newPed.object_label = latest_id++;
             bool transformed = transformPointToGlobal(pedestrian_vision_vector->header, pd_vector[ii].cluster.centroid, newPed.cluster.centroid);
             if(!transformed) return;
             ROS_DEBUG_STREAM( "Creating new pedestrian from vision with id #" << latest_id << " at x:" << newPed.cluster.centroid.x << " y:" << newPed.cluster.centroid.y);
             lPedInView.pd_vector.push_back(newPed);

     }

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

    listener_->transformPoint(local_point.header, global_point, local_point);
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
void data_assoc::pedVisionCallback(sensing_on_road::pedestrian_vision_batchConstPtr pedestrian_vision_vector)
{
    std::vector<sensing_on_road::pedestrian_vision> pd_vector = pedestrian_vision_vector->pd_vector;
    ROS_DEBUG(" Entering vision call back with lPedInView %d and ped vector size of %d", lPedInView.pd_vector.size(), pd_vector.size());

	//pedestrian_vision_vector.pd_vector[].cluster.centroid;
	/// loop over clusters to match with existing lPedInView
    for(int jj=0; jj< lPedInView.pd_vector.size(); jj++)
    {
        double minDist=10000;
        int minID=-1;
        for(int ii=0; ii < pd_vector.size(); ii++)
        {
            geometry_msgs::Point32 global_point;
            //change the distance metric to angular based

            bool transformed = transformPointToGlobal(pedestrian_vision_vector->header, pd_vector[ii].cluster.centroid, global_point);
            if(!transformed) return;
            double currDist = dist(lPedInView.pd_vector[jj].cluster.centroid, global_point);
            if( (currDist < minDist) && currDist>-1)
            {
                minDist = currDist;
                minID = ii;
            }
        }
        //maybe we can add image features to check similarity
        if(minDist < NN_MATCH_THRESHOLD)
        {


            /// if cluster matched, remove from contention
            if(-1 != minID)
            {
                ROS_DEBUG("From Vision: Cluster %d matched with dist %lf with decision flag %d", lPedInView.pd_vector[jj].object_label, minDist, pd_vector[minID].decision_flag);
                //polling added to filter out some noise
                if(lPedInView.pd_vector[jj].confidence>=threshold_)
                {
                    if(pd_vector[minID].decision_flag)
                    {
                        if(lPedInView.pd_vector[jj].confidence+poll_inc_<=1.0) lPedInView.pd_vector[jj].confidence+=poll_inc_;
                    }
                    else
                    {
                        if(lPedInView.pd_vector[jj].confidence-poll_dec_>=threshold_) lPedInView.pd_vector[jj].confidence-=poll_dec_;
                    }
                }
                else
                {
                    if(pd_vector[minID].decision_flag) lPedInView.pd_vector[jj].confidence+=poll_inc_;
                    else if(lPedInView.pd_vector[jj].confidence-poll_dec_>=0) lPedInView.pd_vector[jj].confidence-=poll_inc_;
                }
                /// remove minID element
                if(pd_vector.size())
                    pd_vector.erase(pd_vector.begin()+minID);
            }
        }

    }

    for(int ii=0 ; ii<pd_vector.size(); ii++)
    {

        if(pd_vector[ii].decision_flag)//(pedestrian_vision_vector.pd_vector[ii].cluster.centroid.x!=0 || pedestrian_vision_vector.pd_vector[ii].cluster.centroid.y!=0)
        {
            sensing_on_road::pedestrian_vision newPed;
            newPed.object_label = latest_id++;
            bool transformed = transformPointToGlobal(pedestrian_vision_vector->header, pd_vector[ii].cluster.centroid, newPed.cluster.centroid);
            if(!transformed) return;
            ROS_DEBUG_STREAM( "Creating new pedestrian from vision with id #" << latest_id << " at x:" << newPed.cluster.centroid.x << " y:" << newPed.cluster.centroid.y);
            lPedInView.pd_vector.push_back(newPed);
        }
    }

    publishPed();
    ROS_DEBUG_STREAM("pedVision callback end");

}

void data_assoc::pedClustCallback(feature_detection::clustersConstPtr cluster_vector)
{
    frame_id_ = cluster_vector->header.frame_id;
    ROS_DEBUG_STREAM( " Entering pedestrian call back with lPedInView " << lPedInView.pd_vector.size() );
	/// loop over clusters to match with existing lPedInView
    std::vector<feature_detection::cluster> clusters = cluster_vector->clusters;
    feature_detection::clusters clusters_visualize;
    clusters_visualize.header = cluster_vector->header;
    lPedInView.header = cluster_vector->header;
    lPedInView.header.frame_id = global_frame_;
	for(int jj=0; jj< lPedInView.pd_vector.size(); jj++)
	{
		double minDist=10000;
		int minID=-1;
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
			}
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

void data_assoc::publishPed()
{
    dataAssoc_experimental::PedDataAssoc_vector lPed;
    sensor_msgs::PointCloud pc;
    ROS_DEBUG_STREAM("publishPed start");
    pc.header.frame_id = global_frame_;
    pc.header.stamp = ros::Time::now();
    for(int ii=0; ii <lPedInView.pd_vector.size(); ii++)
    {
        //if(lPedInView.pd_vector[ii].confidence>=threshold_)
        {
            geometry_msgs::Point32 p;
            p = lPedInView.pd_vector[ii].cluster.centroid;
            p.z = lPedInView.pd_vector[ii].object_label;
            pc.points.push_back(p);

            camera_project::CvRectangle rect;
            geometry_msgs::PointStamped pt_src, pt_dest;
            pt_src.header = lPedInView.header;
            pt_src.point.x = lPedInView.pd_vector[ii].cluster.centroid.x;
            pt_src.point.y = lPedInView.pd_vector[ii].cluster.centroid.y;
            pt_src.point.z = lPedInView.pd_vector[ii].cluster.centroid.z;
            try {
                rect = projector.project(pt_src, lPedInView.pd_vector[ii].cluster.width, 2);
            } catch( std::out_of_range & e ) {
                ROS_WARN("out of range: %s", e.what());
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