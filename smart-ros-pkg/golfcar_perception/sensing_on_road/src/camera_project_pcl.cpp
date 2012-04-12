#include "camera_project_node.h"

#include <tf/message_filter.h>
#include <message_filters/subscriber.h>

#include <feature_detection/clusters.h>


class camera_project_pcl : protected camera_project_node
{
public:
    camera_project_pcl();

private:
    /// Subscribes to the 'camera_project_pcl_in' topic (type feature_detection/clusters).
    /// Messages on this topic describe the position of a set of pedestrians as
    /// detected by the source sensor (laser, kinect, stereo camera, etc.).
    /// Each pedestrian is represented as a PointCloud.
    /// They will be projected to camera coordinates (pixels) and published by
    /// ped_vision_pub_.
    message_filters::Subscriber<feature_detection::clusters> ped_pcl_sub_;

    /// We use a tf::MessageFilter, connected to ped_pcl_sub_, so that messages
    /// are cached until a transform is available.
    tf::MessageFilter<feature_detection::clusters> * ped_pcl_sub_tf_filter_;

    /// Callback function for ped_pcl_sub_tf_filter_
    void pcl_in_CB(const boost::shared_ptr<const feature_detection::clusters> &);
};


camera_project_pcl::camera_project_pcl()
{
    ped_pcl_sub_.subscribe(nh_, "camera_project_in", 10);
    ped_pcl_sub_tf_filter_ = new tf::MessageFilter<feature_detection::clusters>(ped_pcl_sub_, tf_, camera_frame_id_, 10);
    ped_pcl_sub_tf_filter_->registerCallback(boost::bind(&camera_project_pcl::pcl_in_CB, this, _1));
    ped_pcl_sub_tf_filter_->setTolerance(ros::Duration(0.05));
}


void camera_project_pcl::pcl_in_CB(const boost::shared_ptr<const feature_detection::clusters> & clusters_ptr)
{
    // For each cluster detected by the sensor, create a corresponding ROI
    // (a rectangle) in the camera image

    sensing_on_road::pedestrian_vision_batch ped_vision_batch_msg;
    ped_vision_batch_msg.header = clusters_ptr->header;

    geometry_msgs::PointStamped pt_src, pt_dest;
    pt_src.header = clusters_ptr->header;

    // For each
    for( unsigned i=0; i<clusters_ptr->clusters.size(); i++ )
    {
        // Convert to PointStamped
        pt_src.point.x = clusters_ptr->clusters[i].centroid.x;
        pt_src.point.y = clusters_ptr->clusters[i].centroid.y;
        pt_src.point.z = clusters_ptr->clusters[i].centroid.z;

        camera_project::CvRectangle rect;
        try {
            rect = project(pt_src, clusters_ptr->clusters[i].width, object_height_);
        } catch( std::out_of_range & e ) {
            ROS_WARN("out of range: %s", e.what());
            continue;
        } catch( tf::TransformException & e ) {
            ROS_WARN("camera project tf error: %s", e.what());
            return;
        }

        sensing_on_road::pedestrian_vision msg;
        msg.cluster = clusters_ptr->clusters[i];
        msg.object_label = clusters_ptr->clusters[i].id;
        msg.decision_flag = false;
        msg.complete_flag = true;
        msg.disz = clusters_ptr->clusters[i].centroid.x;
        setRectMsg(rect, &msg);
        ped_vision_batch_msg.pd_vector.push_back(msg);
    }

   // if( ! ped_vision_batch_msg.pd_vector.empty() )
        ped_vision_pub_.publish(ped_vision_batch_msg);
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "camera_project_pcl");
    camera_project_pcl * camera_project_pcl_node = new camera_project_pcl();
    ros::spin();
    delete camera_project_pcl_node;
}
