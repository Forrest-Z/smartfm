#include "camera_project_node.h"

#include <sensing_on_road/pedestrian_laser_batch.h>


class camera_project_baoxing : protected camera_project_node
{
public:
    camera_project_baoxing();

private:
    /// Subscribes to the 'ped_laser_batch' topic (type pedestrian_laser_batch).
    /// Messages on this topic describe the position of a set of pedestrians as
    /// detected by the laser.
    /// They will be projected to camera coordinates (pixels) and published by
    /// ped_vision_pub_.
    ros::Subscriber ped_laser_batch_sub_;

    /// Callback function for ped_laser_batch_sub_
    void ped_laser_batch_CB(const sensing_on_road::pedestrian_laser_batch &);
};


camera_project_baoxing::camera_project_baoxing()
{
    ped_laser_batch_sub_ = nh_.subscribe("ped_laser_batch", 2,
        &camera_project_baoxing::ped_laser_batch_CB, this);
}


void camera_project_baoxing::ped_laser_batch_CB(const sensing_on_road::pedestrian_laser_batch &pd_laser_para)
{
    // For each pedestrian detected by the laser, create a corresponding ROI
    // (a rectangle) in the camera image.

    sensing_on_road::pedestrian_vision_batch ped_vision_batch_msg;
    ped_vision_batch_msg.header = pd_laser_para.header;

    // For each pedestrian detected by the laser
    for(unsigned i=0; i<pd_laser_para.pedestrian_laser_features.size(); i++)
    {
        // a shortcut
        const sensing_on_road::pedestrian_laser & ped_laser =
                                    pd_laser_para.pedestrian_laser_features[i];

        // we are only interested in pedestrians that are not too close and not too far
        if( ! fmutil::isWithin(ped_laser.pedestrian_laser.point.x, 0.5, 20) )
            continue;

        camera_project::CvRectangle rect;
        try {
            rect = project(ped_laser.pedestrian_laser, ped_laser.size, object_height_);
        } catch( std::out_of_range & e ) {
            ROS_WARN("out of range: %s", e.what());
            continue;
        } catch( tf::TransformException & e ) {
            ROS_WARN("camera project tf error: %s", e.what());
            return;
        }

        sensing_on_road::pedestrian_vision msg;
        msg.cluster.centroid.x = ped_laser.pedestrian_laser.point.x;
        msg.cluster.centroid.y = ped_laser.pedestrian_laser.point.y;
        msg.cluster.centroid.z = ped_laser.pedestrian_laser.point.z;
        msg.confidence = ped_laser.confidence;
        msg.decision_flag = false;
        msg.complete_flag = true;
        msg.object_label = ped_laser.object_label;
        msg.disz = ped_laser.pedestrian_laser.point.x;
        setRectMsg(rect, &msg);
        ped_vision_batch_msg.pd_vector.push_back(msg);
    }

    if( ! ped_vision_batch_msg.pd_vector.empty() )
        ped_vision_pub_.publish(ped_vision_batch_msg);
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "camera_project_baoxingor");
    camera_project_baoxing * camera_project_baoxing_node = new camera_project_baoxing();
    ros::spin();
    delete camera_project_baoxing_node;
}
