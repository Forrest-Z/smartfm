#ifndef __CAMERA_PROJECT_NODE_H__
#define __CAMERA_PROJECT_NODE_H__

#include "camera_project.h"
#include <sensing_on_road/pedestrian_vision_batch.h>


class camera_project_node : public camera_project::camera_projector
{
public:
    camera_project_node();

protected:
    ros::NodeHandle nh_;

    /// Frame ID of the camera (where to project)
    std::string camera_frame_id_;

    /// We need to know a priori the height of the object we want to detect
    double object_height_;


    /// Publishes result of projection on topic 'ped_vision_batch' (type
    /// pedestrian_vision_batch). Messages on this topic describe the position
    /// of a set of pedestrians as detected by the laser, in camera coordinates
    /// (pixels).
    ros::Publisher ped_vision_pub_;
};


/// A helper function to fill in the rectangle part in the pedestrian_vision
/// message from the Rectangle information.
void setRectMsg(const camera_project::CvRectangle & rect, sensing_on_road::pedestrian_vision * msg);

#endif
