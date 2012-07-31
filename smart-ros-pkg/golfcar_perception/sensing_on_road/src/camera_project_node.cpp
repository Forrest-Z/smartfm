#include "camera_project_node.h"


camera_project_node::camera_project_node()
{
    nh_.param("camera_frame_id", camera_frame_id_, std::string("camera_front_base"));
    nh_.param("object_height", object_height_, 2.0);
    ped_vision_pub_ = nh_.advertise<sensing_on_road::pedestrian_vision_batch>("camera_project_out", 2);
}

void setRectMsg(const camera_project::CvRectangle & rect, sensing_on_road::pedestrian_vision * msg)
{
    msg->x          = rect.upper_left.x;
    msg->y          = rect.upper_left.y;
    msg->width      = rect.lower_right.x - rect.upper_left.x;
    msg->height     = rect.lower_right.y - rect.upper_left.y;

    msg->cvRect_x1  = rect.upper_left.x;
    msg->cvRect_y1  = rect.upper_left.y;
    msg->cvRect_x2  = rect.lower_right.x;
    msg->cvRect_y2  = rect.lower_right.y;
}
