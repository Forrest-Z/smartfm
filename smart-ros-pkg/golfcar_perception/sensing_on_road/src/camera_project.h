#ifndef __CAMERA_PROJECT_H__
#define __CAMERA_PROJECT_H__

/*
 * This class is to project object position in 3D world onto 2D image;
 * Reference: "Fast Extrinsic Calibration of a Laser Rangefinder to a Camera"
 * http://www.ri.cmu.edu/publication_view.html?pub_id=5293
 */

#include <cmath>

#include <ros/ros.h>
#include <geometry_msgs/Point32.h>
#include <sensor_msgs/PointCloud.h>
#include <tf/transform_listener.h>
#include <tf/message_filter.h>
#include <message_filters/subscriber.h>

#include <fmutil/fm_math.h>

#include <sensing_on_road/pedestrian_laser.h>
#include <sensing_on_road/pedestrian_laser_batch.h>
#include <sensing_on_road/pedestrian_vision.h>
#include <sensing_on_road/pedestrian_vision_batch.h>

#include <feature_detection/cluster.h>
#include <feature_detection/clusters.h>


/// A structure that holds the camera calibration parameters.
struct CameraCalibrationParameters
{
    double fc[2];   ///< focal_length
    double cc[2];   ///< principal_point
    double alpha_c; ///< skew_coeff
    double kc[5];   ///< distortions

    int width;  ///< frame's width
    int height; ///< frame's height

    /// constructor (sets the values of the parameters).
    CameraCalibrationParameters();
};


/// A structure to represent integer point (pixel)
struct IntPoint {
    int x, y;
};

/// A structure to represent a rectangle in the image (pixel coords)
struct Rectangle {
    IntPoint upper_left; ///< lower left corner
    IntPoint size; ///< width and height
};


class camera_project
{
public:
    camera_project();

protected:
    ros::NodeHandle nh_;

    /// Frame ID of the camera (where to project)
    std::string camera_frame_id_;

    /// We need to know a priori the height of the object we want to detect
    double object_height_;



    /// Parameters of the camera
    CameraCalibrationParameters cam_param_;



    /// Publishes result of projection on topic 'ped_vision_batch' (type
    /// pedestrian_vision_batch). Messages on this topic describe the position
    /// of a set of pedestrians as detected by the laser, in camera coordinates
    /// (pixels).
    ros::Publisher ped_vision_pub_;



    tf::TransformListener tf_;

    /// A helper function to make a rectangle from the centroid position and width
    Rectangle makeRectangle(const geometry_msgs::PointStamped & centroid_in, double width);

    /// A helper function. transform a 3D point in camera coordinates into a
    /// 2D point in pixel coordinates. Takes into account the camera parameters.
    IntPoint projection(const geometry_msgs::Point32 &pt);
};


/// A helper function to fill in the rectangle part in the pedestrian_vision
/// message from the Rectangle information.
void setRectMsg(const Rectangle & rect, sensing_on_road::pedestrian_vision & msg);

#endif
