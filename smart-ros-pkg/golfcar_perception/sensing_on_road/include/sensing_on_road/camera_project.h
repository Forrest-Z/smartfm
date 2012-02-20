#ifndef __CAMERA_PROJECT_H__
#define __CAMERA_PROJECT_H__

/*
 * This class is to project object position in 3D world onto 2D image;
 * Reference: "Fast Extrinsic Calibration of a Laser Rangefinder to a Camera"
 * http://www.ri.cmu.edu/publication_view.html?pub_id=5293
 */

#include <cmath>

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/PointStamped.h>

#include <fmutil/fm_math.h>

namespace camera_project
{

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
    int width;
    int height;
};

struct FloatPoint {
    float x, y;
};

struct CvRectangle {
    FloatPoint upper_left;
    FloatPoint lower_right;
};


/// Convert a Rectangle to a CvRectangle
CvRectangle convertRect(const Rectangle & r);

/// Convert a CvRectangle to a Rectangle
Rectangle convertRect(const CvRectangle & r);


class camera_projector
{
public:
    /// constructor with default parameter values
    camera_projector();

    void setCameraFrameID(const std::string &);

    void setCameraFOV(const double &fov);

    /// Project a rectangle from real world to camera.
    ///
    /// The input rectangle is defined by its centroid position and its size.
    /// The output rectangle is defined by the position of its upper left corner
    /// and its size.
    /// @param centroid: the centroid of the input rectangle (and its frame)
    /// @param width: the width of the input rectangle.
    /// @param height: the height of the input rectangle.
    /// @return the rectangle in pixel coordinates
    CvRectangle project(const geometry_msgs::PointStamped & centroid, double width, double height) const;


protected:
    /// Frame ID of the camera (where to project). Defaults to "usb_cam"
    std::string camera_frame_id_;

    // Field of view of the camera. Defaults to 70 degree
    double fov_;
    /// Parameters of the camera
    CameraCalibrationParameters cam_param_;

    tf::TransformListener tf_;

    /// A helper function. transform a 3D point in camera coordinates into a
    /// 2D point in pixel coordinates. Takes into account the camera parameters.
    IntPoint projection(const geometry_msgs::Point32 &pt) const;
};


} //namespace camera_project

#endif
