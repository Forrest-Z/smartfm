#include "camera_project.h"


const double PIXEL_ALLOWANCE = 30;
const double LASER_MOUNTING_HEIGHT = 1.0;


CameraCalibrationParameters::CameraCalibrationParameters()
{
    fc[0]   = 471.21783;
    fc[1]   = 476.11993;

    cc[0]   = 322.35359;
    cc[1]   = 183.44632;

    kc[0]   = 0.00083;
    kc[1]   = -0.03521;
    kc[2]   = 0.00135;
    kc[3]   = 0.00048;
    kc[4]   = 0;

    alpha_c = 0;

    width  = 640;
    height = 360;
}


camera_project::camera_project()
{
    nh_.param("camera_frame_id", camera_frame_id_, std::string("usb_cam"));
    nh_.param("object_height", object_height_, 2.0);
    ped_vision_pub_ = nh_.advertise<sensing_on_road::pedestrian_vision_batch>("camera_project_out", 2);
}


// A helper function to make a rectangle from the centroid position and width
Rectangle camera_project::makeRectangle (
    const geometry_msgs::PointStamped & centroid_in,
    double width)
{
    //transform from source frame to camera frame
    geometry_msgs::PointStamped ped_camera;
    tf_.transformPoint(camera_frame_id_, centroid_in, ped_camera);

    //tranform from ROS-convention to Vision-convention;
    geometry_msgs::Point32 tmp;
    tmp.x = -ped_camera.point.y;
    tmp.y =  ped_camera.point.z;
    tmp.z =  ped_camera.point.x;

    // project to frame (pixel coordinates)
    IntPoint centroid = projection(tmp);

    //the central point should fall inside of the picture.
    if( ! fmutil::isWithin(centroid.x, 0, cam_param_.width) ||
        ! fmutil::isWithin(centroid.y, 0, cam_param_.height) )
        throw std::out_of_range("centroid does not fall inside the picture");

    float dx_coef  = cam_param_.fc[0]/tmp.z;
    float dy_coef  = cam_param_.fc[1]/tmp.z;

    int lower_x = centroid.x - dx_coef*width/2 - PIXEL_ALLOWANCE;
    int lower_y = centroid.y - dx_coef*LASER_MOUNTING_HEIGHT - PIXEL_ALLOWANCE;
    int upper_x = centroid.x + dy_coef*width/2 + PIXEL_ALLOWANCE;
    int upper_y = centroid.y + dy_coef*(object_height_-LASER_MOUNTING_HEIGHT) + PIXEL_ALLOWANCE;

    Rectangle rect;
    rect.upper_left.x = lower_x<0 ? 0 : lower_x;
    rect.size.x = (upper_x>cam_param_.width ? cam_param_.width : upper_x) - rect.upper_left.x;
    rect.upper_left.y = lower_y<0 ? 0 : lower_y;
    rect.size.y = (upper_y>cam_param_.height ? cam_param_.height : upper_y) - rect.upper_left.y;
    return rect;
}


// A helper function. transform a 3D point in camera coordinates into a
// 2D point in pixel coordinates. Takes into account the camera parameters.
IntPoint camera_project::projection(const geometry_msgs::Point32 &pt)
{
    if(pt.z <= 0.0) throw std::out_of_range("z<0");

    double xn0 = pt.x / pt.z;
    double xn1 = pt.y / pt.z;

    double r2power = pow(xn0,2) + pow(xn1,2);

    double dx0 = 2*cam_param_.kc[2]*xn0*xn1 + cam_param_.kc[3]*(r2power+2*pow(xn0,2));
    double dx1 = cam_param_.kc[2]*(r2power+2*pow(xn0,2))*2*cam_param_.kc[3]*xn0*xn1;

    double xd0 = ( 1 + cam_param_.kc[0]*r2power + cam_param_.kc[1]*pow(r2power,2)
                     + cam_param_.kc[4]*pow(r2power,3) ) * xn0 + dx0;
    double xd1 = ( 1 + cam_param_.kc[0]*r2power + cam_param_.kc[1]*pow(r2power,2)
                     + cam_param_.kc[4]*pow(r2power,3) ) * xn1 + dx1;

    int pixel_x = cam_param_.fc[0]*xd0 + cam_param_.alpha_c*cam_param_.fc[0]*xd1 + cam_param_.cc[0];
    int pixel_y = cam_param_.fc[1]*xd1 + cam_param_.cc[1];

    IntPoint res;
    res.x = pixel_x;
    res.y = cam_param_.height - pixel_y; //remember to change coordinate upside-down;
    return res;
}


void setRectMsg(const Rectangle & rect, sensing_on_road::pedestrian_vision & msg)
{
    msg.x          = rect.upper_left.x;
    msg.y          = rect.upper_left.y;
    msg.width      = rect.size.x;
    msg.height     = rect.size.y;

    msg.cvRect_x1  = msg.x;
    msg.cvRect_y1  = msg.y;
    msg.cvRect_x2  = msg.x + msg.width;
    msg.cvRect_y2  = msg.y + msg.height;
}
