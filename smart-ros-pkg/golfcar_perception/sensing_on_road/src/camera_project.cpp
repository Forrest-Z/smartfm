#include <sensing_on_road/camera_project.h>


namespace camera_project
{

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


camera_projector::camera_projector() : camera_frame_id_("usb_cam"), fov_(70), pixel_allowance_(PIXEL_ALLOWANCE),
        laser_mounting_height_(LASER_MOUNTING_HEIGHT)

{
}

void camera_projector::setCameraFrameID(const std::string & id)
{
    camera_frame_id_ = id;
}

void camera_projector::setPixelAllowance(const double pixel)
{
    pixel_allowance_ = pixel;
}

void camera_projector::setLaserMountingHeight(const double height)
{
    laser_mounting_height_ = height;
}
void camera_projector::setCameraFOV(const double &fovInDegree)
{
    fov_ = fovInDegree;
}
// A helper function to make a rectangle from the centroid position and width
CvRectangle camera_projector::project(
    const geometry_msgs::PointStamped & centroid_in,
    double width, double height) const
{

    //transform from source frame to camera frame
    geometry_msgs::PointStamped pt_camera;
    tf_.transformPoint(camera_frame_id_, centroid_in, pt_camera);

    //tranform from ROS-convention to Vision-convention;
    geometry_msgs::Point32 tmp;
    tmp.x = -pt_camera.point.y;
    tmp.y =  pt_camera.point.z;
    tmp.z =  pt_camera.point.x;

    // project to frame (pixel coordinates)
    IntPoint centroid = projection(tmp);
    double angular_dist = atan2(pt_camera.point.x, -pt_camera.point.y);
    angular_dist = angular_dist/M_PI*180;
    //the central point should fall inside of the picture.
    if( ! fmutil::isWithin(centroid.x, 0, cam_param_.width) ||
        ! fmutil::isWithin(centroid.y, 0, cam_param_.height) ||
        ! fmutil::isWithin(angular_dist, fov_/2.0, 180-fov_/2.0))
        throw std::out_of_range("centroid does not fall inside the picture");

    float dx_coef  = cam_param_.fc[0]/tmp.z;
    float dy_coef  = cam_param_.fc[1]/tmp.z;

    float x1 = centroid.x - dx_coef*width/2 - pixel_allowance_;
    float y1 = centroid.y - dx_coef*laser_mounting_height_ - pixel_allowance_;
    float x2 = centroid.x + dy_coef*width/2 + pixel_allowance_;
    float y2 = centroid.y + dy_coef*(height-laser_mounting_height_) + pixel_allowance_;

    CvRectangle rect;
    rect.upper_left.x = x1<0 ? 0 : x1;
    rect.upper_left.y = y1<0 ? 0 : y1;
    rect.lower_right.x = x2>cam_param_.width ? cam_param_.width : x2;
    rect.lower_right.y = y2>cam_param_.height ? cam_param_.height : y2;
    return rect;
}


// A helper function. transform a 3D point in camera coordinates into a
// 2D point in pixel coordinates. Takes into account the camera parameters.
IntPoint camera_projector::projection(const geometry_msgs::Point32 &pt) const
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


CvRectangle convertRect(const Rectangle & r)
{
    CvRectangle cvr;
    cvr.upper_left.x  = r.upper_left.x;
    cvr.upper_left.y  = r.upper_left.y;
    cvr.lower_right.x  = r.upper_left.x + r.width;
    cvr.lower_right.y  = r.upper_left.y + r.height;
    return cvr;
}

Rectangle convertRect(const CvRectangle & r)
{
    Rectangle rect;
    rect.upper_left.x = r.upper_left.x;
    rect.upper_left.y = r.upper_left.y;
    rect.width = r.lower_right.x - r.upper_left.x;
    rect.height = r.lower_right.y - r.lower_right.x;
    return rect;
}


} //namespace camera_project
