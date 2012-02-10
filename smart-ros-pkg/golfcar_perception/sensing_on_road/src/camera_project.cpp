/*
 * This class is to project object position in 3D world onto 2D image;
 * Reference: "Fast Extrinsic Calibration of a Laser Rangefinder to a Camera"
 * http://www.ri.cmu.edu/publication_view.html?pub_id=5293
 */

#include <vector>
#include <cmath>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <geometry_msgs/Point32.h>
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


const double MAX_HEIGHT = 2.0;
const double PIXEL_ALLOWANCE = 30;
const double LASER_MOUNTING_HEIGHT = 1.0;


/// A structure that holds the camera calibration parameters.
struct CameraCalibrationParameters
{
    double fc[2];   ///< focal_length
    double cc[2];   ///< principal_point
    double alpha_c; ///< skew_coeff
    double kc[5];   ///< distortions

    int width;  ///< frame's width
    int height; ///< frame's height

    CameraCalibrationParameters()
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

};


CameraCalibrationParameters gCamera;


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

private:
    /// Frame ID of the camera (where to project)
    std::string camera_frame_id_;

    /// Subscribes to the 'ped_laser_batch' topic (type pedestrian_laser_batch).
    /// Messages on this topic describe the position of a set of pedestrians as
    /// detected by the laser.
    /// They will be projected to camera coordinates (pixels) and published by
    /// ped_vision_pub_.
    ros::Subscriber ped_laser_batch_sub_;

    /// Publishes result of projection on topic 'ped_vision_batch' (type
    /// pedestrian_vision_batch). Messages on this topic describe the position
    /// of a set of pedestrians as detected by the laser, in camera coordinates
    /// (pixels).
    ros::Publisher ped_vision_pub_;

    /// Subscribes to the 'camera_project_pcl_in' topic (type feature_detection/clusters).
    /// Messages on this topic describe the position of a set of pedestrians as
    /// detected by the source sensor (laser, kinect, stereo camera, etc.).
    /// Each pedestrian is represented as a PointCloud.
    /// They will be projected to camera coordinates (pixels) and published by
    /// ped_vision_pub_.
    message_filters::Subscriber<feature_detection::clusters> ped_pcl_sub_;

    tf::TransformListener tf_;

    /// We use a tf::MessageFilter, connected to ped_pcl_sub_, so that messages
    /// are cached until a transform is available.
    tf::MessageFilter<feature_detection::clusters> * ped_pcl_sub_tf_filter_;

    /// Callback function for ped_laser_batch_sub_
    void ped_laser_batch_CB(const sensing_on_road::pedestrian_laser_batch &);

    /// Callback function for ped_pcl_sub_tf_filter_
    void pcl_in_CB(const boost::shared_ptr<const feature_detection::clusters> &);

    /// A helper function to make a rectangle from the centroid position and width
    Rectangle makeRectangle(const geometry_msgs::PointStamped & centroid_in, double width);

    /// A helper function. transform a 3D point in camera coordinates into a
    /// 2D point in pixel coordinates. Takes into account the camera parameters.
    IntPoint projection(const geometry_msgs::Point32 &pt);
};


camera_project::camera_project()
{
    ros::NodeHandle nh;
    nh.param("camera_frame_id", camera_frame_id_, std::string("usb_cam"));

    // setup the PointCloud channel
    ped_pcl_sub_.subscribe(nh, "pedestrian_clusters", 10);
    ped_pcl_sub_tf_filter_ = new tf::MessageFilter<feature_detection::clusters>(ped_pcl_sub_, tf_, camera_frame_id_, 10);
    ped_pcl_sub_tf_filter_->registerCallback(boost::bind(&camera_project::pcl_in_CB, this, _1));
    ped_pcl_sub_tf_filter_->setTolerance(ros::Duration(0.05));

    // setup the ped_laser_batch channel
    ped_laser_batch_sub_ = nh.subscribe("ped_laser_batch", 2, &camera_project::ped_laser_batch_CB, this);

    // setup the result publisher
    ped_vision_pub_ = nh.advertise<sensing_on_road::pedestrian_vision_batch>("ped_vision_batch", 2);
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
    if( ! fmutil::isWithin(centroid.x, 0, gCamera.width) ||
        ! fmutil::isWithin(centroid.y, 0, gCamera.height) )
        throw std::out_of_range("centroid does not fall inside the picture");

    float dx_coef  = gCamera.fc[0]/tmp.z;
    float dy_coef  = gCamera.fc[1]/tmp.z;

    int lower_x = centroid.x - dx_coef*width/2 - PIXEL_ALLOWANCE;
    int lower_y = centroid.y - dx_coef*LASER_MOUNTING_HEIGHT - PIXEL_ALLOWANCE;
    int upper_x = centroid.x + dy_coef*width/2 + PIXEL_ALLOWANCE;
    int upper_y = centroid.y + dy_coef*(MAX_HEIGHT-LASER_MOUNTING_HEIGHT) + PIXEL_ALLOWANCE;

    Rectangle rect;
    rect.upper_left.x = lower_x<0 ? 0 : lower_x;
    rect.size.x = (upper_x>gCamera.width ? gCamera.width : upper_x) - rect.upper_left.x;
    rect.upper_left.y = lower_y<0 ? 0 : lower_y;
    rect.size.y = (upper_y>gCamera.height ? gCamera.height : upper_y) - rect.upper_left.y;
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

    double dx0 = 2*gCamera.kc[2]*xn0*xn1 + gCamera.kc[3]*(r2power+2*pow(xn0,2));
    double dx1 = gCamera.kc[2]*(r2power+2*pow(xn0,2))*2*gCamera.kc[3]*xn0*xn1;

    double xd0 = ( 1 + gCamera.kc[0]*r2power + gCamera.kc[1]*pow(r2power,2)
                     + gCamera.kc[4]*pow(r2power,3) ) * xn0 + dx0;
    double xd1 = ( 1 + gCamera.kc[0]*r2power + gCamera.kc[1]*pow(r2power,2)
                     + gCamera.kc[4]*pow(r2power,3) ) * xn1 + dx1;

    int pixel_x = gCamera.fc[0]*xd0 + gCamera.alpha_c*gCamera.fc[0]*xd1 + gCamera.cc[0];
    int pixel_y = gCamera.fc[1]*xd1 + gCamera.cc[1];

    IntPoint res;
    res.x = pixel_x;
    res.y = gCamera.height - pixel_y; //remember to change coordinate upside-down;
    return res;
}


void setRectMsg(const Rectangle & rect, sensing_on_road::pedestrian_vision & msg)
{
    msg.x         = rect.upper_left.x;
    msg.y         = rect.upper_left.y;
    msg.width     = rect.size.x;
    msg.height    = rect.size.y;

    msg.cvRect_x1 = msg.x;
    msg.cvRect_y1 = msg.y;
    msg.cvRect_x2 = msg.x + msg.width;
    msg.cvRect_y2 = msg.y + msg.height;
}


////////////////////////////////////////////////////////////////////////////////


void camera_project::pcl_in_CB(const boost::shared_ptr<const feature_detection::clusters> & clusters_ptr)
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

        Rectangle rect;
        try {
            rect = makeRectangle(pt_src, clusters_ptr->clusters[i].width);
        } catch( std::out_of_range & e ) {
            ROS_WARN("out of range: %s", e.what());
            continue;
        } catch( tf::TransformException & e ) {
            ROS_WARN("camera project tf error: %s", e.what());
            return;
        }

        sensing_on_road::pedestrian_vision msg;
        msg.decision_flag = false;
        msg.complete_flag = true;
        msg.disz   = clusters_ptr->clusters[i].centroid.x;
        setRectMsg(rect, msg);
        ped_vision_batch_msg.pd_vector.push_back(msg);
    }

    if( ! ped_vision_batch_msg.pd_vector.empty() )
        ped_vision_pub_.publish(ped_vision_batch_msg);
}


void camera_project::ped_laser_batch_CB(const sensing_on_road::pedestrian_laser_batch &pd_laser_para)
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

        Rectangle rect;
        try {
            rect = makeRectangle(ped_laser.pedestrian_laser, ped_laser.size);
        } catch( std::out_of_range & e ) {
            ROS_WARN("out of range: %s", e.what());
            continue;
        } catch( tf::TransformException & e ) {
            ROS_WARN("camera project tf error: %s", e.what());
            return;
        }

        sensing_on_road::pedestrian_vision msg;
        msg.confidence = ped_laser.confidence;
        msg.decision_flag = false;
        msg.complete_flag = true;
        msg.object_label = ped_laser.object_label;
        msg.disz = ped_laser.pedestrian_laser.point.x;
        setRectMsg(rect, msg);
        ped_vision_batch_msg.pd_vector.push_back(msg);
    }

    if( ! ped_vision_batch_msg.pd_vector.empty() )
        ped_vision_pub_.publish(ped_vision_batch_msg);
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "camera_projector");
    camera_project * camera_project_node = new camera_project();
    ros::spin();
    delete camera_project_node;
}

