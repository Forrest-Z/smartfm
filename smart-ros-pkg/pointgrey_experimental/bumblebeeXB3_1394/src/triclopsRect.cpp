/*
 * triclopsStereo.cpp
 *
 *  Created on: Feb 28, 2012
 *      Author: golfcar
 */

#include "xb3.h"
#include <image_transport/subscriber_filter.h>
#include <message_filters/time_synchronizer.h>
using namespace std;
using namespace xb3;

class triclops_rect
{
public:

    string short_cal_, wide_cal_;
    TriclopsContext wideTriclops_, shortTriclops_;
    dc1394camera_t*  camera_;
    string camera_name_;
    PGRStereoCamera_t stereoCamera_;
    ros::NodeHandle nh_, priv_nh_;
    image_transport::ImageTransport it_;
    image_transport::Publisher nl_rect_pub_, nr_rect_pub_, wl_rect_pub_, wr_rect_pub_;

    string uri_nl_, uri_nr_, uri_wl_, uri_wr_;
    ros::Publisher info_nl_pub_, info_nr_pub_, info_wl_pub_, info_wr_pub_;
    camera_info_manager::CameraInfoManager c_info_narrow_left_, c_info_narrow_right_, c_info_wide_left_, c_info_wide_right_;
    image_transport::SubscriberFilter left_sub_, center_sub_, right_sub_;



    triclops_rect() : priv_nh_("~"), it_(nh_), c_info_narrow_left_(ros::NodeHandle(nh_, "bumblebee/tri_left"),"bumblebee/narrow_left"),
            c_info_narrow_right_(ros::NodeHandle(nh_,"bumblebee/tri_right"),"bumblebee/narrow_right"),
            c_info_wide_left_(ros::NodeHandle(nh_, "bumblebee/tri_wide_left"), "bumblebee/wide_left"),
            c_info_wide_right_(ros::NodeHandle(nh_, "bumblebee/tri_wide_right"), "bumblebee/wide_right")
    {

        camera_name_ = string("bumblebee/");
        priv_nh_.param("short_calibration_file", short_cal_, string(""));
        priv_nh_.param("wide_calibration_file", wide_cal_, string(""));

        left_sub_.subscribe(it_,camera_name_+"raw_left/image_raw",1);
        center_sub_.subscribe(it_,camera_name_+"raw_center/image_raw",1);
        right_sub_.subscribe(it_,camera_name_+"raw_right/image_raw",1);

        nl_rect_pub_ = it_.advertise(camera_name_+"tri_left/image_raw",1);
        nr_rect_pub_ = it_.advertise(camera_name_+"tri_right/image_raw",1);
        wl_rect_pub_ = it_.advertise(camera_name_+"tri_wide_left/image_raw",1);
        wr_rect_pub_ = it_.advertise(camera_name_+"tri_wide_right/image_raw",1);

        priv_nh_.param("uri_nl", uri_nl_, string("package://bumblebeeXB3_1394/calibration/tri_narrow_left.ini"));
        priv_nh_.param("uri_nr", uri_nr_, string("package://bumblebeeXB3_1394/calibration/tri_narrow_right.ini"));
        priv_nh_.param("uri_wl", uri_wl_, string("package://bumblebeeXB3_1394/calibration/tri_wide_left.ini"));
        priv_nh_.param("uri_wr", uri_wr_, string("package://bumblebeeXB3_1394/calibration/tri_wide_right.ini"));

        info_nl_pub_ = nh_.advertise<sensor_msgs::CameraInfo>(camera_name_+"tri_left/camera_info", 1);
        info_nr_pub_ = nh_.advertise<sensor_msgs::CameraInfo>(camera_name_+"tri_right/camera_info", 1);
        info_wl_pub_ = nh_.advertise<sensor_msgs::CameraInfo>(camera_name_+"tri_wide_left/camera_info", 1);
        info_wr_pub_ = nh_.advertise<sensor_msgs::CameraInfo>(camera_name_+"tri_wide_right/camera_info", 1);

        c_info_narrow_left_.loadCameraInfo(uri_nl_);
        c_info_narrow_right_.loadCameraInfo(uri_nr_);
        c_info_wide_left_.loadCameraInfo(uri_wl_);
        c_info_wide_right_.loadCameraInfo(uri_wr_);

        assert(getTriclopsContext(short_cal_, wide_cal_, shortTriclops_, wideTriclops_));
        message_filters::TimeSynchronizer<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::Image> sync(left_sub_, center_sub_, right_sub_, 10);
        sync.registerCallback(boost::bind(&triclops_rect::callback, this, _1, _2, _3));


        ros::spin();
    }

    ~triclops_rect(){};

private:

    void callback(const sensor_msgs::ImageConstPtr& left_image, const sensor_msgs::ImageConstPtr& center_image, const sensor_msgs::ImageConstPtr& right_image)
    {

        TriclopsColorImage rectLeft, rectCenter, rectShortRight, rectWideRight;
        cv_bridge::CvImageConstPtr left_cv_ptr, center_cv_ptr, right_cv_ptr;

        TriclopsInput  colorInput;

        unsigned char* pucRightRGB;
        unsigned char* pucLeftRGB;
        unsigned char* pucCenterRGB;
        pucRightRGB       = NULL;
        pucLeftRGB        = NULL;
        pucCenterRGB      = NULL;

        left_cv_ptr = cv_bridge::toCvShare(left_image, sensor_msgs::image_encodings::RGB8);
        center_cv_ptr = cv_bridge::toCvShare(center_image, sensor_msgs::image_encodings::RGB8);
        right_cv_ptr = cv_bridge::toCvShare(right_image, sensor_msgs::image_encodings::RGB8);

        pucRightRGB = (unsigned char*)right_cv_ptr->image.data;
        pucLeftRGB = (unsigned char*)left_cv_ptr->image.data;
        pucCenterRGB = (unsigned char*)center_cv_ptr->image.data;

        // copy the RGB buffer into an input structure

        int ecol(1280), erow(960);
        // allocation color processing buffers
        assert((left_cv_ptr->image.cols == ecol)&&(right_cv_ptr->image.cols == ecol)&&(center_cv_ptr->image.cols==ecol));
        assert((left_cv_ptr->image.rows == erow)&&(right_cv_ptr->image.rows == erow)&&(center_cv_ptr->image.rows==erow));
        // allocate a color input for color image rectification
        colorInput.nrows      = erow;
        colorInput.ncols      = ecol;
        colorInput.rowinc     = ecol*4;
        colorInput.inputType  = TriInp_RGB_32BIT_PACKED;
        colorInput.u.rgb32BitPacked.data
        = (void*) (new unsigned char[colorInput.nrows*colorInput.rowinc]);

        // rectify the left and right image with both the short and wide contexts
        convertColorTriclopsInput( &colorInput, pucLeftRGB );
        triclopsRectifyColorImage( wideTriclops_, TriCam_LEFT, &colorInput, &rectLeft );
        convertColorTriclopsInput( &colorInput, pucCenterRGB );
        triclopsRectifyColorImage( shortTriclops_, TriCam_LEFT, &colorInput, &rectCenter );
        convertColorTriclopsInput( &colorInput, pucRightRGB );
        triclopsRectifyColorImage( wideTriclops_, TriCam_RIGHT, &colorInput, &rectWideRight);
        convertColorTriclopsInput( &colorInput, pucRightRGB );
        triclopsRectifyColorImage( shortTriclops_, TriCam_RIGHT, &colorInput, &rectShortRight);

        cv::Mat tmp;
        ros::Time stamp = left_image->header.stamp;

        sensor_msgs::Image image_pub;
        triclopsColorImageToCvImage( rectLeft, tmp, "wide_left", false);
        image_pub=publishImage(wl_rect_pub_,tmp, camera_name_+"wide", stamp);
        publishCInfo(info_wl_pub_, c_info_wide_left_, image_pub);

        triclopsColorImageToCvImage( rectCenter, tmp, "narrow_left", false);
        image_pub=publishImage(nl_rect_pub_,tmp, camera_name_+"narrow", stamp);
        publishCInfo(info_nl_pub_, c_info_narrow_left_, image_pub);

        triclopsColorImageToCvImage( rectWideRight, tmp, "wide_right", false);
        image_pub=publishImage(wr_rect_pub_,tmp, camera_name_+"wide", stamp);
        publishCInfo(info_wr_pub_, c_info_wide_right_, image_pub);

        triclopsColorImageToCvImage( rectShortRight, tmp, "narrow_right", false);
        image_pub=publishImage(nr_rect_pub_,tmp, camera_name_+"narrow", stamp);
        publishCInfo(info_nr_pub_, c_info_narrow_right_, image_pub);
         //remember to clear this off for memory leak
         delete[] (unsigned char*)colorInput.u.rgb32BitPacked.data;
    }
};







//=============================================================================
// MAIN
//
int main( int argc, char *argv[] )
{
    ros::init(argc, argv, "triclopsRect");
    triclops_rect TriclopsRect;

    return 0;
}

