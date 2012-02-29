/*
 * xb3.cpp
 *
 *  Created on: Feb 24, 2012
 *      Author: golfcar
 */

//=============================================================================
// System Includes
//=============================================================================
#include <stdlib.h>
#include <stdio.h>
#include <assert.h>

#include <dc1394/conversions.h>
#include <dc1394/control.h>
#include <dc1394/utils.h>


//=============================================================================
// PGR Includes
//=============================================================================
#include "pgr_registers.h"
#include "pgr_stereocam.h"
#include <triclops/pnmutils.h>
#include <triclops/triclopsimageio.h>

#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <image_transport/image_transport.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv/cv.h>
#include <opencv2/highgui/highgui.hpp>

#include <sensor_msgs/PointCloud.h>
#include <camera_info_manager/camera_info_manager.h>

using namespace std;

class xb3
{
public:

    dc1394camera_t*  camera_;
    string camera_name_;
    PGRStereoCamera_t stereoCamera_;
    ros::NodeHandle nh_, priv_nh_;
    image_transport::ImageTransport it_;

    image_transport::Publisher wide_left_pub_, narrow_left_pub_, wide_right_pub_, narrow_right_pub_;
    camera_info_manager::CameraInfoManager c_info_narrow_left_, c_info_narrow_right_, c_info_wide_left_, c_info_wide_right_;
    string uri_nl_, uri_nr_, uri_wl_, uri_wr_;
    ros::Publisher info_nl_pub_, info_nr_pub_, info_wl_pub_, info_wr_pub_, narrow_pc_pub_, wide_pc_pub_;
    xb3() : priv_nh_("~"), it_(nh_),  c_info_narrow_left_(ros::NodeHandle(nh_, "bumblebee/left"),"bumblebee/narrow_left"),
            c_info_narrow_right_(ros::NodeHandle(nh_,"bumblebee/right"),"bumblebee/narrow_right"),
            c_info_wide_left_(ros::NodeHandle(nh_, "bumblebee/wide_left"), "bumblebee/wide_left"),
            c_info_wide_right_(ros::NodeHandle(nh_, "bumblebee/wide_right"), "bumblebee/wide_right")
    {

        camera_name_ = string("bumblebee/");
        narrow_left_pub_ = it_.advertise(camera_name_+"left/image_raw", 1);
        wide_left_pub_ = it_.advertise(camera_name_+"wide_left/image_raw", 1);
        narrow_right_pub_ = it_.advertise(camera_name_+"right/image_raw", 1);
        wide_right_pub_ = it_.advertise(camera_name_+"wide_right/image_raw", 1);


        info_nl_pub_ = nh_.advertise<sensor_msgs::CameraInfo>(camera_name_+"left/camera_info", 1);
        info_nr_pub_ = nh_.advertise<sensor_msgs::CameraInfo>(camera_name_+"right/camera_info", 1);
        info_wl_pub_ = nh_.advertise<sensor_msgs::CameraInfo>(camera_name_+"wide_left/camera_info", 1);
        info_wr_pub_ = nh_.advertise<sensor_msgs::CameraInfo>(camera_name_+"wide_right/camera_info", 1);

        priv_nh_.param("uri_nl", uri_nl_, string("package://bumblebeeXB3_1394/narrow_left.ini"));
        priv_nh_.param("uri_nr", uri_nr_, string("package://bumblebeeXB3_1394/narrow_right.ini"));
        priv_nh_.param("uri_wl", uri_wl_, string("package://bumblebeeXB3_1394/wide_left.ini"));
        priv_nh_.param("uri_wr", uri_wr_, string("package://bumblebeeXB3_1394/wide_right.ini"));

        if(!initialize()) return;
        publish_image();

    }

    ~xb3()
    {
        printf( "Stop transmission\n" );
        //  Stop data transmission
        if ( dc1394_video_set_transmission( stereoCamera_.camera, DC1394_OFF ) != DC1394_SUCCESS )
        {
            fprintf( stderr, "Couldn't stop the camera?\n" );
        }

        // close camera
        cleanup_and_exit( camera_ );
    }

private:

    void publish_image()
    {
        // size of buffer for all images at mono8
        unsigned int   nBufferSize = stereoCamera_.nRows *
                stereoCamera_.nCols *
                stereoCamera_.nBytesPerPixel;

        // allocate a buffer to hold the de-interleaved images
        unsigned char* pucDeInterlacedBuffer = new unsigned char[ nBufferSize ];



        // define all the buffers you could ever want, color or mono
        // leave allocation till later
        // color buffers
        // buffer for the 3 color images color processed and stacked
        unsigned char* pucRGBBuffer      = NULL;
        // buffer for the green channel that approximates the mono signal
        unsigned char* pucGreenBuffer    = NULL;
        unsigned char* pucRightRGB       = NULL;
        unsigned char* pucLeftRGB        = NULL;
        unsigned char* pucCenterRGB      = NULL;


        TriclopsInput  colorInput;

        // allocation color processing buffers
        if ( stereoCamera_.bColor )
        {
            pucRGBBuffer      = new unsigned char[ 3 * nBufferSize ];
            pucGreenBuffer        = new unsigned char[ nBufferSize ];
            // allocate a color input for color image rectification
            colorInput.nrows      = stereoCamera_.nRows;
            colorInput.ncols      = stereoCamera_.nCols;
            colorInput.rowinc     = stereoCamera_.nCols*4;
            colorInput.inputType  = TriInp_RGB_32BIT_PACKED;
            colorInput.u.rgb32BitPacked.data
            = (void*) (new unsigned char[colorInput.nrows*colorInput.rowinc]);
        }

        int i=0;
        while(nh_.ok())
        {
            i++;

            TriclopsInput wideInput, shortInput;
            assert(stereoCamera_.bColor);
            {
                // get the images from the capture buffer and do all required
                // processing
                extractImagesColorXB3( &stereoCamera_,
                                       DC1394_BAYER_METHOD_NEAREST,
                                       pucDeInterlacedBuffer,
                                       pucRGBBuffer,
                                       pucGreenBuffer,
                                       &pucRightRGB,
                                       &pucLeftRGB,
                                       &pucCenterRGB,
                                       &shortInput,
                                       &wideInput );
            }



            ros::Time time = ros::Time::now();
            sensor_msgs::Image image_pub;
            cv::Size image_size(640, 480);

            cv::Mat original_left(stereoCamera_.nRows, stereoCamera_.nCols,CV_8UC3);
            original_left.data = (uchar*)pucCenterRGB;
            cv::cvtColor(original_left, original_left, CV_BGR2RGB);
            cv::resize(original_left, original_left, image_size);
            image_pub = publishImage(narrow_left_pub_, original_left, camera_name_+"narrow", time);
            publishCInfo(info_nl_pub_, uri_nl_, c_info_narrow_left_, image_pub);

            cv::Mat original_wide_left(stereoCamera_.nRows, stereoCamera_.nCols,CV_8UC3);
            original_wide_left.data = (uchar*)pucLeftRGB;
            cv::cvtColor(original_wide_left, original_wide_left, CV_BGR2RGB);
            cv::resize(original_wide_left, original_wide_left, image_size);
            image_pub = publishImage(narrow_right_pub_, original_wide_left, camera_name_+"wide", time);
            publishCInfo(info_wl_pub_, uri_wl_, c_info_wide_left_, image_pub);

            cv::Mat original_right(stereoCamera_.nRows, stereoCamera_.nCols,CV_8UC3);
            original_right.data = (uchar*)pucRightRGB;
            cv::cvtColor(original_right, original_right, CV_BGR2RGB);
            cv::resize(original_right, original_right, image_size);
            image_pub = publishImage(narrow_right_pub_, original_right, camera_name_+"narrow", time);
            publishCInfo(info_nr_pub_, uri_nr_, c_info_narrow_right_, image_pub);
            image_pub = publishImage(narrow_right_pub_, original_right, camera_name_+"wide", time);
            publishCInfo(info_wr_pub_, uri_wr_, c_info_wide_right_, image_pub);
            ros::spinOnce();
        }

    }

    void publishCInfo(ros::Publisher& pub, string uri, camera_info_manager::CameraInfoManager& manager, sensor_msgs::Image img)
    {
        sensor_msgs::CameraInfo c_info;
        manager.loadCameraInfo(uri);
        assert(manager.isCalibrated());
        c_info = manager.getCameraInfo();
        c_info.header = img.header;
        pub.publish(c_info);

    }

    sensor_msgs::Image publishImage(const image_transport::Publisher& pub, cv::Mat& img, string frame_id, const ros::Time& stamp)
    {
        sensor_msgs::Image image;
        cv_bridge::CvImage cvImage;
        cvImage.image = img;
        cvImage.toImageMsg(image);
        image.encoding = sensor_msgs::image_encodings::BGR8;
        image.header.stamp = stamp;
        image.header.frame_id = frame_id;
        pub.publish(image);
        return image;
    }

    bool initialize()
    {
        //===================================================================
        // Find cameras on the 1394 buses
        dc1394_t * d;
        dc1394camera_list_t * list;
        dc1394error_t    err;
        unsigned int nThisCam;


        d = dc1394_new ();

        // Enumerate cameras connected to the PC
        err = dc1394_camera_enumerate (d, &list);

        if ( err != DC1394_SUCCESS )
        {
            fprintf( stderr, "Unable to look for cameras\n\n"
                     "Please check \n"
                     "  - if the kernel modules `ieee1394',`raw1394' and `ohci1394' are loaded \n"
                     "  - if you have read/write access to /dev/raw1394\n\n");
            return false;
        }

        //  get the camera nodes and describe them as we find them
        if (list->num == 0)
        {
            fprintf( stderr, "No cameras found!\n");
            return false;
        }


        printf( "There were %d camera(s) found attached to your PC\n", list->num  );

        // Identify cameras. Use the first stereo camera that is found
        for ( nThisCam = 0; nThisCam < list->num; nThisCam++ )
        {
            camera_ = dc1394_camera_new(d, list->ids[nThisCam].guid);

            if(!camera_)
            {
                printf("Failed to initialize camera with guid %lx", list->ids[nThisCam].guid);
                continue;
            }

            printf( "Camera %d model = '%s'\n", nThisCam, camera_->model );

            if ( isStereoCamera(camera_))
            {

                printf( "Using this camera\n" );
                break;
            }

            dc1394_camera_free(camera_);
        }

        if ( nThisCam == list->num )
        {
            printf( "No stereo cameras were detected\n" );
            return 0;
        }
        ROS_INFO("Resetting 1394 bus");
        if (DC1394_SUCCESS != dc1394_camera_reset(camera_))
        {
            cleanup_and_exit( camera_ );
            fprintf( stderr, "Unable to reset camera.\n");
            return 0;
        }
        ROS_INFO("Reset Complete");

        // query information about this stereo camera
        err = queryStereoCamera( camera_, &stereoCamera_ );
        if ( err != DC1394_SUCCESS )
        {
            fprintf( stderr, "Cannot query all information from camera\n" );
            cleanup_and_exit( camera_ );
        }

        if ( stereoCamera_.model != BUMBLEBEEXB3 )
        {
            fprintf( stderr, "Stereo camera found was not a BB XB3\n" );
            cleanup_and_exit( camera_ );
        }

        // override the default nBytesPerPixel to be 3, because we really do
        // want to run at 3 camera mode
        stereoCamera_.nBytesPerPixel = 3;

        // set the capture mode
        printf( "Setting stereo video capture mode\n" );
        err = setStereoVideoCapture( &stereoCamera_ );
        if ( err != DC1394_SUCCESS )
        {
            fprintf( stderr, "Could not set up video capture mode\n" );
            cleanup_and_exit( stereoCamera_.camera );
        }

        // have the camera start sending us data
        printf( "Start transmission\n" );
        err = startTransmission( &stereoCamera_ );
        if ( err != DC1394_SUCCESS )
        {
            fprintf( stderr, "Unable to start camera iso transmission\n" );
            cleanup_and_exit( stereoCamera_.camera );
        }

        return true;
    }


    void cleanup_and_exit( dc1394camera_t* camera )
    {
        dc1394_capture_stop( camera );
        dc1394_video_set_transmission( camera, DC1394_OFF );
        dc1394_camera_free( camera );
        exit( 0 );
    }
};


int main( int argc, char *argv[] )
{
    ros::init(argc, argv, "xb3");
    xb3 xb3;

    return 0;
}

