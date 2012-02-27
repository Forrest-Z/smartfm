/*
 * xb3.cpp
 *
 *  Created on: Feb 24, 2012
 *      Author: golfcar
 */

/**************************************************************************
 *
 * Title:   xb3stereo
 * Copyright:   (C) 2006,2007,2008 Don Murray donm@ptgrey.com
 *
 * Description:
 *
 *    Get an image set from a Bumblebee or Bumblebee2 via DMA transfer
 *    using libdc1394 and process it with the Triclops stereo
 *    library. Based loosely on 'grab_gray_image' from libdc1394 examples.
 *
 *-------------------------------------------------------------------------
 *     License: LGPL
 *
 *  This library is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU Lesser General Public
 *  License as published by the Free Software Foundation; either
 *  version 2.1 of the License, or (at your option) any later version.
 *
 *  This library is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *  Lesser General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public
 *  License along with this library; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 *************************************************************************/

//=============================================================================
// Copyright © 2006,2007,2008 Point Grey Research, Inc. All Rights Reserved.
//
// This software is the confidential and proprietary information of Point
// Grey Research, Inc. ("Confidential Information").  You shall not
// disclose such Confidential Information and shall use it only in
// accordance with the terms of the license agreement you entered into
// with Point Grey Research Inc.
//
// PGR MAKES NO REPRESENTATIONS OR WARRANTIES ABOUT THE SUITABILITY OF THE
// SOFTWARE, EITHER EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
// IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR
// PURPOSE, OR NON-INFRINGEMENT. PGR SHALL NOT BE LIABLE FOR ANY DAMAGES
// SUFFERED BY LICENSEE AS A RESULT OF USING, MODIFYING OR DISTRIBUTING
// THIS SOFTWARE OR ITS DERIVATIVES.
//
//=============================================================================

//=============================================================================
//
// xb3stereo.cpp
//
//=============================================================================

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

static unsigned char dmap[768] =
{ 150, 150, 150,
  107, 0, 12,
  106, 0, 18,
  105, 0, 24,
  103, 0, 30,
  102, 0, 36,
  101, 0, 42,
  99, 0, 48,
  98, 0, 54,
  97, 0, 60,
  96, 0, 66,
  94, 0, 72,
  93, 0, 78,
  92, 0, 84,
  91, 0, 90,
  89, 0, 96,
  88, 0, 102,
  87, 0, 108,
  85, 0, 114,
  84, 0, 120,
  83, 0, 126,
  82, 0, 131,
  80, 0, 137,
  79, 0, 143,
  78, 0, 149,
  77, 0, 155,
  75, 0, 161,
  74, 0, 167,
  73, 0, 173,
  71, 0, 179,
  70, 0, 185,
  69, 0, 191,
  68, 0, 197,
  66, 0, 203,
  65, 0, 209,
  64, 0, 215,
  62, 0, 221,
  61, 0, 227,
  60, 0, 233,
  59, 0, 239,
  57, 0, 245,
  56, 0, 251,
  55, 0, 255,
  54, 0, 255,
  52, 0, 255,
  51, 0, 255,
  50, 0, 255,
  48, 0, 255,
  47, 0, 255,
  46, 0, 255,
  45, 0, 255,
  43, 0, 255,
  42, 0, 255,
  41, 0, 255,
  40, 0, 255,
  38, 0, 255,
  37, 0, 255,
  36, 0, 255,
  34, 0, 255,
  33, 0, 255,
  32, 0, 255,
  31, 0, 255,
  29, 0, 255,
  28, 0, 255,
  27, 0, 255,
  26, 0, 255,
  24, 0, 255,
  23, 0, 255,
  22, 0, 255,
  20, 0, 255,
  19, 0, 255,
  18, 0, 255,
  17, 0, 255,
  15, 0, 255,
  14, 0, 255,
  13, 0, 255,
  11, 0, 255,
  10, 0, 255,
  9, 0, 255,
  8, 0, 255,
  6, 0, 255,
  5, 0, 255,
  4, 0, 255,
  3, 0, 255,
  1, 0, 255,
  0, 4, 255,
  0, 10, 255,
  0, 16, 255,
  0, 22, 255,
  0, 28, 255,
  0, 34, 255,
  0, 40, 255,
  0, 46, 255,
  0, 52, 255,
  0, 58, 255,
  0, 64, 255,
  0, 70, 255,
  0, 76, 255,
  0, 82, 255,
  0, 88, 255,
  0, 94, 255,
  0, 100, 255,
  0, 106, 255,
  0, 112, 255,
  0, 118, 255,
  0, 124, 255,
  0, 129, 255,
  0, 135, 255,
  0, 141, 255,
  0, 147, 255,
  0, 153, 255,
  0, 159, 255,
  0, 165, 255,
  0, 171, 255,
  0, 177, 255,
  0, 183, 255,
  0, 189, 255,
  0, 195, 255,
  0, 201, 255,
  0, 207, 255,
  0, 213, 255,
  0, 219, 255,
  0, 225, 255,
  0, 231, 255,
  0, 237, 255,
  0, 243, 255,
  0, 249, 255,
  0, 255, 255,
  0, 255, 249,
  0, 255, 243,
  0, 255, 237,
  0, 255, 231,
  0, 255, 225,
  0, 255, 219,
  0, 255, 213,
  0, 255, 207,
  0, 255, 201,
  0, 255, 195,
  0, 255, 189,
  0, 255, 183,
  0, 255, 177,
  0, 255, 171,
  0, 255, 165,
  0, 255, 159,
  0, 255, 153,
  0, 255, 147,
  0, 255, 141,
  0, 255, 135,
  0, 255, 129,
  0, 255, 124,
  0, 255, 118,
  0, 255, 112,
  0, 255, 106,
  0, 255, 100,
  0, 255, 94,
  0, 255, 88,
  0, 255, 82,
  0, 255, 76,
  0, 255, 70,
  0, 255, 64,
  0, 255, 58,
  0, 255, 52,
  0, 255, 46,
  0, 255, 40,
  0, 255, 34,
  0, 255, 28,
  0, 255, 22,
  0, 255, 16,
  0, 255, 10,
  0, 255, 4,
  2, 255, 0,
  8, 255, 0,
  14, 255, 0,
  20, 255, 0,
  26, 255, 0,
  32, 255, 0,
  38, 255, 0,
  44, 255, 0,
  50, 255, 0,
  56, 255, 0,
  62, 255, 0,
  68, 255, 0,
  74, 255, 0,
  80, 255, 0,
  86, 255, 0,
  92, 255, 0,
  98, 255, 0,
  104, 255, 0,
  110, 255, 0,
  116, 255, 0,
  122, 255, 0,
  128, 255, 0,
  133, 255, 0,
  139, 255, 0,
  145, 255, 0,
  151, 255, 0,
  157, 255, 0,
  163, 255, 0,
  169, 255, 0,
  175, 255, 0,
  181, 255, 0,
  187, 255, 0,
  193, 255, 0,
  199, 255, 0,
  205, 255, 0,
  211, 255, 0,
  217, 255, 0,
  223, 255, 0,
  229, 255, 0,
  235, 255, 0,
  241, 255, 0,
  247, 255, 0,
  253, 255, 0,
  255, 251, 0,
  255, 245, 0,
  255, 239, 0,
  255, 233, 0,
  255, 227, 0,
  255, 221, 0,
  255, 215, 0,
  255, 209, 0,
  255, 203, 0,
  255, 197, 0,
  255, 191, 0,
  255, 185, 0,
  255, 179, 0,
  255, 173, 0,
  255, 167, 0,
  255, 161, 0,
  255, 155, 0,
  255, 149, 0,
  255, 143, 0,
  255, 137, 0,
  255, 131, 0,
  255, 126, 0,
  255, 120, 0,
  255, 114, 0,
  255, 108, 0,
  255, 102, 0,
  255, 96, 0,
  255, 90, 0,
  255, 84, 0,
  255, 78, 0,
  255, 72, 0,
  255, 66, 0,
  255, 60, 0,
  255, 54, 0,
  255, 48, 0,
  255, 42, 0,
  255, 36, 0,
  255, 30, 0,
  255, 24, 0,
  255, 18, 0,
  255, 12, 0,
  255,  6, 0,
  255,  0, 0,
};

class xb3
{
public:

    string short_cal_, wide_cal_;
    TriclopsContext wideTriclops_, shortTriclops_;
    dc1394camera_t*  camera_;
    string camera_name_;
    PGRStereoCamera_t stereoCamera_;
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;

    image_transport::Publisher wide_left_pub_, narrow_left_pub_, wide_right_pub_, narrow_right_pub_;
    camera_info_manager::CameraInfoManager c_info_narrow_left_, c_info_narrow_right_, c_info_wide_left_, c_info_wide_right_;
    string uri_nl_, uri_nr_, uri_wl_, uri_wr_;
    ros::Publisher info_nl_pub_, info_nr_pub_, info_wl_pub_, info_wr_pub_;
    xb3() : it_(nh_), c_info_narrow_left_(nh_, string("bumblebee/narrow_left")),
            c_info_narrow_right_(nh_, string("bumblebee/narrow_right")),
            c_info_wide_left_(nh_, string("bumblebee/wide_left")),
            c_info_wide_right_(nh_, string("bumblebee/wide_right"))
    {
        ros::NodeHandle n("~");
        camera_name_ = string("bumblebee/");
        narrow_left_pub_ = it_.advertise(camera_name_+"narrow_left/image_raw", 1);
        wide_left_pub_ = it_.advertise(camera_name_+"wide_left/image_raw", 1);
        narrow_right_pub_ = it_.advertise(camera_name_+"narrow_right/image_raw", 1);
        wide_right_pub_ = it_.advertise(camera_name_+"wide_right/image_raw", 1);

        info_nl_pub_ = nh_.advertise<sensor_msgs::CameraInfo>(camera_name_+"narrow_left/camera_info", 1);
        info_nr_pub_ = nh_.advertise<sensor_msgs::CameraInfo>(camera_name_+"narrow_right/camera_info", 1);
        info_wl_pub_ = nh_.advertise<sensor_msgs::CameraInfo>(camera_name_+"wide_left/camera_info", 1);
        info_wr_pub_ = nh_.advertise<sensor_msgs::CameraInfo>(camera_name_+"wide_right/camera_info", 1);

        n.param("uri_nl", uri_nl_, string("package://bumblebeeXB3_1394/narrow_left.yaml"));
        n.param("uri_nr", uri_nr_, string("package://bumblebeeXB3_1394/narrow_right.yaml"));
        n.param("uri_wl", uri_wl_, string("package://bumblebeeXB3_1394/wide_left.yaml"));
        n.param("uri_wr", uri_wr_, string("package://bumblebeeXB3_1394/wide_right.yaml"));
        n.param("short_calibration_file", short_cal_, string(""));
        n.param("wide_calibration_file", wide_cal_, string(""));
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

            printf( "Grab %d\n", i );

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


            // rectify the left and right image with both the short and wide contexts
            TriclopsColorImage rectLeft, rectCenter, rectShortRight, rectWideRight;


            // copy the RGB buffer into an input structure
            convertColorTriclopsInput( &colorInput, pucLeftRGB );
            triclopsRectifyColorImage( wideTriclops_, TriCam_LEFT, &colorInput, &rectLeft );


            convertColorTriclopsInput( &colorInput, pucCenterRGB );;
            triclopsRectifyColorImage( shortTriclops_, TriCam_LEFT, &colorInput, &rectCenter );

            //the right cam appears to be the same no matter which triclops context is used
            //to save bandwidth, we will not publish this
            convertColorTriclopsInput( &colorInput, pucRightRGB );
            triclopsRectifyColorImage( wideTriclops_, TriCam_RIGHT, &colorInput, &rectWideRight);
            convertColorTriclopsInput( &colorInput, pucRightRGB );
            triclopsRectifyColorImage( shortTriclops_, TriCam_RIGHT, &colorInput, &rectShortRight);

            cv::Mat tmp, right;
            ros::Time time = ros::Time::now();
            triclopsColorImageToCvImage( rectLeft, tmp, "wide_left", false);
            publishImage(wide_left_pub_, tmp, camera_name_+"wide_left", time);
            publishCInfo(info_wl_pub_, uri_wl_, c_info_wide_left_);

            triclopsColorImageToCvImage( rectCenter, tmp, "narrow_left", false);
            publishImage(narrow_left_pub_, tmp, camera_name_+"narrow_left", time);
            publishCInfo(info_nl_pub_, uri_nl_, c_info_narrow_left_);

            triclopsColorImageToCvImage( rectWideRight, tmp, "wide_right", false);
            publishImage(wide_right_pub_, tmp, camera_name_+"wide_right", time);
            publishCInfo(info_wr_pub_, uri_wr_, c_info_wide_right_);

            triclopsColorImageToCvImage( rectShortRight, tmp, "narrow_right", false);
            sensor_msgs::Image right_image = publishImage(wide_right_pub_, tmp, camera_name_+"narrow_right", time);
            publishCInfo(info_nr_pub_, uri_wl_, c_info_narrow_right_);

            //now the stereo processing
            triclopsRectify( shortTriclops_, &shortInput);
            triclopsRectify( wideTriclops_, &wideInput);
            triclopsStereo( shortTriclops_ );
            triclopsStereo( wideTriclops_ );
            TriclopsImage16 shortDisparity, wideDisparity;
            triclopsGetImage16( shortTriclops_, TriImg16_DISPARITY,
                                TriCam_REFERENCE, &shortDisparity );
            triclopsGetImage16( wideTriclops_, TriImg16_DISPARITY,
                                TriCam_REFERENCE, &wideDisparity );
            sensor_msgs::PointCloud pc;
            pc.header.seq = i;
            pc.header.stamp = ros::Time::now();
            pc.header.frame_id = "bumblebee";
            disparityToPointCloud (shortDisparity, shortTriclops_, right_image, pc);
            /*char filename[100];
            sprintf( filename, "short-disparity-%02d.pgm", i );
            printf("Disparity columns: %d rows: %d\n", shortDisparity.ncols, shortDisparity.nrows);
            triclopsSaveImage16( &shortDisparity,filename );*/
            pc_pub_.publish(pc);



            ros::spinOnce();
        }

    }


    void disparityToPointCloud(TriclopsImage16& depthImage16_, TriclopsContext& triclops_, sensor_msgs::Image& right_image, sensor_msgs::PointCloud& cloud_)
    {
        //cv::imshow("right_image", right_image);
        //cvWaitKey(2);
        unsigned short* disparityPixel = depthImage16_.data;
        //unsigned char* rectPixel = &right_image.data;

        geometry_msgs::Point32 p;

        cloud_.channels.resize(1);
        cloud_.channels[0].name = "rgb";


        for( int row=0; row<depthImage16_.nrows; ++row ){
            for( int col=0; col<depthImage16_.ncols; ++col ){
                if ( *disparityPixel < 0xFF00 && *disparityPixel>0)
                {
                    triclopsRCD16ToXYZ( triclops_, row, col, *disparityPixel, &p.y, &p.z, &p.x );
                    p.z = - p.z;
                    p.y = - p.y;
                    cloud_.points.push_back(p);

                    /*cloud_.channels[1].values.push_back(*disparityPixel);
                    cloud_.channels[2].values.push_back(col);
                    cloud_.channels[3].values.push_back(row);*/
                }
                else
                {
                    p.x = 0.0;
                    p.y = 0.0;
                    p.z = 0.0;
                }
                int rgb = (right_image.data[row*320+col+2] << 16) | (right_image.data[row*320+col+1] << 8) | right_image.data[row*320+col+0];
                float float_rgb = *(float*)&rgb;
                cloud_.channels[0].values.push_back(float_rgb);

                disparityPixel++;
                //rectPixel += 3;
            }
            //rectPixel += (right_image.step-depthImage16_.ncols*3);
        }

    void publishCInfo(ros::Publisher& pub, string uri, camera_info_manager::CameraInfoManager& manager)
    {
        sensor_msgs::CameraInfo c_info;
        manager.loadCameraInfo(uri_wl_);
        c_info = manager.getCameraInfo();
        pub.publish(c_info);

    }

    void triclopsColorImageToCvImage (TriclopsColorImage& input, cv::Mat& img, string text, bool show_image)
    {
        cv::Mat tmp_img(input.nrows, input.ncols,CV_8UC1);

        std::vector<cv::Mat> merge_img;

        tmp_img.data = (uchar*)input.blue; merge_img.push_back(tmp_img);
        tmp_img.data = (uchar*)input.green; merge_img.push_back(tmp_img);
        tmp_img.data = (uchar*)input.red; merge_img.push_back(tmp_img);
        cv::merge( merge_img, img );
        if(show_image)
        {
            cv::imshow(text, img);
            cvWaitKey(2);
        }

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
        char*        szShortCal  = (char*)short_cal_.c_str();
        char*        szWideCal   = (char*)wide_cal_.c_str();

        TriclopsError e;

        printf( "Getting TriclopsContexts from files... \n" );
        e = triclopsGetDefaultContextFromFile( &shortTriclops_,  szShortCal);
        if ( e != TriclopsErrorOk )
        {
            fprintf( stderr, "Can't get short context from file\n" );
            return false;
        }

        e = triclopsGetDefaultContextFromFile( &wideTriclops_, szWideCal);
        if ( e != TriclopsErrorOk )
        {
            fprintf( stderr, "Can't get wide context from file\n" );
            return false;
        }
        printf( "...done\n" );

        // make sure we are in subpixel mode
        triclopsSetSubpixelInterpolation( wideTriclops_, 1 );
        triclopsSetSubpixelInterpolation( shortTriclops_, 1 );

        // make sure we are only using one thread. Triclops crash with multiple threads
        triclopsSetMaxThreadCount( wideTriclops_, 1);
        triclopsSetMaxThreadCount( shortTriclops_, 1);

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

    void convertColorTriclopsInput( TriclopsInput* colorInput, unsigned char* pucRGB )
    {
        unsigned char* pucInputData = (unsigned char*) colorInput->u.rgb32BitPacked.data;
        for ( int i = 0, j = 0; i < colorInput->nrows * colorInput->ncols*3; )
        {
            // get R, G and B
            pucInputData[j+2] = pucRGB[i++];
            pucInputData[j+1] = pucRGB[i++];
            pucInputData[j] = pucRGB[i++];
            // increment the Input counter once more to skip the "U" byte
            j += 4;
        }
        return;
    }

    int writePgm( char* szFilename, unsigned char* pucBuffer, int width, int height )
    {
        FILE* stream;
        stream = fopen( szFilename, "wb" );
        if( stream == NULL)
        {
            perror( "Can't open image file" );
            return 1;
        }

        fprintf( stream, "P5\n%u %u 255\n", width, height );
        fwrite( pucBuffer, width, height, stream );
        fclose( stream );
        return 0;
    }

    int writePpm( char* szFilename, unsigned char* pucBuffer, int width, int height )
    {
        FILE* stream;
        stream = fopen( szFilename, "wb" );
        if( stream == NULL)
        {
            perror( "Can't open image file" );
            return 1;
        }

        fprintf( stream, "P6\n%u %u 255\n", width, height );
        fwrite( pucBuffer, 3*width, height, stream );
        fclose( stream );
        return 0;
    }

    void cleanup_and_exit( dc1394camera_t* camera )
    {
        dc1394_capture_stop( camera );
        dc1394_video_set_transmission( camera, DC1394_OFF );
        dc1394_camera_free( camera );
        exit( 0 );
    }
};







//=============================================================================
// MAIN
//
int main( int argc, char *argv[] )
{
    ros::init(argc, argv, "xb3");
    xb3 xb3;

    return 0;
}

