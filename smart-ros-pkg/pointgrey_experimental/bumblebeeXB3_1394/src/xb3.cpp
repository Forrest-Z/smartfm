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

#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <image_transport/image_transport.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv/cv.h>
#include <opencv2/highgui/highgui.hpp>

#include <camera_info_manager/camera_info_manager.h>

using namespace std;

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

            cv::Mat tmp;
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
            publishImage(wide_right_pub_, tmp, camera_name_+"narrow_right", time);
            publishCInfo(info_nr_pub_, uri_wl_, c_info_narrow_right_);
            ros::spinOnce();
        }

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
    void publishImage(const image_transport::Publisher& pub, cv::Mat& img, string frame_id, const ros::Time& stamp)
    {
        sensor_msgs::Image image;
        cv_bridge::CvImage cvImage;
        cvImage.image = img;
        cvImage.toImageMsg(image);
        image.encoding = sensor_msgs::image_encodings::BGR8;
        image.header.stamp = stamp;
        image.header.frame_id = frame_id;
        pub.publish(image);
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

        if (DC1394_SUCCESS != dc1394_camera_reset(camera_))
        {
            cleanup_and_exit( camera_ );
            fprintf( stderr, "Unable to reset camera.\n");
            return 0;
        }


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

