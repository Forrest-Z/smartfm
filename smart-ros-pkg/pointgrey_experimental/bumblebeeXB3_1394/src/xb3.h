/*
 * xb3.h
 *
 *  Created on: Feb 29, 2012
 *      Author: golfcar
 */

#ifndef XB3_H_
#define XB3_H_

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

namespace xb3
{

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
    }


    void disparityToPointCloud(TriclopsImage16& depthImage16_, TriclopsContext& triclops_, cv::Mat& right_image, sensor_msgs::PointCloud& cloud_)
    {

        unsigned short* disparityPixel = depthImage16_.data;
        //unsigned char* rectPixel = &right_image.data;

        geometry_msgs::Point32 p;

        cloud_.channels.resize(1);
        cloud_.channels[0].name = "rgb";
        cloud_.points.clear();
        //only process CV_8U with 3 channel
        assert( right_image.type() == CV_8UC3);

        vector<cv::Mat> right_image_3ch;
        cv::split(right_image, right_image_3ch);
        uchar* blue = right_image_3ch[0].data;
        uchar* green = right_image_3ch[1].data;
        uchar* red = right_image_3ch[2].data;
        assert(right_image.cols == depthImage16_.ncols && right_image.rows == depthImage16_.nrows);
        for( int row=0; row<depthImage16_.nrows; ++row ){
            for( int col=0; col<depthImage16_.ncols; ++col ){
                if ( *disparityPixel < 0xFF00 && *disparityPixel>0)
                {
                    triclopsRCD16ToXYZ( triclops_, row, col, *disparityPixel, &p.y, &p.z, &p.x );
                    int array_ptr = row*right_image.cols+col;
                    p.z = - p.z;
                    p.y = - p.y;
                    cloud_.points.push_back(p);
                    int rgb = (red[array_ptr] << 16) | (green[array_ptr] << 8) | blue[array_ptr];
                    float float_rgb = *reinterpret_cast<float*>(&rgb);
                    cloud_.channels[0].values.push_back(float_rgb);
                }
                else
                {
                    p.x = 0.0;
                    p.y = 0.0;
                    p.z = 0.0;
                }


                disparityPixel++;
            }
        }
    }

    void cleanup_and_exit( dc1394camera_t* camera )
    {
        dc1394_video_set_transmission( camera, DC1394_OFF );
        dc1394_capture_stop( camera );
        dc1394_camera_free( camera );
        exit( 0 );
    }

    bool initialize_camera(PGRStereoCamera_t& stereoCamera_)
    {
        //===================================================================
        // Find cameras on the 1394 buses
        dc1394_t * d;
        dc1394camera_list_t * list;
        dc1394error_t    err;
        dc1394camera_t* camera_;
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
//            dc1394_cleanup_iso_channels_and_bandwidth(camera_);
            //camera_ = dc1394_camera_new(d, list->ids[nThisCam].guid);
            if(!camera_)
            {
                printf("Failed to initialize camera with guid %lx", list->ids[nThisCam].guid);
                continue;
            }

            ROS_INFO("Resetting 1394 bus");
            if (DC1394_SUCCESS != dc1394_camera_reset(camera_))
            {
                cleanup_and_exit( camera_ );
                fprintf( stderr, "Unable to reset camera.\n");
                return 0;
            }
            ROS_INFO("Reset Complete");

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

    void publishCInfo(ros::Publisher& pub, camera_info_manager::CameraInfoManager& manager, sensor_msgs::Image img)
    {
        sensor_msgs::CameraInfo c_info;
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

    bool getTriclopsContext(string short_cal_, string wide_cal_, TriclopsContext& shortTriclops_, TriclopsContext& wideTriclops_)
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

        return true;
    }
/*

        // size of buffer for all images at mono8
        nBufferSize = stereoCamera_.nRows *
                stereoCamera_.nCols *
                stereoCamera_.nBytesPerPixel;

        // allocate a buffer to hold the de-interleaved images
        pucDeInterlacedBuffer = new unsigned char[ nBufferSize ];



        // define all the buffers you could ever want, color or mono
        // leave allocation till later
        // color buffers
        // buffer for the 3 color images color processed and stacked
        pucRGBBuffer      = NULL;
        // buffer for the green channel that approximates the mono signal
        pucGreenBuffer    = NULL;
        pucRightRGB       = NULL;
        pucLeftRGB        = NULL;
        pucCenterRGB      = NULL;

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
        return true;
    }

 void stereoProcess(const ros::TimerEvent& event)
    {
        if(pub_count_==0) return;
        triclopsRectify( shortTriclops_, &shortInput);
        triclopsRectify( wideTriclops_, &wideInput);
        triclopsStereo( shortTriclops_ );
        triclopsStereo( wideTriclops_ );
        TriclopsImage16 shortDisparity, wideDisparity;
        triclopsGetImage16( shortTriclops_, TriImg16_DISPARITY,
                            TriCam_REFERENCE, &shortDisparity );
        triclopsGetImage16( wideTriclops_, TriImg16_DISPARITY,
                            TriCam_REFERENCE, &wideDisparity );

        //now process and publish narrow point cloud
        sensor_msgs::PointCloud narrow_pc;
        TriclopsColorImage rectRight;
        convertColorTriclopsInput( &colorInput, pucRightRGB );
        triclopsRectifyColorImage( shortTriclops_, TriCam_RIGHT, &colorInput, &rectRight);
        cv::Mat right;
        triclopsColorImageToCvImage( rectRight, right, "right", true);
        narrow_pc.header.stamp = stamp_;
        narrow_pc.header.frame_id = "xb3_base";
        disparityToPointCloud (shortDisparity, shortTriclops_, right, narrow_pc);
        narrow_pc_pub_.publish(narrow_pc);

        //and wide point cloud
        sensor_msgs::PointCloud wide_pc;
        wide_pc.header.stamp = stamp_;
        wide_pc.header.frame_id = "xb3_base";
        disparityToPointCloud (wideDisparity, wideTriclops_, right, wide_pc);
        wide_pc_pub_.publish(wide_pc);
    }

*/
};
#endif /* XB3_H_ */
