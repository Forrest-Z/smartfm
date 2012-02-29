/*
 * triclopsStereo.cpp
 *
 *  Created on: Feb 28, 2012
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
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <image_transport/subscriber_filter.h>
using namespace std;

class triclops_stereo
{
public:

    string short_cal_, wide_cal_;
    TriclopsContext wideTriclops_, shortTriclops_;
    dc1394camera_t*  camera_;
    string camera_name_;
    PGRStereoCamera_t stereoCamera_;
    ros::NodeHandle nh_, priv_nh_;
    image_transport::ImageTransport it_;
    int i;
    //image_transport::Publisher wide_left_pub_, narrow_left_pub_, wide_right_pub_, narrow_right_pub_;
    ros::Publisher narrow_pc_pub_, wide_pc_pub_;
    image_transport::SubscriberFilter left_sub_, center_sub_, right_sub_;
    triclops_stereo() : priv_nh_("~"), it_(nh_)
    {

        camera_name_ = string("bumblebee/");
        priv_nh_.param("short_calibration_file", short_cal_, string(""));
        priv_nh_.param("wide_calibration_file", wide_cal_, string(""));
        if(!initialize()) return;

        left_sub_.subscribe(it_,camera_name_+"raw_left/image_raw",20);
        center_sub_.subscribe(it_,camera_name_+"raw_center/image_raw",20);
        right_sub_.subscribe(it_,camera_name_+"raw_right/image_raw",20);

        narrow_pc_pub_ = nh_.advertise<sensor_msgs::PointCloud>(camera_name_+"triclops_narrow", 20);
        wide_pc_pub_ = nh_.advertise<sensor_msgs::PointCloud>(camera_name_+"triclops_wide", 20);
        message_filters::TimeSynchronizer<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::Image> sync(left_sub_, center_sub_, right_sub_, 10);
        sync.registerCallback(boost::bind(&triclops_stereo::callback, this, _1, _2, _3));
        i=0;
        //publish_image();
        ros::spin();
    }

    ~triclops_stereo(){};

private:
    void callback(const sensor_msgs::ImageConstPtr& left_image, const sensor_msgs::ImageConstPtr& center_image, const sensor_msgs::ImageConstPtr& right_image)
    {
        ROS_INFO("Callback received!");
        i++;
        unsigned char* pucRightRGB       = NULL;
        unsigned char* pucLeftRGB        = NULL;
        unsigned char* pucCenterRGB      = NULL;

        cv_bridge::CvImageConstPtr left_cv_ptr, center_cv_ptr, right_cv_ptr;
        left_cv_ptr = cv_bridge::toCvShare(left_image, sensor_msgs::image_encodings::RGB8);
        center_cv_ptr = cv_bridge::toCvShare(center_image, sensor_msgs::image_encodings::RGB8);
        right_cv_ptr = cv_bridge::toCvShare(right_image, sensor_msgs::image_encodings::RGB8);

        pucRightRGB = (unsigned char*)right_cv_ptr->image.data;
        pucLeftRGB = (unsigned char*)left_cv_ptr->image.data;
        pucCenterRGB = (unsigned char*)center_cv_ptr->image.data;

        // copy the RGB buffer into an input structure
        TriclopsInput  colorInput;
        TriclopsColorImage rectLeft, rectCenter, rectShortRight, rectWideRight;
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

        cv::Mat tmp, right;
        triclopsColorImageToCvImage( rectLeft, tmp, "wide_left", false);
        triclopsColorImageToCvImage( rectCenter, tmp, "narrow_left", false);
        triclopsColorImageToCvImage( rectWideRight, tmp, "wide_right", false);
        triclopsColorImageToCvImage( rectShortRight, right, "narrow_right", false);


        //now the stereo processing
        TriclopsInput wideInput, shortInput;

        wideInput.inputType     = TriInp_RGB;
        wideInput.nrows         = erow;
        wideInput.ncols         = ecol;
        wideInput.rowinc        = ecol;
        wideInput.u.rgb.red     = pucRightRGB;
        wideInput.u.rgb.green   = pucLeftRGB;
        wideInput.u.rgb.blue    = pucLeftRGB;

        shortInput.inputType    = TriInp_RGB;
        shortInput.nrows        = erow;
        shortInput.ncols        = ecol;
        shortInput.rowinc       = ecol;
        shortInput.u.rgb.red    = pucRightRGB;
        shortInput.u.rgb.green  = pucCenterRGB;
        shortInput.u.rgb.blue   = pucCenterRGB;

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
        narrow_pc.header.seq = i;
        narrow_pc.header.stamp = left_image->header.stamp;
        narrow_pc.header.frame_id = "bumblebee";
        disparityToPointCloud (shortDisparity, shortTriclops_, right, narrow_pc);
        narrow_pc_pub_.publish(narrow_pc);

        //and wide point cloud
        sensor_msgs::PointCloud wide_pc;
        wide_pc.header.seq = i;
        wide_pc.header.stamp = left_image->header.stamp;
        wide_pc.header.frame_id = "bumblebee";
        disparityToPointCloud (wideDisparity, wideTriclops_, right, wide_pc);
        wide_pc_pub_.publish(wide_pc);


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

    void publishCInfo(ros::Publisher& pub, string uri, camera_info_manager::CameraInfoManager& manager, sensor_msgs::Image img)
    {
        sensor_msgs::CameraInfo c_info;
        manager.loadCameraInfo(uri);
        assert(manager.isCalibrated());
        c_info = manager.getCameraInfo();
        c_info.header = img.header;
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
    ros::init(argc, argv, "triclopsStereo");
    triclops_stereo TriclopsStereo;

    return 0;
}

