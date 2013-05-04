/*
 * triclopsStereo.cpp
 *
 *  Created on: Feb 28, 2012
 *      Author: golfcar
 */

#include "xb3.h"

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
    int i;
    //image_transport::Publisher wide_left_pub_, narrow_left_pub_, wide_right_pub_, narrow_right_pub_;
    ros::Publisher narrow_pc_pub_, wide_pc_pub_;
    image_transport::SubscriberFilter left_sub_, center_sub_, right_sub_;
    triclops_rect() : priv_nh_("~"), it_(nh_)
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

