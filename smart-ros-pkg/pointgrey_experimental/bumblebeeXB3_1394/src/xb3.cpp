/*
 * xb3.cpp
 *
 *  Created on: Feb 24, 2012
 *      Author: golfcar
 */

#include "xb3.h"

using namespace xb3;

class xb3_main
{
public:

    dc1394camera_t*  camera_;
    string camera_name_, short_cal_, wide_cal_;
    PGRStereoCamera_t stereoCamera_;
    ros::NodeHandle nh_, priv_nh_;
    image_transport::ImageTransport it_;

    image_transport::Publisher wide_left_pub_, narrow_left_pub_, wide_right_pub_, narrow_right_pub_;
    image_transport::Publisher raw_left_pub_, raw_center_pub_, raw_right_pub_;
    camera_info_manager::CameraInfoManager c_info_narrow_left_, c_info_narrow_right_, c_info_wide_left_, c_info_wide_right_;
    TriclopsContext wideTriclops_, shortTriclops_;
    TriclopsInput wideInput, shortInput;
    ros::Timer timerStereo, timerPublish;
    string uri_nl_, uri_nr_, uri_wl_, uri_wr_;
    ros::Publisher info_nl_pub_, info_nr_pub_, info_wl_pub_, info_wr_pub_, narrow_pc_pub_, wide_pc_pub_;
    int pub_count_;
    ros::Time stamp_;
    xb3_main() : priv_nh_("~"), it_(nh_),  c_info_narrow_left_(ros::NodeHandle(nh_, "bumblebee/left"),"bumblebee/narrow_left"),
            c_info_narrow_right_(ros::NodeHandle(nh_,"bumblebee/right"),"bumblebee/narrow_right"),
            c_info_wide_left_(ros::NodeHandle(nh_, "bumblebee/wide_left"), "bumblebee/wide_left"),
            c_info_wide_right_(ros::NodeHandle(nh_, "bumblebee/wide_right"), "bumblebee/wide_right")
    {
        camera_name_ = string("bumblebee/");
        narrow_left_pub_ = it_.advertise(camera_name_+"left/image_raw", 1);
        wide_left_pub_ = it_.advertise(camera_name_+"wide_left/image_raw", 1);
        narrow_right_pub_ = it_.advertise(camera_name_+"right/image_raw", 1);
        wide_right_pub_ = it_.advertise(camera_name_+"wide_right/image_raw", 1);

        raw_left_pub_ = it_.advertise(camera_name_+"raw_left/image_raw", 1);
        raw_center_pub_ = it_.advertise(camera_name_+"raw_center/image_raw", 1);
        raw_right_pub_ = it_.advertise(camera_name_+"raw_right/image_raw", 1);

        info_nl_pub_ = nh_.advertise<sensor_msgs::CameraInfo>(camera_name_+"left/camera_info", 1);
        info_nr_pub_ = nh_.advertise<sensor_msgs::CameraInfo>(camera_name_+"right/camera_info", 1);
        info_wl_pub_ = nh_.advertise<sensor_msgs::CameraInfo>(camera_name_+"wide_left/camera_info", 1);
        info_wr_pub_ = nh_.advertise<sensor_msgs::CameraInfo>(camera_name_+"wide_right/camera_info", 1);

        narrow_pc_pub_ = nh_.advertise<sensor_msgs::PointCloud>(camera_name_+"triclops_narrow", 20);
        wide_pc_pub_ = nh_.advertise<sensor_msgs::PointCloud>(camera_name_+"triclops_wide", 20);

        priv_nh_.param("uri_nl", uri_nl_, string("package://bumblebeeXB3_1394/calibration/narrow_left.ini"));
        priv_nh_.param("uri_nr", uri_nr_, string("package://bumblebeeXB3_1394/calibration/narrow_right.ini"));
        priv_nh_.param("uri_wl", uri_wl_, string("package://bumblebeeXB3_1394/calibration/wide_left.ini"));
        priv_nh_.param("uri_wr", uri_wr_, string("package://bumblebeeXB3_1394/calibration/wide_right.ini"));

        c_info_narrow_left_.loadCameraInfo(uri_nl_);
        c_info_narrow_right_.loadCameraInfo(uri_nr_);
        c_info_wide_left_.loadCameraInfo(uri_wl_);
        c_info_wide_right_.loadCameraInfo(uri_wr_);

        priv_nh_.param("short_calibration_file", short_cal_, string(""));
        priv_nh_.param("wide_calibration_file", wide_cal_, string(""));
        pub_count_ = 0;
        if(!initialize_camera(camera_, stereoCamera_)) return;
        start_process();
        ros::spin();
    }

    ~xb3_main()
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

    void start_process()
    {

        unsigned int   nBufferSize;
        unsigned char* pucDeInterlacedBuffer;
        unsigned char* pucRGBBuffer;
        unsigned char* pucGreenBuffer;
        unsigned char* pucRightRGB;
        unsigned char* pucLeftRGB;
        unsigned char* pucCenterRGB;
        TriclopsInput  colorInput;
        // buffer for the 3 color images color processed and stacked
        pucRGBBuffer      = NULL;
        // buffer for the green channel that approximates the mono signal
        pucGreenBuffer    = NULL;
        pucRightRGB       = NULL;
        pucLeftRGB        = NULL;
        pucCenterRGB      = NULL;
        // size of buffer for all images at mono8
        nBufferSize = stereoCamera_.nRows *
                stereoCamera_.nCols *
                stereoCamera_.nBytesPerPixel;

        // allocate a buffer to hold the de-interleaved images
        pucDeInterlacedBuffer = new unsigned char[ nBufferSize ];

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
        while(ros::ok())
        {
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
            stamp_ = time;
            sensor_msgs::Image image_pub;
            cv::Size image_size(320, 240);

            cv::Mat original_left(stereoCamera_.nRows, stereoCamera_.nCols,CV_8UC3);
            original_left.data = (uchar*)pucCenterRGB;
            cv::cvtColor(original_left, original_left, CV_BGR2RGB);
            publishImage(raw_center_pub_, original_left, camera_name_, time);
            cv::resize(original_left, original_left, image_size);
            image_pub = publishImage(narrow_left_pub_, original_left, camera_name_+"narrow", time);
            publishCInfo(info_nl_pub_, c_info_narrow_left_, image_pub);

            cv::Mat original_wide_left(stereoCamera_.nRows, stereoCamera_.nCols,CV_8UC3);
            original_wide_left.data = (uchar*)pucLeftRGB;
            cv::cvtColor(original_wide_left, original_wide_left, CV_BGR2RGB);
            publishImage(raw_left_pub_, original_wide_left, camera_name_, time);
            cv::resize(original_wide_left, original_wide_left, image_size);
            image_pub = publishImage(wide_left_pub_, original_wide_left, camera_name_+"wide", time);
            publishCInfo(info_wl_pub_, c_info_wide_left_, image_pub);

            cv::Mat original_right(stereoCamera_.nRows, stereoCamera_.nCols,CV_8UC3);
            original_right.data = (uchar*)pucRightRGB;
            cv::Mat original_right_rgb;
            cv::cvtColor(original_right, original_right_rgb, CV_BGR2RGB);
            publishImage(raw_right_pub_, original_right_rgb, camera_name_, time);
            cv::resize(original_right_rgb, original_right_rgb, image_size);
            image_pub = publishImage(narrow_right_pub_, original_right_rgb, camera_name_+"narrow", time);
            publishCInfo(info_nr_pub_, c_info_narrow_right_, image_pub);
            image_pub = publishImage(wide_right_pub_, original_right_rgb, camera_name_+"wide", time);
            publishCInfo(info_wr_pub_, c_info_wide_right_, image_pub);
            pub_count_++;
        }
    }




};


int main( int argc, char *argv[] )
{
    ros::init(argc, argv, "xb3");
    xb3_main xb3;

    return 0;
}

