/**
 * Reads a movie file, publishes frames and extracted background. Possible to
 * fast forward and slow down, and save the current image and background image
 * to a file. Also publishes clock messages.
 */

#include <ros/ros.h>
#include <ros/time.h>

#include <rosgraph_msgs/Clock.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

#include "MovingObjectDetector.h"

#include <opencv2/highgui/highgui.hpp>

using namespace std;


string format_time_frame(double t);
string format_time_file(double t);



int main( int argc, char **argv )
{
    ros::init(argc, argv, "play_video");

    // attempt to open the movie file
    if( argc<=1 ) {
        ROS_FATAL("You must provide a movie file as argument");
        return 1;
    }

    cv::VideoCapture cap(argv[1]);
    if ( !cap.isOpened() ) {
        ROS_FATAL("Could not open %s", argv[1]);
        return 1;
    }

    cout <<"Press F (fast) to increase play back speed, S (slow) to slow down, "
            <<"D to return to normal speed, C to save the current and "
            <<"background frame in current directory." <<endl <<endl;

    double fps = cap.get(CV_CAP_PROP_FPS);
    int speedFactor = 0;
    double inv_fps = 1.0/fps;
    double time = 0;
    bool quit = false;

    ros::NodeHandle n;
    image_transport::ImageTransport it(n);
    ros::Publisher clockPub = n.advertise<rosgraph_msgs::Clock>("/clock", 10);
    image_transport::Publisher framePub = it.advertise("camera", 10);
    image_transport::Publisher backgroundPub = it.advertise("background", 10);

    cv_bridge::CvImage frame;
    frame.header.frame_id = "camera";
    frame.header.seq = 0;
    frame.header.stamp.fromSec(0);
    frame.encoding = "bgr8";

    cv_bridge::CvImage backgroundImg;
    backgroundImg.header.frame_id = "camera";
    backgroundImg.header.seq = 0;
    backgroundImg.header.stamp.fromSec(0);
    backgroundImg.encoding = "bgr8";

    // background extraction
    Background background;
    ros::NodeHandle nh("~");
    nh.param("alpha", background.alpha, 0.005);
    ROS_INFO("background alpha value: %f", background.alpha);

    cv::namedWindow("car detect", CV_WINDOW_NORMAL);

    while( ! quit )
    {

        // always capture one frame. If fast forwarding (i.e. speedFactor>0)
        // then capture more. Update background each time.
        // cannot rely on cap.get(CV_CAP_PROP_POS_MSEC) to get the time....
        for( unsigned i=0; true; i++ ) {
            cap >> frame.image;
            background.add(frame.image);
            time += inv_fps;
            if( speedFactor<=0 || i>=pow(speedFactor,2) )
                break;
        }


        rosgraph_msgs::Clock clockmsg;
        clockmsg.clock.fromSec(time);
        clockPub.publish(clockmsg);

        frame.header.seq++;
        frame.header.stamp.fromSec(time);
        framePub.publish(frame.toImageMsg());

        backgroundImg.header.seq++;
        backgroundImg.header.stamp.fromSec(time);
        backgroundImg.image = background.getImg();
        backgroundPub.publish(backgroundImg.toImageMsg());

        // listen to user input; control playback speed
        int wait = speedFactor<0 ? pow(2,-speedFactor) : 1;
        while( ! quit && wait-- > 0 )
        {
            // display the current frame, with time and speed information
            cv::Mat displayImg = frame.image.clone();
            stringstream ss;
            ss << format_time_frame(time) <<", speed=";
            if( speedFactor<0 ) ss <<"1/" <<pow(2,-speedFactor);
            else ss <<"x" <<pow(2,speedFactor);
            cv::putText(displayImg, ss.str(), cv::Point(30,50),
                            cv::FONT_HERSHEY_COMPLEX_SMALL, 2,
                            CV_RGB(0,255,555), 1, CV_AA);
            cv::imshow("car detect", displayImg);

            // get user input
            int key = cv::waitKey((int)(inv_fps*1000));

            if( key=='q' ) quit = true;
            else if( key=='f' ) {
                if( speedFactor<0 ) {
                    wait -= pow(2,-speedFactor-1);
                }
                speedFactor++;
            }
            else if( key=='s' ) speedFactor--;
            else if( key=='d' ) speedFactor = wait = 0;
            else if( key=='c' ) {
                string prefix = format_time_file(time);
                cv::imwrite(prefix + "_background.png", backgroundImg.image);
                cv::imwrite(prefix +"_frame.png", frame.image);
            }
        }
    }

    return 0;
}

string format_time_frame(double t)
{
    int mins = t/60;
    t -= mins*60;
    int secs = t;
    t -= secs;
    int ms = t*1000;
    char buf[100];
    snprintf(buf, 99, "%02d:%02d.%03d", mins, secs, ms);
    return string(buf);
}

string format_time_file(double t)
{
    int mins = t/60;
    t -= mins*60;
    int secs = t;
    t -= secs;
    int ms = t*1000;
    char buf[100];
    snprintf(buf, 99, "%02d_%02d_%03d", mins, secs, ms);
    return string(buf);
}

