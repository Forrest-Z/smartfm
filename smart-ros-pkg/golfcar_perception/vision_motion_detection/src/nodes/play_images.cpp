/**
 * Reads background and frames from an image directory and publishes them. This
 * allows to test some static aspects of blob detection.
 */

#include <stdio.h>
#include <sys/types.h>
#include <dirent.h>
#include <errno.h>

#include <ros/ros.h>

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>

using namespace std;

int img_number = 0;
vector< pair<string,string> > images;

image_transport::Publisher framePub, backgroundPub;
cv_bridge::CvImage frameImg, backgroundImg;


void getImages(string dir);
void reload(int, void *);

void timerCallback(const ros::WallTimerEvent & e)
{
    frameImg.header.stamp = backgroundImg.header.stamp = ros::Time::now();
    frameImg.header.seq++;
    backgroundImg.header.seq++;
    framePub.publish( frameImg.toImageMsg() );
    backgroundPub.publish( backgroundImg.toImageMsg() );
}


int main( int argc, char **argv )
{
    ros::init(argc, argv, "play_images");

    if( argc<=1 ) {
        ROS_FATAL("You need to provide the image directory as argument.");
        return 1;
    }

    getImages(argv[1]);

    cv::namedWindow("source image", CV_WINDOW_NORMAL);
    cv::createTrackbar("image", "source image", &img_number, images.size(), reload);

    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    framePub = it.advertise("camera", 10);
    backgroundPub = it.advertise("background", 10);

    frameImg.header.frame_id = "camera";
    frameImg.header.seq = 0;
    frameImg.header.stamp.fromSec(0);
    frameImg.encoding = "bgr8";

    backgroundImg.header = frameImg.header;
    backgroundImg.encoding = frameImg.encoding;

    reload(0, 0);

    ros::WallTimer timer = nh.createWallTimer(ros::WallDuration(0.25), timerCallback);


    while(1) {
        int key = cv::waitKey(30);
        if( key=='q' ) break;
        else if( key=='n' ) {
            img_number++;
            reload(img_number, 0);
            cv::setTrackbarPos("image", "source image", img_number);
        }
        else if( key=='b' ) {
            img_number--;
            reload(img_number, 0);
            cv::setTrackbarPos("image", "source image", img_number);
        }
        ros::spinOnce();
    }


    return 0;
}





vector<string> listfiles(string dir)
{
    DIR *dp;
    if((dp  = opendir(dir.c_str())) == NULL) {
        stringstream err;
        err << "Error(" << errno << ") opening " << dir;
        throw runtime_error(err.str());
    }

    vector<string> files;
    struct dirent *dirp;
    while ((dirp = readdir(dp)) != NULL) {
        if( dirp->d_type == DT_REG ) {
            //cout <<dirp->d_name <<endl;
            files.push_back(dir + '/' + dirp->d_name);
        }
    }

    closedir(dp);
    return files;
}

void getImages(string dir)
{
    vector<string> files = listfiles(dir);
    sort(files.begin(), files.end());
    for( unsigned i=0; i<files.size()-1; i+=2 ) {
        images.push_back( pair<string,string>(files[i],files[i+1]) );
        //cout <<i/2 <<": " <<files[i] <<", " <<files[i+1] <<endl;
    }
}

void reload(int n, void *)
{
    if( n >= (int)images.size() ) img_number = images.size()-1;
    if( n < 0 ) img_number = 0;

    backgroundImg.image = cv::imread(images[img_number].first, 1);
    frameImg.image = cv::imread(images[img_number].second, 1);

    cv::imshow("source image", frameImg.image);
}
