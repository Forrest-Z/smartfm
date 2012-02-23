#include <stdio.h>
#include <sys/types.h>
#include <dirent.h>
#include <errno.h>

#include <iostream>
#include <sstream>
#include <vector>

#include <cv.h>
#include <highgui.h>

#include "MovingObjectDetector.h"
#include "BlobExtractor.h"
#include "BlobFilter.h"

using namespace std;
using namespace cv;

Mat background, frame;

int diff_threshold_slider = 70;
const int diff_threshold_max = 255;

int size_threshold_slider = 500;
const int size_threshold_max = 5000;

int dilate_size_slider = 40;
const int dilate_size_max = 100;

int erode_size_slider = 40;
const int erode_size_max = 100;

int img_number = 0;
vector< pair<string,string> > images;

void getImages(string dir);
void process(int, void *);
void reload(int);


int main( int argc, char **argv )
{
    namedWindow("car detection", CV_WINDOW_NORMAL);
    namedWindow("diff", CV_WINDOW_NORMAL);

    getImages("images");

    createTrackbar("diff threshold", "diff", &diff_threshold_slider, diff_threshold_max, process);
    createTrackbar("dilate size", "diff", &dilate_size_slider, dilate_size_max, process);
    createTrackbar("erode size", "diff", &erode_size_slider, erode_size_max, process);
    createTrackbar("size threshold", "car detection", &size_threshold_slider, size_threshold_max, process);

    reload(0);

    while(1) {
        int key = waitKey(0);
        if( key=='q' ) break;
        else if( key=='n' ) reload(++img_number);
        else if( key=='b' ) reload(--img_number);
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

void reload(int n)
{
    if( n >= (int)images.size() ) img_number = images.size()-1;
    if( n < 0 ) img_number = 0;

    background = imread(images[img_number].first, 1);
    frame = imread(images[img_number].second, 1);

    process(0,0);
}

void process(int, void *)
{
    MovingObjectDetector detector;
    detector.diff_thresh = diff_threshold_slider;
    detector.dilate_size = dilate_size_slider;
    detector.erode_size = erode_size_slider;

    detector.diff(frame, background);

    BlobExtractor blob_extractor;
    blob_extractor.extract(detector.diffImg, 0);

    Mat display = frame.clone();

    BlobFilterArea areaFilter(size_threshold_slider);

    for( unsigned i=0; i<blob_extractor.blobs.size(); i++ )
    {
        const Blob & blob = blob_extractor.blobs[i];
        if( areaFilter.check(blob) )
        {
            blob.drawContour(display, CV_RGB(255,0,0));
            blob.drawCentroid(display, CV_RGB(255,0,0));
        }
    }

    imshow("car detection", display);
    imshow("diff", detector.diffImg);
}

