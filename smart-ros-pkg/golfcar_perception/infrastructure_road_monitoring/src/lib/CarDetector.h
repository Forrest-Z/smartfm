#ifndef CARDETECTOR_H_
#define CARDETECTOR_H_

#include <stdio.h>

#include <iostream>
#include <vector>
#include <sstream>

#include <cv.h>
#include <highgui.h>

#include <boost/bind.hpp>

#include "MovingObjectDetector.h"
#include "BlobExtractor.h"
#include "BlobFilter.h"
#include "BlobTracker.h"


class CarDetector
{
public:

    Background background;
    MovingObjectDetector detector;
    BlobExtractor blob_extractor;
    BlobFilterArea areaFilter;
    TrackMatcherNNT trackMatcher;
    BlobTracker tracker;

    int frame_width, frame_height;


    CarDetector();

    double adaptiveThresholdFn(const Track & track, const Blob & b);

    /// Update the detector with the frame (except background)
    bool update(cv::Mat frame, double time);

    void display(cv::Mat & displayImg);
};

#endif /* CARDETECTOR_H_ */
