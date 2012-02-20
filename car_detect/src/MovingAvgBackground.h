/*
 * MovingAvgBackground.h
 *
 *  Created on: Jan 31, 2012
 *      Author: brice
 */

#ifndef MOVINGAVGBACKGROUND_H_
#define MOVINGAVGBACKGROUND_H_

#include <deque>
#include <cv.h>

class MovingAvgBackground
{
protected:
    cv::Mat background;

public:
    virtual void add(cv::Mat) = 0;
    cv::Mat getBackgroundImage() const { return background; }
};

class ManualMovingAvgBackground : public MovingAvgBackground
{
    unsigned size; //number of images to consider for the background
    unsigned skip; //number of images to skip between two images used for the background
    unsigned nRefresh; //refresh the background after than many images have been added to it

    unsigned count; //number of images skipped
    unsigned refreshCount; //number of images used for background since last refresh

    std::deque<cv::Mat> images;

    void refresh();

public:
    ManualMovingAvgBackground(unsigned size);
    void setSize(unsigned n) { size=n; images.resize(n); }
    void setSkip(unsigned n) { skip=n; }
    void setRefresh(unsigned n) { nRefresh=n; }
    void add(cv::Mat);
};

class WeightedMovingAvgBackground : public MovingAvgBackground
{
    bool initialized;
    bool weight;

public:
    WeightedMovingAvgBackground(float w);
    void add(cv::Mat);
};

#endif /* MOVINGAVGBACKGROUND_H_ */
