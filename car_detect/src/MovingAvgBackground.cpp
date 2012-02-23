/*
 * MovingAvgBackground.cpp
 *
 *  Created on: Jan 31, 2012
 *      Author: brice
 */

#include "MovingAvgBackground.h"

#include <iostream>
using namespace std;

using namespace cv;

ManualMovingAvgBackground::ManualMovingAvgBackground(unsigned size)
{
    this->size = size;
    skip = 24;
    nRefresh = 1;
    count = 0;
    refreshCount = 0;
}

void ManualMovingAvgBackground::refresh()
{
    float n = images.size();
    background = images.front().clone() / n;
    for( unsigned i=1; i<n; i++ )
        background += images[i]/n;

}

void ManualMovingAvgBackground::add(cv::Mat image)
{
    if( images.empty() ) {
        background = image.clone();
        images.push_back(image);
        return;
    }

    if( ++count < skip ) return;

    // using an incremental algorithm for computing the average, so that we
    // do not have to recompute the sum each time.

    images.push_back(image);
    float n = images.size();
    background *= (n-1)/n;
    background += image/n;
    count = 0;

    if( images.size() < size ) {
        cout <<"Added one image: history size is " <<images.size() <<endl;
        return;
    }

    if( ++refreshCount >= nRefresh ) {
        cout <<"Refreshing" <<endl;
        refreshCount = 0;
        refresh();
    }
    else {
        cout <<"Refresh count = " <<refreshCount <<endl;
        background *= n/(n-1);
        background -= images.front()/(n-1);
    }
    images.pop_front();
}


WeightedMovingAvgBackground::WeightedMovingAvgBackground(float w)
: initialized(false), weight(w)
{
}

void WeightedMovingAvgBackground::add(cv::Mat image)
{
    if( ! initialized ) {
        background = image.clone();
        initialized = true;
        return;
    }

    //cout <<"image.depth()=" <<image.depth() <<" background.depth()=" <<background.depth() <<endl;

    // this does not work: format not supported ...
    //accumulateWeighted(image*1.0, background, weight);

    // this is supposed to be the algo for accumulateWeighted, but it does not
    // work: background is always equals to the last picture...
    background = background*(1.0-weight) + image*weight;
}
