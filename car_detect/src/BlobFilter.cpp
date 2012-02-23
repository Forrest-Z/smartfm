#include "BlobFilter.h"

#include <stdexcept>

void BlobFilter::filter(std::vector<Blob> & blobs)
{
    std::vector<unsigned> idx;
    for( unsigned i=0; i<blobs.size(); i++ )
        if( check(blobs[i]) )
            idx.push_back(i);

    if( idx.size()==blobs.size() )
        return;

    std::vector<Blob> tmp;
    for( unsigned i=0; i<idx.size(); i++ )
        tmp.push_back(blobs[idx[i]]);

    blobs = tmp;
}

BlobFilterArea::BlobFilterArea() : threshold_(0) { }

BlobFilterArea::BlobFilterArea(double threshold) : threshold_(threshold) { }

bool BlobFilterArea::check(const Blob & blob)
{
    return cv::contourArea(cv::Mat(blob.contour)) > threshold_;
}


bool BlobFilterPosition::check(const Blob & blob)
{
    throw std::logic_error("not implemented");
    return true;
}

