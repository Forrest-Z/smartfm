#include <vision_motion_detection/BlobFilter.h>

#include <stdexcept>

void BlobFilter::filter(std::vector<Blob> & blobs) const
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

vision_motion_detection::Blobs
BlobFilter::filter(const vision_motion_detection::Blobs & inmsg) const
{
    vision_motion_detection::Blobs outmsg;
    outmsg.header = inmsg.header;
    for(unsigned i=0; i<inmsg.blobs.size(); i++)
    {
        Blob blob = blobMsgToData(inmsg.blobs[i], inmsg.header.stamp.toSec());
        if( this->check(blob) )
            outmsg.blobs.push_back( inmsg.blobs[i] );
    }
    return outmsg;
}

BlobFilterArea::BlobFilterArea() : threshold_(0) { }

BlobFilterArea::BlobFilterArea(double threshold) : threshold_(threshold) { }

bool BlobFilterArea::check(const Blob & blob) const
{
    return cv::contourArea(cv::Mat(blob.contour)) > threshold_;
}


bool BlobFilterPosition::check(const Blob & blob) const
{
    throw std::logic_error("not implemented");
    return true;
}

