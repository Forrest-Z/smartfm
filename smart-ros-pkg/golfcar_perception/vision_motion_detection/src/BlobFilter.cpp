#include <vision_motion_detection/BlobFilter.h>

#include <stdexcept>

std::vector<Blob> BlobFilter::filter(const std::vector<Blob> & blobs) const
{
    std::vector<Blob> res;
    for( unsigned i=0; i<blobs.size(); i++ )
        if( check(blobs[i]) )
            res.push_back(blobs[i]);
    return res;
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
    return area_.empty() || area_.is_inside(blob.centroid);
}

