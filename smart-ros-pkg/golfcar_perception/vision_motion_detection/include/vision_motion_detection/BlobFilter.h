#ifndef BLOBFILTER_H_
#define BLOBFILTER_H_

#include <vector>

#include <opencv2/opencv.hpp>

#include <vision_motion_detection/data_types.h>
#include <vision_motion_detection/Polygon.h>


class BlobFilter
{
public:
    virtual ~BlobFilter() { }
    virtual bool check(const Blob & blob) const = 0;
    void filter(std::vector<Blob> & blobs) const;
    vision_motion_detection::Blobs filter(const vision_motion_detection::Blobs &) const;
};


class BlobFilterArea : public BlobFilter
{
public:
    double threshold_;
    BlobFilterArea();
    explicit BlobFilterArea(double threshold);

    /// Returns true if the area of the blob is greater than threshold_
    virtual bool check(const Blob & blob) const;
};

class BlobFilterPosition : public BlobFilter
{
public:
    /// if centroid falls within that area then accept the blob
    Polygon area;

    virtual bool check(const Blob & blob) const;
};

#endif /* BLOBFILTER_H_ */
