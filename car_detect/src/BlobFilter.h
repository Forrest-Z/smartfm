#ifndef BLOBFILTER_H_
#define BLOBFILTER_H_

#include <vector>

#include <cv.h>

#include "BlobExtractor.h"


class BlobFilter
{
public:
    virtual bool check(const Blob & blob) = 0;
    void filter(std::vector<Blob> & blobs);
};


class BlobFilterArea : public BlobFilter
{
public:
    double threshold_;
    BlobFilterArea();
    explicit BlobFilterArea(double threshold);

    /// Returns true if the area of the blob is greater than threshold_
    virtual bool check(const Blob & blob);
};

class BlobFilterPosition : public BlobFilter
{
public:
    /// if centroid falls within that area then accept the blob
    std::vector<cv::Point> area;

    virtual bool check(const Blob & blob);
};

#endif /* BLOBFILTER_H_ */
