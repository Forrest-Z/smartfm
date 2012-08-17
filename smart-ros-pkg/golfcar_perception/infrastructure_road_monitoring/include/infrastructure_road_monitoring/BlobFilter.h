#ifndef BLOBFILTER_H_
#define BLOBFILTER_H_

#include <vector>

#include <opencv2/opencv.hpp>

#include <infrastructure_road_monitoring/data_types.h>
#include <infrastructure_road_monitoring/Polygon.h>


class BlobFilter
{
public:
    virtual bool check(const Blob & blob) const = 0;
    void filter(std::vector<Blob> & blobs) const;
    infrastructure_road_monitoring::Blobs filter(const infrastructure_road_monitoring::Blobs &) const;
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
