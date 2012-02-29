#ifndef BLOB_H_
#define BLOB_H_

#include <vector>
#include <cv.h>

class Blob
{
public:
    std::vector<cv::Point> contour;
    cv::Point centroid;
    double timestamp; ///< time in seconds when this blob was extracted

    Blob();

    void drawContour(cv::Mat & displayFrame, cv::Scalar color) const;
    void drawCentroid(cv::Mat & displayFrame, cv::Scalar color, int radius=5, int thickness=-1) const;
};

#endif /* BLOB_H_ */
