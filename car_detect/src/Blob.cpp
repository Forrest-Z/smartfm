#include "Blob.h"


Blob::Blob() : timestamp(0.0)
{

}

void Blob::drawContour(cv::Mat & displayFrame, cv::Scalar color) const
{
    std::vector< std::vector<cv::Point> > v;
    v.push_back( contour );
    cv::drawContours( displayFrame, v, -1, color, 2, 8 );
}

void Blob::drawCentroid(cv::Mat & displayFrame, cv::Scalar color, int radius, int thickness) const
{
    cv::circle(displayFrame, centroid, radius, color, thickness);
}
