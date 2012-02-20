#ifndef BLOBEXTRACTOR_H_
#define BLOBEXTRACTOR_H_

#include <vector>
#include <cv.h>

#include "Blob.h"

class BlobExtractor
{
public:
    std::vector<Blob> blobs;

    BlobExtractor();

    /// extract blobs from the binary frame
    void extract(cv::Mat inFrameBin, double time);

    /// display the blobs on the given frame
    void display(cv::Mat & displayFrame, cv::Scalar color=CV_RGB(255,0,0));
};

#endif /* BLOBEXTRACTION_H_ */
