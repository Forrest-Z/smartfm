#ifndef __BLOB_EXTRACTOR_H__
#define __BLOB_EXTRACTOR_H__

#include <opencv2/highgui/highgui.hpp>
#include <vision_motion_detection/data_types.h>
#include <vision_motion_detection/Blobs.h>
#include <vision_motion_detection/Polygon.h>


class BlobExtractor
{
public:
    BlobExtractor();

    unsigned diff_thresh_; ///< threshold used when converting diff image to binary
    int dilate_size_; ///< how much dilation should be applied
    int erode_size_;  ///< how much erosion should be applied
    int blurring_size_; ///< size of the blurring kernel

    bool view_intermediate_images_; ///< display images resulting from blurring, erosion, dilation, threshold, etc.

    /// Computes the difference between frame and background and extracts blobs
    std::vector<Blob> extract(cv::Mat frame_in, cv::Mat background_in, double time);

    /// set the region of interest
    void set_roi(Polygon);

private:
    cv::Mat diffImg_; ///< the resulting binary image.

    /// Compute the difference between frame and background (result goes to diffImg_)
    void diff(cv::Mat frame, cv::Mat background);

    void dilate(cv::Mat &);
    void erode(cv::Mat &);

    /// extract blobs from the binary frame
    std::vector<Blob> extract(cv::Mat inFrameBin, double time);

    /// Region of interest defined as a polygon
    Polygon roi_poly_;

    /// Masks corresponding to the ROI
    cv::Mat bin_mask_, rgb_mask_;

    /// Creates the masks bin_mask_ and rgb_mask_ corresponding to roi_poly_
    void setMask();
};

#endif // __BLOB_EXTRACTOR_H__
