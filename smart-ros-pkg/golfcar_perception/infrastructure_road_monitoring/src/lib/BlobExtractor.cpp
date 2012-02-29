#include "BlobExtractor.h"

using namespace std;


BlobExtractor::BlobExtractor()
{

}

void BlobExtractor::extract(cv::Mat bin_input, double time)
{
    blobs.clear();

    // a temp mat is required, otherwise diffImg is no longer binary and
    // display is messed up.
    cv::Mat tmp = bin_input.clone();

    // Connected points
    vector<vector<cv::Point> > v;

    /* CV_RETR_EXTERNAL retrives only the extreme outer contours
     * CV_RETR_LIST retrieves all of the contours and puts them in the list
     * CV_RETR_CCOMP retrieves all of the contours and organizes them into a two-level hierarchy: on the top level are the external boundaries of the components, on the second level are the boundaries of the holes
     * CV_RETR_TREE retrieves all of the contours and reconstructs the full hierarchy of nested contours
     */
    /* CV_CHAIN_CODE outputs contours in the Freeman chain code. All other methods output polygons (sequences of vertices)
     * CV_CHAIN_APPROX_NONE translates all of the points from the chain code into points
     * CV_CHAIN_APPROX_SIMPLE compresses horizontal, vertical, and diagonal segments and leaves only their end points
     * CV_CHAIN_APPROX_TC89_L1,CV_CHAIN_APPROX_TC89_KCOS applies one of the flavors of the Teh-Chin chain approximation algorithm.
     * CV_LINK_RUNS uses a completely different contour retrieval algorithm by linking horizontal segments of 1’s. Only the CV_RETR_LIST retrieval mode can be used with this method.
     */
    cv::findContours(tmp, v, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);

    // Approximate each contour by a polygon
    for( unsigned i=0; i<v.size(); i++ )
    {
        Blob blob;
        cv::Mat vi(v[i]);
        //cv::approxPolyDP( vi, blob.contour, 10, true );
        blob.contour = v[i];
        cv::Moments m = cv::moments(vi);
        blob.centroid = cv::Point(m.m10/m.m00, m.m01/m.m00);
        blob.timestamp = time;
        blobs.push_back(blob);
    }
}

void BlobExtractor::display(cv::Mat & display, cv::Scalar color)
{
    // Draw polygonal contour
    if( ! blobs.empty() ) {
        //cout <<"Found " <<contours_poly.size() <<" countours large enough" <<endl;
        for( unsigned i=0; i<blobs.size(); i++ ) {
            blobs[i].drawContour(display, color);
            blobs[i].drawCentroid(display, color, 5, -1);
        }
    }
}
