#include <opencv2/opencv.hpp>
#include <geometry_msgs/Point.h>

inline geometry_msgs::Point toGeometryMsgsPoint(const cv::Point2f& point)
{
  geometry_msgs::Point pt;
  pt.x = static_cast<double>(point.x);
  pt.y = static_cast<double>(point.y);
  pt.z = 0.0;
  return(pt);
};

inline cv::Point2f toOpenCVPoint(const geometry_msgs::Point& point)
{
  return(cv::Point2f(point.x, point.y));
}