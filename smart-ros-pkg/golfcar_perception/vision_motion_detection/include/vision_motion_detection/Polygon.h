#ifndef POLYGON_H_
#define POLYGON_H_

#include <vector>
#include <string>

#include <opencv2/opencv.hpp>

#include <ros/ros.h>

class Polygon : public std::vector<cv::Point>
{
public:
    std::string to_str() const;
    XmlRpc::XmlRpcValue to_XmlRpc() const;
    void from_XmlRpc(XmlRpc::XmlRpcValue xmlrpc);
    void draw(cv::Mat & frame, cv::Scalar color) const;
    bool is_inside(cv::Point p) const;
    Polygon::iterator add_to_countour(int x, int y);
    float dist_to_segment(int x, int y, const Polygon::iterator & it, const Polygon::iterator & jt);
    Polygon::iterator find_closest_point(int x, int y);
};

#endif /* POLYGON_H_ */
