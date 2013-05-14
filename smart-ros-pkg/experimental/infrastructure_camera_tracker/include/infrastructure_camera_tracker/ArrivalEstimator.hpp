/*
 * ArrivalEstimator.hpp
 *
 * Copyright 2013 pvt   <pedro@smart.mit.edu>
 *                      <pvazteixeira@gmail.com>
 */
#ifndef ARRIVAL_HPP
#define ARRIVAL_HPP

#include <opencv2/opencv.hpp>

class ArrivalEstimator
{
  private:
    cv::Point2f line_points[2];
    cv::Point3f theta;
    cv::Scalar color;
    unsigned short int id;

    static unsigned short int id_count;
    // static bool user_added_point;
    // static cv::Point2f new_point;

    static cv::Point3f getLineParameters(const cv::Point2f& pt_1, const cv::Point2f& pt_2)
    {
      cv::Point2f normal, tangent = pt_2 - pt_1;
      normal.x = tangent.y;
      normal.y = -tangent.x;
      normal = (1 / cv::norm(normal)) * normal;   // normalize to get a versor

      cv::Point3f theta;
      theta.x = normal.x;
      theta.y = normal.y;
      theta.z = normal.dot(pt_1);

      return(theta);
    };

  public:
    ArrivalEstimator()
    {
      id = id_count++;
      color = cv::Scalar(0, 255, 0);
    };

    ArrivalEstimator(cv::Scalar line_color)
    {
      id = id_count++;
      color = line_color;
    };

    ArrivalEstimator(const cv::Point2f& pt1, const cv::Point2f& pt2)
    {
      id = id_count++;
      color = cv::Scalar(0, 255, 0);
      theta = getLineParameters(pt1, pt2);
    };

    ~ArrivalEstimator()
    {

    };

    bool getIntersection( const cv::Point2f& position,
                          const cv::Point2f& velocity,
                          cv::Point2f& intersection_point,
                          float& time_to_arrival,
                          float& x) const;

    cv::Point2f getIntersectionPoint(const cv::Point3f& line) const
    {
      cv::Mat A = cv::Mat::zeros(2, 2, CV_32F), B = cv::Mat::zeros(2, 1, CV_32F);

      A.at<float>(0, 0) = theta.x;
      A.at<float>(0, 1) = theta.y;
      B.at<float>(0) = theta.z;

      A.at<float>(1, 0) = line.x;
      A.at<float>(1, 1) = line.y;
      B.at<float>(1) = line.z;

      B = (A.inv() * B);
      // a conversion from Mat to Point2f would be more elegant
      cv::Point2f x(B.at<float>(0), B.at<float>(1));
      A.release();
      B.release();
      return(x);
    };

    // 'wrapper' function to compute intersection given two points defining a line
    cv::Point2f getIntersectionPoint(const cv::Point2f& pt_1, const cv::Point2f& pt_2) const
    {
      cv::Point3f line = getLineParameters(pt_1, pt_2);
      return(this->getIntersectionPoint(line));
    };

    // void getUserInputLine(const std::string& window_name, cv::Mat& frame)
    // {
    //   cv::setMouseCallback(window_name, onMouse, 0);
    //   unsigned short int new_point_count = 0;
    //   cv::imshow(window_name, frame);
    //   std::cout << "Please define the arrival line by left-clicking on two points in window \"" << window_name << "\"\n";
    //   for (;;)
    //   {
    //     if (user_added_point && 0 == new_point_count)
    //     {
    //       line_points[0] = new_point;
    //       ++new_point_count;
    //       user_added_point = false;
    //       cv::circle(frame, line_points[0], 1, color, 1, CV_AA, 0);
    //       cv::imshow(window_name, frame);
    //     }
    //     else if (user_added_point && 1 == new_point_count)
    //     {
    //       if ( new_point == line_points[0])
    //       {
    //         user_added_point = false;
    //         continue;
    //       }
    //       else
    //       {
    //         line_points[1] = new_point;
    //         //++new_point_count;
    //         user_added_point = false;
    //         cv::circle(frame, line_points[1], 1, color, -1, CV_AA, 0);
    //         cv::line(frame, line_points[0], line_points[1], color, 1, CV_AA, 0);
    //         cv::imshow(window_name, frame);
    //         break;
    //       }
    //     }
    //     else
    //     {
    //        cvWaitKey(10);
    //     }
    //   }
    //   theta = getLineParameters(line_points[0], line_points[1]);
    // };

    // static void onMouse ( const int event,
    //                       const int x,
    //                       const int y,
    //                       int /*flags*/,
    //                       void* /*param*/)
    // {
    //   if (event == CV_EVENT_LBUTTONDOWN)
    //   {
    //     new_point = cv::Point2f(static_cast<float>(x), static_cast<float>(y));
    //     user_added_point = true;
    //   }
    // }

    void print(std::ostream& out) const
    {
      out << "arrival line [id = " << id << "]:\n";
      out << "point 1 = " << line_points[0] << "\n";
      out << "point 2 = " << line_points[1] << "\n";
      out << "theta = " << theta << "\n";
    };

    void print(std::ostream& out, std::string pre) const
    {
      out << pre << "arrival line [id = " << id << "]:\n";
      out << pre << "point 1 = " << line_points[0] << "\n";
      out << pre << "point 2 = " << line_points[1] << "\n";
      out << pre << "theta = " << theta << "\n";
    };

    bool validateIntersectionPoint(const cv::Point2f& point) const
    {
      bool in_x = ((std::min(line_points[0].x, line_points[1].x) <= point.x) && (point.x <= std::max(line_points[0].x, line_points[1].x)));
      bool in_y = ((std::min(line_points[0].y, line_points[1].y) <= point.y) && (point.y <= std::max(line_points[0].y, line_points[1].y)));

      return(in_x && in_y);
    };
};

#endif
