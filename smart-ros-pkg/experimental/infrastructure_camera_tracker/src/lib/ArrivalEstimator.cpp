/*
* ArrivalEstimator.cpp
*
* Copyright 2013 pvt   <pedro@smart.mit.edu>
*                      <pvazteixeira@gmail.com>
*/
#include <infrastructure_camera_tracker/ArrivalEstimator.hpp>

// cv::Point2f ArrivalEstimator::new_point = cv::Point2f(0, 0);
// bool ArrivalEstimator::user_added_point = false;
unsigned short int ArrivalEstimator::id_count = 0;

bool ArrivalEstimator::getIntersection(const cv::Point2f& position,
                                  const cv::Point2f& velocity,
                                  cv::Point2f& intersection_point,
                                  float& time_to_arrival,
                                  float& x) const
{
  cv::Point2f pt_2 = position + velocity;
  intersection_point = this->getIntersectionPoint(position, pt_2);
  if (this->validateIntersectionPoint(intersection_point))
  {
    // compute arrival time
    cv::Point2f delta = intersection_point - position;
    time_to_arrival = ((fabs(velocity.x) > fabs(velocity.y)) ? (delta.x / velocity.x) : (delta.y / velocity.y));
    //time_to_arrival = std::max(delta.x/velocity.x, delta.y/velocity.y); // shady
    if (time_to_arrival < 0)
    {
      return(false);
    }
    else
    {
      // compute x
      cv::Point2f tangent = line_points[1] - line_points[0];
      tangent = tangent * (1 / cv::norm(tangent));
      delta = intersection_point - line_points[0];
      x = ((fabs(tangent.x) > fabs(tangent.y)) ? (delta.x / tangent.x) : (delta.y / tangent.y));
      return(true);
    }
  }
  else
  {
    return(false);
  }
}