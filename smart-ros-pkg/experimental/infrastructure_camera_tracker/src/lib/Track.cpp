/*
* Track.cpp
*
* Copyright 2013 pvt   <pedro@smart.mit.edu>
*                      <pvazteixeira@gmail.com>
*/

#include <infrastructure_camera_tracker/Track.hpp>

unsigned long int Track::id_count = 0;

double Track::getLength(void) const
{
  double d = 0;
  std::deque<cv::Point2f>::iterator it1, it2;
  for (it1 = points->begin(), it2 = it1+1; it2 < points->end(); ++it1, ++it2)
  {
    d += cv::norm(*it2 - *it1);
  }
  return(d);
}

/*
 * returns the average of the last n terms
 */
bool Track::getMAVelocity(const unsigned int& n, cv::Point2f& vel) const
{
  if (n < 1  || n > deltas->size() )
  {
    return false;
  }
  /*
  else if ( n > deltas->size())
  {
    cv::Point2f ma(0, 0);
    for (std::deque<cv::Point2f>::reverse_iterator rit = deltas->rbegin(); rit < deltas->rend(); ++rit)
    {
      ma += *rit;
    }
    vel = (1.0 / n) * ma;
    return true;
  }
  */
  else
  {
    cv::Point2f ma(0, 0);
    for (std::deque<cv::Point2f>::reverse_iterator rit = deltas->rbegin(); rit < deltas->rbegin()+n; ++rit)
    {
      ma += *rit;
    }
    vel = (1.0 / n) * ma;
    return true;
  }
}

cv::Point2f Track::getVelocity(void) const
{
  long unsigned int s = points->size();
  cv::Point2f v(0, 0);  // if inactive, it will return (0,0)
  if (active && s > 1)
  {
    v = deltas->back();
  }
  return(v);
}

void Track::update( const cv::Point2f &new_point,
                    const unsigned char &status,
                    const double &t)
{
  if (active)
  {
    if (points->size())
    {
      deltas->push_back(new_point-points->back());
    }
    points->push_back(cv::Point2f(new_point));
    times->push_back(static_cast<double>(t));
    (1 == static_cast<int>(status)) ? active = true : active = false;
  }
}