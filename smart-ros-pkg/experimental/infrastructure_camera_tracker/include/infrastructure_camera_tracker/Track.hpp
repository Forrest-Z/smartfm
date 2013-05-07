/*
* Track.hpp
*
* 2013 pvt   <pedro@smart.mit.edu>
*            <pvazteixeira@gmail.com>
*/

// include guards prevent Track.hpp from being included multiple times
#ifndef TRACK_HPP
#define TRACK_HPP

#include <deque>
#include <vector>
#include <iostream>
#include <fstream>

#include <opencv2/opencv.hpp>

class Track
{
  private:
    std::deque<cv::Point2f>* points;
    std::deque<cv::Point2f>* deltas; // difference between subsequent points
    std::deque<double>* times;
    static long unsigned int id_count;
    long unsigned int id;
    bool active;

	public:

    Track(void) // default constructor
    {
      active = true;
      points = new std::deque<cv::Point2f>;
      deltas = new std::deque<cv::Point2f>;
      //dist = new std::deque<double>;
      times = new std::deque<double>;
      id = id_count++;
    };

    Track(const Track& other) // copy constructor
    {
      active = other.active;
      points = new std::deque<cv::Point2f>;
      deltas = new std::deque<cv::Point2f>;
      times = new std::deque<double>;
      for (unsigned int i=0; i < other.points->size(); i++)
      {
        points->push_back( other.points->at(i) );
        times->push_back( other.times->at(i) );
      }
      for (unsigned int i = 0; i < other.deltas->size(); ++i)
      {
        deltas->push_back( other.deltas->at(i) );
      }
      id = id_count++;
    };

    ~Track()  // destructor
    {
      delete(points);
      delete(deltas);
      delete(times);
    };

    Track& operator=( const Track& rhs )  // assignment operator
    {
      active = rhs.active;
      points->clear();
      deltas->clear();
      times->clear();
      for (unsigned int i=0; i<rhs.points->size(); ++i)
      {
        points->push_back( rhs.points->at(i) );
        times->push_back( rhs.times->at(i) );
      }
      for (unsigned int i=0; i<rhs.deltas->size(); ++i)
      {
        deltas->push_back( rhs.deltas->at(i) );
      }

      return(*this);
    };

    inline void drawLabel(cv::Mat& dst, const std::string& label) const
    {
      cv::Point2f o = points->back();
      cv::putText(dst, label, o, cv::FONT_HERSHEY_SIMPLEX, 0.25, cv::Scalar(255, 255, 255), 1);
    };

    void get(std::vector<cv::Point2f>& positions, std::vector<double>& timestamps, long unsigned int& tid) const
    {
      positions.resize(points->size());
      timestamps.resize(times->size());
      std::deque<cv::Point2f>::iterator itp = points->begin();
      std::deque<double>::iterator itt = times->begin();
      for(unsigned int i = 0; (itp!=points->end() && itt!=times->end()); ++i, ++itp, ++itt)
      {
        positions[i] = *itp;
        timestamps[i] = *itt;
      }
      tid = id;
    }

    inline cv::Point2f getLast(void) const
    {
      return(points->back());
    };

    double getLength(void) const;

    inline double getNetLength(void) const
    {
      return(cv::norm(points->back() - points->front()));
    };

    bool getMAVelocity(const unsigned int& n, cv::Point2f& vel) const;

    cv::Point2f getVelocity(void) const;

    inline bool isActive(void) const
    {
      return(active);
    };

    void plot(cv::Mat& dst, const int& radius, const std::vector<cv::Scalar>& colormap) const
    {
      std::deque<cv::Point2f>::iterator it1 = points->begin();
      std::deque<cv::Point2f>::iterator it2 = it1 + 1;
      cv::Point2f delta;
      int i;
      for ( ; it2 != points->end(); ++it1, ++it2)
      {
        delta = *it2 - *it1;
        i = (unsigned int)((180.0/CV_PI)*(atan2(delta.y, delta.x) + CV_PI));
        //cv::circle(dst, *it2, radius, colormap[i], -1, CV_AA, 0 );
        cv::line(dst, *it2, *it1, colormap.at(i), 1, 8, 0);
      }
    };

    inline void plotLast(cv::Mat& dst, const int& radius = 3) const
    {
      cv::Scalar color = (active?cv::Scalar(0, 255, 0):cv::Scalar(0, 0, 255));
      cv::circle(dst, points->back(), radius, color, -1, CV_AA, 0 );
    };

    inline void plotLast(cv::Mat& dst, const cv::Scalar& color, const int& radius = 3) const
    {
      cv::circle(dst, points->back(), radius, color, -1, CV_AA, 0 );
    };

    inline void print(std::ostream& out) const
    {
      // print points
      out << "Track " << id << ": (";
      for (unsigned int i = 0; i < points->size(); i++)
      {
        out << times->at(i) << ", " << points->at(i).x << ", " << points->at(i).y << "; ";
      }
      out << ")\n";
    };

    inline void printDelta(void) const
    {
      std:: cout << "           [";
      for (unsigned int i = 0; i < deltas->size(); i++)
      {
        std::cout << times->at(i+1) << ", " << deltas->at(i).x << ", " << deltas->at(i).y << "; ";
      }
      std:: cout << "]\n";
    };

    inline void push_back(const cv::Point2f& point, const double& t)
    {
      active = true;
      if (points->size())
      {
        deltas->push_back(point-points->back());
      }
      points->push_back(point);
      times->push_back(t);
    };

    inline void reset(void)
    {
      points->clear();
      times->clear();
      active = true;
    };

    inline long unsigned int size(void) const
    {
      return(points->size());
    };

    void update( const cv::Point2f& new_point,
                 const bool& status,
                 const double& t);

    void update(  const cv::Point2f &new_point,
                  const unsigned char &status,
                  const double &t);
};
#endif
