#ifndef TRACK_BUILDER_HPP
#define TRACK_BUILDER_HPP

#include <opencv2/opencv.hpp>
#include <map>
#include <vector>
#include <infrastructure_camera_tracker/Track.hpp>

class TrackBuilder
{
  public:
    TrackBuilder();
    ~TrackBuilder();

    void add( std::vector<long unsigned int>& ids,
      std::vector<cv::Point2f>& positions,
      std::vector<double>& timestamps);

    void clear(void)
    {
      tracks.clear();
    };

    inline std::vector<Track> getActiveTracks(void) const
    {
      std::vector<Track> result;
      for(std::map<long unsigned int, Track>::const_iterator it = tracks.begin(); it != tracks.end(); ++it)
      {
        if (it->second.isActive())
        {
          result.push_back(it->second);
        }
      }
      return(result);
    };

    // std::vector<Track> getMovingTracks(void) const;

    inline std::vector<Track> getTracks(void) const
    {
      std::vector<Track> result;
      for(std::map<long unsigned int, Track>::const_iterator it = tracks.begin(); it != tracks.end(); ++it)
      {
        result.push_back(it->second);
      }
      return(result);
    };

    void update(std::vector<long unsigned int>& ids,
      std::vector<cv::Point2f>& positions,
      std::vector<double>& timestamps,
      std::vector<bool>& active);

  private:
    std::map<long unsigned int, Track> tracks;
    // std::map<long unsigned int, bool> moving;


};

#endif
