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
  private:
    std::map<long unsigned int, Track> tracks;

    std::vector<Track> getActiveTracks(void);
    std::vector<Track> getMovingTracks(void);
    std::vector<Track> getTracks(void);
    void update(std::vector<long unsigned int> ids,
                std::vector<cv::Point2f> positions,
                std::vector<double> timestamps,
                std::vector<bool> active);
};

#endif
