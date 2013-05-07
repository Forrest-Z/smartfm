#include <infrastructure_camera_tracker/TrackBuilder.hpp>

TrackBuilder::TrackBuilder()
{

};

TrackBuilder::~TrackBuilder()
{

};

std::vector<Track> TrackBuilder::getMovingTracks(void) const
{

};

void TrackBuilder::add( std::vector<long unsigned int>& ids,
                        std::vector<cv::Point2f>& positions,
                        std::vector<double>& timestamps)
{
  Track temp_track;
  for(unsigned int i = 0; i < ids.size(); ++i)
  {
    tracks.
  }
};

void TrackBuilder::update(  std::vector<long unsigned int>& ids,
                            std::vector<cv::Point2f>& positions,
                            std::vector<double>& timestamps,
                            std::vector<bool>& active,
{
  for(unsigned int i = 0; i < ids.size(); ++i)
  {

  }
};