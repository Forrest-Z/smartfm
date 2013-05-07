#include <infrastructure_camera_tracker/TrackBuilder.hpp>

TrackBuilder::TrackBuilder():tracks()
{

};

TrackBuilder::~TrackBuilder()
{

};

void TrackBuilder::add( std::vector<long unsigned int>& ids,
                        std::vector<cv::Point2f>& positions,
                        std::vector<double>& timestamps)
{
  Track temp_track;
  for(unsigned int i = 0; i < ids.size(); ++i)
  {
    // could do: stitch tracks here
    temp_track = Track();
    temp_track.update(positions[i], true, timestamps[i]);
    tracks[ids[i]] = temp_track;
    // tracks.emplace(ids[i], temp_track); // C++11
  }
};

void TrackBuilder::update(  std::vector<long unsigned int>& ids,
                            std::vector<cv::Point2f>& positions,
                            std::vector<double>& timestamps,
                            std::vector<bool>& active)
{
  for(unsigned int i = 0; i < ids.size(); ++i)
  {
    tracks[ids[i]].update(positions[i], active[i], timestamps[i]);
  }
};