#include "ArrivalEstimatorNode.hpp"
#include "cv_msg_bridge.hpp"

ArrivalEstimatorNode::ArrivalEstimatorNode(): node_handle("~")
{
  feature_subscriber = node_handle.subscribe("features", 100, &ArrivalEstimatorNode::featureCallback, this);
  line_subscriber = node_handle.subscribe("arrival_line", 100, &ArrivalEstimatorNode::lineCallback, this);
  arrival_publisher = node_handle.advertise<infrastructure_camera_tracker::ArrivalSet>("arrivals", 1000);
  track_publisher = node_handle.advertise<infrastructure_camera_tracker::TrackSet>("tracks", 1000);
};

ArrivalEstimatorNode::~ArrivalEstimatorNode()
{

};

void ArrivalEstimatorNode::featureCallback(const infrastructure_camera_tracker::TrackedFeatureSet::ConstPtr& msg)
{
  namespace ict = infrastructure_camera_tracker;

  last_header = msg->header;

  unsigned int N = msg->features.size();
  std::vector<long unsigned int> ids(N);
  std::vector<cv::Point2f> points(N);
  std::vector<double> timestamps(N);
  std::vector<bool> active(N);

  for (unsigned int i = 0; i < N; ++i)
  {
    ids[i]        = msg->features[i].id;
    points[i]     = cv::Point2f(msg->features[i].position.x, msg->features[i].position.y);
    timestamps[i] = msg->features[i].time;
    active[i]     = msg->features[i].active;
  }

  if (msg->new_set)
  {
    // the arrival estimator can also publish tracks as it needs them for its own estimation
    ROS_INFO("Publising track set");
    publishTracks();
    track_builder.clear(); // if stitching isn't implemented
    track_builder.add(ids, points, timestamps);
  }
  else
  {
    ict::ArrivalSet message;
    ict::TrackedFeature feature;

    track_builder.update(ids, points, timestamps, active);

    // perform arrival estimation
    std::vector<Track> tracks = track_builder.getTracks();    // not very efficient

    // loop variables
    cv::Point2f arrival_point, feature_velocity;
    float arrival_time, x_arrival;

    // compute intersection
    int total_arrivals=0;// number
    for(unsigned int i = 0; i < tracks.size(); ++i)
    {
      if(tracks[i].isActive())
      {
        // get moving average of the velocity
        if (tracks[i].getMAVelocity(5, feature_velocity))
        {
          if (1 < cv::norm(feature_velocity))
          {
            if (arrival_estimator.getIntersection(tracks[i].getLast(), feature_velocity, arrival_point, arrival_time, x_arrival))
            {
              ++total_arrivals;
              feature.position = toGeometryMsgsPoint(tracks[i].getLast());
              feature.time = tracks[i].getLastTime();
              feature.id = tracks[i].getID();
              feature.active = true;
              message.features.push_back(feature);
              message.velocity.push_back(toGeometryMsgsPoint(feature_velocity));
              message.arrival_point.push_back(toGeometryMsgsPoint(arrival_point));
              message.arrival_time.push_back(arrival_time);
              message.arrival_line   = arrival_line_msg;
            }
          }
        }
      }
    }
    //ROS_INFO("%i arrivals computed", j);
    // publish!
    if (0<total_arrivals)
    {
      arrival_publisher.publish(message);
    }
  }
};

void ArrivalEstimatorNode::lineCallback(const infrastructure_camera_tracker::ArrivalLine::ConstPtr& msg)
{
  ROS_INFO("Received new arrival line position.");

  // std::cout << toOpenCVPoint(msg->first) << ", " <<toOpenCVPoint(msg->second) << std::endl;
  arrival_estimator = ArrivalEstimator(toOpenCVPoint(msg->first), toOpenCVPoint(msg->second));
  arrival_estimator.print();
  // cv::Mat frame(1000, 1000, CV_8UC3, cv::Scalar(0, 0, 0));
  // cv::namedWindow("AEN_debug", cv::WINDOW_AUTOSIZE);
  // cv::imshow("AEN_debug", frame);
  // cv::waitKey(10);
  // cv::line(frame, toOpenCVPoint(msg->first), toOpenCVPoint(msg->second), cv::Scalar(255, 255, 255), 1, CV_AA, 0 );
  // cv::imshow("AEN_debug", frame);
  // cv::waitKey(10);
};

// void ArrivalEstimatorNode::publishArrivals(void) const
// {

// };

void ArrivalEstimatorNode::publishTracks(void) const
{
  namespace ict = infrastructure_camera_tracker;
  ict::TrackSet message;
  // msg/track as vector of features
  // trackset as a vector of msg/track
  std::vector<Track> tracks = track_builder.getTracks();
  std::vector<ict::Track> trackv_msg(tracks.size());

  for(unsigned int i = 0; i < tracks.size(); ++i)
  {
    std::vector<ict::Feature> featurev_msg(tracks[i].size());

    std::vector<cv::Point2f> positions;
    std::vector<double> timestamps;
    long unsigned int id;
    tracks[i].get(positions, timestamps, id);

    for(unsigned int j = 0; j < tracks[i].size(); ++j )
    {
      featurev_msg[j].position = toGeometryMsgsPoint(positions[j]);
      featurev_msg[j].time = timestamps[j];
    }

    trackv_msg[i].track = featurev_msg;
    trackv_msg[i].id = id;
  }

  // message.header = last_header;
  message.header.stamp = ros::Time::now();
  message.tracks = trackv_msg;

  track_publisher.publish(message);
};