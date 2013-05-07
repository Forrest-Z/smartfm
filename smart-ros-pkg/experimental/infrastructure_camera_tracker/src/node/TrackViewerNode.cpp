#include "TrackViewerNode.hpp"

#include <cv_bridge/cv_bridge.h>

cv::Scalar angleToBGR(const int& angle, const unsigned char& alpha)
{
  const double h = (static_cast<double>(angle))/60.0;
  const unsigned char x = static_cast<unsigned char>(255*(1 - fabs(fmod(h, 2) - 1)));

  switch (static_cast<int>(h))
  {
    case 0:
      return(cv::Scalar(0, x, 255, alpha));
    case 1:
      return(cv::Scalar(0, 255, x, alpha));
    case 2:
      return(cv::Scalar(x, 255, 0, alpha));
    case 3:
      return(cv::Scalar(255, x, 0, alpha));
    case 4:
      return(cv::Scalar(255, 0, x, alpha));
    case 5:
      return(cv::Scalar(x, 0, 255, alpha));
    case 6:
      return(cv::Scalar(x, 0, 255, alpha));
    default:
      std::cerr << "[angleToBGR] Something went wrong (h=" << h << ").\n";
  }
  return(cv::Scalar(0, 0, 0, alpha));
}

std::vector<cv::Scalar> getAngleColormap(void)
{
  std::vector<cv::Scalar> result(361);

  for (unsigned int i = 0; i < 361; ++i)
  {
    result[i] = angleToBGR(i, 200);
  }

  return(result);
}

std::vector<cv::Scalar> TrackViewerNode::colormap = getAngleColormap();

TrackViewerNode::TrackViewerNode(): node_handle("~"),
                                    img_transport(node_handle),
                                    img_subscriber(img_transport, "image", 1),
                                    trackset_subscriber(node_handle, "tracks", 1),
                                    synchronizer(sync_policy(1), img_subscriber, trackset_subscriber),
                                    window_name("track viewer")
{
  // bind the callback and tell boost::bind() to forward the first and second arguments it receives
  // boost::function<void (const sensor_msgs::Image::ConstPtr&, const infrastructure_camera_tracker::TrackSet::ConstPtr&)> sync_callback( boost::bind( &TrackViewerNode::callback, this, _1, _2 ) );
  // synchronizer.registerCallback( sync_callback );                                                // this does not work

  synchronizer.registerCallback( boost::bind(&TrackViewerNode::callback, this, _1, _2) );         // but this does

  cv::namedWindow(window_name, cv::WINDOW_AUTOSIZE);
};

TrackViewerNode::~TrackViewerNode()
{
  // close window
};

void TrackViewerNode::callback( const sensor_msgs::Image::ConstPtr& image,
                                const ict::TrackSet::ConstPtr& trackset)
{
  ROS_INFO("received image/trackset pair\n");
  cv::Mat frame = cv_bridge::toCvCopy(image, "bgr8")->image;

  // plot tracks
  for(unsigned int i = 0; i < trackset->tracks.size(); ++i)
  {
    cv::Point2f delta;
    unsigned int angle, j = 0, k = 1;
    for ( ; k < trackset->tracks[i].track.size(); ++j, ++k )
    {
      cv::Point2f a(trackset->tracks[i].track[j].position.x, trackset->tracks[i].track[j].position.y);
      cv::circle( frame, a, 1, cv::Scalar(0, 255, 0), -1, CV_AA, 0 );
      cv::Point2f b(trackset->tracks[i].track[k].position.x, trackset->tracks[i].track[k].position.y);
      cv::circle( frame, b, 1, cv::Scalar(0, 255, 0), -1, CV_AA, 0 );

      delta = b - a;
      angle = static_cast<int>((180.0/CV_PI)*(atan2(delta.y, delta.x) + CV_PI));

      cv::line(frame, a, b, colormap.at(angle), 1, 8, 0);
    }
  }

  // display image
  cv::imshow(window_name, frame);
  cv::waitKey(10);
};
