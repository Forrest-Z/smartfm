/*
 * pedestrian_collector
 * author: Baoxing
 * data:   2013/04/14
 */

#ifndef PEDESTRIAN_COLLECTOR_H
#define PEDESTRIAN_COLLECTOR_H

#include <ros/ros.h>
#include <vector>
#include <sensor_msgs/PointCloud.h>
#include <sensing_on_road/pedestrian_laser_batch.h>
#include <sensing_on_road/pedestrian_vision_batch.h>
#include <feature_detection/clusters.h>
#include <geometry_msgs/Point.h>
#include <tf/transform_listener.h>
#include <tf/message_filter.h>
#include <message_filters/subscriber.h>
#include <geometry_msgs/PolygonStamped.h>
#include <fmutil/fm_math.h>
#include <dynamic_reconfigure/server.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <cv_bridge/CvBridge.h>

using namespace std;
using namespace ros;
using namespace tf;

namespace golfcar_semantics{

	class pedestrian_track
	{
		public:
		int object_label;
		float ped_confidence;
		std::vector<sensing_on_road::pedestrian_vision> ped_track;
	};

    class pedestrian_collector {
        public:
    	pedestrian_collector();
        ~pedestrian_collector();
        std::vector<pedestrian_track> ped_tracks_;

        private:
        ros::NodeHandle nh_, private_nh_;
        tf::TransformListener tf_;
		string map_frame_, base_frame_;
		ros::Subscriber	pedestrian_sub_;
		void pedCallback(const sensing_on_road::pedestrian_vision_batch::ConstPtr& ped_batch_in);

		//three functions;
		void track_updating(const sensing_on_road::pedestrian_vision_batch::ConstPtr& ped_batch_in);
		void track_visualization();
		void track_saving();

        IplImage *visual_image_;
		double map_scale_;
		string map_pic_path_;
        image_transport::ImageTransport it_;
        image_transport::Publisher visual_pub_;
        sensor_msgs::CvBridge bridge_;

        //to do: store the tracks in a file;
        string file_path_;
   };
};

#endif
