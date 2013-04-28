/*
 * pedestrian_induced
 * author: Baoxing
 * date:   2013/04/16
 * description: to learn pedestrian-activity-induced semantics from collected pedestrian tracks, together with a road map;
 * 				offline learning, in the 1st version;
 */

#ifndef GOLFCAR_SEMANTICS_PED_SEMANTIC_H
#define GOLFCAR_SEMANTICS_PED_SEMANTIC_H

#include <ros/ros.h>
#include <vector>
#include <sensor_msgs/PointCloud.h>
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

#include "../data_type/datatype_semantic.h"
#include "../tools/local_track_show.h"
#include "../tools/global_track_show.h"

#include "AM_learner.h"

using namespace std;
using namespace ros;
using namespace tf;

namespace golfcar_semantics{

    class ped_semantics {
        public:
    	ped_semantics(char* image_path, char* file_path, double map_scale, double track_length_threshold);
        ~ped_semantics();

        //extracted ped_semantic information;
        void semantics_learning();

        private:

        void roadmap_loading();
        void pedtrack_loading();
        void track_visualization(vector<track_common> & ped_tracks, CvScalar color, bool plot_ends);
        void ped_EE_extraction();
        void ped_track_classification();

        IplImage *road_image_, *distance_image_, *visualize_image_;

		pd_track_container *track_container_;
        vector<track_common> ped_tracks_;
        vector<track_common> ped_moving_tracks_;
        vector<track_common> ped_static_tracks_;

        double map_scale_;
		char* image_path_, *file_path_;
		global_track_show *global_viewer_;


		size_t track_size_thresh_;
		double track_time_thresh_, track_length_thresh_;

		CvSize local_view_size_;
		double local_show_scale_;
		local_track_show *local_viewer_;
		void trajectory_show();


		AM_learner *activity_map_learner_;
   };
};

#endif
