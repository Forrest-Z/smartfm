#ifndef GOLFCART_SEMANTICS_SIMULTRACKCOLLECTOR_H
#define GOLFCART_SEMANTICS_SIMULTRACKCOLLECTOR_H

#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <sensor_msgs/Imu.h>
#include <string>
#include "geometry_msgs/Point32.h"

#include "../activity_collector/pedestrian_collector.h"

using namespace std;

namespace golfcar_semantics{

	class simul_track_collector {
			public:
			simul_track_collector()
			{
				SimuTrackPath_ = "simulation_track.data";
				pose_sub_ = nh_.subscribe("/amcl_pose", 100, &simul_track_collector::poseCallback, this);
			};
			~simul_track_collector()
			{
				track_saving();
			};

			//accumulate simul_track_;
			void poseCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr msg)
			{
				sensing_on_road::pedestrian_vision simul_ped;
				simul_ped.cluster.last_update = msg->header.stamp;
				simul_ped.cluster.centroid.x  = (float) msg->pose.pose.position.x;
				simul_ped.cluster.centroid.y  = (float) msg->pose.pose.position.y;
				simul_ped.cluster.width  	  = 1.0;
				simul_ped.cluster.depth  	  = 0.5;
				simul_ped.local_centroid.x  = 0.0;
				simul_ped.local_centroid.y  = 0.0;
				simul_track_.ped_track.push_back(simul_ped);
			}

			void track_saving()
			{
				FILE *fp_output;
				if((fp_output=fopen(SimuTrackPath_.c_str(), "a"))==NULL){ROS_ERROR("cannot open output_file\n"); return;}

				simul_track_.ped_confidence = 0.8;
				fprintf(fp_output, "%f\t", simul_track_.ped_confidence);
				fprintf(fp_output, "%ld\n", simul_track_.ped_track.size());

				for(size_t j=0; j<simul_track_.ped_track.size(); j++)
				{
					fprintf(fp_output, "%lf\t", simul_track_.ped_track[j].cluster.last_update.toSec());
					fprintf(fp_output, "%f\t",  simul_track_.ped_track[j].cluster.centroid.x);
					fprintf(fp_output, "%f\t",  simul_track_.ped_track[j].cluster.centroid.y);
					fprintf(fp_output, "%f\t",  simul_track_.ped_track[j].cluster.width);
					fprintf(fp_output, "%f\t",  simul_track_.ped_track[j].cluster.depth);
					fprintf(fp_output, "%f\t",  simul_track_.ped_track[j].local_centroid.x);
					fprintf(fp_output, "%f\t",  simul_track_.ped_track[j].local_centroid.y);
				}
				fprintf(fp_output, "\n");
				fclose(fp_output);
			}

			private:
			ros::NodeHandle nh_;
			ros::Subscriber pose_sub_;
			std::string SimuTrackPath_;
			pedestrian_track simul_track_;
		};
};

int main(int argc, char** argv)
{
	 ros::init(argc, argv, "simu_track_collector_node");
	 ros::NodeHandle n;
	 golfcar_semantics::simul_track_collector simu_track_collector_node;
     ros::spin();
     return 0;
}

#endif
