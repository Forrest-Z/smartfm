#include <ros/ros.h>
#include <ros/time.h>
#include <nav_msgs/OccupancyGrid.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/LaserScan.h>
#include <laser_geometry/laser_geometry.h>
#include <sensor_msgs/PointCloud.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/PointStamped.h>
#include <tf/message_filter.h>
#include <message_filters/subscriber.h>
#include <boost/thread.hpp>
#include <boost/shared_ptr.hpp>
#include <vector>
#include <cmath>
#include "sensing_on_road/line_piece.h"
#include "sensing_on_road/scan_segment.h"
#include "sensing_on_road/segments_one_batch.h"
#include "sensing_on_road/pedestrian_laser.h"
#include "sensing_on_road/pedestrian_laser_batch.h"
#include "sensing_on_road/pedestrian_vision.h"
#include "sensing_on_road/pedestrian_vision_batch.h"

#ifndef SENSING_ON_ROAD_PEDESTRIANS
#define SENSING_ON_ROAD_PEDESTRIANS


namespace sensing_on_road
{

class  pedestrian_laser_history
{
public:
    int object_label;
    std::vector<int> merged_labels;
    ros::Time latest_update_time;

    //"history_status" is used to flag status of the history;
    //"0" is reset value; every time when "data_association" fucntion begins, status of each history is set to "0";
    //"1" means "mutual matching", and the history will be updated by its correponding segment;
    //"2" for "merge";
    //"3" for "split";
    //"4" for "rest" not associated;
    unsigned int history_status;
    int nearest_segment_serial;
    float nearest_distance;
    float velocity;
    float fastest_velocity;
    bool once_for_all;
    std::vector<sensing_on_road::scan_segment> single_segment_history;

    float pedestrian_belief;

    pedestrian_laser_history()
    {
        object_label = 0;
        velocity = 0.0;
        fastest_velocity  = 0.0;
        pedestrian_belief = 0.1;
        once_for_all = false;
    }
    
    ~pedestrian_laser_history() { }
};


class pedestrian_features
{
public:
    pedestrian_features();
    ~pedestrian_features();

private:
    ros::Subscriber segments_batch_sub_;
    void pedestrian_filtering(const sensing_on_road::segments_one_batch& batch_para);
    sensing_on_road::segments_one_batch filtered_segment_batch_;

    void data_association();
    int object_total_number_;
    std::vector <pedestrian_laser_history> 		history_pool_;
    float his_seg_dis(geometry_msgs::Point32 &pt11, geometry_msgs::Point32 &pt12, geometry_msgs::Point32 &pt21, geometry_msgs::Point32 &pt22);
    void history_processing();
    sensing_on_road::pedestrian_laser_batch	new_laser_pedestrians_;
    ros::Publisher laser_pedestrians_pub_;

    void pedestrian_extraction();

    sensor_msgs::PointCloud pedestrians_cloud_;
    ros::Publisher pedestrians_pcl_pub_;

    ros::Subscriber veri_vision_sub_;
    void Bayesfilter(const sensing_on_road::pedestrian_vision_batch &veri_pd_para);
    sensing_on_road::pedestrian_vision_batch veri_batch_;

    void ProbabilityCheck();
    sensing_on_road::pedestrian_vision_batch veri_show_batch_;
    ros::Publisher veri_show_batch_pub_;
    sensor_msgs::PointCloud veri_show_pcl_;
    ros::Publisher veri_show_pcl_pub_;
};

} //namespace sensing_on_road

#endif

