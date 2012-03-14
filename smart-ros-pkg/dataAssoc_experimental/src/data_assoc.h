/*
 * data_assoc.h
 *
 *  Created on: Sep 15, 2011
 *      Author: golfcar
 */

#ifndef DATA_ASSOC_H_
#define DATA_ASSOC_H_
#include <ros/ros.h>
#include <vector>
#include <sensor_msgs/PointCloud.h>
#include <sensing_on_road/pedestrian_laser_batch.h>
#include <sensing_on_road/pedestrian_vision_batch.h>
#include <sensing_on_road/camera_project.h>
#include <feature_detection/clusters.h>
#include <geometry_msgs/Point.h>
#include <tf/transform_listener.h>
#include <tf/message_filter.h>
#include <message_filters/subscriber.h>
#include <dataAssoc_experimental/PedDataAssoc.h>
#include <dataAssoc_experimental/PedDataAssoc_vector.h>
#include <geometry_msgs/PolygonStamped.h>
#include <fmutil/fm_math.h>

#include <dataAssoc_experimental/CameraParamConfig.h>
#include <dynamic_reconfigure/server.h>

#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>

#include "opencv2/opencv.hpp"
#include "cv_bridge/cv_bridge.h"

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

using namespace message_filters;
using namespace cv;
using namespace std;

#define NN_MATCH_THRESHOLD 1.0
#define NN_ANG_MATCH_THRESHOLD 0.0873 //5 degree

struct centroid_local_global
{
    geometry_msgs::Point32 global_centroid;
    geometry_msgs::Point32 local_centroid;
};

struct local_lPedInView
{
    int id;
    geometry_msgs::PointStamped location;
};

struct PedImgHash
{
    int id;
    vector<double> image_hash;
};

struct MergeID
{
    vector<PedImgHash> peds;
};

class MergeList
{
    //management of merge list and to disambiguate clusters based on image color histogram

public:
    vector<MergeID> merged_ids;
    MergeList(ros::NodeHandle nh){};

    void erase_merge_lists(int id)
    {
        //erase the split cluster id and resolve the ambiguity by looking at color histogram
        for(size_t j=0; j<merged_ids.size(); )
        {
            for(size_t k=1; k<merged_ids[j].peds.size(); k++)
            {
                if(merged_ids[j].peds[k].id==id)
                {
                    //assert(merged_ids[j].id.size()==merged_ids[j].image_hash.size());
                    merged_ids[j].peds.erase(merged_ids[j].peds.begin()+k);
                    //merged_ids[j].image_hash.erase(merged_ids[j].image_hash.begin()+k);
                }
            }
            if(merged_ids[j].peds.size() == 1)
            {
                merged_ids.erase(merged_ids.begin()+j);
            }
            else j++;
        }
    }

    void create_merge_lists(PedImgHash source_id,  PedImgHash dest_id)
    {
        //search for current merge list and update the list accordingly
        //will create new list when the merged cluster not found
        MergeID new_merge_id;
        //check if the parent list is already exist
        bool merge_list_exist = false;
        for(size_t j=0; j<merged_ids.size(); j++)
        {
            for(size_t k=0; k<merged_ids[j].peds.size(); k++)
            {
                if(merged_ids[j].peds[k].id==source_id.id)
                {
                    merge_list_exist = true;
                    merged_ids[j].peds.push_back(dest_id);
                    k = merged_ids[j].peds.size();
                }
            }
            if(merge_list_exist) j = merged_ids.size();
        }
        if(!merge_list_exist)
        {
            new_merge_id.peds.push_back(source_id);
            new_merge_id.peds.push_back(dest_id);
            merged_ids.push_back(new_merge_id);
            ROS_WARN("New merge id created with %d %d with lists %d", new_merge_id.peds[0].id, new_merge_id.peds[1].id, merged_ids.size());
        }

    }

    void get_split_id()
    {

    }
};

class data_assoc
{
public:
    data_assoc(int argc, char** argv);
    //data_assoc();
    ~data_assoc();

private:
    ros::NodeHandle nh_;
    image_transport::SubscriberFilter image_sub_;
    image_transport::ImageTransport it_;
    void pedClustCallback(sensor_msgs::ImageConstPtr image,feature_detection::clustersConstPtr cluster_vector);
    void pedVisionAngularCallback(sensor_msgs::PointCloudConstPtr pedestrian_vision_angular);
    void publishPed(Mat img);
    bool transformPointToGlobal(std_msgs::Header header, geometry_msgs::Point32 input_point, geometry_msgs::Point32& output_point);
    bool transformGlobalToLocal(geometry_msgs::PointStamped& global_point, geometry_msgs::PointStamped& local_point);
    void cleanUp();
    void increaseConfidence(int id);
    void updatelPedInViewWithID(int id, sensing_on_road::pedestrian_vision& pd_vector);
    void updatelPedInView(sensing_on_road::pedestrian_vision& update_source, sensing_on_road::pedestrian_vision& update_dest);
    void resetLPedInViewDecisionflag();
    bool getlPedInViewCentroid(int id, geometry_msgs::Point32& centroid);

    void getLocalLPedInView(vector<local_lPedInView>& lPedInView_local);
    ros::Publisher pedPub_;
    ros::Publisher visualizer_;
    string frame_id_, global_frame_, camera_frame_;
    bool use_sim_time_;
    tf::TransformListener *listener_;
    tf::MessageFilter<feature_detection::clusters> * laser_tf_filter_;
    tf::MessageFilter<sensor_msgs::PointCloud> * vision_angular_tf_filter_;
    message_filters::Subscriber<feature_detection::clusters> pedClustSub_;
    message_filters::Subscriber<sensor_msgs::PointCloud> pedVisionAngularSub_;
    sensing_on_road::pedestrian_vision_batch lPedInView;
    double time_out_, poll_inc_, poll_dec_, threshold_;
    camera_project::camera_projector projector;
    std::vector<geometry_msgs::Point32> laser_latest_global_, laser_latest_local_;
    MergeList merge_lists;
    int latest_id;

    dynamic_reconfigure::Server<dataAssoc_experimental::CameraParamConfig> dynamic_server;
    dynamic_reconfigure::Server<dataAssoc_experimental::CameraParamConfig>::CallbackType dynamic_cb;
    void dynamic_callback(dataAssoc_experimental::CameraParamConfig &config, uint32_t level);
    int pixel_padding_;
    double laser_height_;

    void syncCallback(const sensor_msgs::ImageConstPtr image, const feature_detection::clustersConstPtr cluster_vector);
    bool imageProjection(Mat& img, sensing_on_road::pedestrian_vision& ped, bool generate_image_hash);
    void colorHist(cv::Mat& src, vector<double>& image_hash)
    {
        Mat hsv;
        cvtColor(src, hsv, CV_BGR2HSV);

        // let's quantize the hue to 30 levels
        // and the saturation to 32 levels
        int hbins = 10, sbins = 1;
        int histSize[] = {hbins, sbins};
        // hue varies from 0 to 179, see cvtColor
        float hranges[] = { 0, 180 };
        // saturation varies from 0 (black-gray-white) to
        // 255 (pure spectrum color)
        float sranges[] = { 0, 256 };
        const float* ranges[] = { hranges, sranges };
        MatND hist;
        // we compute the histogram from the 0-th and 1-st channels
        int channels[] = {0, 1};

        calcHist( &hsv, 1, channels, Mat(), // do not use mask
                  hist, 2, histSize, ranges,
                  true, // the histogram is uniform
                  false );
        double maxVal=0;
        minMaxLoc(hist, 0, &maxVal, 0, 0);

        int scale = 10;
        Mat histImg = Mat::zeros(sbins*scale, hbins*10, CV_8UC3);
        image_hash.clear();
        for( int h = 0; h < hbins; h++ )
            for( int s = 0; s < sbins; s++ )
            {
                float binVal = hist.at<float>(h, s);
                //int intensity = cvRound(binVal*255/maxVal);
                image_hash.push_back(binVal/maxVal);
                //cout << image_hash[image_hash.size()-1] << ' ';
                /*cvRectangle( histImg, Point(h*scale, s*scale),
                                     Point( (h+1)*scale - 1, (s+1)*scale - 1),
                                     Scalar::all(intensity),
                                     CV_FILLED );*/
            }

        //cout<<endl;
    }
};

#endif /* DATA_ASSOC_H_ */
