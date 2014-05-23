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
        //check if the parent list or dest_id is already exist
        bool src_exist = false, dest_exist = false;
        int merge_no;
        for(size_t j=0; j<merged_ids.size(); j++)
            for(size_t k=0; k<merged_ids[j].peds.size(); k++)
                if(merged_ids[j].peds[k].id==source_id.id) { src_exist = true; merge_no = j;break;}
        for(size_t j=0; j<merged_ids.size(); j++)
                    for(size_t k=0; k<merged_ids[j].peds.size(); k++)
                        if(merged_ids[j].peds[k].id==dest_id.id) dest_exist = true;

        if(!dest_exist && src_exist) merged_ids[merge_no].peds.push_back(dest_id);


        if(!src_exist)
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
    //void pedClustCallback(sensor_msgs::ImageConstPtr image,feature_detection::clustersConstPtr cluster_vector);
	void pedClustCallback(feature_detection::clustersConstPtr cluster_vector);
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
    ros::Publisher filtered_cluster_pub_;
    ros::Publisher pedPub_;
    ros::Publisher visualizer_;
    ros::Publisher visualize_good_object_;

    string frame_id_, global_frame_, camera_frame_;
    bool use_sim_time_;
    tf::TransformListener *listener_;
    tf::MessageFilter<feature_detection::clusters> * laser_tf_filter_;
    tf::MessageFilter<sensor_msgs::PointCloud> * vision_angular_tf_filter_;
    //message_filters::Subscriber<feature_detection::clusters> pedClustSub_;
	ros::Subscriber pedClustSub_;
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
    double laser_height_, color_cost_, dist_cost_, cost_threshold_, merge_dist_;
    unsigned char color_downsample(unsigned char color);
    void updatelPedInViewWithNewCluster(feature_detection::clusters& cluster_vector, Mat& img);
    void updateMergeList();
    void updateImageHash(Mat& img);
    void checkMergedlPedInView(Mat& img);
    void syncCallback(const sensor_msgs::ImageConstPtr image, const feature_detection::clustersConstPtr cluster_vector);
    bool imageProjection(Mat& img, std_msgs::Header& header, sensing_on_road::pedestrian_vision& ped, bool generate_image_hash);
    void getColorDiff(vector<double>& first, vector<double>& second, double& diff);
    void colorHist(cv::Mat& src, vector<double>& image_hash)
    {
        //cout<<"Start colorHist"<<endl;
        /*vector<Mat> split_bgr;
        split_bgr.resize(3);
        split(src, split_bgr);
        image_hash.erase(image_hash.begin(), image_hash.end());
        image_hash.resize(7*7*7);
        for(int i=0; i<split_bgr[1].cols; i++)
            for(int j=0; j<split_bgr[1].rows; j++)
            {
                int b = split_bgr[0].at<uchar>(j, i); b/36;
                int g = split_bgr[1].at<uchar>(j, i); g/36;
                int r = split_bgr[2].at<uchar>(j, i); r/36;
                cout<<"B: "<<b<<" G "<<g<<" R "<<r<<endl;
                image_hash[b*49+g*7+r]+=1.0;
            }
        for(size_t i=0; i<image_hash.size(); i++) cout<<image_hash[i]<<" ";
        cout<<endl<<"End colorHist"<<endl;*/
        int image_size = src.cols * src.rows;
        //no histogram is available when image_size is zero
        if(image_size == 0)
        {
            //image_hash.clear();
            return;
        }
        Mat hsv;
        cvtColor(src, hsv, CV_BGR2HSV);

        // let's quantize the hue to 30 levels
        // and the saturation to 32 levels
        int hbins = 15, sbins = 16;
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

        //minMaxLoc(hist, 0, &maxVal, 0, 0);

        int scale = 10;
        Mat histImg = Mat::zeros(sbins*scale, hbins*10, CV_8UC3);
        image_hash.clear();


        //cout <<"Image size "<< image_size<<" "<<src.cols<<" "<<src.rows;


        for( int h = 0; h < hbins; h++ )
            for( int s = 0; s < sbins; s++ )
            {
                float binVal = hist.at<float>(h, s);
                image_hash.push_back(binVal/image_size);
                //cout << image_hash[image_hash.size()-1] << ' ';
            }

        //cout<<"Accumulated: "<<accumulated_binval<<endl;
    }
};

#endif /* DATA_ASSOC_H_ */
