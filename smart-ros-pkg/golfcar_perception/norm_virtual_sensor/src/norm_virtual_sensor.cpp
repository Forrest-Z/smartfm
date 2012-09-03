/*
 * norm_virtual_sensor.cpp
 *
 *  Created on: Sep 2, 2012
 *      Author: demian
 */

#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <tf/transform_listener.h>
#include <tf/message_filter.h>
#include <message_filters/subscriber.h>
#include <laser_geometry/laser_geometry.h>
#include <fmutil/fm_math.h>
#include <fmutil/fm_stopwatch.h>

#include "pcl/point_cloud.h"
#include "pcl_ros/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl/ros/conversions.h"
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/voxel_grid.h>

using namespace std;

class AccumulateData
{
    const string target_frame_;
    const double min_dist_;
    const unsigned int scan_buffer_;
    laser_geometry::LaserProjection projector_;
    tf::StampedTransform last_transform_;
    tf::StampedTransform sensor_transform_;
    vector<sensor_msgs::PointCloud> data_;

    bool checkDistance(const tf::StampedTransform& newTf)
    {
        tf::Transform odom_old_new  = last_transform_.inverse() * newTf;
        float tx, ty;
        tx = -odom_old_new.getOrigin().y();
        ty =  odom_old_new.getOrigin().x();
        float mov_dis = sqrtf(tx*tx + ty*ty);

        if(mov_dis > min_dist_) return true;
        else return false;
    }

public:

    bool new_data_;

    AccumulateData(string target_frame, double min_dist, double data_dist): target_frame_(target_frame), min_dist_(min_dist),
            scan_buffer_(data_dist/min_dist+15.0), new_data_(false)
    {
        cout<<"Accumulation at "<<scan_buffer_<<" scans "<<endl;
    }

    //add latest observation to the sensor and erase the old buffer if necessary
    void addData(sensor_msgs::PointCloud2 &src, tf::TransformListener &tf)
    {
        sensor_msgs::PointCloud scan;
        sensor_msgs::convertPointCloud2ToPointCloud(src, scan);
        tf::StampedTransform latest_transform;

        tf.lookupTransform(target_frame_, scan.header.frame_id, scan.header.stamp, latest_transform);
        sensor_transform_ = latest_transform;
        tf.transformPointCloud(target_frame_, scan, scan);
        if(data_.size() == 0)
        {
            last_transform_ = latest_transform;
            insertPointCloud(scan);
            return;
        }

        if(checkDistance(latest_transform))
        {
            if(insertPointCloud(scan))
            {
                last_transform_ = latest_transform;
                new_data_ = true;
            }
        }
    }

    void addData(sensor_msgs::LaserScan &scan, tf::TransformListener &tf)
    {
        tf::StampedTransform latest_transform;
        tf.lookupTransform(target_frame_, scan.header.frame_id, scan.header.stamp, latest_transform);
        sensor_transform_ = latest_transform;
        if(data_.size() == 0 )
        {
            //only happens during initialization
            last_transform_ = latest_transform;
            insertData(scan, tf);
            return;
        }

        if(checkDistance(latest_transform))
        {
            if(insertData(scan, tf))
            {
                last_transform_ = latest_transform;
                new_data_ = true;
            }
        }
    }

    bool insertPointCloud(sensor_msgs::PointCloud &scan)
    {
        data_.insert(data_.begin(), scan);
        if(data_.size()>scan_buffer_) data_.resize(scan_buffer_);
        return true;
    }

    bool insertData(sensor_msgs::LaserScan &scan, tf::TransformListener &tf)
    {
        sensor_msgs::PointCloud laser_cloud;

        try{projector_.transformLaserScanToPointCloud(target_frame_, scan, laser_cloud, tf);}
        catch (tf::TransformException& e){ ROS_ERROR("%s",e.what());return false;}

        return insertPointCloud(laser_cloud);
    }

    //obtain accumulated points so far that is sorted from new to old transformed by target frame
    bool getLatestAccumulated(vector<sensor_msgs::PointCloud> &data)
    {
        data = data_;

        //A simple flag to say we have new accumulated data
        bool new_data = new_data_;
        new_data_ = false;
        return new_data;
    }


};


class NormVirtualSensor
{
    AccumulateData *laser_accumulate_;
    string target_frame_;
    double norm_radius_search_, min_move_dist_;
    vector<pcl::PointCloud<pcl::PointXYZRGBNormal> > accumulated_normals_;
    ros::Publisher accumulated_pub_;
    ros::Publisher final_pc2_pub_, final_pc_pub_;
    ros::Publisher latest_normal_pub_;
    tf::TransformListener *tf_;
    tf::StampedTransform cur_sensor_trans_;
    message_filters::Subscriber<sensor_msgs::LaserScan> laser_scan_sub_;
    tf::MessageFilter<sensor_msgs::LaserScan>       *laser_scan_filter_;

    message_filters::Subscriber<sensor_msgs::PointCloud2> pointcloud_sub_;
    tf::MessageFilter<sensor_msgs::PointCloud2>       *pointcloud_filter_;

    inline float labelToRGB(uint8_t i)
    {
        //pack scan number into RGB
        uint8_t r = i, g = 0, b = 0; // Example: Red color
        uint32_t rgb = ((uint32_t)r << 16 | (uint32_t)g << 8 | (uint32_t)b);
        return *reinterpret_cast<float*>(&rgb);
    }

    pcl::PointCloud<pcl::PointXYZRGB> labelScan(vector<sensor_msgs::PointCloud> &data, std_msgs::Header header)
    {
        pcl::PointCloud<pcl::PointXYZRGB> pcl;

        for(size_t i=0; i<data.size(); i++)
        {

            sensor_msgs::PointCloud2 temp;
            sensor_msgs::convertPointCloudToPointCloud2(data[i], temp);
            pcl::PointCloud<pcl::PointXYZ> pcl_temp;
            pcl::fromROSMsg(temp,  pcl_temp);
            pcl::PointCloud<pcl::PointXYZRGB> pclxyzrgbnorm_temp;
            pclxyzrgbnorm_temp.resize(pcl_temp.size());

            for(size_t j=0; j<pcl_temp.points.size(); j++)
            {
                pclxyzrgbnorm_temp.points[j].x = pcl_temp[j].x;
                pclxyzrgbnorm_temp.points[j].y = pcl_temp[j].y;
                pclxyzrgbnorm_temp.points[j].z = pcl_temp[j].z;
                pclxyzrgbnorm_temp.points[j].rgb = labelToRGB(i);
            }
            pcl += pclxyzrgbnorm_temp;
        }
        pcl_downsample(pcl);
        cout<<"Downsampled size "<<pcl.size()<<endl;
        sensor_msgs::PointCloud concatenated_pts;
        concatenated_pts.header = header;
        concatenated_pts.header.frame_id = target_frame_;
        concatenated_pts.points.resize(pcl.size());
        for(size_t i=0; i<pcl.size(); i++)
        {
            concatenated_pts.points[i].x = pcl.points[i].x;
            concatenated_pts.points[i].y = pcl.points[i].y;
            concatenated_pts.points[i].z = pcl.points[i].z;
        }
        accumulated_pub_.publish(concatenated_pts);
        return pcl;
    }

    pcl::PointCloud<pcl::PointXYZRGBNormal> normalEstimation(pcl::PointCloud<pcl::PointXYZRGB>& input)
    {
        //if(input.points.size() == 0) return;
        //down sampling moved to rolling windows
        fmutil::Stopwatch sw;

        // Create the normal estimation class, and pass the input dataset to it
        pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;
        ne.setInputCloud (input.makeShared());

        // Create an empty kdtree representation, and pass it to the normal estimation object.
        // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
        pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB> ());
        ne.setSearchMethod (tree);

        // Output datasets
        pcl::PointCloud<pcl::Normal> cloud_normals;

        // Use all neighbors in a sphere of radius 10cm
        ne.setRadiusSearch (norm_radius_search_);

        // Set use point using current sensor pose
        double viewpoint[] = {cur_sensor_trans_.getOrigin().x(), cur_sensor_trans_.getOrigin().y(), cur_sensor_trans_.getOrigin().z()};
        cout<<"Setting view point with ";
        for(size_t i=0; i<3; i++) cout<<viewpoint[i]<<" ";
        cout<<endl;
        ne.setViewPoint(viewpoint[0], viewpoint[1], viewpoint[2]);
        // Compute the features
        sw.start("Compute normals");
        ne.compute (cloud_normals);
        sw.end();
        // concatentate the fileds
        pcl::PointCloud<pcl::PointXYZRGBNormal> point_normals;
        sw.start("Concatenate fields");
        pcl::concatenateFields(input, cloud_normals, point_normals);
        sw.end();
        //cout<<"Point normal "<<point_normals.points[0].normal_x<<' '<<point_normals.points[0].normal_y<<' '<<point_normals.points[0].normal_z<<endl;
        // publish normal using visualization marker

        return point_normals;

    }

    pcl::PointCloud<pcl::PointXYZRGBNormal> filterNormal(pcl::PointCloud<pcl::PointXYZRGBNormal>& pcl_cloud)
    {
        fmutil::Stopwatch sw;
        stringstream ss;
        ss<<"Filter clouds with "<<pcl_cloud.size()<<" pts";
        sw.start(ss.str());
        unsigned int count_filtered=0, count_correct=0, count_raw=0;

        //absolutely stunningly quick!
        pcl::ConditionAnd<pcl::PointXYZRGBNormal>::Ptr range_cond (new pcl::ConditionAnd<pcl::PointXYZRGBNormal> ());
        range_cond->addComparison (pcl::FieldComparison<pcl::PointXYZRGBNormal>::ConstPtr (new
                pcl::FieldComparison<pcl::PointXYZRGBNormal> ("normal_z", pcl::ComparisonOps::LT, 0.5)));
        //range_cond->addComparison (pcl::FieldComparison<pcl::PointNormal>::ConstPtr (new
        //      pcl::FieldComparison<pcl::PointNormal> ("z", pcl::ComparisonOps::GT, 0.0)));
        pcl::ConditionalRemoval<pcl::PointXYZRGBNormal> condrem (range_cond);
        condrem.setInputCloud (pcl_cloud.makeShared());
        condrem.filter (pcl_cloud);

        pcl::ConditionAnd<pcl::PointXYZRGBNormal>::Ptr range_cond2 (new pcl::ConditionAnd<pcl::PointXYZRGBNormal> ());
        range_cond2->addComparison (pcl::FieldComparison<pcl::PointXYZRGBNormal>::ConstPtr (new
                pcl::FieldComparison<pcl::PointXYZRGBNormal> ("normal_z", pcl::ComparisonOps::GT, -0.5)));
        //range_cond2->addComparison (pcl::FieldComparison<pcl::PointNormal>::ConstPtr (new
        //      pcl::FieldComparison<pcl::PointNormal> ("z", pcl::ComparisonOps::GT, 0.0)));
        pcl::ConditionalRemoval<pcl::PointXYZRGBNormal> condrem2 (range_cond2);
        condrem2.setInputCloud (pcl_cloud.makeShared());
        condrem2.filter (pcl_cloud);
        sw.end();

        //perform density based filtering
        sw.start("Density filtering");
        pcl::PointCloud<pcl::PointXYZRGBNormal> radius_filtered_pcl2;// = pcl_cloud;
        pcl::RadiusOutlierRemoval<pcl::PointXYZRGBNormal> outrem;
        outrem.setInputCloud(pcl_cloud.makeShared());
        outrem.setRadiusSearch(0.125);
        outrem.setMinNeighborsInRadius(5);
        outrem.filter(radius_filtered_pcl2);
        sw.end();
        cout<<"Remaining clouds: "<<radius_filtered_pcl2.size()<<endl;
        return radius_filtered_pcl2;
    }

    void pcl_downsample(pcl::PointCloud<pcl::PointXYZRGB> &point_cloud)
    {
        pcl::VoxelGrid<pcl::PointXYZRGB> sor;
        // always good not to use in place filtering as stated in
        // http://www.pcl-users.org/strange-effect-of-the-downsampling-td3857829.html
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_msg_filtered (new pcl::PointCloud<pcl::PointXYZRGB> ());
        float downsample_size_ = 0.05;
        sor.setInputCloud(point_cloud.makeShared());
        sor.setLeafSize (downsample_size_, downsample_size_, downsample_size_);
        pcl::PointIndicesPtr pi;
        sor.filter (*input_msg_filtered);
        /*cout<<"Downsampled colors ";
        for(size_t i=0; i<input_msg_filtered->points.size(); i++)
        {
            cout<<input_msg_filtered->points[i].rgb<<" ";
        }
        cout<<endl;*/
        point_cloud = * input_msg_filtered;
    }

    void pcCallback(sensor_msgs::PointCloud2ConstPtr scan)
    {
        sensor_msgs::PointCloud2 pc2 = *scan;
        tf_->lookupTransform(target_frame_, scan->header.frame_id, scan->header.stamp, cur_sensor_trans_);
        laser_accumulate_->addData(pc2, *tf_);
        mainProcess(scan->header);
    }

    void scanCallback(sensor_msgs::LaserScanConstPtr scan)
    {
        sensor_msgs::LaserScan ls = *scan;
        tf_->lookupTransform(target_frame_, scan->header.frame_id, scan->header.stamp, cur_sensor_trans_);
        laser_accumulate_->addData(ls, *tf_);
        mainProcess(scan->header);
    }

    pcl::PointCloud<pcl::PointXYZRGBNormal> getLabeledPointCloud( pcl::PointCloud<pcl::PointXYZRGBNormal> &input, unsigned int label)
    {
        pcl::PointCloud<pcl::PointXYZRGBNormal> single_pcl;
        pcl::ConditionAnd<pcl::PointXYZRGBNormal>::Ptr range_cond (new pcl::ConditionAnd<pcl::PointXYZRGBNormal> ());
        range_cond->addComparison (pcl::FieldComparison<pcl::PointXYZRGBNormal>::ConstPtr (new
                pcl::FieldComparison<pcl::PointXYZRGBNormal> ("rgb", pcl::ComparisonOps::EQ, labelToRGB(label))));

        pcl::ConditionalRemoval<pcl::PointXYZRGBNormal> condrem (range_cond);
        condrem.setInputCloud (input.makeShared());
        condrem.filter (single_pcl);
        return single_pcl;
    }

    void mainProcess(std_msgs::Header header)
    {
        fmutil::Stopwatch sw;
        sw.start("++++Total++++++: ");
        vector<sensor_msgs::PointCloud> data;
        if(laser_accumulate_->getLatestAccumulated(data))
        {
            //RGB used as the scan_no
            pcl::PointCloud<pcl::PointXYZRGB> pcl_labeled = labelScan(data, header);
            pcl::PointCloud<pcl::PointXYZRGBNormal> temp = normalEstimation(pcl_labeled);
            pcl::PointCloud<pcl::PointXYZRGBNormal> processed_pcl = filterNormal(temp);
            pcl::PointCloud<pcl::PointXYZRGBNormal> single_pcl = getLabeledPointCloud(processed_pcl, (unsigned int)(data.size()/2.0));


            //get the latest strip of pcl which can be considered as unstable but useful to ensure latest information is incorporated when reconstruction
            pcl::PointCloud<pcl::PointXYZRGBNormal> latest_pcl;
            for(size_t i= 0; i<(unsigned int)(data.size()/2.0); i++)
            {
                latest_pcl += getLabeledPointCloud(processed_pcl, i);
            }
            cout<<"Injecting latest normal with "<<latest_pcl.size()<<" pts"<<endl;
            //rebuild normals with old and new normals
            accumulated_normals_.insert(accumulated_normals_.begin(), single_pcl);
            if(accumulated_normals_.size()>100) accumulated_normals_.resize(100);

            pcl::PointCloud<pcl::PointXYZRGBNormal> rebuild_normal;
            for(size_t i=0; i<accumulated_normals_.size(); i++)
            {
                rebuild_normal+=accumulated_normals_[i];
            }
            rebuild_normal+=latest_pcl;

            sensor_msgs::PointCloud2 out;
            pcl::toROSMsg(rebuild_normal, out);
            out.header = header;
            out.header.frame_id = target_frame_;
            final_pc2_pub_.publish(out);
            sensor_msgs::PointCloud out_pc_legacy;
            sensor_msgs::convertPointCloud2ToPointCloud(out, out_pc_legacy);
            final_pc_pub_.publish(out_pc_legacy);
            pcl::toROSMsg(latest_pcl, out);
            out.header = header;
            out.header.frame_id = target_frame_;
            latest_normal_pub_.publish(out);
            sw.end();
        }

        cout<<endl;
    }

public:
    NormVirtualSensor(): norm_radius_search_(0.1), min_move_dist_(0.02)
    {
        ros::NodeHandle n;

        tf_ = new tf::TransformListener();

        target_frame_ = "/odom";
        laser_accumulate_ = new AccumulateData(target_frame_, min_move_dist_, norm_radius_search_);

        laser_scan_sub_.subscribe(n, "scan_in", 10);
        laser_scan_filter_ = new tf::MessageFilter<sensor_msgs::LaserScan>(laser_scan_sub_, *tf_, target_frame_, 10);
        laser_scan_filter_->registerCallback(boost::bind(&NormVirtualSensor::scanCallback, this, _1));

        pointcloud_sub_.subscribe(n, "laser_surface_removed", 10);
        pointcloud_filter_ = new tf::MessageFilter<sensor_msgs::PointCloud2>(pointcloud_sub_, *tf_, target_frame_, 10);
        pointcloud_filter_->registerCallback(boost::bind(&NormVirtualSensor::pcCallback, this, _1));

        accumulated_pub_ = n.advertise<sensor_msgs::PointCloud>("accumulated_pts", 10);
        final_pc2_pub_ = n.advertise<sensor_msgs::PointCloud2>("single_pcl", 10);
        final_pc_pub_ = n.advertise<sensor_msgs::PointCloud>("pc_legacy_out", 10);
        latest_normal_pub_ = n.advertise<sensor_msgs::PointCloud2>("latest_normal", 10);
        ros::spin();
    }

};

int main(int argc, char** argcv)
{
    ros::init(argc, argcv, "norm_virtual_sensor");
    NormVirtualSensor nvs;
    return 0;
}


