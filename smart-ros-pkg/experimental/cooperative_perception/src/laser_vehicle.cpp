/*
 * laser_vehicle.cpp
 *
 *  Created on: Sep 19, 2012
 *      Author: demian
 */

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PolygonStamped.h>
#include <fmutil/fm_math.h>
#include <fmutil/fm_stopwatch.h>

#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <tf/message_filter.h>
#include <message_filters/subscriber.h>

#include "pcl_ros/point_cloud.h"
#include "pcl/ros/conversions.h"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/centroid.h>
#include <pcl/common/eigen.h>
#include <pcl/common/common.h>
#include <pcl/common/pca.h>
#include <laser_geometry/laser_geometry.h>

using std::string;
using std::cout;
using std::endl;
using std::vector;

struct laser_special
{
    vector<double> angles;
    vector<double> ranges;
};

class LaserRangePoint
{
    sensor_msgs::PointCloud pts;

    laser_special laser;

public:
    LaserRangePoint(sensor_msgs::LaserScan &scan, tf::TransformListener &tf, string target_frame)
    {
        //scan.range_max += 2.0;
        laser_geometry::LaserProjection projector;

        try{projector.transformLaserScanToPointCloud(target_frame, scan, pts, tf, laser_geometry::channel_option::Index|laser_geometry::channel_option::Distance);}
        catch (tf::TransformException& e){ ROS_ERROR("%s",e.what());return;}

        for(size_t i=0; i<pts.points.size(); i++)
        {


            laser.angles.push_back(pts.channels[0].values[i]*scan.angle_increment+scan.angle_min);
            laser.ranges.push_back(pts.channels[1].values[i]);

            //project all to virtual plane
            pts.points[i].z = 0;

            //cout<<i<<" "<<laser.ranges[i]*sin(laser.angles[i])<<" "<<laser.ranges[i]*cos(laser.angles[i])<<" "<<pts.points[i].x<<" "<<pts.points[i].y<<endl;

        }

    }

    laser_special getRanges()
    {
        return laser;
    }

    sensor_msgs::PointCloud getPts()
    {
        return pts;
    }

};

class LaserVehicle
{
    message_filters::Subscriber<sensor_msgs::LaserScan> laser_scan_sub_;
    tf::MessageFilter<sensor_msgs::LaserScan>       *laser_scan_filter_;

    ros::NodeHandle *nh_;
    tf::TransformListener *tf_;

    ros::Publisher poly_pub_, segmented_pub_, filter_res_pub_;
    string target_frame_;
    laser_geometry::LaserProjection projector_;
    double disConti_thresh_;
    void scanCallback(sensor_msgs::LaserScanConstPtr scan)
    {
        sensor_msgs::PointCloud laser_cloud;
        sensor_msgs::LaserScan laser_scan = *scan;

        LaserRangePoint laser_range_pt(laser_scan, *tf_, target_frame_);
        if(laser_range_pt.getRanges().ranges.size() == 0) return;

        findDiscontinuousPoint(laser_range_pt);
        //detectVehicle(laser_cloud);
    }

    void findDiscontinuousPoint(LaserRangePoint &lrp)
    {
        laser_special ls = lrp.getRanges();
        vector<double> ranges = ls.ranges;
        vector<double> angles = ls.angles;
        vector<double> filter_res;
        vector<geometry_msgs::Point32> filter_pts;
        sensor_msgs::PointCloud segmented_pts, filter_res_vis;
        segmented_pts.header = lrp.getPts().header;
        for(size_t i=3; i<ranges.size()-3; i++)
        {
            /*double t1 = ranges[i-5] + ranges[i-4] + ranges[i-3];
            double t2 = ranges[i+3] + ranges[i+4] + ranges[i+5];
            double t3 = ranges[i-2] + ranges[i-1] + ranges[i];
            double t4 = ranges[i+2] + ranges[i+1] + ranges[i];*/
            double t1 = ranges[i-3] + ranges[i-2];
            double t2 = ranges[i+2] + ranges[i+3];
            double t3 = ranges[i-1] + ranges[i];
            double t4 = ranges[i+1] + ranges[i];
            double r = t1 + t2 - t3 - t4;
            filter_res.push_back(r);
            filter_pts.push_back(lrp.getPts().points[i]);
            geometry_msgs::Point32 p = lrp.getPts().points[i];

            //although direct assignment is possible using lrp.getPts(), below is for consistency checking
            //p.x = ranges[i] * cos(angles[i]);
            //p.y = ranges[i] * sin(angles[i]);
            p.z = r*0.1;
            filter_res_vis.points.push_back(p);

        }
        filter_res_vis.header = lrp.getPts().header;
        filter_res_pub_.publish(filter_res_vis);


        //segmentation using discontinuous point
        unsigned int pre_minmax_pt = 2;
        unsigned int minmax_pt_count = 0;


        for(size_t ip=2; ip<filter_res.size()-2; ip++)
        {
            float temp_responsein2 = filter_res[ip-2];
            float temp_responsein1 = filter_res[ip-1];
            float temp_responsei   = filter_res[ip];
            float temp_responseip1 = filter_res[ip+1];
            float temp_responseip2 = filter_res[ip+2];
            //try to find local "maxima" and "minima";
            bool  temp_maxima=(temp_responsei>=temp_responsein1)&&(temp_responsei>temp_responsein2)&&(temp_responsei>=temp_responseip1)&&(temp_responsei>temp_responseip2);
            bool  temp_minima=(temp_responsei<=temp_responsein1)&&(temp_responsei<temp_responsein2)&&(temp_responsei<=temp_responseip1)&&(temp_responsei<temp_responseip2);
            bool  maxima2=false;
            bool  minima2=false;

            if(temp_maxima||temp_minima)
            {
                if(temp_maxima &&(temp_responsei>disConti_thresh_)) {maxima2=true;}
                else if(temp_minima&&(temp_responsei<-disConti_thresh_)) {minima2=true;}
                else{}
            }

            if(maxima2||minima2)
            {

                if(ip - pre_minmax_pt > 3)
                for(size_t j=pre_minmax_pt; j<ip; j++)
                {
                    geometry_msgs::Point32 p = filter_pts[j];
                    //p.x = filter_res[ip];
                    //p.y = ip;
                    //p.z = minmax_pt_count;
                    p.z = minmax_pt_count;// 0.1*filter_res[ip];
                    segmented_pts.points.push_back(p);
                }
                pre_minmax_pt = ip;
                minmax_pt_count++;
            }
        }
        //adding final segment
        if(filter_res.size()-2 - pre_minmax_pt > 3)
        {
            if(filter_res.size()-2 - pre_minmax_pt > 3)
                for(size_t j=pre_minmax_pt; j<filter_res.size()-2; j++)
                {
                    geometry_msgs::Point32 p = filter_pts[j];
                    //p.x = filter_res[ip];
                    //p.y = ip;
                    //p.z = minmax_pt_count;
                    p.z = minmax_pt_count;// 0.1*filter_res[ip];
                    segmented_pts.points.push_back(p);
                }
            pre_minmax_pt = filter_res.size()-2;
            minmax_pt_count++;

        }
        /*if(segmented_pts.points.size()>3)
        {
            pcl::PointCloud<pcl::PointXYZ> segmented_pcl;
            sensor_msgs::PointCloud2 segmented_pts2;
            sensor_msgs::convertPointCloudToPointCloud2(segmented_pts, segmented_pts2);
            pcl::fromROSMsg(segmented_pts2, segmented_pcl);

            pcl::PCA<pcl::PointXYZ> pca;
            pca.setInputCloud(segmented_pcl.makeShared());
            Eigen::Matrix3f eigen_vectors = pca.getEigenVectors();
            Eigen::Vector3f eigen_values = pca.getEigenValues();
            cout<<eigen_values<<endl;
            cout<<eigen_vectors<<endl<<endl;

            geometry_msgs::Point32 pca_vector_pts;
            pca_vector_pts.x = eigen_vectors(0);
            pca_vector_pts.y = eigen_vectors(1);
            pca_vector_pts.z = 1.0;
            segmented_pts.points.push_back(pca_vector_pts);
            pca_vector_pts.x = eigen_vectors(3);
            pca_vector_pts.y = eigen_vectors(4);
            pca_vector_pts.z = 2.0;
            segmented_pts.points.push_back(pca_vector_pts);
        }*/
        segmented_pub_.publish(segmented_pts);
    }
    void detectVehicle(sensor_msgs::PointCloud laser_cloud)
    {

        //simply draw polygon for now
        geometry_msgs::PolygonStamped poly;
        poly.header = laser_cloud.header;
        poly.polygon.points.insert(poly.polygon.points.begin(), laser_cloud.points.begin(), laser_cloud.points.end());
        poly_pub_.publish(poly);

        // smoothing with a 3rd order curve
        for(size_t i=0; i<laser_cloud.points.size(); i++)
        {
            cout<<i<<" "<<laser_cloud.points[i].x<<" "<<laser_cloud.points[i].y<<endl;
        }
        //find angle between 3 points
        /*
        vector<double> angle_bet_pts;
        for(size_t i=1; i<laser_cloud.points.size()-1; i++)
        {
            double angle1 = atan2(laser_cloud.points[i-1].y-laser_cloud.points[i].y, laser_cloud.points[i-1].x - laser_cloud.points[i].x);
            double angle2 = atan2(laser_cloud.points[i].y-laser_cloud.points[i+1].y, laser_cloud.points[i].x - laser_cloud.points[i+1].x);

            double angle_diff = angle2 - angle1;
            if(angle_diff > M_PI/2) angle_diff=- M_PI/2;
            if(angle_diff < -M_PI/2) angle_diff+= M_PI/2;
            cout<<i<<": "<<angle1<<"-"<<angle2<<" = "<<angle_diff<<endl;
        }*/

        //calculate angle difference between 1st angle to another
        /*for(size_t i=1; i<angle_bet_pts.size(); i++)
        {
            double angle_diff = angle_bet_pts[i] - angle_bet_pts[i-1];
            if(angle_diff > M_PI/2) angle_diff=- M_PI/2;
            if(angle_diff < -M_PI/2) angle_diff
        }*/
        /*sensor_msgs::PointCloud2 laser_cloud2;
        pcl::PointCloud<pcl::PointXYZ> laser_pcl;
        sensor_msgs::convertPointCloudToPointCloud2(laser_cloud, laser_cloud2);
        pcl::fromROSMsg(laser_cloud2, laser_pcl);
        segmentation(laser_pcl);*/
    }

public:
    LaserVehicle(): nh_(new ros::NodeHandle), tf_(new tf::TransformListener)
    {
        ros::NodeHandle private_nh("~");
        private_nh.param("target_frame", target_frame_, string("/base_link"));
        private_nh.param("discontinue_thres", disConti_thresh_, 0.45);
        laser_scan_sub_.subscribe(*nh_, "scan_in", 10);
        laser_scan_filter_ = new tf::MessageFilter<sensor_msgs::LaserScan>(laser_scan_sub_, *tf_, target_frame_, 10);
        laser_scan_filter_->registerCallback(boost::bind(&LaserVehicle::scanCallback, this, _1));
        poly_pub_ = nh_->advertise<geometry_msgs::PolygonStamped>("laser_polygon", 10);
        segmented_pub_ = nh_->advertise<sensor_msgs::PointCloud>("segmented_dist_p", 10);
        filter_res_pub_ = nh_->advertise<sensor_msgs::PointCloud>("filter_response", 10);
        cout<<"LH initialized"<<endl;
    }
};

int main(int argc, char** argcv)
{
    ros::init(argc, argcv, "laser_vehicle");
    LaserVehicle lv;
    ros::spin();
    return 0;
}
