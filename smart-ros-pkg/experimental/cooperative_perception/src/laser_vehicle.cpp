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

    ros::Publisher poly_pub_, segmented_pub_, filter_res_pub_, filter_size_pub_, vehicle_pub_;
    string target_frame_;
    int filter_pts_;
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

    inline pcl::PointCloud<pcl::PointXYZ> pointcloudToPCL(sensor_msgs::PointCloud &pc)
    {
        sensor_msgs::PointCloud2 pc2;
        sensor_msgs::convertPointCloudToPointCloud2(pc, pc2);
        pcl::PointCloud<pcl::PointXYZ> pcl;
        pcl::fromROSMsg(pc2, pcl);
        return pcl;
    }

    double findYawLeastSquare(pcl::PointCloud<pcl::PointXYZ> pts, int filter_pt)
    {
        //adapted from www.ccas.ru/mmes/educat/lab04k/02/least-squares.c
          double SUMx, SUMy, SUMxy, SUMxx, slope,
                 y_intercept;
          SUMx = 0; SUMy = 0; SUMxy = 0; SUMxx = 0;

          //erase some points near the edge to calculate only more stable points
          pcl::PointCloud<pcl::PointXYZ> p;
          for(size_t i=filter_pt; i<pts.size()-filter_pt;i++)
              p.push_back(pts.points[i]);

          for (size_t i=0; i<p.size(); i++) {
            SUMx = SUMx + p[i].x;
            SUMy = SUMy + p[i].y;
            SUMxy = SUMxy + p[i].x*p[i].y;
            SUMxx = SUMxx + p[i].x*p[i].x;
          }
          slope = ( SUMx*SUMy - p.size()*SUMxy ) / ( SUMx*SUMx - p.size()*SUMxx );
          y_intercept = ( SUMy - slope*SUMx ) / p.size();

          double y1 = y_intercept, x1 = 0;
          double y2 = slope + y_intercept, x2 = 1;
          double yaw = atan2(y1-y2, x1-x2);
          if(slope>=0) return yaw+M_PI/2;
          else return yaw-M_PI/2;
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

        vector<pcl::PointCloud<pcl::PointXYZ> > segmented_pcl;
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
                {
                    pcl::PointCloud<pcl::PointXYZ> pxyz;
                    for(size_t j=pre_minmax_pt; j<ip; j++)
                    {
                        geometry_msgs::Point32 p = filter_pts[j];
                        p.z = minmax_pt_count;// 0.1*filter_res[ip];
                        segmented_pts.points.push_back(p);
                        pxyz.push_back(pcl::PointXYZ(p.x, p.y, 0.0));
                    }
                    segmented_pcl.push_back(pxyz);
                }
                pre_minmax_pt = ip;
                minmax_pt_count++;
            }
        }
        //adding final segment
        if(filter_res.size()-2 - pre_minmax_pt > 3)
        {
                pcl::PointCloud<pcl::PointXYZ> pxyz;
                for(size_t j=pre_minmax_pt; j<filter_res.size()-2; j++)
                {
                    geometry_msgs::Point32 p = filter_pts[j];
                    p.z = minmax_pt_count;// 0.1*filter_res[ip];
                    segmented_pts.points.push_back(p);
                    pxyz.push_back(pcl::PointXYZ(p.x, p.y, 0.0));
                }
                segmented_pcl.push_back(pxyz);
        }

        //the points now contains only smooth curve, checking for length using simple bounding box and yaw with least square fit
        //and select the nearest one
        double final_yaw=0.0, nearest_dist=1e9;
        geometry_msgs::Point final_point;

        for(size_t i=0; i<segmented_pcl.size();)
        {
            pcl::PointXYZ pt_max, pt_min;
            pcl::getMinMax3D(segmented_pcl[i], pt_min, pt_max);
            double yaw = findYawLeastSquare(segmented_pcl[i], filter_pts_);
            double bounding_dist = fmutil::distance(pt_max.x, pt_max.y, pt_min.x, pt_min.y);

            if(bounding_dist < 1.5 && bounding_dist > 0.5 && fabs(yaw) < 45.0/180*M_PI)
            {
                i++;
                //final selection of curve using nearest dist
                pcl::PointXYZ pt_center((pt_max.x + pt_min.x)/2, (pt_max.y + pt_min.y)/2, 0);
                double bumper_dist = fmutil::distance(pt_center.x, pt_center.y, 0, 0);
                if(bumper_dist < nearest_dist)
                {
                    final_yaw = yaw;
                    final_point.x = pt_center.x;
                    final_point.y = pt_center.y;
                    nearest_dist = bumper_dist;
                }
            }
            else  segmented_pcl.erase(segmented_pcl.begin()+i);
        }
        pcl::PointCloud<pcl::PointXYZ> size_filtered_pcl;
        for(size_t i=0; i<segmented_pcl.size(); i++)
        {
            for(size_t j=0; j<segmented_pcl[i].size(); j++)
            {
                pcl::PointXYZ p = segmented_pcl[i].points[j];
                p.z = i;
                size_filtered_pcl.push_back(p);
            }
        }
        sensor_msgs::PointCloud2 size_filtered_pc2;
        pcl::toROSMsg(size_filtered_pcl, size_filtered_pc2);
        size_filtered_pc2.header = segmented_pts.header;
        filter_size_pub_.publish(size_filtered_pc2);

        segmented_pub_.publish(segmented_pts);
        if(segmented_pts.points.size()>0)
            publishVehicle(final_yaw, final_point, segmented_pts.header);
    }
    void publishVehicle(double yaw, geometry_msgs::Point pt, std_msgs::Header &header)
    {
        tf::Quaternion q;
        q.setRPY(0, 0, yaw);
        geometry_msgs::Quaternion vehicle_quat;
        tf::quaternionTFToMsg(q, vehicle_quat);

        geometry_msgs::PoseStamped vehicle_pose;
        vehicle_pose.header = header;
        geometry_msgs::Pose temp_pose;
        temp_pose.position = pt;
        temp_pose.orientation = vehicle_quat;
        vehicle_pose.pose = temp_pose;
        vehicle_pub_.publish(vehicle_pose);
    }

public:
    LaserVehicle(): nh_(new ros::NodeHandle), tf_(new tf::TransformListener)
    {
        ros::NodeHandle private_nh("~");
        private_nh.param("target_frame", target_frame_, string("/base_link"));
        private_nh.param("discontinue_thres", disConti_thresh_, 0.45);
        private_nh.param("filter_pts", filter_pts_, 3);
        laser_scan_sub_.subscribe(*nh_, "scan_in", 10);
        laser_scan_filter_ = new tf::MessageFilter<sensor_msgs::LaserScan>(laser_scan_sub_, *tf_, target_frame_, 10);
        laser_scan_filter_->registerCallback(boost::bind(&LaserVehicle::scanCallback, this, _1));
        poly_pub_ = nh_->advertise<geometry_msgs::PolygonStamped>("laser_polygon", 10);
        segmented_pub_ = nh_->advertise<sensor_msgs::PointCloud>("segmented_dist_p", 10);
        filter_res_pub_ = nh_->advertise<sensor_msgs::PointCloud>("filter_response", 10);
        filter_size_pub_ = nh_->advertise<sensor_msgs::PointCloud2>("size_filtered", 10);
        vehicle_pub_ = nh_->advertise<geometry_msgs::PoseStamped>("vehicle_pose", 10);
        cout<<"LV initialized"<<endl;
    }
};

int main(int argc, char** argcv)
{
    ros::init(argc, argcv, "laser_vehicle");
    LaserVehicle lv;
    ros::spin();
    return 0;
}
