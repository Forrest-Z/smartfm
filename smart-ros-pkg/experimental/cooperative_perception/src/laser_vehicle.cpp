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

#include "nearest_neighbor_tracking.h"

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
    double angle_increment;

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
        angle_increment = scan.angle_increment;

    }

    laser_special getRanges()
    {
        return laser;
    }

    double getAngleInc()
    {
        return angle_increment;
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
    tf::TransformBroadcaster *tf_broadcaster_;

    ros::Publisher poly_pub_, segmented_pub_, filter_res_pub_, filter_size_pub_, vehicle_pub_;
    ros::Publisher DP1_pub_, DP2_pub_, raw_RA_pub_, FR_RA_pub_, DP_RA_pub_;

    NearestNeighborTracking *nnt_;

    string target_frame_, veh_frame_;
    int filter_pts_;
    laser_geometry::LaserProjection projector_;
    bool target_set_;
    double disConti_thresh_, car_width_, width_tol_, angle_tol_, range_thres_;
    tf::StampedTransform latest_trans_;
    void scanCallback(sensor_msgs::LaserScanConstPtr scan)
    {
        sensor_msgs::PointCloud laser_cloud;
        sensor_msgs::LaserScan laser_scan = *scan;
        laser_scan.range_max = 30.0;
        LaserRangePoint laser_range_pt(laser_scan, *tf_, target_frame_);
        if(laser_range_pt.getRanges().ranges.size() == 0) return;

        //findDiscontinuousPoint(laser_range_pt);
        //DP_Extraction(laser_range_pt);
        simpleEuclideanExtraction(laser_range_pt);
        //detectVehicle(laser_cloud);

        //need to keep sending the data to keep the tf tree alive
        latest_trans_.stamp_ = ros::Time::now();
        tf_broadcaster_->sendTransform(latest_trans_);
    }

    inline pcl::PointCloud<pcl::PointXYZ> pointcloudToPCL(sensor_msgs::PointCloud &pc)
    {
        sensor_msgs::PointCloud2 pc2;
        sensor_msgs::convertPointCloudToPointCloud2(pc, pc2);
        pcl::PointCloud<pcl::PointXYZ> pcl;
        pcl::fromROSMsg(pc2, pcl);
        return pcl;
    }

    double findYawLeastSquarePCA(pcl::PointCloud<pcl::PointXYZ> pts, int filter_pt)
    {
        pcl::PointCloud<pcl::PointXYZ> p;
        //if(filter_pt> pts.points.size()-1) return M_PI;
        //if(pts.size()-filter_pt < 0) return M_PI;
        if(pts.size()<(unsigned int)(2*filter_pt)) return M_PI;
        for(size_t i=filter_pt; i<pts.size()-filter_pt;i++)
        {
            p.push_back(pts.points[i]);
        }
        if(p.size()<3) return M_PI;
        pcl::PCA<pcl::PointXYZ> pca;
        pca.setInputCloud(p.makeShared());
        Eigen::Matrix3f eigen_vec = pca.getEigenVectors();

        return atan2(eigen_vec(0), -eigen_vec(1));
    }
    double findYawLeastSquare(pcl::PointCloud<pcl::PointXYZ> pts, int filter_pt)
    {
        //changing the coordinate into normal xy coordinate
        if(pts.size()<(unsigned int)(2*filter_pt)) return M_PI;
        for(size_t i=0; i<pts.size(); i++)
        {
            double x = - pts.points[i].y;
            double y = pts.points[i].x;
            pts.points[i].x = x;
            pts.points[i].y = y;
        }
        //adapted from www.ccas.ru/mmes/educat/lab04k/02/least-squares.c
        double SUMx, SUMy, SUMxy, SUMxx, slope,
        y_intercept;
        SUMx = 0; SUMy = 0; SUMxy = 0; SUMxx = 0;

        //erase some points near the edge to calculate only more stable points
        pcl::PointCloud<pcl::PointXYZ> p;
        //cout<<"----------Start-----------"<<endl;
        for(size_t i=filter_pt; i<pts.size()-filter_pt;i++)
        {
            p.push_back(pts.points[i]);
            //cout<<pts.points[i].x<<" "<<pts.points[i].y<<endl;
        }


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

        //using the changed coordinate to properly calculate the orientation (x = -y, y = x)
        //cout<<slope<<endl;
        //cout<<"-----------end-------------"<<endl;
        double yaw = atan2(x1-x2, -(y1-y2));
        //if(slope>=0) return yaw+M_PI/2;
        //else return yaw-M_PI/2;
        return yaw+M_PI/2;
    }

    void simpleEuclideanExtraction(LaserRangePoint &lrp)
    {
        sensor_msgs::PointCloud laser_cloud  = lrp.getPts();

        ros::NodeHandle private_nh("~");
        double euc_dist_range;
        //smallest should be 1.0, which is really the nearest range possible from 2 beams
        private_nh.param("euc_dist_range", euc_dist_range, 1.1);
        int segmented_count = 0;
        //cout<<"----------Start-----------"<<endl;
        vector<pcl::PointCloud<pcl::PointXYZ> > segmented_pts;
        pcl::PointCloud<pcl::PointXYZ> segmented_pt;
        for(size_t i=1; i<laser_cloud.points.size(); i++)
        {
            double dist = fmutil::distance<geometry_msgs::Point32>(laser_cloud.points[i], laser_cloud.points[i-1]);
            double index_diff = laser_cloud.channels[0].values[i] - laser_cloud.channels[0].values[i-1];
            double angle_inc = index_diff * lrp.getAngleInc();
            double r = laser_cloud.channels[1].values[i-1];
            double euc_dist_thres = fabs(euc_dist_range*angle_inc*r);
            double range_diff = fabs(laser_cloud.channels[1].values[i] - laser_cloud.channels[1].values[i-1]);
            pcl::PointXYZ pxyz(laser_cloud.points[i].x, laser_cloud.points[i].y, 0.0);

            //adding another criteria, the range since the euc_dist_thres fails when euc_dist_range is to large
            if(dist>euc_dist_thres || range_diff>range_thres_)
            {
                segmented_count++;
                laser_cloud.points[i].z = segmented_count;
                //begin new collection
                segmented_pts.push_back(segmented_pt);
                segmented_pt.clear();
                segmented_pt.push_back(pxyz);
            }
            else
            {
                laser_cloud.points[i].z = segmented_count;
                segmented_pt.push_back(pxyz);
            }
           // cout <<segmented_count<<": "<<euc_dist_thres<<" "<<dist<<" "<<angle_inc<<" "<<r<<endl;
        }
        //push the last point if necessary
        if(segmented_pt.points.size()>0) segmented_pts.push_back(segmented_pt);
        //cout<<"-----------end-------------"<<endl;

        segmented_pub_.publish(laser_cloud);

        selectVehicle(segmented_pts, laser_cloud.header);
    }

    void DP_Extraction(LaserRangePoint &lrp)
    {
        sensor_msgs::PointCloud laser_cloud_laser_  = lrp.getPts();
        sensor_msgs::LaserScan laser_scan_;
        int     disFirst_multi_ = 4;
        double  disConti_thresh_ = 0.3;

        //first-order discontinuous points DP1;
        std::vector<unsigned int> DP1_IDs_;
        sensor_msgs::PointCloud DP1_pcl_;

        //second-order discontinuous points DP2;
        std::vector<unsigned int> DP2_IDs_;
        sensor_msgs::PointCloud DP2_pcl_;

        sensor_msgs::PointCloud raw_RA_pcl_, FR_RA_pcl_, DP_RA_pcl_;
        //to visulize data processing of DP2; x Range, y Angle, noted as RA; "filter response" is noted as FR

        DP1_IDs_.clear();
        DP1_pcl_.points.clear();
        DP1_pcl_.header=laser_cloud_laser_.header;


        for(unsigned int ip=0; ip<laser_cloud_laser_.points.size(); ip++)
        {
            if(ip==0) continue;
            unsigned int serial_tempn1 = (unsigned int)(laser_cloud_laser_.channels[0].values[ip-1]);
            unsigned int serial_tempp0 = (unsigned int)(laser_cloud_laser_.channels[0].values[ip]);
            float  range_tempn1 = laser_cloud_laser_.channels[1].values[ip-1];
            float  range_tempp0 = laser_cloud_laser_.channels[1].values[ip];
            if(fabsf(range_tempp0-range_tempn1)>lrp.getAngleInc() * range_tempp0 * disFirst_multi_ && (serial_tempp0-serial_tempn1)==1)
            {
                //DP1_IDs_ is optional;
                DP1_IDs_.push_back(ip);
                DP1_pcl_.points.push_back(laser_cloud_laser_.points[ip]);
            }
        }
        DP1_pub_.publish(DP1_pcl_);

        DP2_IDs_.clear();
        DP2_pcl_.points.clear();
        DP2_pcl_.header=laser_cloud_laser_.header;
        raw_RA_pcl_.points.clear();
        FR_RA_pcl_.points.clear();
        DP_RA_pcl_.points.clear();
        raw_RA_pcl_.header = laser_cloud_laser_.header;
        FR_RA_pcl_.header  = laser_cloud_laser_.header;
        DP_RA_pcl_.header  = laser_cloud_laser_.header;
        geometry_msgs::Point32 raw_RA_pt;
        geometry_msgs::Point32 FR_RA_pt;
        geometry_msgs::Point32 DP_RA_pt;

        //"raw_RA_pcl_";
        for(unsigned int ip=0; ip<laser_cloud_laser_.points.size(); ip++)
        {
            float           range_temp  = laser_cloud_laser_.channels[1].values[ip];
            //multiplied by 0.1 when shown, just for showing purpose here;
            raw_RA_pt.x = range_temp;
            raw_RA_pt.y = ip * 0.1;
            raw_RA_pt.z = 0;
            raw_RA_pcl_.points.push_back(raw_RA_pt);
        }


        //"FR_RA_pcl_";
        for(unsigned int ip=0; ip<raw_RA_pcl_.points.size(); ip++)            // pay attention to i;
        {
            float response_temp =0;

            //*******Higher order (5+1+5) to overcome noise**************
            if(ip<=4||ip>=raw_RA_pcl_.points.size()-5){response_temp = raw_RA_pcl_.points[ip].x;}
            else{
                response_temp = raw_RA_pcl_.points[ip+5].x+raw_RA_pcl_.points[ip+4].x+raw_RA_pcl_.points[ip+3].x+raw_RA_pcl_.points[ip-3].x+raw_RA_pcl_.points[ip-4].x+raw_RA_pcl_.points[ip-5].x;
                response_temp = response_temp-(raw_RA_pcl_.points[ip+2].x+raw_RA_pcl_.points[ip+1].x+raw_RA_pcl_.points[ip].x+raw_RA_pcl_.points[ip].x+raw_RA_pcl_.points[ip-1].x+raw_RA_pcl_.points[ip-2].x);
            }
            FR_RA_pt.x = response_temp;
            FR_RA_pt.y = raw_RA_pcl_.points[ip].y;
            FR_RA_pt.z = 0;
            FR_RA_pcl_.points.push_back(FR_RA_pt);
        }


        //find local minima and maxima to determine Discontinous Points(DP);
        for (unsigned int ip=0; ip<FR_RA_pcl_.points.size(); ip++)
        {
            if(4<ip && ip<FR_RA_pcl_.points.size()-5)
            {
                float temp_responsein2 = FR_RA_pcl_.points[ip-2].x;
                float temp_responsein1 = FR_RA_pcl_.points[ip-1].x;
                float temp_responsei   = FR_RA_pcl_.points[ip].x;
                float temp_responseip1 = FR_RA_pcl_.points[ip+1].x;
                float temp_responseip2 = FR_RA_pcl_.points[ip+2].x;
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
                    DP2_IDs_.push_back(ip);
                    DP_RA_pcl_.points.push_back(FR_RA_pcl_.points[ip]);
                    geometry_msgs::Point32 p = laser_cloud_laser_.points[ip];
                    //just to visualize the max/min pt
                    if(maxima2) p.z = 1.0;
                    else p.z = 0.0;
                    DP2_pcl_.points.push_back(p);
                }
            }
        }
        DP2_pub_.publish(DP2_pcl_);
        raw_RA_pub_.publish(raw_RA_pcl_);
        FR_RA_pub_.publish(FR_RA_pcl_);
        DP_RA_pub_.publish(DP_RA_pcl_);
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
        //cout<<"-------------start----------"<<endl;
        for(size_t i=3; i<ranges.size()-3; i++)
        {
            double t1 = ranges[i-5] + ranges[i-4] + ranges[i-3];
            double t2 = ranges[i+3] + ranges[i+4] + ranges[i+5];
            double t3 = ranges[i-2] + ranges[i-1] + ranges[i];
            double t4 = ranges[i+2] + ranges[i+1] + ranges[i];
            /*double t1 = ranges[i-3] + ranges[i-2];
            double t2 = ranges[i+2] + ranges[i+3];
            double t3 = ranges[i-1] + ranges[i];
            double t4 = ranges[i+1] + ranges[i];*/
            double r = t1 + t2 - t3 - t4;
            filter_res.push_back(r);
            filter_pts.push_back(lrp.getPts().points[i]);
            geometry_msgs::Point32 p = lrp.getPts().points[i];

            //although direct assignment is possible using lrp.getPts(), below is for consistency checking
            //p.x = ranges[i] * cos(angles[i]);
            //p.y = ranges[i] * sin(angles[i]);
            p.z = r*0.1;
            filter_res_vis.points.push_back(p);

            // cout<<p.x<<" "<<p.y<<" "<<p.z<<endl;

        }
        //cout<<"-------------end----------"<<endl;
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
        selectVehicle(segmented_pcl, segmented_pts.header);
        segmented_pub_.publish(segmented_pts);
    }

    void selectVehicle(vector<pcl::PointCloud<pcl::PointXYZ> > &segmented_pcl, std_msgs::Header header)
    {
        double final_yaw=0.0, nearest_dist=1e9;
        geometry_msgs::Point final_point;
        //cout<<"-------------start----------"<<endl;
        if(target_set_)
        {
            vector<geometry_msgs::PoseStamped> measured_poses;
            //cout<<"-------------start----------"<<endl;
            for(size_t i=0; i<segmented_pcl.size();i++)
            {
                if(segmented_pcl[i].size()<2) continue;

                pcl::PointXYZ pt_max, pt_min;
                pcl::getMinMax3D(segmented_pcl[i], pt_min, pt_max);
                double yaw = findYawLeastSquare(segmented_pcl[i], filter_pts_);
                pcl::PointXYZ pt_center((pt_max.x + pt_min.x)/2, (pt_max.y + pt_min.y)/2, 0);
                final_point.x = pt_center.x;
                final_point.y = pt_center.y;
                //cout<<"Added "<<final_point.x <<" "<<final_point.y<<" "<<yaw<<endl;
                measured_poses.push_back(convertToPoseStamped(yaw, final_point, header));
            }
            //cout<<"-------------end----------"<<endl;
            geometry_msgs::PoseStamped final_pose = nnt_->updateMeasurement(measured_poses);
            this->publishVehicle(final_pose);
        }
        else
        {
            for(size_t i=0; i<segmented_pcl.size();)
            {
                pcl::PointXYZ pt_max, pt_min;
                pcl::getMinMax3D(segmented_pcl[i], pt_min, pt_max);
                double yaw = findYawLeastSquare(segmented_pcl[i], filter_pts_);
                double bounding_dist = fmutil::distance(pt_max.x, pt_max.y, pt_min.x, pt_min.y);
                cout<<i<<": "<<bounding_dist<<" "<<yaw<<" "<<angle_tol_/180*M_PI<<endl;
                if(bounding_dist < car_width_+width_tol_ && bounding_dist > car_width_-width_tol_ && fabs(yaw) < angle_tol_/180*M_PI)
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
            //cout<<"-------------end----------"<<endl;
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
            size_filtered_pc2.header = header;
            filter_size_pub_.publish(size_filtered_pc2);
            //final_point.x += 0.45*cos(final_yaw);
            //final_point.y += 0.45*sin(final_yaw);
            if(size_filtered_pcl.points.size()>0)
            {
                geometry_msgs::PoseStamped vehicle_pose = convertToPoseStamped(final_yaw, final_point, header);
                nnt_ = new NearestNeighborTracking(vehicle_pose);
                target_set_ = true;
            }
        }


    }
    geometry_msgs::PoseStamped convertToPoseStamped(double yaw, geometry_msgs::Point pt, std_msgs::Header &header)
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
        return vehicle_pose;
    }

    void publishVehicle(geometry_msgs::PoseStamped vehicle_pose)
    {

        vehicle_pub_.publish(vehicle_pose);

        tf::StampedTransform trans(tf::Transform(), vehicle_pose.header.stamp, vehicle_pose.header.frame_id, veh_frame_);
        tf::poseMsgToTF(vehicle_pose.pose, trans);
        latest_trans_ = trans;
    }

public:
    LaserVehicle(): nh_(new ros::NodeHandle), tf_(new tf::TransformListener), tf_broadcaster_(new tf::TransformBroadcaster), target_set_(false)
    {

        ros::NodeHandle private_nh("~");
        private_nh.param("target_frame", target_frame_, string("/base_link"));
        private_nh.param("discontinue_thres", disConti_thresh_, 0.3);
        private_nh.param("filter_pts", filter_pts_, 0);
        private_nh.param("car_width", car_width_, 1.0);
        private_nh.param("width_tolerance", width_tol_, 0.5);
        private_nh.param("detect_angle_tolerance", angle_tol_, 90.0);
        private_nh.param("range_thres", range_thres_, 0.1);
        private_nh.param("detected_vehicle_frame", veh_frame_, string("/robot_1/base_link"));
        std::string::size_type delimiter_position( nh_->getNamespace().find('/') );

        //the getNamespace() return //namespace for some reason...
        //only search at most 2 delimited text, which is sufficient in this simple case
        /*std::string part1, part2;
        if ( std::string::npos != delimiter_position )
        {
            part1 =  nh_->getNamespace().substr(0, delimiter_position);
            part2 =  nh_->getNamespace().substr(delimiter_position+1) ;
        }
        veh_frame_ = string( part2 + "/detected_vehicle");*/

        laser_scan_sub_.subscribe(*nh_, "scan_in", 10);
        laser_scan_filter_ = new tf::MessageFilter<sensor_msgs::LaserScan>(laser_scan_sub_, *tf_, target_frame_, 10);
        laser_scan_filter_->registerCallback(boost::bind(&LaserVehicle::scanCallback, this, _1));
        poly_pub_ = nh_->advertise<geometry_msgs::PolygonStamped>("laser_polygon", 10);
        segmented_pub_ = nh_->advertise<sensor_msgs::PointCloud>("segmented_dist_p", 10);
        filter_res_pub_ = nh_->advertise<sensor_msgs::PointCloud>("filter_response", 10);
        filter_size_pub_ = nh_->advertise<sensor_msgs::PointCloud2>("size_filtered", 10);
        vehicle_pub_ = nh_->advertise<geometry_msgs::PoseStamped>("vehicle_pose", 10);

        DP1_pub_ = nh_->advertise<sensor_msgs::PointCloud>("DP1", 10);
        DP2_pub_ = nh_->advertise<sensor_msgs::PointCloud>("DP2", 10);
        raw_RA_pub_ = nh_->advertise<sensor_msgs::PointCloud>("raw_RA", 10);
        FR_RA_pub_ = nh_->advertise<sensor_msgs::PointCloud>("FR_RA", 10);
        DP_RA_pub_ = nh_->advertise<sensor_msgs::PointCloud>("DP_RA", 10);
        cout<<"LV initialized with detected frame at "<<veh_frame_<<endl;
    }
};

int main(int argc, char** argcv)
{
    ros::init(argc, argcv, "laser_vehicle");
    LaserVehicle lv;
    ros::spin();
    return 0;
}
