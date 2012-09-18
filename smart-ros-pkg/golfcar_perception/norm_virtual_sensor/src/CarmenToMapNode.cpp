/*
 * CarmenToMapNode.cpp
 *
 *  Created on: Sep 16, 2012
 *      Author: demian
 */

#include <ros/ros.h>
#include "sensors/LaserReading.h"
#include "sensorstream/CarmenLog.h"
#include "sensorstream/LogSensorStream.h"
#include "sensorstream/SensorStream.h"
#include "utils/SimpleMinMaxPeakFinder.h"
#include <occupancy_grid_utils/ray_tracer.h>
#include <tf/transform_listener.h>

#include <boost/circular_buffer.hpp>
#include <boost/optional.hpp>
#include <boost/foreach.hpp>

using namespace std;

LogSensorStream m_sensorReference_(NULL,NULL);
occupancy_grid_utils::OverlayClouds *overlay_;
nav_msgs::MapMetaData mapMetaData_;
string global_frame_;

typedef boost::shared_ptr<occupancy_grid_utils::LocalizedCloud> CloudPtr;
typedef boost::shared_ptr<occupancy_grid_utils::LocalizedCloud const> CloudConstPtr;
typedef boost::circular_buffer<CloudConstPtr> CloudBuffer;

void computeBoundingBox(double &bbX, double &bbY, double &min_x, double &min_y){
    double minX =  1e17, minY =  1e17;
    double maxX = -1e17, maxY = -1e17;
    unsigned int position = m_sensorReference_.tell();
    m_sensorReference_.seek(0,END);
    unsigned int last = m_sensorReference_.tell();
    m_sensorReference_.seek(0);

    std::string bar(50, ' ');
    bar[0] = '#';
    unsigned int progress = 0;

    while(!m_sensorReference_.end()){
    unsigned int currentProgress = (m_sensorReference_.tell()*100)/last;
    if (progress < currentProgress){
        progress = currentProgress;
        bar[progress/2] = '#';
        std::cout << "\rComputing Bounding Box  [" << bar << "] " << (m_sensorReference_.tell()*100)/last << "%" << std::flush;
    }
    const LaserReading* lreadReference = dynamic_cast<const LaserReading*>(m_sensorReference_.next());
    if (lreadReference){
        const std::vector<Point2D>& points = lreadReference->getWorldCartesian();
        const std::vector<double>& rho = lreadReference->getRho();
        for(unsigned int i = 0; i < points.size(); i++){
        if(rho[i] >= lreadReference->getMaxRange()) continue;
        minX = minX < points[i].x ? minX : points[i].x;
        minY = minY < points[i].y ? minY : points[i].y;
        maxX = maxX > points[i].x ? maxX : points[i].x;
        maxY = maxY > points[i].y ? maxY : points[i].y;
        }
    }
    }
    bbX = (maxX - minX); bbY = (maxY - minY);
    m_sensorReference_.seek(position);
    min_x = minX, min_y = minY;
    std::cout << " done." << std::endl;

}

void computeMap(CloudBuffer &clouds, sensor_msgs::PointCloud &ros_clouds){
    unsigned int position = m_sensorReference_.tell();
    m_sensorReference_.seek(0,END);
    unsigned int last = m_sensorReference_.tell();
    m_sensorReference_.seek(0);

    std::string bar(50, ' ');
    bar[0] = '#';
    unsigned int progress = 0;


    while(!m_sensorReference_.end()){
        unsigned int currentProgress = (m_sensorReference_.tell()*100)/last;
        if (progress < currentProgress){
            progress = currentProgress;
            bar[progress/2] = '#';
            std::cout << "\rComputing Map  [" << bar << "] " << (m_sensorReference_.tell()*100)/last << "%" << last<<" "<< std::flush;
        }
        const LaserReading* lreadReference = dynamic_cast<const LaserReading*>(m_sensorReference_.next());

        ros_clouds.header.frame_id = global_frame_;
        if (lreadReference){
            const std::vector<Point2D>& points = lreadReference->getCartesian();
            const OrientedPoint2D pose = lreadReference->getLaserPose();
            const std::vector<double>& rho = lreadReference->getRho();

            sensor_msgs::PointCloud pc;
            pc.header.frame_id = global_frame_;

            for(unsigned int i = 0; i < points.size(); i+=2){
                if(rho[i] >= lreadReference->getMaxRange()) continue;
                geometry_msgs::Point32 pt;
                pt.x = points[i].x; pt.y = points[i].y;
                pc.points.push_back(pt);
            }
            ros_clouds.points.insert(ros_clouds.points.begin(), pc.points.begin(), pc.points.end());
            //cout<<pc.points.size()<<" ";
            CloudPtr localized_cloud(new occupancy_grid_utils::LocalizedCloud());
            //occupancy_grid_utils::LocalizedCloud localized_cloud;
            localized_cloud->header = pc.header;
            localized_cloud->sensor_pose.orientation = tf::createQuaternionMsgFromYaw(pose.theta);
            geometry_msgs::Point p;
            p.x = pose.x; p.y = pose.y; p.z = 0.0;
            localized_cloud->sensor_pose.position = p;
            localized_cloud->cloud = pc;
            clouds.push_back(localized_cloud);
        }
    }
    m_sensorReference_.seek(position);
    cout<<endl;
    std::cout << " done." << std::endl;
}

int main(int argc, char** argcv)
{
    ros::init(argc, argcv, "carmenToMap");
    ros::NodeHandle nh;
    ros::Publisher map_pub_ = nh.advertise<nav_msgs::OccupancyGrid>("/map", 1, true);
    ros::Publisher pc_pub_ = nh.advertise<sensor_msgs::PointCloud>("/map_pts", 1, true);
    string map_file;
    double resolution, min_occ, max_range;
    ros::NodeHandle private_nh("~");
    private_nh.param("map_file", map_file, string(""));
    private_nh.param("global_frame", global_frame_, string("/map"));
    private_nh.param("resolution", resolution, 0.05);
    private_nh.param("min_occ", min_occ, 0.07);
    private_nh.param("max_range", max_range, 80.0);
    CarmenLogWriter writer;
    CarmenLogReader reader;
    m_sensorReference_ = LogSensorStream(&reader, &writer);
    m_sensorReference_.load(map_file);
    double sizeX, sizeY, minX, minY;
    computeBoundingBox(sizeX, sizeY, minX, minY);
    std::cout << "Bounding Box = " << sizeX << "x" << sizeY << std::endl;
    std::cout << "Min Box = " << minX << "x" << minY << std::endl;
    nav_msgs::OccupancyGrid grid_info;
    grid_info.info.height = sizeY/resolution;
    grid_info.info.width = sizeX/resolution;
    geometry_msgs::Pose origin;
    origin.orientation.w = 1.0;
    origin.position.x = minX;
    origin.position.y = minY;
    grid_info.info.origin = origin;
    grid_info.info.resolution = resolution;
    grid_info.info.origin.orientation.w = 1.0;

    //occupancy_grid_utils::DEFAULT_MAX_DISTANCE;min_pass_through=occupancy_grid_utils::DEFAULT_MIN_PASS_THROUGH;
    occupancy_grid_utils::OverlayClouds overlay =
            occupancy_grid_utils::createCloudOverlay(grid_info, global_frame_, min_occ, max_range, 2);
    //vector<occupancy_grid_utils::LocalizedCloud> clouds_buffer;
    CloudBuffer clouds_buffer(3000);
    sensor_msgs::PointCloud pts;
    computeMap(clouds_buffer,pts);

    pts.header.frame_id = global_frame_;
    int count = 0;
    cout<<"cloud_buffer size: "<<clouds_buffer.size()<<endl;
    BOOST_FOREACH  (CloudConstPtr cloud, clouds_buffer)
    {
        //occupancy_grid_utils::LocalizedCloud::ConstPtr cloud_ptr = new occupancy_grid_utils::LocalizedCloud(cloud);
        occupancy_grid_utils::addCloud(&overlay, cloud);
        if(count++<200)
            pts.points.insert(pts.points.begin(), cloud->cloud.points.begin(), cloud->cloud.points.end());
    }

    pc_pub_.publish(pts);
    nav_msgs::OccupancyGrid::ConstPtr grid = occupancy_grid_utils::getGrid(overlay);
    map_pub_.publish(grid);
    ros::spin();
    return 0;
}
