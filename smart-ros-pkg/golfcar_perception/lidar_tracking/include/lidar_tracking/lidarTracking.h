#ifndef LIDAR_TRACKING_H
#define LIDAR_TRACKING_H

#include <sensor_msgs/LaserScan.h>
#include <ros/ros.h>
#include <deque>
#include <tf/transform_listener.h>

using namespace std;
using namespace ros;

class Cluster{
    public:
        double x;
        double y;
        double x_dot;
        double y_dot;
        double r;
        int count;
        double last_seen;

        Cluster();
        void addPoint(double x, double y, double t);
        void getVelocity(double* x_dot, double* y_dot);

    private:
        bool fitLine(double* slope, double* intercept);

        struct Point{double x, y, xx, xy, t;};
        deque<Point> points;
        double sumX;
        double sumY;
        double sumXX;
        double sumXY;
        int n;
        double curr_x_dot;
        double curr_y_dot;
        bool points_changed;
};

class LidarTracking{
    public:
        LidarTracking();
        void velocityCallback(const geometry_msgs::TwistConstPtr& msg);
        void lidarCallback(const sensor_msgs::LaserScanConstPtr& msg);
        void getClusters(vector<float> ranges, vector<float> angles, vector<Cluster>* raw_clusters);
        void forwardPropogateClusters(double t);
        void matchClusters(vector<Cluster> clusters, double t);
        void linearExtrapolate(ros::Time stamp);
    private:
        ros::Publisher dynObs_pub;
        ros::Publisher marker_pub;
        ros::Subscriber scan_sub;
        ros::Subscriber vel_sub;
        ros::NodeHandle n;
        vector<Cluster> clusters;
        double last_t;
        bool turning;
        string laser_link;
        string odom_frame;
        tf::TransformListener tf_;
};

#endif

