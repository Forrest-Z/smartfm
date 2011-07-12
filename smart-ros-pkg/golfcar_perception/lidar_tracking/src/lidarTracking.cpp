#include <iostream>

#include <dynamic_obs_msgs/DynamicObstacles.h>
#include <dynamic_obs_msgs/DynamicObstacle.h>
#include <dynamic_obs_msgs/DynObsTrajectory.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <ros/ros.h>
#include <stdio.h>
#include <lidar_tracking/lidarTracking.h>
#include <geometry_msgs/Pose.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

using namespace std;
using namespace ros;
using namespace dynamic_obs_msgs;

#define VISUALIZE_DYNAMIC_OBSTACLES 1
#define VISUALIZE_OBSTACLE_TRAILS   1

#define MAX_COUNT 1600
#define MIN_COUNT 0
#define DYN_OBS_COUNT_THRESH 40

#define CLUSTER_TIMEOUT 15.0
#define CLUSTER_TOO_FAR 0.4
#define CLUSTER_MIN_VELOCITY 0.2
#define CLUSTER_MAX_ANGLE_DIFF (90*M_PI/180)
#define MAX_RADIUS_CHANGE 0.1

#define MAX_POINTS 100
#define MAP_PADDING 0.1

#define MAX_TURNING 0.1

#define MAX(a,b) (a > b ? a : b)

    LidarTracking::LidarTracking()
: tf_(ros::NodeHandle(), ros::Duration(10),	true)
{
    dynObs_pub = n.advertise<dynamic_obs_msgs::DynamicObstacles>("dynamic_obstacles", 1);

#if VISUALIZE_DYNAMIC_OBSTACLES
    marker_pub = n.advertise<visualization_msgs::MarkerArray>("/visualization_marker_array", 1);
#endif

    n.param("laser_link", laser_link,std::string("sick_laser"));
    n.param("map_frame",map_frame,std::string("map"));

    turning = false;
    bool track_when_turning;
    n.param("track_when_turning",track_when_turning, false);

    if(!track_when_turning)
        vel_sub = n.subscribe("cmd_vel", 1, &LidarTracking::velocityCallback,this);

    scan_sub = n.subscribe("sick_scan", 1, &LidarTracking::lidarCallback,this);
}

void LidarTracking::velocityCallback(const geometry_msgs::TwistConstPtr& msg){
    turning = fabs(msg->angular.z) > MAX_TURNING || (msg->angular.z != 0 && msg->linear.x == 0);
}

void LidarTracking::lidarCallback(const sensor_msgs::LaserScanConstPtr& msg)
{
    bool useScan = !turning;

    vector<float> angles;
    vector<float> ranges;
    int num_rays = msg->ranges.size();
    angles.reserve(num_rays);
    ranges.reserve(num_rays);
    float ang = msg->angle_min;
    for(int i=0; i<num_rays; i++){
        ranges.push_back(msg->ranges[i]);
        angles.push_back(ang);
        ang += msg->angle_increment;
    }

    double t = msg->header.stamp.toSec();
    vector<Cluster> temp;
    if(useScan)
        getClusters(ranges, angles, &temp);
    forwardPropogateClusters(t);
    if(useScan)
        matchClusters(temp, t);
    linearExtrapolate(msg->header.stamp);
    last_t = t;
}

void LidarTracking::getClusters(vector<float> ranges, vector<float> angles, vector<Cluster>* raw_clusters)
{
    // all in meters
    float minDist = 0.3;                        //min dist of cluster from laser
    float maxDist = 75;                         //max --,,--
    float clusterThreshold = 0.4;               // see below
    unsigned int clusterNMin = 4;               // min laser points in cluster
    float minWidth = 0.1;                       // min cluster width  r*theta
    float maxWidth = 0.7;                       // max --,,--

    geometry_msgs::PointStamped msg_out;
    geometry_msgs::PointStamped msg_in;
    msg_in.header.stamp = ros::Time();
    msg_in.header.frame_id = laser_link;
    msg_in.point.z = 0;

    for(unsigned int i=0; i<ranges.size(); i++)
    {
        unsigned int j;
        for(j=i; j<ranges.size()-1; j++)
        {
            if(fabs(ranges[j+1]-ranges[j]) > clusterThreshold)
                break;
        }
        //make sure cluster is large enough and isn't on the edge of our FOV
        if(j-i >= clusterNMin && i!=0 && j!=ranges.size()-1)
        {
            int upperMid = ceil(((float)(j+i))/2.);
            int lowerMid = (j+i)/2;
            float dist = (ranges[upperMid]+ranges[lowerMid])/2;
            float width = dist*(angles[j]-angles[i]);                                   //dist*tan((angles[j]-angles[i])/2);
            if(width <= maxWidth && width >= minWidth && dist <= maxDist && dist >= minDist)
            {
                Cluster c;
                float theta = (angles[j]+angles[i])/2;

                msg_in.point.x = dist*cos(theta);
                msg_in.point.y = dist*sin(theta);
                try{
                    tf_.transformPoint(map_frame, msg_in, msg_out);
                }
                catch (tf::TransformException ex){
                    ROS_ERROR("%s",ex.what());
                }

                c.x = msg_out.point.x;
                c.y = msg_out.point.y;
                c.r = width/2;

            }
        }
        i=j;
    }
    printf("%d raw clusters found\n",(int)raw_clusters->size());
}

// propagates clusters between scans
void LidarTracking::forwardPropogateClusters(double t)
{
    double dt = t-last_t;
    for(unsigned int i=0; i<clusters.size(); i++)
    {
        clusters[i].x += clusters[i].x_dot*dt;
        clusters[i].y += clusters[i].y_dot*dt;
        if(clusters[i].count > MIN_COUNT)
            clusters[i].count--;
        if(t-clusters[i].last_seen > CLUSTER_TIMEOUT)
        {
            clusters.erase(clusters.begin()+i);
            i--;
        }
    }
}

void LidarTracking::matchClusters(vector<Cluster> raw_clusters, double t)
{

    for(unsigned int i=0; i<raw_clusters.size(); i++)
    {
        raw_clusters[i].x_dot = 0;
        raw_clusters[i].y_dot = 0;

        int closest_idx = -1;
        double closest_dist = 100;
        for(unsigned int j=0; j<clusters.size(); j++)
        {
            double dx = raw_clusters[i].x - clusters[j].x;
            double dy = raw_clusters[i].y - clusters[j].y;
            double dist = sqrt(dx*dx+dy*dy);
            if(dist < closest_dist)
            {
                closest_dist = dist;
                closest_idx = j;
            }
        }
        if(closest_idx == -1 || closest_dist > CLUSTER_TOO_FAR)
        {
            //no match: make new cluster with low count
            printf("no match...\n");
            raw_clusters[i].x_dot = 0;
            raw_clusters[i].y_dot = 0;
            raw_clusters[i].last_seen = t;
            raw_clusters[i].count = 0;
            clusters.push_back(raw_clusters[i]);
            clusters.back().addPoint(raw_clusters[i].x,raw_clusters[i].y,t);
        }
        else
        {
            //this cluster is a match
            double dt = t-last_t;
            double old_x = clusters[closest_idx].x-clusters[closest_idx].x_dot*dt;
            double old_y = clusters[closest_idx].y-clusters[closest_idx].y_dot*dt;
            double new_x_dot = (raw_clusters[i].x - old_x)/dt;
            double new_y_dot = (raw_clusters[i].y - old_y)/dt;
            double x_dot = (clusters[closest_idx].x_dot + new_x_dot)/2;
            double y_dot = (clusters[closest_idx].y_dot + new_y_dot)/2;

            double mag_old = sqrt(clusters[closest_idx].x_dot*clusters[closest_idx].x_dot + clusters[closest_idx].y_dot*clusters[closest_idx].y_dot);
            double mag_new = sqrt(new_x_dot*new_x_dot + new_y_dot*new_y_dot);
            double dir_diff = acos(clusters[closest_idx].x_dot/mag_old*new_x_dot/mag_new + clusters[closest_idx].y_dot/mag_old*new_y_dot/mag_new);
            double v = sqrt(x_dot*x_dot + y_dot*y_dot);
            printf("v_old=(%f,%f) v_new(%f,%f)\n",clusters[closest_idx].x_dot,clusters[closest_idx].y_dot,new_x_dot,new_y_dot);

            clusters[closest_idx].addPoint(raw_clusters[i].x,raw_clusters[i].y,t);
            clusters[closest_idx].getVelocity(&x_dot, &y_dot);

            if(v < CLUSTER_MIN_VELOCITY || (!isnan(dir_diff) && dir_diff > CLUSTER_MAX_ANGLE_DIFF))
            {
                /*
                //|| fabs(clusters[closest_idx].r - raw_clusters[i].r) > MAX_RADIUS_CHANGE)
                //this cluster is a match but is consistently standing still
                //we should either do nothing or maybe reduce the count even more?
                */
                printf("bad match (%f,%f)\n",v,dir_diff*180/M_PI);
                clusters[closest_idx].last_seen = t;
                clusters[closest_idx].x = (clusters[closest_idx].x + raw_clusters[i].x)/2;
                clusters[closest_idx].y = (clusters[closest_idx].y + raw_clusters[i].y)/2;

                if(clusters[closest_idx].count > MIN_COUNT)
                    clusters[closest_idx].count--;

                clusters[closest_idx].x_dot = x_dot;
                clusters[closest_idx].y_dot = y_dot;
            }
            else
            {
                //this cluster is a match and is moving
                //update pose, velocity, and increase count
                printf("good match!\n");
                clusters[closest_idx].last_seen = t;
                clusters[closest_idx].x = (clusters[closest_idx].x + raw_clusters[i].x)/2;
                clusters[closest_idx].y = (clusters[closest_idx].y + raw_clusters[i].y)/2;
                clusters[closest_idx].x_dot = x_dot;
                clusters[closest_idx].y_dot = y_dot;
                clusters[closest_idx].r = (clusters[closest_idx].r + raw_clusters[i].r)/2;
                if(clusters[closest_idx].count < MAX_COUNT)
                    clusters[closest_idx].count += 8;

            }
        }
    }
    printf("%d clusters\n",(int)clusters.size());
}

void LidarTracking::linearExtrapolate(ros::Time stamp)
{
    double pred_sec = 5;
    double t_gran = 0.1;
    dynamic_obs_msgs::DynamicObstacles msg;
    msg.header.stamp = stamp;
    msg.header.frame_id = map_frame;

    int max_count = 0;
    for(unsigned int c=0; c<clusters.size(); c++)
    {
        if(clusters[c].count > max_count)
            max_count = clusters[c].count;
        if(clusters[c].count < DYN_OBS_COUNT_THRESH)
            continue;

        double x = clusters[c].x;
        double y = clusters[c].y;
        double x_dot, y_dot;
        clusters[c].getVelocity(&x_dot, &y_dot);
        double t = stamp.toSec();

        DynObsTrajectory traj;
        traj.exists_after = false;
        traj.probability = 1;
        for(int i=0; i<pred_sec/t_gran+1; i++)
        {
            geometry_msgs::PoseWithCovarianceStamped p;
            double timestep = i*t_gran;

            p.pose.pose.position.x = x + x_dot*timestep;
            p.pose.pose.position.y = y + y_dot*timestep;
            p.header.stamp = ros::Time(t + timestep);
            p.header.frame_id = map_frame;
            for(int i=0; i<36; i++)
                p.pose.covariance[i] = 0;

            //variance x
            p.pose.covariance[0] = 0.01;
            //variance y
            p.pose.covariance[7] = 0.01;
            p.pose.pose.orientation.w = 1;
            traj.points.push_back(p);
        }

        DynamicObstacle o;
        o.radius = clusters[c].r;
        o.trajectories.push_back(traj);
        msg.dyn_obs.push_back(o);
    }

    dynObs_pub.publish(msg);

    printf("max_count=%d\n",max_count);
    printf("%d dynamic obstacles\n",(int)msg.dyn_obs.size());

#if VISUALIZE_DYNAMIC_OBSTACLES
    int obs_idx = 0;
    int trail_idx = 0;
    int cluster_idx = 0;
    int dynObs_idx = 0;
    visualization_msgs::MarkerArray ma;
    for(unsigned int i=0; i<clusters.size(); i++){
        //if(clusters[i].count < DYN_OBS_COUNT_THRESH)
        //continue;

        geometry_msgs::Pose pose;
        pose.position.x = clusters[i].x;
        pose.position.y = clusters[i].y;
        pose.position.z = 0;

        visualization_msgs::Marker marker;
        marker.header = msg.header;
        marker.type = visualization_msgs::Marker::CYLINDER;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose = pose;
        marker.scale.x = clusters[i].r*2;
        marker.scale.y = clusters[i].r*2;
        marker.scale.z = 0.1;
        if(clusters[i].count < DYN_OBS_COUNT_THRESH){
            marker.color.r = 0.0f;
            marker.color.g = 0.0f;
            marker.color.b = 1.0f;
            marker.ns = "cluster";
            marker.id = cluster_idx;
            cluster_idx++;
        }
        else{
            marker.color.r = 1.0f;
            marker.color.g = 0.0f;
            marker.color.b = 0.0f;
            marker.ns = "dynamic_obstacle";
            marker.id = dynObs_idx;
            dynObs_idx++;
        }
        marker.color.a = 1.0;
        marker.lifetime = ros::Duration(CLUSTER_TIMEOUT-(stamp.toSec()-clusters[i].last_seen)); //ros::Duration();
        ma.markers.push_back(marker);

#if VISUALIZE_OBSTACLE_TRAILS
        if(clusters[i].count >= DYN_OBS_COUNT_THRESH){
            vector<geometry_msgs::PoseWithCovarianceStamped> trail = msg.dyn_obs[obs_idx].trajectories[0].points;
            obs_idx++;
            for(unsigned int j=0; j<trail.size(); j+=20){
                geometry_msgs::Pose pose;
                pose.position.x = trail[j].pose.pose.position.x;
                pose.position.y = trail[j].pose.pose.position.y;
                pose.position.z = 0;

                visualization_msgs::Marker marker;
                marker.header = msg.header;
                marker.ns = "obstacle_trail";
                marker.id = trail_idx;
                trail_idx++;
                marker.type = visualization_msgs::Marker::CYLINDER;
                marker.action = visualization_msgs::Marker::ADD;
                marker.pose = pose;
                marker.scale.x = clusters[i].r*2;
                marker.scale.y = clusters[i].r*2;
                marker.scale.z = 0.1;
                marker.color.r = 1.0f;
                marker.color.g = 0.0f;
                marker.color.b = 0.0f;
                marker.color.a = MAX(1.0-((double)j)/trail.size(),0.1);
                marker.lifetime = ros::Duration(0.5); //ros::Duration();
                ma.markers.push_back(marker);
            }
        }
#endif


    }
    marker_pub.publish(ma);
#endif

}

Cluster::Cluster()
{
    n = 0;
    sumX = 0;
    sumY = 0;
    sumXX = 0;
    sumXY = 0;
    curr_x_dot = 0;
    curr_y_dot = 0;
    points_changed = false;
}

void Cluster::addPoint(double x, double y, double t)
{
    if(n == MAX_POINTS){
        sumX -= points.front().x;
        sumY -= points.front().y;
        sumXX -= points.front().xx;
        sumXY -= points.front().xy;
        points.pop_front();
    }
    else
        n++;

    Point p;
    p.x = x;
    p.y = y;
    p.xx = x*x;
    p.xy = x*y;
    p.t = t;
    points.push_back(p);
    sumX += p.x;
    sumY += p.y;
    sumXX += p.xx;
    sumXY += p.xy;

    points_changed = true;
}

void Cluster::getVelocity(double* x_dot, double* y_dot)
{
    if(points_changed)
    {
        if(n < 2)
        {
            curr_x_dot = 0;
            curr_y_dot = 0;
        }
        else
        {
            double slope, intercept, ang1;
            if(fitLine(&slope, &intercept))
                ang1 = atan2(slope,1);
            else
                ang1 = M_PI/2;
            double dy = points.back().y-points.front().y;
            double dx = points.back().x-points.front().x;
            double ang2 = atan2(dy, dx);

            double ang_diff = ang2 - ang1;
            if(ang_diff < -M_PI) 
                ang_diff += 2*M_PI;
            if(ang_diff > M_PI) 
                ang_diff -= 2*M_PI;

            if(fabs(ang_diff) > M_PI/2)
                ang1 += M_PI;

            double mag = sqrt(dx*dx+dy*dy)/(points.back().t-points.front().t);
            curr_x_dot = mag*cos(ang1);
            curr_y_dot = mag*sin(ang1);
        }
        points_changed = false;
    }

    *x_dot = curr_x_dot;
    *y_dot = curr_y_dot;
}

bool Cluster::fitLine(double* slope, double* intercept)
{
    double Sx = n*sumXX - sumX*sumX;
    if(Sx == 0)
        return false;
    double Sxy = n*sumXY - sumX*sumY;
    double m = Sxy/Sx;
    double b = (sumY - m*sumX)/n;
    *slope = m;
    *intercept = b;
    return true;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "lidar_tracking");
    LidarTracking track;
    ros::spin();
}

