#include <math.h>

#include <string>
#include <vector>

#include <ros/ros.h>
#include <ros/console.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Point32.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <nav_core/base_local_planner.h>
#include <costmap_2d/costmap_2d.h>
#include <costmap_2d/costmap_2d_publisher.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/cost_values.h>

#include <pluginlib/class_list_macros.h>

#include "golfcar_purepursuit.h"
#include <pnc_msgs/move_status.h>
#include <pnc_msgs/poi.h>

using namespace std;
using namespace costmap_2d;

namespace golfcar_purepursuit
{


class PurePursuitBase : public nav_core::BaseLocalPlanner
{
public:
    void initialize(string name, tf::TransformListener* tf, costmap_2d::Costmap2DROS* costmap_ros);
    bool computeVelocityCommands(geometry_msgs::Twist& cmd_vel);
    bool isGoalReached();
    bool setPlan(const vector<geometry_msgs::PoseStamped>& orig_global_plan);

    PurePursuit* pp_;

private:
    ros::Subscriber poi_sub_;
    ros::Time expected_end_;
    tf::TransformListener* tf_;
    ros::Subscriber odom_sub_;
    ros::Publisher l_plan_pub_,g_plan_pub_,clear_space_pub_,move_status_pub_;

    geometry_msgs::PolygonStamped clear_space_;
    costmap_2d::Costmap2DROS* costmap_ros_;
    geometry_msgs::PoseStamped robot_pose;
    vector<geometry_msgs::PointStamped> lookahead_points;

    bool path_flag_;
    double slow_speed_, maximum_speed_, highrisk_speed_;
    bool forward_;
    double steer_angle_;
    double tracking_dist_;

    bool stopped_;
    bool goalreached_;
    int waypointPassed_;

    void UpdatePosition();
    void golfcar_direction(geometry_msgs::Point32 p);
    void getFillCells(vector<MapLocation>& footprint);

    void PoiCallback(pnc_msgs::poi poi);
    double getSigDist(int *type);
    double getIntDist(geometry_msgs::Point* int_point);
    pnc_msgs::poi poi_;
};


void PurePursuitBase::initialize(string name, tf::TransformListener* tf,
                                 costmap_2d::Costmap2DROS* costmap_ros)
{
    ros::NodeHandle global_node;
    ros::NodeHandle private_nh("~/" + name);
    private_nh.param("slow_speed", slow_speed_, 0.5);
    private_nh.param("maximum_speed", maximum_speed_, 1.5);
    private_nh.param("highrisk_speed", highrisk_speed_, 0.5);
    private_nh.param("tracking_distance",tracking_dist_,10.0);
    l_plan_pub_ = global_node.advertise<nav_msgs::Path>("local_plan", 1);
    g_plan_pub_ = global_node.advertise<nav_msgs::Path>("global_plan", 1);
    clear_space_pub_ = global_node.advertise<geometry_msgs::PolygonStamped>("clear_space",1);
    move_status_pub_ = global_node.advertise<pnc_msgs::move_status>("move_status",1);
    poi_sub_ = global_node.subscribe("/poi", 1, &PurePursuitBase::PoiCallback, this);
    tf_ = tf;
    costmap_ros_ = costmap_ros;

    pp_ = new PurePursuit(costmap_ros_->getGlobalFrameID());
    forward_ = true;
    stopped_ = false;
    goalreached_=false;
    waypointPassed_ =-1;

    cout <<name <<endl;
}

void PurePursuitBase::PoiCallback(pnc_msgs::poi poi)
{
    poi_ = poi;
}

void PurePursuitBase::UpdatePosition()
{
    tf::Stamped<tf::Pose> global_pose;
    costmap_ros_->getRobotPose(global_pose);
    tf::poseStampedTFToMsg(global_pose, robot_pose);
}

double PurePursuitBase::getIntDist(geometry_msgs::Point* int_point)
{
    //the intersections points is strictly increasing, getting the distance is easier
    for(unsigned int i=0; i<poi_.int_pts.size();i++)
    {
        if(poi_.int_pts[i] >= pp_->path_n_)
        {
            double dist;
            (*int_point) = pp_->path_.poses[poi_.int_pts[i]].pose.position;
            if(pp_->current_pos_to_point_dist(poi_.int_pts[i], &dist)) return dist;
            else return -1;
        }
    }
    return -1;
}
double PurePursuitBase::getSigDist(int *type)
{
    if(poi_.sig_pts.size()==0) return -1;
    vector<double> distances;
    for(unsigned int i=0; i<poi_.sig_pts.size();i++)
    {

        if(poi_.sig_pts[i].points>=pp_->path_n_)
        {
            double dist;
            if(pp_->current_pos_to_point_dist(poi_.sig_pts[i].points, &dist)) distances.push_back(dist);
            else distances.push_back(numeric_limits<double>::max());
        }
        else distances.push_back(numeric_limits<double>::max());
    }
    double min_dist = distances[0];
    *type = poi_.sig_pts[0].type;
    for(unsigned int i=1; i<distances.size();i++)
    {
        if(distances[i]<min_dist)
        {
            min_dist = distances[i];
            *type = poi_.sig_pts[i].type;
        }
    }
    if(min_dist == numeric_limits<double>::max()) return -1;
    else return min_dist;
}
bool PurePursuitBase::computeVelocityCommands(geometry_msgs::Twist& cmd_vel){
    PurePursuitBase::UpdatePosition();
    vector<double> proposed_velocities;
    pnc_msgs::move_status move_status;
    pp_->vehicle_base_ = robot_pose.pose;

    double steer_angle,dist_to_goal;
    geometry_msgs::Point int_point;
    path_flag_= pp_->steering_control(steer_angle,dist_to_goal);
    move_status.dist_to_goal = dist_to_goal;
    move_status.dist_to_ints = getIntDist(&int_point);
    int sig_type=-1;
    move_status.dist_to_sig = getSigDist(&sig_type);
    move_status.sig_type = sig_type;
    move_status.int_point = int_point;
    if(path_flag_){


        steer_angle_=steer_angle;
        goalreached_=false;

        move_status.steer_angle= steer_angle_;

        //implementation of collision detection
        geometry_msgs::PointStamped pointst;

        pointst.header.frame_id = costmap_ros_->getBaseFrameID();
        vector<geometry_msgs::Point> robot_footprint=costmap_ros_->getRobotFootprint();
        //a rectangle footprint is assumed starting from lower left
        geometry_msgs::Point robot_FP_UL = robot_footprint[1], robot_FP_UR = robot_footprint[2];

        pointst.header.stamp = ros::Time();
        ros::Time current_time = ros::Time::now();
        lookahead_points.clear();
        //collision detection area
        pointst.point = robot_FP_UL; lookahead_points.push_back(pointst);
        pointst.point.x = robot_FP_UL.x+tracking_dist_*cos(steer_angle_); pointst.point.y = robot_FP_UL.y+tracking_dist_*sin(steer_angle_);  lookahead_points.push_back(pointst);
        pointst.point.x = robot_FP_UR.x+tracking_dist_*cos(steer_angle_);pointst.point.y = robot_FP_UR.y+tracking_dist_*sin(steer_angle_);  lookahead_points.push_back(pointst);
        pointst.point = robot_FP_UR;  lookahead_points.push_back(pointst);

        clear_space_.header.stamp = ros::Time();
        clear_space_.header.frame_id = costmap_ros_->getGlobalFrameID();
        clear_space_.polygon.points.clear();

        geometry_msgs::PointStamped temp;
        geometry_msgs::Point32 tempPoint32;
        vector<geometry_msgs::Point32> polygon_in_world;
        //get proper transform to convert world coordinate to costmap coordinate

        for(unsigned int i=0;i<lookahead_points.size();i++)
        {
            try{
                lookahead_points[i].header.stamp = ros::Time();
                tf_->transformPoint(costmap_ros_->getGlobalFrameID(),lookahead_points[i],temp);
                tempPoint32.x = temp.point.x;
                tempPoint32.y = temp.point.y;
                polygon_in_world.push_back(tempPoint32);
            }
            catch(tf::TransformException& e){
                cout << e.what();
                return false;
            }

        }

        //adding the new polygons for visualization and further process for costmap
        vector<geometry_msgs::Point32> observed_polygon;
        observed_polygon.push_back(polygon_in_world[0]);
        observed_polygon.push_back(polygon_in_world[3]);
        observed_polygon.push_back(polygon_in_world[2]);
        observed_polygon.push_back(polygon_in_world[1]);

        //for visualization
        for(unsigned int i=0;i<observed_polygon.size();i++)
            clear_space_.polygon.points.push_back(observed_polygon[i]);
        clear_space_pub_.publish(clear_space_);

        //evaluate on the cost map
        vector<MapLocation> critical_polygon_costmap;
        vector<MapLocation> observed_polygon_costmap;
        MapLocation ml;
        Costmap2D c2d;
        costmap_ros_->getCostmapCopy(c2d);

        vector<MapLocation> observed_polygon_cells;
        vector<MapLocation> critical_polygon_cells;

        for(unsigned int i=0;i<observed_polygon.size();i++)
        {
            c2d.worldToMap(observed_polygon[i].x, observed_polygon[i].y, ml.x, ml.y);
            observed_polygon_costmap.push_back(ml);
        }
        c2d.polygonOutlineCells(observed_polygon_costmap,observed_polygon_cells);

        //get fill cells using function from base_local_planner
        PurePursuitBase::getFillCells(observed_polygon_cells);

        //initialize obstacles as 99 meter
        double wx=99,wy=99;
        for(unsigned int i=0;i<observed_polygon_cells.size();i++)
        {
            //find out the nearest obstacles

            int cells_cost = (int)c2d.getCost(observed_polygon_cells[i].x, observed_polygon_cells[i].y);

            if(cells_cost==254)
            {
                double tempx, tempy;
                c2d.mapToWorld(observed_polygon_cells[i].x,observed_polygon_cells[i].y,tempx,tempy);
                geometry_msgs::PointStamped obs_point;
                geometry_msgs::PointStamped obs_point_base;
                try{

                    obs_point.header.stamp = ros::Time();
                    obs_point.header.frame_id = costmap_ros_->getGlobalFrameID();
                    obs_point.point.x = tempx;
                    obs_point.point.y = tempy;
                    tf_->transformPoint(costmap_ros_->getBaseFrameID(),obs_point,obs_point_base);
                }
                catch(tf::TransformException& e){
                    cout << e.what();
                    cout<<"p3"<<endl;
                    return false;
                }
                tempx = obs_point_base.point.x;
                tempy = obs_point_base.point.y;
                if((wx*wx+wy*wy)>(tempx*tempx+tempy*tempy))
                {
                    wx = tempx;
                    wy = tempy;
                }

            }
        }
        //now that wx and wy is the nearest obstacle, send it to speed profile generator for it
        ROS_DEBUG_STREAM("Nearest Obstacle in "<< costmap_ros_->getBaseFrameID()<<": "<<wx<<' '<<wy);
        move_status.obstacles_dist = sqrt(wx*wx + wy*wy) - robot_FP_UL.x;

        move_status.acc_dec = true;
        if(waypointPassed_!=pp_->path_n_)
        {
            ROS_INFO("Path %d/%d", pp_->path_n_, (int)pp_->path_.poses.size()-1);
            waypointPassed_=pp_->path_n_;
        }

    }
    else{


        if(pp_->path_n_<(int)pp_->path_.poses.size()-1)
        {
            move_status.emergency=-1;
            move_status.steer_angle = steer_angle_;
            ROS_WARN("Steering control at unknown state");
        }
        else
        {
            move_status.acc_dec = false;
            move_status.steer_angle = 0;
            if(!goalreached_)
            {
                ROS_INFO("No path found");
                goalreached_=true;
            }
        }


    }
    move_status_pub_.publish(move_status);
    return true;

}


bool PurePursuitBase::isGoalReached(){
    if(!path_flag_ && pp_->path_n_<(int)pp_->path_.poses.size()-1)
    {
        ROS_INFO("Goal reached");
        return true;
    }
    else return false;
    ROS_INFO("Goal %d/%d",pp_->path_n_,(int)pp_->path_.poses.size()-1);
}


bool PurePursuitBase::setPlan(const vector<geometry_msgs::PoseStamped>& orig_global_plan){
    path_flag_=true;
    pp_-> path_.poses = orig_global_plan;
    PurePursuitBase::UpdatePosition();
    vector<geometry_msgs::PoseStamped>::iterator it;
    it = pp_->path_.poses.begin();
    //fix me: not ideal if the vehicle is already travel some path,
    //it will follow the shortest path from current pos to the first point!
    //fixed: changes to path by extending the next start point and overlap with previous path
    //pp_-> path_.poses.insert(it, robot_pose);

    pp_-> initialized_ = false;
    pp_-> dist_to_final_point = 100;
    pp_-> path_n_ =0;

    return true;
}


//taken from base_local_planner
void PurePursuitBase::getFillCells(vector<MapLocation>& footprint){
    //quick bubble sort to sort pts by x
    MapLocation swap, pt;
    unsigned int i = 0;
    while(i < footprint.size() - 1){
        if(footprint[i].x > footprint[i + 1].x){
            swap = footprint[i];
            footprint[i] = footprint[i + 1];
            footprint[i + 1] = swap;
            if(i > 0)
                --i;
        }
        else
            ++i;
    }

    i = 0;
    MapLocation min_pt;
    MapLocation max_pt;
    unsigned int min_x = footprint[0].x;
    unsigned int max_x = footprint[footprint.size() -1].x;
    //walk through each column and mark cells inside the footprint
    for(unsigned int x = min_x; x <= max_x; ++x){
        if(i >= footprint.size() - 1)
            break;

        if(footprint[i].y < footprint[i + 1].y){
            min_pt = footprint[i];
            max_pt = footprint[i + 1];
        }
        else{
            min_pt = footprint[i + 1];
            max_pt = footprint[i];
        }

        i += 2;
        while(i < footprint.size() && footprint[i].x == x){
            if(footprint[i].y < min_pt.y)
                min_pt = footprint[i];
            else if(footprint[i].y > max_pt.y)
                max_pt = footprint[i];
            ++i;
        }

        //loop though cells in the column
        for(unsigned int y = min_pt.y; y < max_pt.y; ++y){
            pt.x = x;
            pt.y = y;
            footprint.push_back(pt);
        }
    }
}

} //namespace golfcar_purepursuit



PLUGINLIB_DECLARE_CLASS(golfcar_purepursuit, PurePursuitBase, golfcar_purepursuit::PurePursuitBase, nav_core::BaseLocalPlanner)
