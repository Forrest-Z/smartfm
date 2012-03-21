#include <math.h>

#include <ros/ros.h>
#include <ros/console.h>

#include <pluginlib/class_list_macros.h>
#include <nav_core/base_global_planner.h>

#include <tf/transform_listener.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Point32.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>

#include <costmap_2d/costmap_2d.h>
#include <costmap_2d/costmap_2d_publisher.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/cost_values.h>

#include <StationPath.h>

#include <pnc_msgs/poi.h>

using namespace std;




namespace golfcar_gp
{


class GlobalPlan : public nav_core::BaseGlobalPlanner
{
public:
    void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros);
    bool makePlan(const geometry_msgs::PoseStamped& start,
                  const geometry_msgs::PoseStamped& goal,
                  std::vector<geometry_msgs::PoseStamped>& plan);

    ros::Publisher g_plan_pub_;
    ros::Publisher slowZone_pub_;
    ros::Publisher poi_pub_;
    ros::Subscriber stage_sub_;

    StationPaths sp_;
};



void GlobalPlan::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros){
    ros::NodeHandle global_node;
    g_plan_pub_ = global_node.advertise<nav_msgs::Path>("global_plan", 1, true);
    slowZone_pub_ = global_node.advertise<geometry_msgs::PoseArray>("slowZone", 1, true);
    poi_pub_ = global_node.advertise<pnc_msgs::poi>("poi", 1, true);

    ROS_INFO("gplan initialized");
}

void add_pts(vector<int>& pts, vector<pnc_msgs::sig_pt>* poi, int type)
{
    pnc_msgs::sig_pt sig_pt;
    for(unsigned int i=0; i<pts.size(); i++)
    {
        sig_pt.points = pts[i];
        sig_pt.type = type;
        poi->push_back(sig_pt);
    }
}

bool GlobalPlan::makePlan(const geometry_msgs::PoseStamped&  start,
                          const geometry_msgs::PoseStamped& goal,
                          std::vector<geometry_msgs::PoseStamped>& plan)
{
    //all in according to the pixels from global map
    geometry_msgs::Pose2D target;
    StationPath station_path = sp_.getPath(sp_.knownStations()((int)goal.pose.position.x),
                                           sp_.knownStations()((int)goal.pose.position.y));
    geometry_msgs::PoseArray slowZones;
    slowZones.header.frame_id = "/map";
    slowZones.header.stamp = ros::Time::now();

    for(unsigned int i=0;i<sp_.slowZones_.size();i++)
    {
        geometry_msgs::Pose pose;
        pose.orientation.w = 1.0;
        pose.position.x = sp_.slowZones_[i].x_;
        pose.position.y = sp_.slowZones_[i].y_;
        pose.position.z = sp_.slowZones_[i].r_;
        slowZones.poses.push_back(pose);
    }

    slowZone_pub_.publish(slowZones);

    //publish point of interest
    pnc_msgs::poi poi;
    add_pts(station_path.leftsig_pts_, &poi.sig_pts, 0);
    add_pts(station_path.rightsig_pts_, &poi.sig_pts, 1);
    add_pts(station_path.offsig_pts_, &poi.sig_pts, 2);

    //poi.rightsig_pts.insert(poi.rightsig_pts.begin(),station_path.rightsig_pts_.begin(),station_path.rightsig_pts_.end());
    //poi.offsig_pts.insert(poi.offsig_pts.begin(),station_path.offsig_pts_.begin(),station_path.offsig_pts_.end());// = station_path.leftsig_pts_;
    poi.int_pts.insert(poi.int_pts.begin(),station_path.ints_pts_.begin(),station_path.ints_pts_.end());// = station_path.leftsig_pts_;

    poi_pub_.publish(poi);

    vector<geometry_msgs::PoseStamped> final_targets;
    geometry_msgs::PoseStamped ps;

    nav_msgs::Path p;
    p.poses.resize(station_path.size());
    for(unsigned int i=0; i<station_path.size(); i++)
    {
        p.poses[i].header.frame_id = "/map";
        p.poses[i].header.stamp = ros::Time::now();
        p.poses[i].pose.position.x = station_path[i].x_;
        p.poses[i].pose.position.y = station_path[i].y_;
        p.poses[i].pose.orientation.w = 1.0;
        geometry_msgs::PoseStamped pl;
        pl.header.frame_id = "/map";
        pl.header.stamp = ros::Time::now();
        pl.pose.position.x = station_path[i].x_;
        pl.pose.position.y = station_path[i].y_;
        pl.pose.orientation.w = 1.0;
        plan.push_back(pl);
    }
    cout<<"Plan with "<<plan.size()<<" points sent."<<endl;
    p.header.stamp = ros::Time();
    p.header.frame_id = "/map";

    //p.poses.push_back(start);
    //p.poses.push_back(goal);
    g_plan_pub_.publish(p);
    //plan.push_back(targets);
    return true;
}

};

PLUGINLIB_DECLARE_CLASS(golfcar_gp, GlobalPlan, golfcar_gp::GlobalPlan, nav_core::BaseGlobalPlanner)

