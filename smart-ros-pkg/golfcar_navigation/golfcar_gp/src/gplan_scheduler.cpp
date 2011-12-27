#include <math.h>

#include <pluginlib/class_list_macros.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/PoseArray.h>
#include <golfcar_gp/gplan_scheduler.h>

using namespace std;


PLUGINLIB_DECLARE_CLASS(golfcar_gp, GlobalPlan, golfcar_gp::GlobalPlan, nav_core::BaseGlobalPlanner)


namespace golfcar_gp
{


void GlobalPlan::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros){
    ros::NodeHandle global_node;
    g_plan_pub_ = global_node.advertise<nav_msgs::Path>("global_plan", 1);
    slowZone_pub_ = global_node.advertise<geometry_msgs::PoseArray>("slowZone", 1, true);

    ROS_INFO("gplan initialized");
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
