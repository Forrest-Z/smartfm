#ifndef PED_PATHPLAN_ROS_H_
#define PED_PATHPLAN_ROS_H_

#include <vector>
#include <ros/ros.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <nav_msgs/Path.h>
#include <tf/transform_datatypes.h>
#include <nav_core/base_global_planner.h>
#include <nav_msgs/GetPlan.h>
#include <ped_pathplan/pathplan.h>

namespace ped_pathplan {

    class PathPlanROS : public nav_core::BaseGlobalPlanner {
    public:
        PathPlanROS();
        PathPlanROS(std::string name, costmap_2d::Costmap2DROS* costmap_ros);
        void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros);
        bool makePlan(const geometry_msgs::PoseStamped& start, 
                const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan);

        bool makePlanService(nav_msgs::GetPlan::Request& req, nav_msgs::GetPlan::Response& resp);
		void publishPlan(const std::vector<geometry_msgs::PoseStamped>& path);

    protected:
      void updateCostmap(); 
      void mapToWorld(double mx, double my, double& wx, double& wy);
      bool initialized, allow_unknown;
      boost::shared_ptr<PathPlan> planner;
      costmap_2d::Costmap2D costmap;
      costmap_2d::Costmap2DROS* costmap_ros;
      std::string tf_prefix;
      boost::mutex mutex;
      ros::ServiceServer make_plan_srv;
      ros::Publisher plan_pub;
    };
}


#endif
