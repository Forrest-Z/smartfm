/** Local planner based on the pure-pursuit algorithm.
 *
 * The pure pursuit algorithm computes the steering command that will make the
 * vehicle follow the path. Basically, it looks ahead for a point on the path
 * and track that point.
 *
 * This is a ROS plugin for the navigation stack, implementing the front-end
 * nav_core::BaseLocalPlanner interface. The back end pure pursuit algoritm is
 * implemented in golfcar_purepursuit.h/cpp.
 */

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
#include <fmutil/fm_math.h>

#define POLY_S vector<geometry_msgs::PointStamped>
#define POLY vector<geometry_msgs::Point32>

using namespace std;
using namespace costmap_2d;

namespace golfcar_purepursuit
{


class PurePursuitBase : public nav_core::BaseLocalPlanner
{
public:
	// Functions required by nav_core::BaseLocalPlanner
    virtual ~PurePursuitBase();
    void initialize(string name, tf::TransformListener* tf, costmap_2d::Costmap2DROS* costmap_ros);
    bool computeVelocityCommands(geometry_msgs::Twist& cmd_vel);
    bool isGoalReached();
    bool setPlan(const vector<geometry_msgs::PoseStamped>& orig_global_plan);

private:
    PurePursuit* pp_;

    ros::Subscriber poi_sub_;
    tf::TransformListener* tf_;
    ros::Publisher l_plan_pub_, g_plan_pub_, clear_space_pub_, move_status_pub_;

    costmap_2d::Costmap2DROS* costmap_ros_;
    geometry_msgs::PoseStamped robot_pose_;
    pnc_msgs::poi poi_;

    int waypointPassed_;
    bool path_flag_;
    bool stopped_;
    bool goalreached_;
    bool forward_;
    double slow_speed_;
    double maximum_speed_;
    double highrisk_speed_;
    double steer_angle_;
    double tracking_dist_;

    void UpdatePosition();
    void golfcar_direction(geometry_msgs::Point32 p);
    void getFillCells(vector<MapLocation>& footprint);
    void PoiCallback(pnc_msgs::poi poi);
    double getSigDist(int *type);
    double getIntDist(geometry_msgs::Point* int_point);
    POLY_S get_lookahead_points();
    POLY convert_to_costmap_coord(const POLY_S &);
	void publish_clear_space(const POLY &);
	vector<MapLocation> get_costmap_cell_values(const POLY & poly);
	geometry_msgs::Point find_nearest_obstacle(const vector<MapLocation> & cells);
};


PurePursuitBase::~PurePursuitBase()
{
    delete pp_;
}


/// Required by the BaseLocalPlanner interface.
void PurePursuitBase::initialize(string name, tf::TransformListener* tf,
                                 costmap_2d::Costmap2DROS* costmap_ros)
{
	tf_ = tf;
	costmap_ros_ = costmap_ros;
	forward_ = true;
	stopped_ = false;
	goalreached_ = false;
	waypointPassed_ = -1;

    ros::NodeHandle global_node;
    ros::NodeHandle private_nh("~/" + name);

    private_nh.param("slow_speed", slow_speed_, 0.5);
    private_nh.param("maximum_speed", maximum_speed_, 1.5);
    private_nh.param("highrisk_speed", highrisk_speed_, 0.5);
    private_nh.param("tracking_distance",tracking_dist_, 10.0);

    l_plan_pub_ = global_node.advertise<nav_msgs::Path>("local_plan", 1);
    g_plan_pub_ = global_node.advertise<nav_msgs::Path>("global_plan", 1);
    clear_space_pub_ = global_node.advertise<geometry_msgs::PolygonStamped>("clear_space",1);
    move_status_pub_ = global_node.advertise<pnc_msgs::move_status>("move_status",1);
    poi_sub_ = global_node.subscribe("/poi", 1, &PurePursuitBase::PoiCallback, this);

    pp_ = new PurePursuit(costmap_ros_->getGlobalFrameID());
}

void PurePursuitBase::PoiCallback(pnc_msgs::poi poi)
{
    poi_ = poi;
}

void PurePursuitBase::UpdatePosition()
{
    tf::Stamped<tf::Pose> global_pose;
    costmap_ros_->getRobotPose(global_pose);
    tf::poseStampedTFToMsg(global_pose, robot_pose_);
}

double PurePursuitBase::getIntDist(geometry_msgs::Point* int_point)
{
	
    //the intersections points is strictly increasing, getting the distance is easier
    for( unsigned i=0; i<poi_.int_pts.size(); i++ )
    {
        if( poi_.int_pts[i] >= pp_->path_n_ )
        {
            double dist;
            *int_point = pp_->path_.poses[poi_.int_pts[i]].pose.position;
            if( pp_->current_pos_to_point_dist_simple(poi_.int_pts[i], &dist) )
            {
				ROS_DEBUG("int_pt %d, path_n %d", poi_.int_pts[i], pp_->path_n_);
				return dist;
			}
            else return -1;
        }
    }
    return -1;
}

double PurePursuitBase::getSigDist(int *type)
{
    if( poi_.sig_pts.size()==0 ) return -1;

    vector<double> distances;
    for( unsigned i=0; i<poi_.sig_pts.size(); i++ )
    {
        if(poi_.sig_pts[i].points>=pp_->path_n_)
        {
            double dist;
            if( pp_->current_pos_to_point_dist(poi_.sig_pts[i].points, &dist) )
            	distances.push_back(dist);
            else
            	distances.push_back(numeric_limits<double>::max());
        }
        else
            distances.push_back(numeric_limits<double>::max());
    }

    double min_dist = distances[0];
    *type = poi_.sig_pts[0].type;
    for( unsigned i=1; i<distances.size(); i++ )
    {
        if( distances[i] < min_dist )
        {
            min_dist = distances[i];
            *type = poi_.sig_pts[i].type;
        }
    }
    return min_dist == numeric_limits<double>::max() ? -1 : min_dist;
}

// Compute rectangle used for collision detection
POLY_S PurePursuitBase::get_lookahead_points()
{
	POLY_S lookahead_points;
	geometry_msgs::PointStamped pointst;
	pointst.header.frame_id = costmap_ros_->getBaseFrameID();
	pointst.header.stamp = ros::Time();
	vector<geometry_msgs::Point> robot_footprint = costmap_ros_->getRobotFootprint();
	//a rectangle footprint is assumed starting from lower left
	geometry_msgs::Point robot_FP_UL = robot_footprint[1];
	geometry_msgs::Point robot_FP_UR = robot_footprint[2];

	//collision detection area
	pointst.point = robot_FP_UL;
	lookahead_points.push_back(pointst);

	pointst.point.x = robot_FP_UL.x+tracking_dist_*cos(steer_angle_);
	pointst.point.y = robot_FP_UL.y+tracking_dist_*sin(steer_angle_);
	lookahead_points.push_back(pointst);

	pointst.point.x = robot_FP_UR.x+tracking_dist_*cos(steer_angle_);
	pointst.point.y = robot_FP_UR.y+tracking_dist_*sin(steer_angle_);
	lookahead_points.push_back(pointst);

	pointst.point = robot_FP_UR;
	lookahead_points.push_back(pointst);

	return lookahead_points;
}

//convert world coordinate to costmap coordinate
POLY PurePursuitBase::convert_to_costmap_coord(const POLY_S & in_poly)
{
	POLY polygon_in_world;
	for( unsigned i=0; i<in_poly.size(); i++ )
	{
		geometry_msgs::PointStamped inPt = in_poly[i], temp;
		inPt.header.stamp = ros::Time();
		tf_->transformPoint(costmap_ros_->getGlobalFrameID(), inPt, temp);
		geometry_msgs::Point32 tempPoint32;
		tempPoint32.x = temp.point.x;
		tempPoint32.y = temp.point.y;
		polygon_in_world.push_back(tempPoint32);
	}

	POLY out_poly;
	out_poly.push_back(polygon_in_world[0]);
	out_poly.push_back(polygon_in_world[3]);
	out_poly.push_back(polygon_in_world[2]);
	out_poly.push_back(polygon_in_world[1]);
	return out_poly;
}

void PurePursuitBase::publish_clear_space(const POLY & poly)
{
	geometry_msgs::PolygonStamped clear_space_msg;
	clear_space_msg.header.stamp = ros::Time();
	clear_space_msg.header.frame_id = costmap_ros_->getGlobalFrameID();
	clear_space_msg.polygon.points = poly;
	clear_space_pub_.publish(clear_space_msg);
}

// Retrieve cost map values on this polygon
vector<MapLocation> PurePursuitBase::get_costmap_cell_values(const POLY & poly)
{
	Costmap2D c2d;
	costmap_ros_->getCostmapCopy(c2d);
	vector<MapLocation> poly_cm;
	for( unsigned i=0; i<poly.size(); i++ )
	{
		MapLocation ml;
		c2d.worldToMap(poly[i].x, poly[i].y, ml.x, ml.y);
		poly_cm.push_back(ml);
	}

	vector<MapLocation> poly_cells;
	c2d.polygonOutlineCells(poly_cm, poly_cells);
	getFillCells(poly_cells);

	return poly_cells;
}

geometry_msgs::Point PurePursuitBase::find_nearest_obstacle(const vector<MapLocation> & cells)
{
	Costmap2D c2d;
	costmap_ros_->getCostmapCopy(c2d);

	double dmin = numeric_limits<double>::max();
	geometry_msgs::Point pmin;
	pmin.x = 99;
	pmin.y = 99;

	for( unsigned i=0; i<cells.size(); i++ )
	{
		if( (int)c2d.getCost(cells[i].x, cells[i].y)==254 )
		{
			geometry_msgs::PointStamped obs_point;
			obs_point.header.stamp = ros::Time();
			obs_point.header.frame_id = costmap_ros_->getGlobalFrameID();
	        c2d.mapToWorld(cells[i].x, cells[i].y,
	                obs_point.point.x, obs_point.point.y);

			geometry_msgs::PointStamped obs_point_base;
			tf_->transformPoint(costmap_ros_->getBaseFrameID(),
					obs_point, obs_point_base);

			double d = fmutil::mag(obs_point_base.point.x, obs_point_base.point.y);
			if( d < dmin )
			{
				pmin.x = obs_point_base.point.x;
				pmin.y = obs_point_base.point.y;
				dmin = d;
			}
		}
	}
	return pmin;
}

/// Required by the BaseLocalPlanner interface.
bool PurePursuitBase::computeVelocityCommands(geometry_msgs::Twist& cmd_vel)
{
    ROS_DEBUG("PurePursuitBase::computeVelocityCommands");
    UpdatePosition();
    pp_->vehicle_base_ = robot_pose_.pose;

    pnc_msgs::move_status move_status;

    double steer_angle, dist_to_goal;
    geometry_msgs::Point int_point;
    path_flag_= pp_->steering_control(&steer_angle, &dist_to_goal);
    move_status.dist_to_goal = dist_to_goal;
    move_status.dist_to_ints = getIntDist(&int_point);
    int sig_type=-1;
    move_status.dist_to_sig = getSigDist(&sig_type);
    move_status.sig_type = sig_type;
    move_status.int_point = int_point;

    if(path_flag_)
    {
        steer_angle_ = steer_angle;
        goalreached_ = false;
        move_status.steer_angle = steer_angle_;

        try
        {
			POLY observed_polygon = convert_to_costmap_coord( get_lookahead_points() );
			publish_clear_space(observed_polygon);

			vector<MapLocation> observed_polygon_cells =
					get_costmap_cell_values(observed_polygon);

			geometry_msgs::Point pmin = find_nearest_obstacle(observed_polygon_cells);
			geometry_msgs::Point robot_FP_UL = costmap_ros_->getRobotFootprint()[1];
	        move_status.obstacles_dist = fmutil::mag(pmin.x, pmin.y) - robot_FP_UL.x;

            ROS_DEBUG_STREAM("Nearest Obstacle in " << costmap_ros_->getBaseFrameID()
                    <<": " <<pmin.x <<' ' <<pmin.y <<". robot_FP_UL.x=" <<robot_FP_UL.x
                    <<". obstacles_dist=" <<move_status.obstacles_dist);

	        move_status.path_exist = true;
	        if( waypointPassed_ != pp_->path_n_ )
	        {
	            ROS_INFO("Path %d/%d", pp_->path_n_, (int)pp_->path_.poses.size()-1);
	            waypointPassed_ = pp_->path_n_;
	        }
        }
        catch (tf::TransformException & e) {
        	cout <<e.what();
        	return false;
        }
    }
    else
    {
        move_status.path_exist = false;
        if( pp_->path_n_ < (int) pp_->path_.poses.size()-1 )
        {
            //move_status.emergency = -1;
            move_status.steer_angle = steer_angle_;
            ROS_WARN("Steering control at unknown state");
        }
        else
        {

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

/// Required by the BaseLocalPlanner interface.
bool PurePursuitBase::isGoalReached()
{
       UpdatePosition();
       pp_->vehicle_base_ = robot_pose_.pose;

       double steer_angle, dist_to_goal;
       pp_->steering_control(&steer_angle, &dist_to_goal);
       //ROS_INFO("Distance to goal: %lf", dist_to_goal);
       if( dist_to_goal< 5.0)
       {
           ROS_INFO("Goal Reached");
           return true;
       }
       return false;
}

/// Required by the BaseLocalPlanner interface.
bool PurePursuitBase::setPlan(const vector<geometry_msgs::PoseStamped>& orig_global_plan)
{
    path_flag_=true;
    pp_-> path_.poses = orig_global_plan;
    UpdatePosition();
    vector<geometry_msgs::PoseStamped>::iterator it;
    it = pp_->path_.poses.begin();
    //fix me: not ideal if the vehicle is already travel some path,
    //it will follow the shortest path from current pos to the first point!
    //fixed: changes to path by extending the next start point and overlap with previous path
    //pp_-> path_.poses.insert(it, robot_pose_);

    pp_-> initialized_ = false;
    pp_-> dist_to_final_point = 100;
    pp_-> path_n_ =0;

    return true;
}


//taken from base_local_planner/trajectory_planner.cpp
void PurePursuitBase::getFillCells(vector<MapLocation>& footprint)
{
    //quick bubble sort to sort pts by x
    MapLocation swap, pt;
    unsigned int i = 0;
    while(i < footprint.size() - 1)
    {
        if(footprint[i].x > footprint[i + 1].x)
        {
            swap = footprint[i];
            footprint[i] = footprint[i + 1];
            footprint[i + 1] = swap;
            if(i > 0) --i;
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
    for(unsigned int x = min_x; x <= max_x; ++x)
    {
        if(i >= footprint.size() - 1)
            break;

        if(footprint[i].y < footprint[i + 1].y)
        {
            min_pt = footprint[i];
            max_pt = footprint[i + 1];
        }
        else
        {
            min_pt = footprint[i + 1];
            max_pt = footprint[i];
        }

        i += 2;
        while(i < footprint.size() && footprint[i].x == x)
        {
            if(footprint[i].y < min_pt.y)
                min_pt = footprint[i];
            else if(footprint[i].y > max_pt.y)
                max_pt = footprint[i];
            ++i;
        }

        //loop though cells in the column
        for(unsigned int y = min_pt.y; y < max_pt.y; ++y)
        {
            pt.x = x;
            pt.y = y;
            footprint.push_back(pt);
        }
    }
}

} //namespace golfcar_purepursuit



PLUGINLIB_DECLARE_CLASS(golfcar_purepursuit, PurePursuitBase, golfcar_purepursuit::PurePursuitBase, nav_core::BaseLocalPlanner)
