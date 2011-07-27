#include <iostream>
#include <ctime>

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <laser_geometry/laser_geometry.h>

// For transform support
#include <tf/tf.h>
#include "tf/transform_broadcaster.h"
#include "tf/transform_listener.h"
#include "tf/message_filter.h"

#include <geometry_msgs/Point32.h>
#include <sensor_msgs/PointCloud.h>
#include <message_filters/subscriber.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PointStamped.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/GridCells.h>

#include <local_map/local_map_msg.h>

#include "dubins_car.hpp"
#include "rrts.hpp"

using namespace std;

typedef DubinsCar::StateType state_t;
typedef DubinsCar::TrajectoryType trajectory_t;
typedef DubinsCar::SystemType system_t;

typedef RRTstar::Vertex <DubinsCar> vertex_t;
typedef RRTstar::Planner <DubinsCar> planner_t; 

class Planner_node
{
    public:
        Planner_node();
        ~Planner_node(){};

    private:
        
        // planner
        system_t system;
        planner_t rrts;
        unsigned int RRT_MAX_ITER;
        float prev_best_cost, curr_best_cost;
        int already_committed;
        bool planner_in_progress;
        bool received_goal;
        bool first_frame;
        float dist_betw_odom;
        int map_skip, map_count;
        geometry_msgs::Point32 curr_goal;

        list<double*> toPublishTraj;
        list<float> toPublishControl;
        
        ros::Time last_good_path_time;
        system_t create_system();
        void change_sampling_region();
        void emergency_replan();
        int project_goal(float &xc, float &yc, float &xs, float &ys, float goalx, float goaly);
        bool root_in_goal();
        bool isFarFromTraj();

        // ros
        ros::NodeHandle nh;
        ros::Subscriber goal_sub;
        ros::Subscriber odom_sub;
        ros::Subscriber map_sub;

        ros::Publisher traj_pub;
        ros::Publisher trajview_pub;
        ros::Publisher obs_pub;
        ros::Publisher tree_pub;
        ros::Publisher vertex_pub;
        ros::Timer planner_timer;
        ros::Timer tree_pub_timer;

        geometry_msgs::Pose odom_prev, odom_now;
        void on_goal(const geometry_msgs::PointStamped& point);
        void on_odom(const nav_msgs::Odometry::ConstPtr & msg);
        void on_map(const local_map::local_map_msg::ConstPtr & local_map);
        void on_planner_timer(const ros::TimerEvent &e);
        void on_tree_pub_timer(const ros::TimerEvent &e);
        void get_plan();

        int publish_traj();
        int publish_tree();
        void publish_grid();
};

Planner_node::Planner_node()
{
    received_goal = 0;
    prev_best_cost = 1e20, curr_best_cost = 1e10;
    already_committed = 0;
    dist_betw_odom = 0;
    map_count = 0;
    map_skip = 1;
    planner_in_progress = false;
    first_frame = 1;
    RRT_MAX_ITER = 500;
    
    curr_goal.x = 0;
    curr_goal.y = 0;
    curr_goal.z = 0;

    // init periodic planner
    planner_timer = nh.createTimer(ros::Duration(0.5), &Planner_node::on_planner_timer, this);
    tree_pub_timer = nh.createTimer(ros::Duration(2.0), &Planner_node::on_tree_pub_timer, this);

    // subscribe to points
    odom_sub = nh.subscribe<nav_msgs::Odometry>("odom", 5, &Planner_node::on_odom, this);
    map_sub = nh.subscribe<local_map::local_map_msg>("localCells", 5, &Planner_node::on_map, this);
    goal_sub = nh.subscribe("pnc_waypoint", 5, &Planner_node::on_goal, this);

    traj_pub = nh.advertise<nav_msgs::Path>("pnc_trajectory", 5);
    trajview_pub = nh.advertise<nav_msgs::Path>("pncview_trajectory", 5);
    obs_pub = nh.advertise<sensor_msgs::PointCloud>("obs_map", 5);
    tree_pub = nh.advertise<sensor_msgs::PointCloud>("rrt_tree", 5);
    vertex_pub = nh.advertise<sensor_msgs::PointCloud>("rrt_vertex", 5);
}

inline float dist(float x1, float y1, float x2, float y2)
{
    return sqrt( (x1-x2)*(x1-x2) + (y1-y2)*(y1-y2) );
}

void Planner_node::on_goal(const geometry_msgs::PointStamped &point)
{
    curr_goal.x = point.point.x;
    curr_goal.y = point.point.y;
    curr_goal.z = point.point.z;
    ROS_INFO("got goal: %f %f %f", curr_goal.x, curr_goal.y, curr_goal.z);
    
    if(first_frame == 0)
    {
        change_sampling_region();

        /*
        // set root to current position
        vertex_t& rootVertex =  rrts.getRootVertex();
        state_t &stateRoot = rootVertex.getState();
        stateRoot[0] = odom_now.position.x;
        stateRoot[1] = odom_now.position.y;
        stateRoot[2] = odom_now.position.z;

        // 2. reinit planner
        rrts.initialize();
        prev_best_cost = 1e10;
        curr_best_cost = 1e20;
        cout<<"initialized new tree"<<endl;
        */

        rrts.checkTree();
        rrts.updateReachability();

        planner_in_progress = true;
        last_good_path_time = ros::Time::now();
    }
}

void Planner_node::on_odom(const nav_msgs::Odometry::ConstPtr & msg)
{
    odom_now.position.x = msg->pose.pose.position.x;
    odom_now.position.y = msg->pose.pose.position.y;
    odom_now.position.z = tf::getYaw(msg->pose.pose.orientation);

    float x1 = odom_now.position.x;
    float y1 = odom_now.position.y;
    
    if(planner_in_progress)
    {
        vertex_t& rootVertex =  rrts.getRootVertex();
        state_t &stateRoot = rootVertex.getState();
        float x2 = stateRoot[0];
        float y2 = stateRoot[1];
        
        if( (sqrt((x1-x2)*(x1-x2) + (y1-y2)*(y1-y2)) < 2) )
        {
            already_committed = 0;
        }
    }
}

void Planner_node::publish_grid()
{
    sensor_msgs::PointCloud pc;
    pc.header.stamp = ros::Time::now();
    pc.header.frame_id = "odom";

    //cout<<"verify: "<< local_map->vals.size() <<" "<< local_map->xsize * local_map->ysize << endl;
    for(int i=0; i< system.xsize; i++)
    {
        for(int j=0; j< system.ysize; j++)
        {
            float tmp = (float)system.map_vals[j + i*system.xsize];
            if( tmp > 50.0)
            {
                //cout<<"1: "<<i<<" "<<j << endl;
                geometry_msgs::Point32 p;
                p.x = system.origin.x + (i - system.xorigin)*system.map_res;
                p.y = system.origin.y + (j - system.yorigin)*system.map_res;
                p.z = 0;

                pc.points.push_back(p);
            }
        }
    }
    obs_pub.publish(pc);
}

void Planner_node::on_map(const local_map::local_map_msg::ConstPtr & local_map)
{
    map_count++;

    if(first_frame)
    {
        first_frame = 0;
        system.map_res = local_map->res;
        system.xsize = local_map->xsize;
        system.ysize = local_map->ysize;
        system.xorigin = local_map->xorigin;
        system.yorigin = local_map->yorigin;
        system.map_width = local_map->width;
        system.map_height = local_map->height;

        system.origin.x = local_map->origin.x;
        system.origin.y = local_map->origin.y;
        system.origin.z = local_map->origin.z;       // this is yaw

        system.map_vals = new unsigned char [system.xsize * system.ysize];

        //change_sampling_region();        
        rrts.setSystem( system );

        // init root
        vertex_t &root = rrts.getRootVertex();  
        state_t &rootState = root.getState();
        rootState[0] = system.origin.x;
        rootState[1] = system.origin.y;
        rootState[2] = system.origin.z;

        // Set planner parameters
        rrts.setGamma (2.0);
        rrts.setGoalSampleFrequency (0.1);

        // Initialize the planner
        rrts.initialize ();

        ROS_INFO("got first frame");
    
        curr_goal.x = system.origin.x;
        curr_goal.y = system.origin.y;
        curr_goal.z = system.origin.z;
    }

    for(int i=0; i< system.xsize; i++)
    {
        for(int j=0; j< system.ysize; j++)
        {
            float car_width = 2.0, car_length = 2.0;

            int yleft = min(system.ysize, (int)(j + car_width/2/system.map_res));
            int yright = max(0, (int)(j - car_width/2/system.map_res));
            int xfront = min(system.xsize, (int)(i + car_length/2/system.map_res));
            int xback = max(0, (int)(i - car_length/2/system.map_res));

            system.map_vals[i + j*system.xsize] = (unsigned char)local_map->vals[i + j*system.xsize];
            if(system.map_vals[i + j*system.xsize] > 10)
            {
                for(int it= xback; it< xfront; it++)
                {
                    for(int jt = yright; jt < yleft; jt++)
                    {
                        system.map_vals[it + jt*system.xsize] = 250 - fabs(it-i) - fabs(jt-j);
                    }
                }
            }
        }
    }

    // write origin in odom frame
    system.origin.x = local_map->origin.x;
    system.origin.y = local_map->origin.y;
    system.origin.z = local_map->origin.z;       // this is yaw

    publish_grid();
}


int Planner_node::project_goal(float &xc, float &yc, float &xs, float &ys, float goalx, float goaly)
{
    int goalx_num = ((goalx - system.origin.x)/system.map_res + system.xorigin);
    int goaly_num = ((goaly - system.origin.y)/system.map_res + system.yorigin);
    if( (goalx_num >=0) && (goalx_num < system.xsize) && (goaly_num >=0) && (goaly_num < system.ysize) )
    {
        ROS_INFO("goal inside: keeping it same");
        xc = goalx; yc = goaly;
        xs = 2.0; yc = 2.0;
        return 1;
    }
    double min_corner_dist = DBL_MAX;
    int corner_num = 0;

    float xmin, ymin;
    vector<geometry_msgs::Point32> corners(4);
    corners[0].x = 0; corners[0].y = 0;
    corners[1].x = (float)system.xsize; corners[1].y = 0;
    corners[2].x = (float)system.xsize; corners[2].y = (float)system.ysize;
    corners[3].x = 0; corners[3].y = (float)system.ysize;

    for(int i=0; i<4; i++)
    {
        corners[i].x = system.origin.x + (corners[i].x - system.xorigin)*system.map_res;
        corners[i].y = system.origin.y + (corners[i].y - system.yorigin)*system.map_res;

        float tmp_dist = dist(corners[i].x, corners[i].y, goalx, goaly);
        if( tmp_dist < min_corner_dist)
        {
            xmin = corners[i].x; ymin = corners[i].y;
            min_corner_dist = tmp_dist;
            corner_num = i;
        }
    }
    //cout<<"min corner: "<< xmin <<" "<< ymin << " corner_num: "<< corner_num<<endl;

    float xmin2, ymin2;
    int corner_bef = ((corner_num - 1)%4 + 4)%4;
    int corner_next = (corner_num + 1)%4;
    
    if( dist( corners[corner_bef].x, corners[corner_bef].y, goalx, goaly) <= dist( corners[corner_next].x, corners[corner_next].y, goalx, goaly) )
    {
        xmin2 = corners[corner_bef].x;
        ymin2 = corners[corner_bef].y;
        //cout<<"min2: "<< xmin2<<" "<< ymin2<<" min2: "<< corner_bef<<endl;
    }
    else
    {
        xmin2 = corners[corner_next].x;
        ymin2 = corners[corner_next].y;
        //cout<<"min2: "<< xmin2<<" "<< ymin2<<" min2: "<< corner_next<<endl;
    }
    
    xc = (xmin + xmin2)/2;
    yc = (ymin + ymin2)/2;
    if( fabs(xmin - xmin2) > fabs(ymin - ymin2) )
    {
        xs = fabs(xmin - xmin2);
        ys = 5.0;
    }
    else
    {
        ys = fabs(ymin - ymin2);
        xs = 5.0;
    }
    ROS_INFO("projected goals: [%f, %f], size: [%f, %f]", xc, yc, xs, ys);

    return 0;
}

void Planner_node::change_sampling_region()
{
    //cout<<"sys origin: "<< system.origin.x<<" "<< system.origin.y<<endl;
    vertex_t &root = rrts.getRootVertex();  
    state_t &rootState = root.getState();
    system.regionOperating.center[0] = rootState[0];
    system.regionOperating.center[1] = rootState[1];
    system.regionOperating.center[2] = 0;
    system.regionOperating.size[0] = 100.0;
    system.regionOperating.size[1] = 100.0;
    system.regionOperating.size[2] = 2.0 * M_PI;
   
    while(curr_goal.z > M_PI)
        curr_goal.z -= 2*M_PI;
    while(curr_goal.z < -M_PI)
        curr_goal.z += 2*M_PI;
    
    system.regionGoal.center[0] = curr_goal.x;
    system.regionGoal.center[1] = curr_goal.y;
    system.regionGoal.center[2] = curr_goal.z;
    system.regionGoal.size[0] = 2;
    system.regionGoal.size[1] = 2;
    system.regionGoal.size[2] = 10/180*M_PI;
}

bool Planner_node::root_in_goal()
{
    vertex_t &rootVertex = rrts.getRootVertex();
    state_t &curr_state = rootVertex.getState();
    return system.isReachingTarget(curr_state);
}

void Planner_node::on_tree_pub_timer(const ros::TimerEvent &e)
{
    publish_tree();
}

void Planner_node::on_planner_timer(const ros::TimerEvent &e)
{
    if(planner_in_progress)
        get_plan();
    return;
}

void Planner_node::get_plan()
{

    // prune the obstructed part of the tree
    rrts.checkTree();
    rrts.updateReachability();
    ROS_INFO("rrt num_vert: %d", rrts.numVertices);
    change_sampling_region();

    if( ! root_in_goal() )
    {
        for(unsigned int i=0; i< RRT_MAX_ITER; i++)
        {
            rrts.iteration();
            if( rrts.numVertices > 3000)
                break;
        }
        rrts.updateReachability();
        curr_best_cost = rrts.getBestVertexCost();
        ROS_INFO("commit status: %d best_curr: %f", already_committed, curr_best_cost);

        // if found traj, copy it and keep publishing, switch root to some node ahead
        if( !already_committed )
        {
            ROS_INFO("prev_cost: %f curr_cost: %f", prev_best_cost, curr_best_cost);
            if( fabs(prev_best_cost - curr_best_cost) < 1.0)
            {
                vertex_t& vertexBest = rrts.getBestVertex ();
                if (&vertexBest == NULL)
                {
                    ROS_DEBUG("get_plan(): best not found");
                }
                else
                {
                    ROS_INFO("traj cost: %f", rrts.getBestVertexCost() );

                    // free memory for toPublishTraj
                    for (list<double*>::iterator iter = toPublishTraj.begin(); iter != toPublishTraj.end(); iter++) 
                    {
                        double* stateRef = *iter;
                        delete stateRef;
                    } 
                    toPublishTraj.clear();
                    toPublishControl.clear();

                    // do checktree here to avoid switchRoot going beyond an obstacle
                    rrts.checkTree();
                    rrts.updateReachability();
                    if(rrts.switchRoot(50, toPublishTraj, toPublishControl))
                    {
                        already_committed = 1;
                        ROS_INFO("switching root");
                    }
                    else
                        ROS_INFO("couldn't switch root");
                }
            }
        }

        prev_best_cost = curr_best_cost;
    }
    
    publish_traj();
}

void Planner_node::emergency_replan()
{
    //1. set root to current position
    vertex_t& rootVertex =  rrts.getRootVertex();
    state_t &stateRoot = rootVertex.getState();
    stateRoot[0] = odom_now.position.x;
    stateRoot[1] = odom_now.position.y;
    stateRoot[2] = odom_now.position.z;
    
    change_sampling_region();

    // 2. reinit planner
    rrts.initialize();
    prev_best_cost = 1e10;
    curr_best_cost = 1e20;
    ROS_INFO("initialized new tree");
    
    already_committed = 0;
    
    get_plan();
    ROS_INFO("finished get_plan emergency");
}

// return true normally, if it finds even one vertex within 1.5 m of car, return false
bool Planner_node::isFarFromTraj()
{
    float cx = odom_now.position.x;
    float cy = odom_now.position.y;
    
    if(toPublishTraj.size() == 0)
        return false;

    for (list<double*>::iterator iter = toPublishTraj.begin(); iter != toPublishTraj.end(); iter++) 
    {
        double* stateRef = *iter;
        
        if( dist(cx, cy, stateRef[0], stateRef[1]) < 3.0)
            return false;
    }
    return true;
}


int Planner_node::publish_traj()
{
    bool late_flag = false;
    if(planner_in_progress)
    {
        ros::Duration time_since_last_path = ros::Time::now() - last_good_path_time;
        //ROS_INFO("Duration: %f", time_since_last_path.toSec());
        if(time_since_last_path.toSec() > 5.0)
            late_flag = true;
        else
            late_flag = false;
    }
    
    bool safe_flag = rrts.isSafeTrajectory(toPublishTraj);
    bool far_flag = isFarFromTraj();

    if( (!safe_flag) || far_flag  || late_flag  )
    {
        ROS_INFO("clearing_trajectory- is_far: %d is_safe: %d is_late: %d", far_flag, safe_flag, late_flag);
        toPublishControl.clear();
    
        // free memory
        for (list<double*>::iterator iter = toPublishTraj.begin(); iter != toPublishTraj.end(); iter++) 
        {
            double* stateRef = *iter;
            delete stateRef;
        } 
        toPublishTraj.clear();
        
        // initiate replan
        emergency_replan();

        return 0;
    }
    else
    {
        if(toPublishTraj.size() > 0)
            last_good_path_time = ros::Time::now();

        nav_msgs::Path traj_msg;
        traj_msg.header.stamp = ros::Time::now();
        traj_msg.header.frame_id = "odom";

        for (list<double*>::iterator iter = toPublishTraj.begin(); iter != toPublishTraj.end(); iter++) 
        {
            double* stateRef = *iter;
            geometry_msgs::PoseStamped p;
            p.header.stamp = ros::Time::now();
            p.header.frame_id = "odom";
            
            ROS_DEBUG(" [%f, %f, %f]", stateRef[0], stateRef[1], stateRef[2]);
            p.pose.position.x = stateRef[0];
            p.pose.position.y = stateRef[1];
            p.pose.position.z = stateRef[2];
            p.pose.orientation.w = 1.0;
            traj_msg.poses.push_back(p);
        }

        int which_pose =0;
        for (list<float>::iterator iter = toPublishControl.begin(); iter != toPublishControl.end(); iter++) 
        {
            traj_msg.poses[which_pose].pose.position.z = *iter;
            //cout<< *iter<<" ";
            which_pose++;
        }
        traj_pub.publish(traj_msg);

        // publish to viewer
        traj_msg.poses.clear();
        for (list<double*>::iterator iter = toPublishTraj.begin(); iter != toPublishTraj.end(); iter++) 
        {
            double* stateRef = *iter;
            geometry_msgs::PoseStamped p;
            p.header.stamp = ros::Time::now();
            p.header.frame_id = "odom";

            p.pose.position.x = stateRef[0];
            p.pose.position.y = stateRef[1];
            p.pose.position.z = 0;
            p.pose.orientation.w = 1.0;
            traj_msg.poses.push_back(p);
        }
        trajview_pub.publish(traj_msg);
    }
    return 1;
}

int Planner_node::publish_tree()
{
    int num_nodes = 0;
    
    num_nodes = rrts.numVertices;

    sensor_msgs::PointCloud pc;
    pc.header.stamp = ros::Time::now();
    pc.header.frame_id = "odom";

    sensor_msgs::PointCloud pc1;
    pc1.header.stamp = ros::Time::now();
    pc1.header.frame_id = "odom";
   
    if (num_nodes > 0) 
    {    
        for (list<vertex_t*>::iterator iter = rrts.listVertices.begin(); iter != rrts.listVertices.end(); iter++) 
        {
            vertex_t &vertexCurr = **iter;
            state_t &stateCurr = vertexCurr.getState ();
            
            geometry_msgs::Point32 p;
            p.x = stateCurr[0];
            p.y = stateCurr[1];
            p.z = 0.0;
            pc.points.push_back(p);
            pc1.points.push_back(p);
            
            vertex_t& vertexParent = vertexCurr.getParent();
            if (&vertexParent != NULL) 
            {
                state_t& stateParent = vertexParent.getState();
                list<double*> trajectory;
                list<float> control;
                if (system.getTrajectory (stateParent, stateCurr, trajectory, control)) 
                {
                    int par_num_states = trajectory.size();
                    if (par_num_states) 
                    {
                        int stateIndex = 0;
                        for (list<double*>::iterator it_state = trajectory.begin(); it_state != trajectory.end(); it_state++) 
                        {
                            double *stateTraj = *it_state;

                            geometry_msgs::Point32 p2;
                            p2.x = stateTraj[0];
                            p2.y = stateTraj[1];
                            p2.z = 0.0;
                            pc.points.push_back(p2);
                            stateIndex++;
                            delete [] stateTraj;
                        }
                    }
                }
            }
        }
    }

    tree_pub.publish(pc);
    vertex_pub.publish(pc1);

    return 1;
}

int main(int argc, char **argv)
{

    ros::init(argc, argv, "golfcar_planner_node");
    ros::NodeHandle n;

    Planner_node my_pnode;

    ros::spin();
};
