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

        system_t create_system();
        void change_sampling_region();
        void emergency_replan();
        int project_goal(float &xc, float &yc, float &xs, float &ys, float goalx, float goaly);
        bool system_in_goal();
        
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

        geometry_msgs::Pose odom_prev, odom_now;
        void on_goal(const geometry_msgs::PointStamped& point);
        void on_odom(const nav_msgs::Odometry::ConstPtr & msg);
        void on_map(const local_map::local_map_msg::ConstPtr & local_map);
        void on_planner_timer(const ros::TimerEvent &e);
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
    planner_timer = nh.createTimer(ros::Duration(0.2), &Planner_node::on_planner_timer, this);

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

void Planner_node::on_goal(const geometry_msgs::PointStamped &point)
{
    curr_goal.x = point.point.x;
    curr_goal.y = point.point.y;
    curr_goal.z = point.point.z;
    cout<<"got goal: "<< curr_goal.x<<" "<< curr_goal.y<<" "<< curr_goal.z << endl;
    already_committed = 0;

    //1. set root to current position
    vertex_t& rootVertex =  rrts.getRootVertex();
    state_t &stateRoot = rootVertex.getState();
    stateRoot[0] = system.origin.x;
    stateRoot[1] = system.origin.y;
    stateRoot[2] = system.origin.z;
    
    change_sampling_region();

    // 2. reinit planner
    rrts.initialize();
    prev_best_cost = 1e10;
    curr_best_cost = 1e20;
    cout<<"initialized new tree"<<endl;

    planner_in_progress = true;
}

void Planner_node::on_odom(const nav_msgs::Odometry::ConstPtr & msg)
{
    odom_now.position.x = msg->pose.pose.position.x;
    odom_now.position.y = msg->pose.pose.position.y;

    float x1 = odom_now.position.x;
    float y1 = odom_now.position.y;
    
    if(planner_in_progress)
    {
        vertex_t& rootVertex =  rrts.getRootVertex();
        state_t &stateRoot = rootVertex.getState();
        float x2 = stateRoot[0];
        float y2 = stateRoot[1];
        
        if( (sqrt((x1-x2)*(x1-x2) + (y1-y2)*(y1-y2)) < 1) )
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

        cout<<"got first frame" << endl;
    
        curr_goal.x = system.origin.x;
        curr_goal.y = system.origin.y;
        curr_goal.z = system.origin.z;
    }

    //cout<<"l xsize: "<< local_map->xsize<<" l ysize:"<< local_map->ysize <<endl;
    //cout<<"xsize: "<< system.xsize<<" ysize:"<<system.ysize<<endl;
    //cout<<"verify: "<< local_map->vals.size() <<" "<< system.xsize*system.ysize << endl;
    for(int i=0; i< system.xsize; i++)
    {
        for(int j=0; j< system.ysize; j++)
        {
            float car_width = 1.0, car_length = 1.6;

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
                        system.map_vals[it + jt*system.xsize] = 250;
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


inline float dist(float x1, float y1, float x2, float y2)
{
    return sqrt( (x1-x2)*(x1-x2) + (y1-y2)*(y1-y2) );
}

int Planner_node::project_goal(float &xc, float &yc, float &xs, float &ys, float goalx, float goaly)
{
    int goalx_num = ((goalx - system.origin.x)/system.map_res + system.xorigin);
    int goaly_num = ((goaly - system.origin.y)/system.map_res + system.yorigin);
    if( (goalx_num >=0) && (goalx_num < system.xsize) && (goaly_num >=0) && (goaly_num < system.ysize) )
    {
        cout<<"goal inside: keeping it same" << endl;
        xc = goalx; yc = goaly;
        xs = 5.0; yc = 5.0;
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
    cout<<"goals: "<< xc <<" "<<yc<<" "<< xs<<" " << ys << endl;
    
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
    system.regionOperating.size[0] = system.map_width;
    system.regionOperating.size[1] = system.map_height;
    system.regionOperating.size[2] = 2.0 * M_PI;
  
     
    system.regionGoal.center[0] = curr_goal.x;
    system.regionGoal.center[1] = curr_goal.y;
    system.regionGoal.center[2] = curr_goal.z;
    system.regionGoal.size[0] = 5;
    system.regionGoal.size[1] = 5;
    system.regionGoal.size[2] = 0.5 * M_PI;
    
    /*
    system.regionGoal.center[0] = 100;
    system.regionGoal.center[1] = 0;
    system.regionGoal.center[2] = 0;
    system.regionGoal.size[0] = 5;
    system.regionGoal.size[1] = 5;
    system.regionGoal.size[2] = 0.5 * M_PI;
    */

    /*
    // project goal
    float goal_th = 0;
    float xc, yc, xs, ys;
    if(project_goal(xc, yc, xs, ys, curr_goal.x, curr_goal.y))
        goal_th = curr_goal.z;
    else
        goal_th = atan2(yc - system.origin.y, xc - system.origin.x);

    system.regionGoal.center[0] = xc;
    system.regionGoal.center[1] = yc;
    system.regionGoal.center[2] = goal_th;
    system.regionGoal.size[0] = xs;
    system.regionGoal.size[1] = ys;
    system.regionGoal.size[2] = 0.3 * M_PI;
    cout<<"goal dir: "<< system.regionGoal.center[2] << endl;   
    */
    
    /* 
    // project goal naive
    float length = dist(curr_goal.x, curr_goal.y, system.origin.x, system.origin.y);
    float xc = (curr_goal.x - system.origin.x)/length*10;
    float yc = (curr_goal.y - system.origin.y)/length*10;
    float goal_th = atan2(curr_goal.y - system.origin.y, curr_goal.x - system.origin.x);

    system.regionGoal.center[0] = xc;
    system.regionGoal.center[1] = yc;
    system.regionGoal.center[2] = goal_th;
    system.regionGoal.size[0] = 5.0;
    system.regionGoal.size[1] = 5.0;
    system.regionGoal.size[2] = 0.3 * M_PI;
    */
}

bool Planner_node::system_in_goal()
{
    state_t curr_state;
    curr_state[0] = system.origin.x;
    curr_state[1] = system.origin.y;
    curr_state[2] = curr_goal.z;

    return system.isReachingTarget(curr_state);
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
    cout<<"num_vert: "<< rrts.numVertices << endl;
    change_sampling_region();
    
    if(system_in_goal())
    {
        cout<<"reached goal: clear trajectories"<<endl;
        already_committed = 1;
        planner_in_progress = 0;
        for (list<double*>::iterator iter = toPublishTraj.begin(); iter != toPublishTraj.end(); iter++) 
        {
            double* stateRef = *iter;
            delete stateRef;
        } 
        toPublishControl.clear();
        toPublishTraj.clear();
        
    }
    
    for(unsigned int i=0; i< RRT_MAX_ITER; i++)
    {
        rrts.iteration();
    }
    curr_best_cost = rrts.getBestVertexCost();
    cout<<"commit status: "<< already_committed << " best_curr: "<< curr_best_cost << endl;

    // if found traj, copy it and keep publishing, switch root to some node ahead
    if(!already_committed)
    {
        if( fabs(prev_best_cost - curr_best_cost) < 10.0)
        {
            cout<<"prev: "<< prev_best_cost << " curr: " << curr_best_cost << endl;
            vertex_t& vertexBest = rrts.getBestVertex ();
            if (&vertexBest == NULL)
            {
                cout<<"new vertex not found"<<endl;
            }
            else
            {
                cout<<"traj cost: "<< rrts.getBestVertexCost() << endl;
                
                // free memory for toPublishTraj
                for (list<double*>::iterator iter = toPublishTraj.begin(); iter != toPublishTraj.end(); iter++) 
                {
                    double* stateRef = *iter;
                    delete stateRef;
                } 
                toPublishTraj.clear();
                toPublishControl.clear();
                
                //get best trajectory currently
                list<double*> bestTrajectory;
                list<float> bestControl;
                rrts.getBestTrajectory(bestTrajectory, bestControl);
                double length = rrts.getTrajectoryLength(bestTrajectory);
                cout<<"length best: "<<length<<endl;
                
                if(rrts.switchRoot(10, toPublishTraj, toPublishControl))
                {
                    already_committed = 1;
                    change_sampling_region();
                    cout<<"switching root"<<endl;
                }
                else
                    cout<<"couldn't switch root" << endl;
            }
        }
    }
    else
    {
        publish_traj();
    }
    
    prev_best_cost = curr_best_cost;

    publish_tree();
}

void Planner_node::emergency_replan()
{
    //1. set root to current position
    vertex_t& rootVertex =  rrts.getRootVertex();
    state_t &stateRoot = rootVertex.getState();
    stateRoot[0] = system.origin.x;
    stateRoot[1] = system.origin.y;
    stateRoot[2] = system.origin.z;
    
    change_sampling_region();

    // 2. reinit planner
    rrts.initialize();
    prev_best_cost = 1e10;
    curr_best_cost = 1e20;
    cout<<"initialized new tree"<<endl;
    
    publish_tree();
    already_committed = 0;
    
    get_plan();
    cout<<"finished get_plan emergency"<<endl;
}

int Planner_node::publish_traj()
{
    
    // 1. check if current trajectory is safe, else stop
    if(! rrts.isSafeTrajectory(toPublishTraj) )
    {
        cout<<"unsafe traj: clearing"<<endl;
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

    nav_msgs::Path traj_msg;
    traj_msg.header.stamp = ros::Time::now();
    traj_msg.header.frame_id = "odom";
    
    for (list<double*>::iterator iter = toPublishTraj.begin(); iter != toPublishTraj.end(); iter++) 
    {
        double* stateRef = *iter;
        geometry_msgs::PoseStamped p;
        p.header.stamp = ros::Time::now();
        p.header.frame_id = "odom";
        //if( (stateRef[0] > 0.001) && (stateRef[0] < 1e100) && (stateRef[1] > 0.001) && (stateRef[1] < 1e100))
        //{
            //cout<<"here"<< endl;
            ROS_DEBUG(" [%f, %f, %f]", stateRef[0], stateRef[1], stateRef[2]);
            p.pose.position.x = stateRef[0];
            p.pose.position.y = stateRef[1];
            p.pose.position.z = stateRef[2];
            p.pose.orientation.w = 1.0;
            traj_msg.poses.push_back(p);
        //}
        //delete [] stateRef;
    }
    
    int which_pose =0;
    for (list<float>::iterator iter = toPublishControl.begin(); iter != toPublishControl.end(); iter++) 
    {
        traj_msg.poses[which_pose].pose.position.z = *iter;
        //cout<< *iter<<" ";
        which_pose++;
    }
    cout<<endl;
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

    cout<<"finished traj pub"<<endl;
    return 1;
}

int Planner_node::publish_tree()
{
    cout<<"start tree pub"<<endl;
    int num_nodes = 0;
    
    num_nodes = rrts.numVertices;

    sensor_msgs::PointCloud pc;
    pc.header.stamp = ros::Time::now();
    pc.header.frame_id = "odom";

    sensor_msgs::PointCloud pc1;
    pc1.header.stamp = ros::Time::now();
    pc1.header.frame_id = "odom";
   
    int count = 0;
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

    cout<<"finished tree pub"<<endl;
    return 1;

}

int main(int argc, char **argv)
{

    ros::init(argc, argv, "golfcar_planner_node");
    ros::NodeHandle n;

    Planner_node my_pnode;

    ros::spin();
};
