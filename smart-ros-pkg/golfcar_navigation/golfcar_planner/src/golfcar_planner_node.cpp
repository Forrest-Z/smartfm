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

        int num_frames_no_plan;
        list<double*> stateList;
        unsigned int RRT_MAX_ITER;
        planner_t rrts;

        int map_skip, map_count;
        bool planner_in_progress;
        bool first_frame;
        system_t system;
        ros::NodeHandle nh;

        ros::Subscriber map_sub;
        ros::Publisher traj_pub;
        ros::Publisher obs_pub;
        ros::Publisher tree_pub;
        ros::Publisher vertex_pub;

        ros::Timer planner_timer;

        system_t create_system();
        void on_map(const local_map::local_map_msg::ConstPtr & local_map);
        void get_plan();
        int get_traj(planner_t *rrts);
        int publish_tree(planner_t *rrts);
        void publish_grid();

        int project_goal(float &xc, float &yc, float &xs, float &ys, float goalx, float goaly);
};

Planner_node::Planner_node()
{

    map_count = 0;
    map_skip = 1;
    planner_in_progress = false;
    num_frames_no_plan = 0;
    first_frame = 1;
    RRT_MAX_ITER = 1000;

    // init periodic planner
    //planner_timer = nh.createTimer(ros::Duration(5.0), &Planner_node::get_plan, this);

    // subscribe to points
    map_sub = nh.subscribe<local_map::local_map_msg>("localCells", 5, &Planner_node::on_map, this);
    traj_pub = nh.advertise<nav_msgs::Path>("pnc_trajectory", 5);
    obs_pub = nh.advertise<sensor_msgs::PointCloud>("obs_map", 5);
    tree_pub = nh.advertise<sensor_msgs::PointCloud>("rrt_tree", 5);
    vertex_pub = nh.advertise<sensor_msgs::PointCloud>("rrt_vertex", 5);
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


        // dummy sys init for getRoot to work
        system.regionOperating.center[0] = system.origin.x;
        system.regionOperating.center[1] = system.origin.y;
        system.regionOperating.center[2] = system.origin.z;
        system.regionOperating.size[0] = system.map_width;
        system.regionOperating.size[1] = system.map_height;
        system.regionOperating.size[2] = 2.0 * M_PI;

        system.regionGoal.center[0] = 100;
        system.regionGoal.center[1] = 0;
        system.regionGoal.center[2] = 0;
        system.regionGoal.size[0] = 5;
        system.regionGoal.size[1] = 5;
        system.regionGoal.size[2] = 0.3 * M_PI;
        
        rrts.setSystem( system );

        // Set planner parameters
        rrts.setGamma (2.0);
        rrts.setGoalSampleFrequency (0.1);
        
        // init root
        vertex_t &root = rrts.getRootVertex();  
        state_t &rootState = root.getState();
        rootState[0] = system.origin.x;
        rootState[1] = system.origin.y;
        rootState[2] = system.origin.z;

        // Initialize the planner
        rrts.initialize ();
        
        cout<<"got first frame" << endl;
    }

    if(map_count % map_skip == 0)
    {
        map_count = 0;
        
        //cout<<"l xsize: "<< local_map->xsize<<" l ysize:"<< local_map->ysize <<endl;
        //cout<<"xsize: "<< system.xsize<<" ysize:"<<system.ysize<<endl;
        //cout<<"verify: "<< local_map->vals.size() <<" "<< system.xsize*system.ysize << endl;
        for(int i=0; i< system.xsize; i++)
        {
            for(int j=0; j< system.ysize; j++)
            {
                float car_width = 0.1, car_length = 0.1;

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
        
        get_plan();
    
        publish_grid();
    }
}

inline float dist(float x1, float y1, float x2, float y2)
{
    return sqrt( (x1-x2)*(x1-x2) + (y1-y2)*(y1-y2) );
}

int Planner_node::project_goal(float &xc, float &yc, float &xs, float &ys, float goalx, float goaly)
{
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
        ys = 2.0;
    }
    else
    {
        ys = fabs(ymin - ymin2);
        xs = 2.0;
    }
    //cout<<"goals: "<< xc <<" "<<yc<<" "<< xs<<" " << ys << endl;
    
    return 0;
}

void Planner_node::get_plan()
{
    // prune the obstructed part of the tree
    rrts.checkTree();
    //cout<<"tree_num_vert: "<< rrts.numVertices<<endl;
    
    //cout<<"sys origin: "<< system.origin.x<<" "<< system.origin.y<<endl;
    system.regionOperating.center[0] = system.origin.x;
    system.regionOperating.center[1] = system.origin.y;
    system.regionOperating.center[2] = system.origin.z;
    system.regionOperating.size[0] = system.map_width;
    system.regionOperating.size[1] = system.map_height;
    system.regionOperating.size[2] = 2.0 * M_PI;

    // project goal

    float xc, yc, xs, ys;
    project_goal(xc, yc, xs, ys, 1000, 1000);

    system.regionGoal.center[0] = xc;
    system.regionGoal.center[1] = yc;
    system.regionGoal.center[2] = atan2(1000 - system.origin.y, 1000 - system.origin.x);
    system.regionGoal.size[0] = xs;
    system.regionGoal.size[1] = ys;
    system.regionGoal.size[2] = 0.3 * M_PI;
    //cout<<"goal dir: "<< system.regionGoal.center[2] << endl;

    for(unsigned int i=0; i< RRT_MAX_ITER; i++)
    {
        rrts.iteration();
    }
    
    rrts.updateReachability();
    
    //cout<<"cost: "<< rrts.getBestVertexCost() << " result: " << get_traj(&rrts) << endl;
    //rrts.switchRoot(1.0);
    publish_tree(&rrts);
}

int Planner_node::get_traj(planner_t *prrts)
{
    vertex_t& vertexBest = prrts->getBestVertex ();

    if (&vertexBest == NULL)
    {
        cout<<"new vertex not found"<<endl;
        return 1;
    }
    else
    {
        stateList.clear();
        prrts->getBestTrajectory (stateList);

        nav_msgs::Path traj_msg;
        traj_msg.header.stamp = ros::Time::now();
        traj_msg.header.frame_id = "odom";

        int stateIndex = 0;
        for (list<double*>::iterator iter = stateList.begin(); iter != stateList.end(); iter++) 
        {
            double* stateRef = *iter;
            geometry_msgs::PoseStamped p;
            p.header.stamp = ros::Time::now();
            p.header.frame_id = "odom";
            p.pose.position.x = stateRef[0];
            p.pose.position.y = stateRef[1];
            p.pose.position.z = 0;
            traj_msg.poses.push_back(p);

            delete [] stateRef;
            stateIndex++;
        }

        traj_pub.publish(traj_msg);

    }
    return 0;
}

int Planner_node::publish_tree(planner_t *prrts)
{
    int num_nodes = 0;
    
    num_nodes = prrts->numVertices;

    sensor_msgs::PointCloud pc;
    pc.header.stamp = ros::Time::now();
    pc.header.frame_id = "odom";

    sensor_msgs::PointCloud pc1;
    pc1.header.stamp = ros::Time::now();
    pc1.header.frame_id = "odom";
    
    if (num_nodes > 0) 
    {    
        for (list<vertex_t*>::iterator iter = prrts->listVertices.begin(); iter != prrts->listVertices.end(); iter++) 
        {
            vertex_t &vertexCurr = **iter;
            state_t &stateCurr = vertexCurr.getState ();
            
            geometry_msgs::Point32 p;
            p.x = stateCurr[0];
            p.y = stateCurr[1];
            p.z = 0.0;
            pc.points.push_back(p);
            pc1.points.push_back(p);
            
            /*
            vertex_t& vertexParent = vertexCurr.getParent();
            if (&vertexParent != NULL) 
            {
                state_t& stateParent = vertexParent.getState();
                list<double*> trajectory;
                if (system.getTrajectory (stateParent, stateCurr, trajectory) == 0) {
                    cout << "ERROR: Trajectory can not be regenerated" << endl;
                    return 0;
                }
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
            */
        }
    }

    /*
    if (tree->num_nodes > 1) 
    {
        tree->num_edges = tree->num_nodes - 1;
        tree->edges = (int32_t **) malloc (tree->num_edges * sizeof(int32_t *));
        tree->traj_edges = (lcmtypes_opttree_traj_t *) malloc (tree->num_edges * sizeof (lcmtypes_opttree_tree_t));

        for (int i = 0; i < tree->num_edges; i++) {
            tree->traj_edges[i].num_states = 0;
            tree->traj_edges[i].states = NULL;
        }

        int edgeIndex = 0;
        for (list<vertex_t*>::iterator iter = planner.listVertices.begin(); iter != planner.listVertices.end(); iter++) {

            vertex_t &vertexCurr = **iter;

            if (&(vertexCurr.getParent()) == NULL) 
                continue;

            int parentIndex = 0;
            bool parentFound = false;
            for (list<vertex_t*>::iterator iterParent = planner.listVertices.begin(); 
                    iterParent != planner.listVertices.end(); iterParent++) {

                vertex_t *vertexParentCurr = *iterParent;

                if ( &(vertexCurr.getParent())  == vertexParentCurr) {

                    parentFound = true; 
                    break;
                }
                parentIndex++;
            }

            if (parentFound == false) {
                cout << "ERROR: No parent found" << endl; 
            }
            tree->edges[edgeIndex] = (int32_t *) malloc (2 * sizeof(int32_t));
            tree->edges[edgeIndex][0] = edgeIndex;
            tree->edges[edgeIndex][1] = parentIndex;

            edgeIndex++;
        }

    }
    else {
        tree->num_edges = 0;
        tree->edges = NULL;
    }
    */
    
    //tree_pub.publish(pc);
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
