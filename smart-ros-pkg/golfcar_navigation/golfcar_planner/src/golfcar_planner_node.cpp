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

#include <sensor_msgs/PointCloud.h>
#include <message_filters/subscriber.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>

#include <local_map/local_map_msg.h>
#include <golfcar_planner/pnc_trajectory.h>

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

        list<double*> stateList;
        unsigned int RRT_MAX_ITER;

        bool first_frame;
        system_t system;
        planner_t rrts;
        ros::NodeHandle nh;

        ros::Subscriber map_sub;
        ros::Publisher traj_pub;
        ros::Timer planner_timer;

        system_t create_system();
        void on_map(const local_map::local_map_msg::ConstPtr & local_map);
        void get_plan(const ros::TimerEvent &event);
        int get_traj();
        void get_tree();
};

Planner_node::Planner_node()
{
    first_frame = 1;
    RRT_MAX_ITER = 1000;

    // init periodic planner
    planner_timer = nh.createTimer(ros::Duration(0.1), &Planner_node::get_plan, this);

    // subscribe to points
    map_sub = nh.subscribe("localCells", 1, &Planner_node::on_map, this);
    traj_pub = nh.advertise<nav_msgs::Path>("pnc_trajectory", 5); 

    // one dummy set system

    system.regionOperating.center[0] = 0.0;
    system.regionOperating.center[1] = 0.0;
    system.regionOperating.center[2] = 0.0;
    system.regionOperating.size[0] = 100.0;
    system.regionOperating.size[1] = 20.0;
    system.regionOperating.size[2] = 2.0 * M_PI;

    // goal straight ahead
    system.regionGoal.center[0] = 50.0;
    system.regionGoal.center[1] = 10.0;
    system.regionGoal.center[2] = 0.0;
    system.regionGoal.size[0] = 5.0;
    system.regionGoal.size[1] = 5.0;
    system.regionGoal.size[2] = 0.1 * M_PI;

    rrts.setSystem( system );

    // init root
    vertex_t &root = rrts.getRootVertex();  
    state_t &rootState = root.getState();
    rootState[0] = 0.0;
    rootState[1] = 0.0;
    rootState[2] = 0.0;

    // Initialize the planner
    rrts.initialize ();

    // Set planner parameters
    rrts.setGamma (2.0);
    rrts.setGoalSampleFrequency (0.1);
}

void Planner_node::on_map(const local_map::local_map_msg::ConstPtr & local_map)
{
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

        system.map_vals = new uint8_t [system.xsize * system.ysize];
    }
    for(int i=0; i< system.xsize* system.ysize; i++)
    {
        system.map_vals[i] = local_map->vals[i];
    }
}

// break out of loop if cost does not decrease for > 500 iters
void Planner_node::get_plan(const ros::TimerEvent & event)
{
    double curr_cost = 1000, prev_cost = 1000;
    for(unsigned int i=0; i< RRT_MAX_ITER; i++)
    {
        rrts.iteration();

        curr_cost = rrts.getBestVertexCost();
        if(i % 500)
        {
            if( (fabs(curr_cost - prev_cost) < 1) && (curr_cost < 1000) )
                break;
            else
                prev_cost = curr_cost;
        }
    }
    get_traj();
}

int Planner_node::get_traj()
{
    vertex_t& vertexBest = rrts.getBestVertex ();

    if (&vertexBest == NULL)
        return 1;

    stateList.clear();
    rrts.getBestTrajectory (stateList);

    nav_msgs::Path traj_msg;
    traj_msg.header.stamp = ros::Time::now();
    traj_msg.header.frame_id = "base_link";

    int stateIndex = 0;
    for (list<double*>::iterator iter = stateList.begin(); iter != stateList.end(); iter++) 
    {
        double* stateRef = *iter;
        geometry_msgs::PoseStamped p;
        p.header.stamp = ros::Time::now();
        p.header.frame_id = "base_link";
        p.pose.position.x = stateRef[0];
        p.pose.position.y = stateRef[1];
        p.pose.position.z = 0;
        traj_msg.poses.push_back(p);
        
        delete [] stateRef;
        stateIndex++;
    }
   
    cout<<"published traj"<<endl;
    traj_pub.publish(traj_msg);
    return 0;
}

/*
   void Planner_node::get_tree()
   {
   tree->num_nodes = planner.numVertices; 


   if (tree->num_nodes > 0) {    


   tree->nodes = (lcmtypes_opttree_node_t *) malloc (tree->num_nodes * sizeof(lcmtypes_opttree_node_t));
   tree->traj_from_parent = (lcmtypes_opttree_traj_t *) malloc (tree->num_nodes * sizeof(lcmtypes_opttree_traj_t));

   int nodeIndex = 0;
   for (list<vertex_t*>::iterator iter = planner.listVertices.begin(); iter != planner.listVertices.end(); iter++) {


   vertex_t &vertexCurr = **iter;
   state_t &stateCurr = vertexCurr.getState ();

   tree->nodes[nodeIndex].nodeid = nodeIndex;
   tree->nodes[nodeIndex].dead_node = false;
   tree->nodes[nodeIndex].state.x = stateCurr[0];
   tree->nodes[nodeIndex].state.y = stateCurr[1];
   tree->nodes[nodeIndex].state.z = 0.0;
   tree->nodes[nodeIndex].distance_from_root = vertexCurr.getCost ();


   vertex_t& vertexParent = vertexCurr.getParent();
   if (&vertexParent != NULL) {
   state_t& stateParent = vertexParent.getState();
   list<double*> trajectory;
   if (system.getTrajectory (stateParent, stateCurr, trajectory) == 0) {
   cout << "ERROR: Trajectory can not be regenerated" << endl;
   return 0;
   }

   tree->traj_from_parent[nodeIndex].num_states = trajectory.size();
   if (tree->traj_from_parent[nodeIndex].num_states) {
   tree->traj_from_parent[nodeIndex].states = (lcmtypes_optsystem_state_t *) 
   malloc (tree->traj_from_parent[nodeIndex].num_states * sizeof (lcmtypes_optsystem_state_t));
   int stateIndex = 0;
   for (list<double*>::iterator it_state = trajectory.begin(); it_state != trajectory.end(); it_state++) {
   double *stateCurr = *it_state;
   tree->traj_from_parent[nodeIndex].states[stateIndex].x = stateCurr[0];
   tree->traj_from_parent[nodeIndex].states[stateIndex].y = stateCurr[1];
   tree->traj_from_parent[nodeIndex].states[stateIndex].z = 0.0;
   stateIndex++;
   delete [] stateCurr;
   }
   }
   else 
   tree->traj_from_parent[nodeIndex].states = NULL;
   }
   else {
   tree->traj_from_parent[nodeIndex].num_states = 0;
   tree->traj_from_parent[nodeIndex].states = NULL;
   }
   nodeIndex++;

   }

   }
   else {
   tree->nodes = NULL;
   }

   if (tree->num_nodes > 1) {
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

lcmtypes_opttree_tree_t_publish (lcm, "OPTTREE_TREE", tree);

lcmtypes_opttree_tree_t_destroy (tree);


return 1;

}
*/

int main(int argc, char **argv)
{

    ros::init(argc, argv, "golfcar_planner_node");
    ros::NodeHandle n;

    Planner_node my_pnode;

    ros::spin();
};
