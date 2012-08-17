#include <iostream>
#include <ctime>

#include <ros/ros.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/Pose.h>
#include <sensor_msgs/PointCloud.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>

#include "dubins_car.hpp"
#include "rrts.hpp"

using namespace std;

typedef DubinsCar::StateType state_t;
typedef DubinsCar::TrajectoryType trajectory_t;
typedef DubinsCar::SystemType system_t;

typedef RRTstar::Vertex <DubinsCar> vertex_t;
typedef RRTstar::Planner <DubinsCar> planner_t; 


class Planner
{
    public:
        Planner();
        ~Planner();

        system_t system;
        planner_t rrts;
        geometry_msgs::Point32 goal;
        bool is_updating_committed_trajectory;
        list<double*> committed_trajectory;
        list<float> committed_control;
        int clear_committed_trajectory();

        // ros
        ros::NodeHandle nh;
        ros::Subscriber goal_sub;
        ros::Subscriber map_sub;
        
        ros::Publisher traj_pub;
        ros::Publisher trajview_pub;
        ros::Publisher tree_pub;
        ros::Publisher vertex_pub;
        //ros::Timer planner_timer;
        ros::Timer tree_pub_timer;

        // functions
        void on_goal(const geometry_msgs::PointStamped &p);
        void on_map(const nav_msgs::OccupancyGrid &map);
        bool root_in_goal();
        void iterate();
        float dist(float x1, float y1, float x2, float y2)
        {
            return sqrt( (x1-x2)*(x1-x2) + (y1-y2)*(y1-y2) );
        }

        void publish_tree();
        void on_tree_pub_timer(const ros::TimerEvent &e)
};

Planner::Planner()
{
    clear_committed_trajectory();
    is_updating_committed_trajectory = false;

    tree_pub_timer = nh.createTimer(ros::Duration(1.0), &Planner::on_tree_pub_timer, this);
    committed_trajectory_pub_timer = nh.createTimer(ros::Duration(0.5), &Planner::on_committed_trajectory_pub_timer, this);
    
    committed_trajectory_pub = nh.advertise<nav_msgs::Path>("pnc_trajectory", 5);
    committed_trajectory_view_pub = nh.advertise<nav_msgs::Path>("pncview_trajectory", 5);
    tree_pub = nh.advertise<sensor_msgs::PointCloud>("rrts_tree", 2);
    vertex_pub = nh.advertise<sensor_msgs::PointCloud>("rrts_vertex", 2);
}

int Planner::clear_committed_trajectory()
{
    for(list<double*>::iterator i=committed_trajectory.begin(); i!=committed_trajectory.end(); i++)
    {
        double* stateRef = *i;
        delete[] stateRef;
    }
    committed_trajectory.clear();

    committed_control.clear(); 
}

// p is (x,y,yaw) in map coords
void Planner::on_goal(const geometry_msgs::PointStamped &p)
{
    goal.x = p.point.x;
    goal.y = p.point.y;
    goal.z = p.point.z;
    while(goal.z > M_PI)
        goal.z = goal.z - 2*M_PI;
    while(goal.z < -M_PI)
        goal.z = goal.z + 2*M_PI;

    ROS_INFO("got goal: %f %f %f", goal.x, goal.y, goal.z);

}

bool Planner::root_in_goal()
{
    vertex_t &rootVertex = rrts.getRootVertex();
    state_t &curr_state = rootVertex.getState();
    return system.isReachingTarget(curr_state);
}

void Planner::iterate()
{

    // 1. if at the end of committed trajectory then clear trajectory and return

    // 2.  
}

void Planner::on_committed_trajectory_pub_timer(const ros::TimerEvent &e)
{
    // this flag is set by iterate() if it is going to change the committed_trajectory
    // hacked semaphore
    if(is_updating_committed_trajectory)
        return;

    nav_msgs::Path traj_msg;
    traj_msg.header.stamp = ros::Time::now();
    traj_msg.header.frame_id = "map";
    
    list<float>::iterator committed_control_iter = committed_control.begin();
    for (list<double*>::iterator iter = committed_trajectory.begin(); iter != committed_trajectory.end(); iter++) 
    {
        double* stateRef = *iter;
        geometry_msgs::PoseStamped p;
        p.header.stamp = ros::Time::now();
        p.header.frame_id = "map";

        p.pose.position.x = stateRef[0];
        p.pose.position.y = stateRef[1];
        p.pose.position.z = *committed_control_iter;        // send control as the third state
        p.pose.orientation.w = 1.0;
        traj_msg.poses.push_back(p);
        
        ROS_DEBUG(" [%f, %f, %f]", p.pose.position.x, p.pose.position.y, p.pose.position.z);

        committed_control_iter++;
    }
    
    committed_trajectory_pub.publish(traj_msg);
    
    // publish to viewer
    traj_msg.poses.clear();
    for (list<double*>::iterator iter = committed_trajectory.begin(); iter != committed_trajectory.end(); iter++) 
    {
        double* stateRef = *iter;
        geometry_msgs::PoseStamped p;
        p.header.stamp = ros::Time::now();
        p.header.frame_id = "map";

        p.pose.position.x = stateRef[0];
        p.pose.position.y = stateRef[1];
        p.pose.position.z = 0;
        p.pose.orientation.w = 1.0;
        traj_msg.poses.push_back(p);
    }
    committed_trajectory_view_pub.publish(traj_msg);
}


void Planner::on_tree_pub_timer(const ros::TimerEvent &e)
{
    publish_tree();
}
int Planner::publish_tree()
{
    int num_nodes = 0;

    num_nodes = rrts.numVertices;

    sensor_msgs::PointCloud pc;
    pc.header.stamp = ros::Time::now();
    pc.header.frame_id = "map";

    sensor_msgs::PointCloud pc1;
    pc1.header.stamp = ros::Time::now();
    pc1.header.frame_id = "map";

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

    return 0;
}


int main(int argc, char **argv)
{

    ros::init(argc, argv, "rrts_node");
    //ros::NodeHandle n;

    Planner my_planner;

    ros::spin();
};
