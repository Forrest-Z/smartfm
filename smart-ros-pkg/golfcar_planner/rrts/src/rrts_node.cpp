#include <iostream>
#include <ctime>

#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Pose.h>
#include <sensor_msgs/PointCloud.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>

#include <tf/tf.h>

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
        int rrts_max_iter;
        
        geometry_msgs::Point32 goal;
        nav_msgs::OccupancyGrid map;
        bool is_first_goal;

        geometry_msgs::Pose car_position;
        bool is_updating_committed_trajectory;
        list<double*> committed_trajectory;
        list<float> committed_control;
        int clear_committed_trajectory();

        // ros
        ros::NodeHandle nh;
        ros::Subscriber goal_sub;
        ros::Subscriber map_sub;
        
        ros::Publisher tree_pub;
        ros::Publisher vertex_pub;
        ros::Publisher committed_trajectory_pub;
        ros::Publisher committed_trajectory_view_pub;
        ros::Timer planner_timer;
        ros::Timer tree_pub_timer;
        ros::Timer committed_trajectory_pub_timer;

        // functions
        void on_goal(const geometry_msgs::Pose::ConstPtr p);
        void on_map(const nav_msgs::OccupancyGrid::ConstPtr og);
        void on_committed_trajectory_pub_timer(const ros::TimerEvent &e);
        
        bool root_in_goal();
        void change_sampling_region();
        void setup_rrts();
        void on_planner_timer(const ros::TimerEvent &e);
        void get_plan();
        float dist(float x1, float y1, float x2, float y2)
        {
            return sqrt( (x1-x2)*(x1-x2) + (y1-y2)*(y1-y2) );
        }

        void publish_tree();
        void on_tree_pub_timer(const ros::TimerEvent &e);
};

Planner::Planner()
{
    clear_committed_trajectory();
    is_updating_committed_trajectory = false;

    planner_timer = nh.createTimer(ros::Duration(0.5), &Planner::on_planner_timer, this);

    tree_pub_timer = nh.createTimer(ros::Duration(1.0), &Planner::on_tree_pub_timer, this);
    committed_trajectory_pub_timer = nh.createTimer(ros::Duration(0.5), &Planner::on_committed_trajectory_pub_timer, this);
     
    committed_trajectory_pub = nh.advertise<nav_msgs::Path>("pnc_trajectory", 2);
    committed_trajectory_view_pub = nh.advertise<nav_msgs::Path>("pncview_trajectory", 2);
    tree_pub = nh.advertise<sensor_msgs::PointCloud>("rrts_tree", 2);
    vertex_pub = nh.advertise<sensor_msgs::PointCloud>("rrts_vertex", 2);

    map_sub = nh.subscribe("local_map", 2, &Planner::on_map, this);
    goal_sub = nh.subscribe("goal", 2, &Planner::on_goal, this);

    rrts_max_iter = 200;
    is_first_goal = true;
}

Planner::~Planner()
{
    clear_committed_trajectory();
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
    
    return 0;
}

// p is (x,y,yaw) in map coords
// assumption: goal is coming less frequently than map
void Planner::on_goal(const geometry_msgs::Pose::ConstPtr p)
{
    double roll=0, pitch=0, yaw=0;
    tf::Quaternion q;
    tf::quaternionMsgToTF(p->orientation, q);
    tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
    
    goal.x = p->position.x;
    goal.y = p->position.y;
    goal.z = yaw;

    ROS_INFO("got goal: %f %f %f", goal.x, goal.y, goal.z);
    
    if(is_first_goal)
    {
        setup_rrts();
        is_first_goal = false;
    }

}

void Planner::on_map(const nav_msgs::OccupancyGrid::ConstPtr og)
{
    // 1. copy the incoming grid into map
    system.map = *og;

    // 2. get pose of car in map frame
    car_position = og->info.origin;
    ROS_INFO("got map");
    //cout<<car_position<<endl;
    
}

bool Planner::root_in_goal()
{
    vertex_t &rootVertex = rrts.getRootVertex();
    state_t &curr_state = rootVertex.getState();
    return system.isReachingTarget(curr_state);
}


void Planner::change_sampling_region()
{
    vertex_t &root = rrts.getRootVertex();  
    state_t &rootState = root.getState();
    
    double cyaw = cos(rootState[2]);
    double syaw = sin(rootState[2]);
    
    // center of the map is the center of the local_map but in /map frame
    // yaw is 0
    system.regionOperating.center[0] = rootState[0] + cyaw*system.map.info.height/4.0*system.map.info.resolution;
    system.regionOperating.center[1] = rootState[1] + syaw*system.map.info.height/4.0*system.map.info.resolution;
    system.regionOperating.center[2] = 0;
    cout<<"rootState: "<< rootState[0]<<" "<<rootState[1]<<" "<<0<<endl;
    
    // just create a large operating region around the car irrespective of the orientation in /map frame
    // yaw is 2*M_PI
    double size = sqrt(pow(system.map.info.height,2), pow(system.map.info.width,2));
    system.regionOperating.size[0] = size;
    system.regionOperating.size[1] = size;
    system.regionOperating.size[2] = 2.0 * M_PI;
    cout<<"regionOperating: "<< system.regionOperating.size[0]<<" "<<system.regionOperating.size[1]<<" "<<system.regionOperating.size[2]<<endl;

    system.regionGoal.center[0] = goal.x;
    system.regionGoal.center[1] = goal.y;
    system.regionGoal.center[2] = goal.z;
    system.regionGoal.size[0] = 1;
    system.regionGoal.size[1] = 1;
    system.regionGoal.size[2] = 10/180*M_PI;
}

void Planner::setup_rrts()
{ 
    rrts.setSystem(system);
    vertex_t &root = rrts.getRootVertex();  
    state_t &rootState = root.getState();
    rootState[0] = car_position.position.x;
    rootState[1] = car_position.position.y;
    double roll=0, pitch=0, yaw=0;
    tf::Quaternion q;
    tf::quaternionMsgToTF(car_position.orientation, q);
    tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
    rootState[2] = yaw;

    change_sampling_region();

    // Set planner parameters
    rrts.setGamma (2.0);
    rrts.setGoalSampleFrequency (0.4);

    // Initialize the planner
    rrts.initialize ();
}

void Planner::get_plan()
{
    rrts.checkTree();
    rrts.updateReachability();
    ROS_INFO("rrt num_vert: %d", rrts.numVertices);
    while(rrts.numVertices < rrts_max_iter)
    {
        rrts.iteration();
        cout<<"rrts.numVertices: "<<rrts.numVertices<<endl;
        getchar();
    }
    double best_cost = rrts.getBestVertexCost();
    if(best_cost < 100)
    {
        is_updating_committed_trajectory = true;
        rrts.switchRoot(10, committed_trajectory, committed_control);
        is_updating_committed_trajectory = false;
    }
    else
    {
        ROS_INFO("223: did not find good path");
    }

}

void Planner::on_planner_timer(const ros::TimerEvent &e)
{
    // 1. if at the end of committed trajectory then clear trajectory and return
    if(!committed_trajectory.empty())
    {
        list<double*>::reverse_iterator riter = committed_trajectory.rbegin();
        double* last_committed_state = *riter;
        if( dist(car_position.position.x, car_position.position.y, last_committed_state[0], last_committed_state[1]) < 0.25)
        {
            clear_committed_trajectory();
            get_plan();
            return;
        }

        // 2. if far from committed trajectory, clear everything and go to 3
        bool is_far_away = true;
        for(list<double*>::iterator i=committed_trajectory.begin(); i!=committed_trajectory.end(); i++)
        {
            double* curr_state = *i;
            if(dist(car_position.position.x, car_position.position.y, curr_state[0], curr_state[1]) < 0.25)
                is_far_away = false;
        }
        if(is_far_away == true)
        {
            setup_rrts();
        }
    }
    else
    {
        // 3. else add more vertices / until you get a good trajectory, copy it to committed trajectory, return
        if(is_first_goal == false)
        {
            /*
            vertex_t &root = rrts.getRootVertex();  
            state_t &rootState = root.getState();
            double t[3] = {rootState[0], rootState[1], rootState[2]};
            cout<<"t: "<< t[0]<<" "<<t[1]<<" "<<t[2]<<endl;
            cout<<system.IsInCollision (t)<<endl;
            */
            get_plan();
        }
    }
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
void Planner::publish_tree()
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
}


int main(int argc, char **argv)
{

    ros::init(argc, argv, "rrts_node");
    //ros::NodeHandle n;

    Planner my_planner;

    ros::spin();
};
