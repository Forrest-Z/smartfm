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
#include <tf/transform_listener.h>

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
        bool is_first_goal, is_first_map;

        geometry_msgs::Point32 car_position;
        tf::TransformListener tf_;
        void get_robot_pose();

        bool is_updating_committed_trajectory;
        void publish_committed_trajectory();
        list<double*> committed_trajectory;
        list<float> committed_control;
        int clear_committed_trajectory();
        bool reached_end_of_committed_trajectory();

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
        float dist(float x1, float y1, float z1=0, float x2=0, float y2=0, float z2=0)
        {
            return sqrt( (x1-x2)*(x1-x2) + (y1-y2)*(y1-y2) + (z1-z2)*(z1-z2));
        }

        void publish_tree();
        void on_tree_pub_timer(const ros::TimerEvent &e);
};

Planner::Planner()
{
    srand(0);

    clear_committed_trajectory();
    is_updating_committed_trajectory = false;

    planner_timer = nh.createTimer(ros::Duration(0.5), &Planner::on_planner_timer, this);

    tree_pub_timer = nh.createTimer(ros::Duration(0.5), &Planner::on_tree_pub_timer, this);
    committed_trajectory_pub_timer = nh.createTimer(ros::Duration(0.5), &Planner::on_committed_trajectory_pub_timer, this);
     
    committed_trajectory_pub = nh.advertise<nav_msgs::Path>("pnc_trajectory", 2);
    committed_trajectory_view_pub = nh.advertise<nav_msgs::Path>("pncview_trajectory", 2);
    tree_pub = nh.advertise<sensor_msgs::PointCloud>("rrts_tree", 2);
    vertex_pub = nh.advertise<sensor_msgs::PointCloud>("rrts_vertex", 2);

    map_sub = nh.subscribe("local_map", 2, &Planner::on_map, this);
    goal_sub = nh.subscribe("goal", 2, &Planner::on_goal, this);

    rrts_max_iter = 200;
    is_first_goal = true;
    is_first_map = true;
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
    double goal_state[3] = {goal.x, goal.y, goal.z};
    cout<<"goal goal: "<< goal.x<<" "<<goal.y<<" "<<goal.z<<endl;
    if(is_first_goal)
    {
        is_first_goal = false;
        cout<<"got first goal"<<endl;
        if(is_first_map == false)
        {
            setup_rrts();
            if(rrts.system->IsInCollision(goal_state))
            {
                cout<<"goal in collision: abort"<<endl;
                exit(0);
            }
        }
    }
    // new goal than previous one, change sampling region
    else if( dist(goal.x, goal.y, goal.z, p->position.z, p->position.y, yaw) > 0.5)
    {
        if(rrts.system->IsInCollision(goal_state))
        {
            cout<<"goal in collision: abort"<<endl;
            exit(0);
        }
        change_sampling_region();
    }
    //ROS_INFO("got goal: %f %f %f", goal.x, goal.y, goal.z);
}

void Planner::get_robot_pose()
{
    tf::Stamped<tf::Pose> map_pose;
    map_pose.setIdentity();
    tf::Stamped<tf::Pose> robot_pose;
    robot_pose.setIdentity();
    robot_pose.frame_id_ = "base_link";
    robot_pose.stamp_ = ros::Time();
    ros::Time current_time = ros::Time::now();
    
    bool transform_is_correct = false;
    try {
        tf_.transformPose("map", robot_pose, map_pose);
    }
    catch(tf::LookupException& ex) {
        ROS_ERROR("No Transform available Error: %s\n", ex.what());
        transform_is_correct = false;
    }
    catch(tf::ConnectivityException& ex) {
        ROS_ERROR("Connectivity Error: %s\n", ex.what());
        transform_is_correct = false;
    }
    catch(tf::ExtrapolationException& ex) {
        ROS_ERROR("Extrapolation Error: %s\n", ex.what());
        transform_is_correct = false;
    }
    // check odom_pose timeout
    if (current_time.toSec() - map_pose.stamp_.toSec() > 0.1) {
        ROS_WARN("Get robot pose transform timeout. Current time: %.4f, odom_pose stamp: %.4f, tolerance: %.4f",
                current_time.toSec(), map_pose.stamp_.toSec(), 0.1);
        transform_is_correct = false;
    }
    transform_is_correct = true;
    
    if(transform_is_correct)
    {
        geometry_msgs::PoseStamped tmp;
        tf::poseStampedTFToMsg(map_pose, tmp);
        
        double roll=0, pitch=0, yaw=0;
        tf::Quaternion q;
        tf::quaternionMsgToTF(tmp.pose.orientation, q);
        tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
        car_position.x = tmp.pose.position.x;
        car_position.y = tmp.pose.position.y;
        car_position.z = yaw;
        //cout<<car_position<<endl;
        
        system.map_origin[0] = car_position.x;
        system.map_origin[1] = car_position.y;
        system.map_origin[2] = car_position.z;
    }
}

void Planner::on_map(const nav_msgs::OccupancyGrid::ConstPtr og)
{
    // 1. copy the incoming grid into map
    system.map = *og;
    
    // 2. get car_position
    get_robot_pose();

    if(is_first_map)
    {
        is_first_map = false;
        cout<<"got first map"<<endl;
        if(is_first_goal == false)
            setup_rrts();
    }
    /*
    cout<<"system.map_origin: "<< system.map_origin[0]<<" "<<system.map_origin[1]<<" "<<system.map_origin[2]<<endl;
    cout<<"is_in_collision: "<< system.IsInCollision (system.map_origin)<<endl;
    cout<<endl;
    */
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
    cout<<"rootState: "<<rootState[0]<<" "<<rootState[1]<<" "<<rootState[2]<<endl;
    
    // center of the map is the center of the local_map but in /map frame
    // yaw is 0
    system.regionOperating.center[0] = 0; //rootState[0] + cyaw*system.map.info.height/4.0*system.map.info.resolution;
    system.regionOperating.center[1] = 0; //rootState[1] + syaw*system.map.info.height/4.0*system.map.info.resolution;
    system.regionOperating.center[2] = 0;
    cout<<"regionOperating: "<< system.regionOperating.center[0]<<" "<<system.regionOperating.center[1]<<" "<<system.regionOperating.center[2]<<endl;
     
    // just create a large operating region around the car irrespective of the orientation in /map frame
    // yaw is 2*M_PI
    double size = sqrt(pow(system.map.info.height,2) + pow(system.map.info.width,2))*system.map.info.resolution;
    system.regionOperating.size[0] = system.map.info.height*system.map.info.resolution;
    system.regionOperating.size[1] = system.map.info.width*system.map.info.resolution;
    system.regionOperating.size[2] = 2.0 * M_PI;
    cout<<"regionOperating: "<< system.regionOperating.size[0]<<" "<<system.regionOperating.size[1]<<" "<<system.regionOperating.size[2]<<endl;

    system.regionGoal.center[0] = (double)goal.x;
    system.regionGoal.center[1] = (double)goal.y;
    system.regionGoal.center[2] = (double)goal.z;
    system.regionGoal.size[0] = 1.0;
    system.regionGoal.size[1] = 1.0;
    system.regionGoal.size[2] = 20.0/180.0*M_PI;
    cout<<"region_goal: "<< system.regionGoal.center[0]<<" "<<system.regionGoal.center[1]<<" "<<system.regionGoal.center[2]<<endl;

}

void Planner::setup_rrts()
{
    cout<<"called setup_rrts"<<endl;

    // get car_position
    get_robot_pose();
    
    rrts.setSystem(system);
    vertex_t &root = rrts.getRootVertex();  
    state_t &rootState = root.getState();
    rootState[0] = car_position.x;
    rootState[1] = car_position.y;
    rootState[2] = car_position.z;
    
    change_sampling_region();

    // Set planner parameters
    rrts.setGamma (2.0);
    rrts.setGoalSampleFrequency (0.3);

    // Initialize the planner
    rrts.initialize ();
    cout<<"setup_rrts complete"<<endl;
}

void Planner::get_plan()
{
    rrts.checkTree();
    rrts.updateReachability();
    if(root_in_goal())
    {
        //cout<<"root in goal"<<endl;
        return;
    }
    //cout<<"after check_tree num_vert: "<< rrts.numVertices<<endl;
    bool found_best_path = false;
    double best_cost=1e20, prev_best_cost=1e20;
    while(!found_best_path)
    {
        rrts.iteration();
        best_cost = rrts.getBestVertexCost();
        if( (best_cost < 25) && (rrts.numVertices > 25))
        {
            if( (prev_best_cost - best_cost) < 0.5)
                found_best_path = true;
        }
        prev_best_cost = best_cost;
        
        if(rrts.numVertices > 200)
            break;
        //cout<<endl;
    }
    cout<<"n: "<< rrts.numVertices<<endl;
    if(found_best_path)
    {
        cout<<"found best path with cost: "<<best_cost<<endl;
        if(committed_trajectory.empty())
        {
            is_updating_committed_trajectory = true;
            if(rrts.switchRoot(10, committed_trajectory, committed_control) == 0)
                cout<<"cannot switch_root: lowerBoundVertex = NULL"<<endl;
            else
            {
                // change sampling region if successful switch_root
                change_sampling_region();
                cout<<"switched root successfully"<<endl;
                cout<<"committed_trajectory len: "<< committed_trajectory.size()<<endl;
            }
            is_updating_committed_trajectory = false;
        }
    }
    else
    {
        ROS_INFO("223: did not find good path");
    }

}

bool Planner::reached_end_of_committed_trajectory()
{
    // latest car_position
    get_robot_pose();
    
    list<double*>::reverse_iterator riter = committed_trajectory.rbegin();
    double* last_committed_state = *riter;
    double delyaw = car_position.x - last_committed_state[2];
    while(delyaw > M_PI)
        delyaw -= 2.0*M_PI;
    while(delyaw < -M_PI)
        delyaw += 2.0*M_PI;

    /*
    if( (fabs(car_position.x - last_committed_state[0]) > system.regionGoal.size[0]/2.0) ||
        (fabs(car_position.y - last_committed_state[1]) > system.regionGoal.size[1]/2.0) || 
        ( fabs(delyaw) > system.regionGoal.size[2]/2.0) )
        return false;
    else
        return true;
    */
    if(dist(car_position.x, car_position.y, 0, last_committed_state[0], last_committed_state[1], 0) < 0.5)
        return true;
    else
        return false;
}

void Planner::on_planner_timer(const ros::TimerEvent &e)
{
    // 1. if at the end of committed trajectory then clear trajectory and go to 3
    if(!committed_trajectory.empty())
    {
        /*
        if(!rrts.isSafeTrajectory(committed_trajectory))
        {
            cout<<"committed trajectory unsafe"<<endl;
            clear_committed_trajectory();
            setup_rrts();
        }
        else 
        */ 
        if(reached_end_of_committed_trajectory())
        {
            cout<<"reached end of committed trajectory"<<endl;
            clear_committed_trajectory();
            setup_rrts();
        }
        else
        {
            // 2. if far from committed trajectory, clear everything and go to 3
            bool is_far_away = true;
            for(list<double*>::iterator i=committed_trajectory.begin(); i!=committed_trajectory.end(); i++)
            {
                double* curr_state = *i;
                if(dist(car_position.x, car_position.y, car_position.z, curr_state[0], curr_state[1], curr_state[2]) < 0.5)
                    is_far_away = false;
            }
            if(is_far_away == true)
            {
                cout<<"is_far_away: emergency replan"<<endl;
                clear_committed_trajectory();
                setup_rrts();
            }
        }
    }
    
    // 3. else add more vertices / until you get a good trajectory, copy it to committed trajectory, return
    if( (is_first_goal == false) && (is_first_map == false) )
    {
        get_plan();
    }
}

void Planner::on_committed_trajectory_pub_timer(const ros::TimerEvent &e)
{
    publish_committed_trajectory();
}

void Planner::publish_committed_trajectory()
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
        
        //printf(" [%f, %f, %f]", p.pose.position.x, p.pose.position.y, p.pose.position.z);
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
    //cout<<"published committed_trajectory"<<endl;
}

void Planner::on_tree_pub_timer(const ros::TimerEvent &e)
{
    publish_tree();
    //cout<<"published tree"<<endl;
}
void Planner::publish_tree()
{
    int num_nodes = rrts.numVertices;

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
            //cout<<"published: "<< p.x<<" "<<p.y<<endl;
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
    //cout<<"published tree"<<endl;
}


int main(int argc, char **argv)
{

    ros::init(argc, argv, "rrts_node");
    //ros::NodeHandle n;

    Planner my_planner;

    ros::spin();
};
