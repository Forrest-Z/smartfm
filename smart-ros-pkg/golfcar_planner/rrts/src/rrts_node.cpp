#include <iostream>
#include <ctime>

#include <ros/ros.h>
#include <std_msgs/Int16MultiArray.h>
#include <std_msgs/Bool.h>
#include <message_filters/subscriber.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/PointCloud.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>

#include <tf/tf.h>
#include <tf/transform_listener.h>

#include <rrts/rrts_status.h>
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
        double planner_dt;

        geometry_msgs::Point32 goal;
        nav_msgs::OccupancyGrid map;
        bool is_first_goal, is_first_map;

        geometry_msgs::Point32 car_position;
        tf::TransformListener tf_;
        int get_robot_pose();

        double max_length_committed_trajectory;
        bool is_updating_committed_trajectory;
        void publish_committed_trajectory();
        list<double*> committed_trajectory;
        list<float> committed_control;
        void publish_control_view_trajectory();

        int clear_committed_trajectory();
        int clear_committed_trajectory_length();
        bool should_send_new_committed_trajectory;
        bool is_first_committed_trajectory;
        bool is_near_end_committed_trajectory(); 
        double state_last_clear[3];

        // ros
        ros::NodeHandle nh;
        ros::Subscriber goal_sub;
        ros::Subscriber map_sub;

        enum status_def{rinc=0, ginc, ginf, ring};
        bool rrts_status[4];

        ros::Publisher rrts_status_pub;
        ros::Timer rrts_status_timer;

        ros::Publisher tree_pub;
        ros::Publisher vertex_pub;
        ros::Publisher control_trajectory_pub;

        ros::Publisher committed_trajectory_pub;
        ros::Publisher committed_trajectory_view_pub;
        ros::Timer planner_timer;
        ros::Timer tree_pub_timer;
        ros::Timer committed_trajectory_pub_timer;

        // functions
        void on_goal(const geometry_msgs::PoseStamped::ConstPtr p);
        void send_rrts_status(const ros::TimerEvent &e);
        void on_map(const nav_msgs::OccupancyGrid::ConstPtr og);
        void on_committed_trajectory_pub_timer(const ros::TimerEvent &e);

        bool root_in_goal();
        bool is_robot_in_collision();
        void change_goal_region();
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
    max_length_committed_trajectory = 15.0;
    
    planner_dt = 0.5;
    planner_timer = nh.createTimer(ros::Duration(planner_dt), &Planner::on_planner_timer, this);
    rrts_status_timer = nh.createTimer(ros::Duration(0.5), &Planner::send_rrts_status, this);

    tree_pub_timer = nh.createTimer(ros::Duration(0.3), &Planner::on_tree_pub_timer, this);
    committed_trajectory_pub_timer = nh.createTimer(ros::Duration(0.3), &Planner::on_committed_trajectory_pub_timer, this);

    committed_trajectory_pub = nh.advertise<nav_msgs::Path>("pnc_trajectory", 2);
    committed_trajectory_view_pub = nh.advertise<nav_msgs::Path>("pncview_trajectory", 2);
    tree_pub = nh.advertise<sensor_msgs::PointCloud>("rrts_tree", 2);
    vertex_pub = nh.advertise<sensor_msgs::PointCloud>("rrts_vertex", 2);
    control_trajectory_pub = nh.advertise<std_msgs::Int16MultiArray>("control_trajectory_msg", 2);	

    map_sub = nh.subscribe("local_map", 2, &Planner::on_map, this);
    goal_sub = nh.subscribe("pnc_nextpose", 2, &Planner::on_goal, this);
    rrts_status_pub = nh.advertise<rrts::rrts_status>("rrts_status", 2);

    is_first_goal = true;
    is_first_map = true;
    for(int i=0; i<4; i++)
        rrts_status[i] = false;
}

Planner::~Planner()
{
    clear_committed_trajectory();
}

int Planner::clear_committed_trajectory()
{
    is_updating_committed_trajectory = true;
    for(list<double*>::iterator i=committed_trajectory.begin(); i!=committed_trajectory.end(); i++)
    {
        double* stateRef = *i;
        delete[] stateRef;
    }
    committed_trajectory.clear();
    committed_control.clear();

    publish_committed_trajectory();

    is_updating_committed_trajectory = false;

    if(get_robot_pose() == 1)
        cout<<"robot_pose failed"<<endl;
    state_last_clear[0] = car_position.x;
    state_last_clear[1] = car_position.y;
    state_last_clear[2] = car_position.z;
    return 0;
}

// keep doing pop_front on the committed_trajectory until length
int Planner::clear_committed_trajectory_length()
{
    if(get_robot_pose() == 1)
        cout<<"robot_pose failed"<<endl;
    bool reached_end = false;
    list<double*>::iterator iter = committed_trajectory.begin();
    int num_delete = 0;
    while (!reached_end)
    {
        double *s1 = *iter;
        if(dist(s1[0], s1[1], 0, car_position.x, car_position.y, 0) < 2.0)
        {
            reached_end = true;
            break;
        }
        iter++;
        num_delete++;
    }
    is_updating_committed_trajectory = true;
    if(reached_end)
    {
        int n = 0;
        while(n < num_delete)
        {
            committed_trajectory.pop_front();
            committed_control.pop_front();
            n++;
        }
    }

    state_last_clear[0] = car_position.x;
    state_last_clear[1] = car_position.y;
    state_last_clear[2] = car_position.z;
    is_updating_committed_trajectory = false;
    return 0;
}

void Planner::send_rrts_status(const ros::TimerEvent &e)
{
    rrts::rrts_status smsg;
    smsg.header.stamp = ros::Time::now();
    smsg.robot_in_collision = rrts_status[rinc];
    smsg.goal_in_collision = rrts_status[ginc];
    smsg.goal_infeasible = rrts_status[ginf];
    smsg.root_in_goal = rrts_status[ring];
    rrts_status_pub.publish(smsg);
}

// p is (x,y,yaw) in map coords
void Planner::on_goal(const geometry_msgs::PoseStamped::ConstPtr ps)
{
    double roll=0, pitch=0, yaw=0;
    tf::Quaternion q;
    tf::quaternionMsgToTF(ps->pose.orientation, q);
    tf::Matrix3x3(q).getRPY(roll, pitch, yaw);

    if(is_first_goal)
    {
        is_first_goal = false;
        cout<<"got first goal"<<endl;
        goal.x = ps->pose.position.x;
        goal.y = ps->pose.position.y;
        goal.z = yaw;
        double goal_state[3] = {goal.x, goal.y, goal.z};
        cout<<"goal goal: "<< goal.x<<" "<<goal.y<<" "<<goal.z<<endl;
        if(is_first_map == false)
        {
            setup_rrts();
            if(rrts.system->IsInCollision(goal_state, false))
            {
                cout<<"goal in collision: stopping"<<endl;
                clear_committed_trajectory();
                setup_rrts();
                rrts_status[ginc] = true;
            }
            rrts_status[ginc] = false;
        }
    }
    // new goal than previous one, change sampling region
    else if( dist(goal.x, goal.y, 0., ps->pose.position.x, ps->pose.position.y, 0.) > 0.5)
    {
        goal.x = ps->pose.position.x;
        goal.y = ps->pose.position.y;
        goal.z = yaw;
        double goal_state[3] = {goal.x, goal.y, goal.z};
        cout<<"goal goal: "<< goal.x<<" "<<goal.y<<" "<<goal.z<<endl;
        if(rrts.system->IsInCollision(goal_state, false))
        {
            cout<<"goal in collision: sending collision"<<endl;
            //clear_committed_trajectory();
            //setup_rrts();
            rrts_status[ginc] = true;
        }
        change_goal_region();
        rrts_status[ginc] = false;
    }
    //ROS_INFO("got goal: %f %f %f", goal.x, goal.y, goal.z);
}

int Planner::get_robot_pose()
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
        //cout<<"get_robot_pose: "<< car_position.x<<" "<<car_position.y<<" "<<car_position.z<<endl;

        /*
           if( (is_first_map==false) && (is_first_goal==false))
           {
           if(rrts.system->IsInCollision(system.map_origin))
           {
           cout<<"current car position in collision abort"<<endl;
           exit(0);
           }
           }
           */
        return 0;
    }
    return 1;
}

void Planner::on_map(const nav_msgs::OccupancyGrid::ConstPtr og)
{
    // 1. copy the incoming grid into map
    system.map = *og;

    bool got_pose = true;
    // 2. get car_position
    if(get_robot_pose() == 1)
    {
        got_pose = false;
        cout<<"robot_pose failed"<<endl;
    }
    if(is_first_map && got_pose)
    {
        state_last_clear[0] = car_position.x;
        state_last_clear[1] = car_position.y;
        state_last_clear[2] = car_position.z;
    }
    if(is_first_map)
    {
        is_first_map = false;
        cout<<"got first map"<<endl;
        if(is_first_goal == false)
            setup_rrts();
    }
    if(got_pose)
    {
        system.map_origin[0] = car_position.x;
        system.map_origin[1] = car_position.y;
        system.map_origin[2] = car_position.z;
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
    bool res = system.isReachingTarget(curr_state); 
    rrts_status[ring] = res;
    return res;
}


void Planner::change_goal_region()
{
    system.regionGoal.center[0] = (double)goal.x;
    system.regionGoal.center[1] = (double)goal.y;
    system.regionGoal.center[2] = (double)goal.z;
    system.regionGoal.size[0] = 1.0;
    system.regionGoal.size[1] = 1.0;
    system.regionGoal.size[2] = 30.0/180.0*M_PI;
    //cout<<"region_goal: "<< system.regionGoal.center[0]<<" "<<system.regionGoal.center[1]<<" "<<system.regionGoal.center[2]<<endl;
}

void Planner::setup_rrts()
{
    //cout<<"called setup_rrts"<<endl;

    // get car_position
    if(get_robot_pose() == 1)
        cout<<"robot_pose failed"<<endl;

    rrts.setSystem(system);
    vertex_t &root = rrts.getRootVertex();  
    state_t &rootState = root.getState();
    rootState[0] = car_position.x;
    rootState[1] = car_position.y;
    rootState[2] = car_position.z;

    system.regionOperating.center[0] = 0;
    system.regionOperating.center[1] = 0;
    system.regionOperating.center[2] = 0;
    //cout<<"regionOperating: "<< system.regionOperating.center[0]<<" "<<system.regionOperating.center[1]<<" "<<system.regionOperating.center[2]<<endl;

    // just create a large operating region around the car irrespective of the orientation in /map frame
    // yaw is 2*M_PI
    system.regionOperating.size[0] = system.map.info.height*system.map.info.resolution;
    system.regionOperating.size[1] = system.map.info.width*system.map.info.resolution;
    system.regionOperating.size[2] = 2.0 * M_PI;

    change_goal_region();

    // Set planner parameters
    rrts.setGamma (2.0);
    rrts.setGoalSampleFrequency (0.4);

    // Initialize the planner
    rrts.initialize ();
    //cout<<"setup_rrts complete"<<endl;

    // planner parameters about the first committed trajectory
    should_send_new_committed_trajectory = false;
    is_first_committed_trajectory = true;
}

bool Planner::is_robot_in_collision()
{
    if((!is_first_map) && (!is_first_goal))
    {
        get_robot_pose();
        double tmp[3] = {car_position.x, car_position.y, car_position.z};
        bool res = rrts.system->IsInCollision(tmp);
        rrts_status[rinc] = res;
        return res;
    }
    else
    {
        rrts_status[rinc] = false;
        return false;
    }
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
    if(is_robot_in_collision())
    {
        cout<<"robot in collision"<<endl;
        clear_committed_trajectory();
        return;
    }
    //cout<<"after check_tree num_vert: "<< rrts.numVertices<<endl;
    bool found_best_path = false;
    double best_cost=rrts.getBestVertexCost();
    double prev_best_cost=best_cost;
    int samples_this_loop = 0;

    ros::Time start_current_call_back = ros::Time::now();
    cout<<"s: "<< rrts.numVertices<<" best_cost: "<<best_cost;
    flush(cout);
    while((!found_best_path) || (rrts.numVertices < 10))
    {
        samples_this_loop += rrts.iteration();
        best_cost = rrts.getBestVertexCost();
        if(best_cost < 5000)
        {
            if( fabs(prev_best_cost - best_cost) < 0.05)
                found_best_path = true;
        }
        //cout<<"n: "<< rrts.numVertices<<endl;
        
        if(samples_this_loop %5 == 0)
            prev_best_cost = best_cost;

        ros::Duration dt = ros::Time::now() - start_current_call_back;
        // give some time to the following code as well
        if(dt.toSec() > 0.8*planner_dt)
            break;
    }
    cout<<" e: "<< rrts.numVertices<<" best_cost: "<< best_cost<<endl;
    if(found_best_path)
    {
        if( should_send_new_committed_trajectory || is_first_committed_trajectory )
        {
            is_updating_committed_trajectory = true;
            if(rrts.switchRoot(max_length_committed_trajectory, committed_trajectory, committed_control) == 0)
            {
                cout<<"cannot switch_root: lowerBoundVertex = NULL"<<endl;
                exit(0);
            }
            else
            {
                // change sampling region if successful switch_root
                change_goal_region();
                cout<<"switched root successfully"<<endl;
                cout<<"committed_trajectory len: "<< committed_trajectory.size()<<endl;
            }
            is_updating_committed_trajectory = false;
            should_send_new_committed_trajectory = false;
            is_first_committed_trajectory = false;
        }
    }
    else if(rrts.numVertices > 200)
    {
        rrts_status[ginf] = true;
        cout<<"did not find best path: reinitializing"<<endl;
        clear_committed_trajectory();
        setup_rrts();
    }
}

bool Planner::is_near_end_committed_trajectory()
{
    // latest car_position
    if(get_robot_pose() == 1)
        cout<<"robot_pose failed"<<endl;

    list<double*>::reverse_iterator riter = committed_trajectory.rbegin();
    double* last_committed_state = *riter;
    double delyaw = car_position.x - last_committed_state[2];
    while(delyaw > M_PI)
        delyaw -= 2.0*M_PI;
    while(delyaw < -M_PI)
        delyaw += 2.0*M_PI;

    if(dist(car_position.x, car_position.y, 0, last_committed_state[0], last_committed_state[1], 0) < 4.0)
        return true;
    else
        return false;
}

void Planner::on_planner_timer(const ros::TimerEvent &e)
{
    cout<<endl;
    
    if(!committed_trajectory.empty())
    {
        // 1. check if trajectory is safe
        if(!rrts.isSafeTrajectory(committed_trajectory))
        {
            cout<<"committed trajectory unsafe"<<endl;
            clear_committed_trajectory();
            setup_rrts();
        }
        // 2. check if it is at the end of the trajectory
        else if(is_near_end_committed_trajectory() && (!root_in_goal()))
        {
            cout<<"appending to committed trajectory"<<endl;
            //clear_committed_trajectory();
            should_send_new_committed_trajectory = true;
            clear_committed_trajectory_length();
            get_plan();
            return;
        }
        // 3. if far from committed trajectory, clear everything
        else
        {
            bool is_far_away = true;
            for(list<double*>::iterator i=committed_trajectory.begin(); i!=committed_trajectory.end(); i++)
            {
                double* curr_state = *i;
                if(dist(car_position.x, car_position.y, 0., curr_state[0], curr_state[1], 0.) < 1.0)
                    is_far_away = false;
            }
            if(is_far_away == true)
            {
                cout<<"is_far_away: emergency replan"<<endl;
                clear_committed_trajectory();
                setup_rrts();
                get_plan();
                return;
            }
        }
    }
    // 4. else add more vertices / until you get a good trajectory, copy it to committed trajectory, return
    if( (is_first_goal == false) && (is_first_map == false) )
    {
        get_plan();
        return;
    }
    
    if( dist(car_position.x, car_position.y, 0, state_last_clear[0], state_last_clear[1], 0) > 2.0)
        clear_committed_trajectory_length();
}

void Planner::publish_control_view_trajectory()
{
    std_msgs::Int16MultiArray tmp;

    for(list<float>::iterator i=committed_control.begin(); i!=committed_control.end(); i++)
    {
        tmp.data.push_back((int)(*i));
    }
    control_trajectory_pub.publish(tmp);
}

void Planner::on_committed_trajectory_pub_timer(const ros::TimerEvent &e)
{
    publish_committed_trajectory();
    publish_control_view_trajectory();
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
            //cout<<"published_rrts_vertex: "<< p.x<<" "<<p.y<<endl;
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

    Planner my_planner;

    ros::spin();
};
