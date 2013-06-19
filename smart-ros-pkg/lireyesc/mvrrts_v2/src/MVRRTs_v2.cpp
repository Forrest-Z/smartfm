// Header files: Standard C++
#include <ctime>

// Header files: Custom [Luis]
#include "MVRRTs_v2.hpp"
#include "Defs_Utilities.hpp"

/** Turning radius of the golf cart.  */

#define GOLF_CART_TURN_RADIUS           3.0

/** Discretization step-size. */

#define GOLF_CART_DISCRETIZATION_STEP   0.05

/** Duration of time interval between transmission of committed trajectory. */

#define INTERVAL_COMMITTED_TRAJ         0.5

/** Minimum number of nodes in the MVRRT* tree. */

#define MIN_SIZE_OF_TREE                2000

/** Maximum length of time the MVRRT* algorithm is allowed to run for (in seconds). */

#define MVRRTSTAR_MAX_RUNTIME           5

/** Rate at which we sample from the local map as opposed to the global one. */

#define RATE_GLOBAL_VS_LOCAL            1.0

/** Heuristic sampling rate for the global map. */

#define HEURISTIC_RATE_GLOBAL           1.0

/** Heuristic sampling rate for the local map. */

#define HEURISTIC_RATE_LOCAL            0.0

Golf_Cart_MVRRTs::Golf_Cart_MVRRTs( const format_t turn_radius, 
        
                                    const format_t delta_x, 
        
                                    const Auto_AB& auto_AB) : 
                                    
    // Sets up all members related to localization
                                    
    map_global (), map_local (), 
        
    tf_listener_base_link_to_map(), 
        
    // Sets up all members related to the MVRRT* algorithm
            
    golf_cart ( turn_radius, delta_x, map_global, map_local, auto_AB, rules_of_road), 
        
    robot_state { 0, 0, 0}, 
            
    goal_region (), 
        
    mvrrts (NULL), mvrrts_status_msg (), commited_traj (), 
            
    // Sets up all members related to ROS functionalities
        
    node (), 

    pub_committed_controls ( node.advertise < std_msgs::Int16MultiArray > ( "control_trajectory", 1) ), 

    pub_committed_traj ( node.advertise < nav_msgs::Path > ( "pnc_trajectory", 1) ), 
            
    pub_committed_traj_RVIZ ( node.advertise < nav_msgs::Path > ( "pncview_trajectory", 1) ), 
            
    pub_mvrrts_nodes ( node.advertise < sensor_msgs::PointCloud > ( "rrts_vertex", 1) ), 

    pub_mvrrts_tree ( node.advertise < sensor_msgs::PointCloud > ( "rrts_tree", 1) ), 
            
    pub_planner_status ( node.advertise < rrts::rrts_status > ( "rrts_status", 1) ), 
            
    sub_goal ( node.subscribe( 
            
        "move_base_simple/goal", 1, &Golf_Cart_MVRRTs::callback_for_goal, this) ), 

    sub_map_global ( node.subscribe( 
            
        "lane_map", 1, &Golf_Cart_MVRRTs::callback_for_map_global, this) ), 
        
    sub_map_local ( node.subscribe( 
            
        "local_map", 1, &Golf_Cart_MVRRTs::callback_for_map_local, this) ), 
    
    timer_pub_committed_traj ( node.createTimer( 
            
        ros::Duration(INTERVAL_COMMITTED_TRAJ), &Golf_Cart_MVRRTs::callback_for_committed_traj, this) ), 
            
    timer_pub_mvrrts ( node.createTimer( 
            
        ros::Duration(INTERVAL_COMMITTED_TRAJ), &Golf_Cart_MVRRTs::callback_for_mvrrts, this) ), 
    
    time_last_replan( ros::Time::now() ), 
            
    // Sets up all switches
            
    locked_committed_traj(false), locked_tree(false), 
            
    // FOR DEBUGGING PURPOSES
    
    DEBUG_TIMER( node.createTimer( ros::Duration(0.2), &Golf_Cart_MVRRTs::DEBUG_CALLBACK, this) ) {
    
    // Sets up the size of the goal region
    
    goal_region.radius_xy = 1.0;
    
    goal_region.radius_th = deg2rad(30);
    
    // Retrieves state of the robot (in global map coordinates)
    
    ros::Rate get_robot_state_loop_rate (10);
    
    while ( !get_robot_state() ) {
        
        ros::spinOnce(); get_robot_state_loop_rate.sleep();
    
    }
    
    ROS_INFO( "Golf_Cart_MVRRTs: Received robot state.");
    
    ROS_DEBUG( "State: ( %.3f , %.3f , %.3f )", robot_state[0], robot_state[1], rad2Deg(robot_state[2]) );
    
    // FOR INITIAL DESIGN PURPOSES

    mvrrts_status_msg.goal_in_collision = false;

    mvrrts_status_msg.goal_infeasible = false;

    mvrrts_status_msg.robot_in_collision = false;

    mvrrts_status_msg.robot_near_root = false;

    mvrrts_status_msg.root_in_goal = false;

    mvrrts_status_msg.switched_root = true;
    
}

Golf_Cart_MVRRTs::~Golf_Cart_MVRRTs() {
    
    if ( mvrrts ) { delete mvrrts; }
    
}

void Golf_Cart_MVRRTs::callback_for_committed_traj( const ros::TimerEvent& event) {
    
    publish_committed_trajectory();
    
}

void Golf_Cart_MVRRTs::callback_for_goal( const geometry_msgs::PoseStamped::ConstPtr msg) {
    
    // Computes angular coordinate of goal state
    
    tf::Quaternion goal_state_q;
    
    tf::quaternionMsgToTF( (msg -> pose).orientation, goal_state_q);
    
    double roll = 0, pitch = 0, yaw = 0;
    
    ( tf::Matrix3x3(goal_state_q) ).getRPY( roll, pitch, yaw);
    
    // Retrieves and stores goal state and prints informative message to ROS-CONSOLE
    
    goal_region.center[0] = (format_t) (msg -> pose).position.x;
    
    goal_region.center[1] = (format_t) (msg -> pose).position.y;
    
    goal_region.center[2] = (format_t) yaw;
    
    ROS_INFO("Received goal state.");
    
    // Verifies that goal state is not in collision
    
    if ( map_global.is_in_collision(goal_region.center) ) {
        
        mvrrts_status_msg.goal_in_collision = true;
        
        ROS_WARN("Goal state is in collision!");
        
    }
    
    else {
    
        mvrrts_status_msg.goal_in_collision = false;
    
        search_for_plan();
    }
    
}

void Golf_Cart_MVRRTs::callback_for_map_global(const nav_msgs::OccupancyGrid& msg) {
    
    map_global.read_map(msg);
    
}

void Golf_Cart_MVRRTs::callback_for_map_local( const pnc_msgs::local_map& msg) {
    
    map_local.read_map(msg);
    
}

void Golf_Cart_MVRRTs::callback_for_mvrrts( const ros::TimerEvent& event) {
    
    // Publishes MVRRT* tree
    
    publish_tree();
    
    // Declares, initializes, and publishes planner status message
    
    rrts::rrts_status current_status    = mvrrts_status_msg;
    
    current_status.header.frame_id      = "/map";
    
    current_status.header.stamp         = ros::Time::now();
    
    pub_planner_status.publish(current_status);
    
}

bool Golf_Cart_MVRRTs::get_robot_state() {
    
    // Declares and initializes identity transform from map to map
    
    tf::Stamped < tf::Pose > map_pose;
    
    map_pose.setIdentity();
    
    // Declares and initializes identity transform from robot to robot
    
    tf::Stamped < tf::Pose > robot_pose;
    
    robot_pose.setIdentity();
    
    // Sets the frame of the robot transform to the base link
    
    robot_pose.frame_id_ = "base_link";
    
    robot_pose.stamp_ = ros::Time();
    
    // Attempts to compute the pose of the robot in the map frame
    
    try { tf_listener_base_link_to_map.transformPose( "map", robot_pose, map_pose); }
    
    catch ( tf::TransformException& exception ) {
        
        ROS_ERROR( "In get_robot_state(): %s\n", exception.what()); return false;
        
    }
    
    // Declares and initializes state (in the map frame) message
    
    geometry_msgs::PoseStamped state;
    
    tf::poseStampedTFToMsg( map_pose, state);
    
    // Computes angular coordinate of state
    
    tf::Quaternion state_q;
    
    tf::quaternionMsgToTF( state.pose.orientation, state_q);
    
    double roll = 0, pitch = 0, yaw = 0;
    
    ( tf::Matrix3x3(state_q) ).getRPY( roll, pitch, yaw);
    
    // Copies planar and angular coordinates of state
    
    robot_state[0] = (format_t) state.pose.position.x;
    
    robot_state[1] = (format_t) state.pose.position.y;
    
    robot_state[2] = (format_t) yaw;
    
    return true;
    
}

void Golf_Cart_MVRRTs::publish_committed_trajectory() {
    
    // If the committed trajectory is not locked (i.e. being modified)
    
    if ( mvrrts && !locked_committed_traj ) {
        
        // Locks committed trajectory
        
        locked_committed_traj = true;
        
        // Declares and initializes path to be committed

        nav_msgs::Path trajectory_msg;

        trajectory_msg.header.stamp = time_last_replan;

        trajectory_msg.header.frame_id = "/map";
        
        // Iterates through states along committed trajectory
        
        for ( auto& state : commited_traj ) {
            
            // Declares and initializes state along path to be committed

            geometry_msgs::PoseStamped path_state;

            path_state.header.stamp          = ros::Time::now();

            path_state.header.frame_id       = "/map";
            
            // Copies planar coordinates of state along trajectory

            path_state.pose.position.x       = (geometry_msgs::Pose::_position_type::_x_type) state[0];

            path_state.pose.position.y       = (geometry_msgs::Pose::_position_type::_y_type) state[1];
            
            // Copies control input of state along trajectory

            path_state.pose.position.z       = (geometry_msgs::Pose::_position_type::_y_type) state[3];;
            
            // Indicates that the car should move forward

            path_state.pose.orientation.w    = +1.0;
            
            // Inserts state and control input into path object

            trajectory_msg.poses.push_back(path_state);

        }

        // Commits (i.e.publishes) path message to the low-level controllers
        
        pub_committed_traj.publish(trajectory_msg);
        
        // Declares and initializes path message to be sent to RVIZ
        
        nav_msgs::Path trajectory_msg_for_RVIZ = trajectory_msg;
        
        // Iterates through states along committed path removing control inputs
        
        for ( unsigned int k = 0; k < trajectory_msg_for_RVIZ.poses.size(); k++) {
            
            trajectory_msg_for_RVIZ.poses[k].pose.position.z  = 0.0;
        }
        
        // Commits (i.e.publishes) path message to RVIZ
        
        pub_committed_traj_RVIZ.publish(trajectory_msg_for_RVIZ);
        
        // Unlocks committed trajectory
        
        locked_committed_traj = false;
        
    }
    
}

void Golf_Cart_MVRRTs::publish_tree() {
    
    // If the tree is not locked (i.e. being modified) then publish it to R-VIZ
    
    if ( mvrrts && !locked_tree ) {
        
        // Locks tree, publishes it, and then unlocks it
        
        locked_tree = true;
        
        mvrrts -> publish_tree( pub_mvrrts_nodes, pub_mvrrts_tree);
        
        locked_tree = false;
        
    }
    
}

bool Golf_Cart_MVRRTs::search_for_plan() {
    
    // Acquires current state of robot and terminates procedure if that fails
    
    if ( !get_robot_state() ) {
        
        ROS_WARN("Method search_for_plan failed because of method get_robot_state.");
        
        return false;
        
    }
    
    // Waits until both committed trajectory and tree are unlocked and then locks them
    
    while ( locked_committed_traj || locked_tree ) {}
    
    locked_committed_traj = locked_tree = true;
    
    // Re-constructs and sets up MVRRT* algorithm object

    delete mvrrts;

    mvrrts = new DubinsCar_MVRRTs( golf_cart, robot_state, goal_region);

    ROS_INFO( "MVRRT* root state in global coordinates: ( %.3f , %.3f , %.3f )", 

              robot_state[0], robot_state[1], robot_state[2]);
    
    // Set up Dubin's Car scaled boxes, sampling rates, and sampling heuristics
    
    mvrrts -> set_DCSB_search_object_params(16.0);
    
    mvrrts -> set_sampling_rates( RATE_GLOBAL_VS_LOCAL, HEURISTIC_RATE_GLOBAL, HEURISTIC_RATE_LOCAL);
    
    map_global.setup_heuristic_sampling_region( robot_state, goal_region);
    
    // Runs MVRRT* algorithm until either the tree is sufficiently large or we run out of time
    
    ROS_INFO( "Running MVRRT* algorithm...");
    
    clock_t start_time = clock();
    
    while ( ( mvrrts -> get_num_vertices() < MIN_SIZE_OF_TREE ) && 
            
          ( ! is_time_to_stop( start_time, MVRRTSTAR_MAX_RUNTIME) ) ) {
        
        mvrrts -> iteration();
    
    }
    
    // Prints size of tree and level of unsafety and retrieves best trajectory (if it exists)

    ROS_INFO( "Done. Size of the tree: %u nodes", mvrrts -> get_num_vertices());
    
    mvrrts -> print_LUS_of_best_trajectory();

    commited_traj = mvrrts -> get_best_traj();
    
    // Records re-plan time
    
    time_last_replan = ros::Time::now();

    // Unlocks committed trajectory and MVRRT* tree

    locked_committed_traj = locked_tree = false;
    
    return true;
        
}

void Golf_Cart_MVRRTs::DEBUG_CALLBACK( const ros::TimerEvent& event) {
    
    get_robot_state();
    
    if ( map_global.is_in_collision(robot_state) ) {
        
        ROS_WARN("ROBOT IN COLLISION according to global map.");
        
    }
    
//    format_t state_local [3];
//    
//    map_local.transform_global_to_local( robot_state, state_local);
//    
//    if ( map_local.is_in_collision(state_local) ) {
//        
//        ROS_WARN("ROBOT IN COLLISION according to local map.");
//        
//    }
//    
//    map_local.DEBUG_PRINT_MAP();
    
}

int main( int argc, char **argv) {
    
    // Sets up the ROS node
    
    ros::init( argc, argv, "mvrrts_v2");
    
    Auto_AB auto_AB;
    
    auto_AB.insert_Auto( new Auto_SS_SP( true, LANE_LEFT, 0.0, 1.0), 0, 0);

    Golf_Cart_MVRRTs MVRRTs_v2 ( GOLF_CART_TURN_RADIUS, GOLF_CART_DISCRETIZATION_STEP, auto_AB);
    
    // Enters an infinite loop controlled by the ROS master node
    
    ros::spin();

    return EXIT_SUCCESS;

}
