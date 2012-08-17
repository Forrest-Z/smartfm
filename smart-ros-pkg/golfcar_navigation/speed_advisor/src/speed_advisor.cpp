#include "speed_advisor.h"


SpeedAttribute SpeedAttribute::generate(const string & description,
        SpeedAttributeDescription element,
        double speed_now, double target_speed,
        double speed_inc, double speed_dec)
{
    SpeedAttribute attr;
    attr.speed_now_ = speed_now;
    attr.description_ = description;
    attr.element_ = element;
    attr.speed_inc_ = speed_inc;
    attr.speed_dec_ = speed_dec;
    attr.target_speed_ = target_speed;

    attr.final_speed_ = speed_now;
    if( speed_now > target_speed )
        attr.final_speed_ = fmutil::max(speed_now + speed_dec, target_speed);
    else if( speed_now < target_speed )
        attr.final_speed_ = fmutil::min(speed_now + speed_inc, target_speed);

    return attr;
}


SpeedAttribute & SpeedSettings::find_min_speed()
{
    assert( !empty() );
    SpeedSettings::iterator min, it;
    for( it=begin()+1, min=begin(); it!=end(); ++it)
        if( it->final_speed_ < min->final_speed_ )
            min = it;
    return *min;
}



SpeedAdvisor::SpeedAdvisor()
{
    ros::NodeHandle nh("~");
    nh.param("max_speed", high_speed_, 2.0);
    nh.param("acc", acc_, 0.5);
    nh.param("max_dec", max_dec_, 2.0);
    nh.param("norm_dec", norm_dec_, 0.7);
    nh.param("robot_frame_id", base_link_, string("base_link"));
    nh.param("map_frame_id", map_id_, string("map"));
    nh.param("dec_ints", dec_ints_, 0.5);
    nh.param("dec_station", dec_station_, 0.5);
    nh.param("ppc_stop_dist", ppc_stop_dist_, 5.0); //the stopping distance from ppc.
    //It was found that the distance given by move_status will reach as small as 4 meter
    //Hence, stopping distance should be at least larger than 4 for the stopping manoeuvre to work

    nh.param("frequency", frequency_,20.0);
    nh.param("tolerance", tolerance_, 0.5);
    nh.param("emergency_zone", e_zone_, 2.0);
    nh.param("slow_zone", slow_zone_, 10.0);
    nh.param("slow_speed", slow_speed_, 1.5);
    nh.param("baselink_carfront_length", baselink_carfront_length_, 2.0);
    nh.param("enterstation_speed",enterstation_speed_, slow_speed_); //by default, enterstation speed is the same as slow speed
    nh.param("stationspeed_dist", stationspeed_dist_, 20.0);
    nh.param("kinematic_acceleration", kinematics_acc_, true);

    nh_.param("use_sim_time", use_sim_time_, false);
    ROS_DEBUG_STREAM("Simulated time is "<<use_sim_time_);

    speed_now_ = 0;
    signal_type_ = -1;

    recommend_speed_pub_= nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1);
    speed_contribute_pub_ = nh_.advertise<pnc_msgs::speed_contribute>("speed_status",1);
    left_blinker_pub_ = nh_.advertise<std_msgs::Bool>("left_blinker",1);
    right_blinker_pub_ = nh_.advertise<std_msgs::Bool>("right_blinker",1);

    move_base_speed_ = nh_.subscribe("/move_status", 1, &SpeedAdvisor::moveSpeedCallback, this);
    slowzone_sub_ = nh_.subscribe("slowZone", 1, &SpeedAdvisor::slowZoneCallback, this);

    timer_ = nh_.createTimer(ros::Duration(1.0/frequency_),
                                &SpeedAdvisor::ControlLoop, this);
}

void SpeedAdvisor::slowZoneCallback(geometry_msgs::PoseArrayConstPtr slowzones)
{
    slowZone_.poses = slowzones->poses;
}

/* A shortcut to add a full brake velocity profile */
#define BRAKE(desc, el) speed_settings.push_back( SpeedAttribute::generate( \
            desc, el, speed_now_, 0, pos_speed, max_neg_speed) )

/** Checks a list of conditions and generate different speed profiles for each.
 * Finally choose the slowest profile.
 * TODO add user interface
 */
void SpeedAdvisor::ControlLoop(const ros::TimerEvent& event)
{
    ROS_DEBUG("Control Loop: t=%f", ros::Time::now().toSec());

    //pre-compute the amount of change in speed
    double max_neg_speed = -max_dec_ / frequency_;
    double pos_speed = acc_ / frequency_;
    double norm_neg_speed = -norm_dec_ / frequency_;

    // Container for the velocity profiles
    SpeedSettings speed_settings;

    // Add a normal speed profile
    speed_settings.push_back( SpeedAttribute::generate(
            "norm zone", SpeedAttribute::norm_zone,
            speed_now_, high_speed_, pos_speed, norm_neg_speed) );

    // Problem with the navigation node (move_base) --> brake
    if( fabs((ros::Time::now()-last_update_).toSec()) > tolerance_ )
        BRAKE("no response from move_base", SpeedAttribute::no_response);

    // PPC couldn't find a path to follow, it may due to localisation error or illegal path received
    if( ! move_status_.path_exist )
        BRAKE("move_base reports no path", SpeedAttribute::path_exist);

    if(move_status_.emergency)
        BRAKE("Emergency!", SpeedAttribute::emergency);

    // Get the position of the front of the robot in map coordinates.
    tf::Stamped<tf::Pose> robot_pose;
    getRobotGlobalPose(robot_pose);
    double yaw = robot_pose.getRotation().getAngle();
    double robot_x = robot_pose.getOrigin().x() + baselink_carfront_length_ * cos(yaw);
    double robot_y = robot_pose.getOrigin().y() + baselink_carfront_length_ * sin(yaw);

    // Check whether we are in a slow zone.
    // Slow zones are encoded in the paths and available in slowZone. They are defined
    // by their centre point (x,y) and size (z)
    for(unsigned i=0; i<slowZone_.poses.size(); i++)
    {
        double dist=fmutil::distance(robot_x, robot_y,
                slowZone_.poses[i].position.x,
                slowZone_.poses[i].position.y);
        if(dist < slowZone_.poses[i].position.z)
        {
            stringstream ss;
            ss <<"Slow zone: " <<i;
            speed_settings.push_back( SpeedAttribute::generate(
                    ss.str(), SpeedAttribute::slow_zone,
                    speed_now_, slow_speed_, pos_speed, norm_neg_speed) );
        }
    }

    // This message will be published later
    pnc_msgs::speed_contribute sc;

    //
    // Dealing with obstacles
    //
    double obs_dist = move_status_.obstacles_dist;

    // Obtsacle is very close (emergency zone)
    if(obs_dist<=e_zone_)
        BRAKE("Obs within emergency zone!", SpeedAttribute::e_zone);

    // Obstacle is not so close (slow zone)
    if( obs_dist < slow_zone_ )
        speed_settings.push_back( SpeedAttribute::generate(
                "Obs within safety zone", SpeedAttribute::warn_brake,
                speed_now_, slow_speed_, pos_speed, norm_neg_speed) );

    // Compute some deceleration value based on the distance to the
    // nearest obstacle

    // Although this is covered by the e_zone and slow_zone,
    // the algorithm below take into account the real kinematics of
    // the vehicle that should ensure that no collision would occur even
    // when slow_zone/e_zone specified is not enough to prevent collision
    // from occurring. The following algorithm also ensure smooth
    // transition of stopping behaviour
    sc.dec_req = pow(speed_now_,2)/(2*(obs_dist-e_zone_));
    if(kinematics_acc_)
    {
        if( sc.dec_req > max_dec_ )
            BRAKE("Obs 1st: Max brake", SpeedAttribute::max_brake);
        else if( sc.dec_req >= norm_dec_ )
            speed_settings.push_back( SpeedAttribute::generate(
                    "Obs 1st: really need to brake", SpeedAttribute::need_brake,
                    speed_now_, 0, pos_speed, -sc.dec_req/frequency_) );
    }

    //
    // Intersections
    //
    if( ! int_h_.is_clear_to_go() )
    {
        // Approaching an intersection, and the flag has not been cleared yet.
        // --> needs to stop.
        // The target velocity (sc.int_rec) is computed based on the distance
        // to the intersection stop point
        
        // todo: This could be problematic when there is a skip in path point
        // by the controller, the vehicle wouldn't stop but proceed to the
        // next point
        
        // todo: The other problem is when the calculation of int dist is
        // inaccurate, causes overshoot due to constraint given by decceleration
        // and the recommended speed is reported as the next point
        // need to add to check that the speed is actally zero 
        // there are some fundamental problem here:
        // --------- The inaacurate calculation of int dist ------------------

        float d = int_h_.dist_to_int() - ppc_stop_dist_;
        sc.int_rec = d<=0 ? 0 : sqrt(2 * dec_ints_ * d);

        ROS_DEBUG_STREAM("Intersection not clear. dist=" << int_h_.dist_to_int()
                <<". Recommended speed: " <<sc.int_rec);

        speed_settings.push_back( SpeedAttribute::generate(
                "Intersection", SpeedAttribute::intersection,
                speed_now_, sc.int_rec, pos_speed, norm_neg_speed) );
    }


    //
    // Approaching the goal:
    // - when closer than stationspeed_dist_, slow down to slow_speed_
    // - compute a deceleration profile that will make the car stop at the goal.
    //

    // Getting near the goal: slowing down to slow_speed_
    if( move_status_.dist_to_goal < stationspeed_dist_ )
        speed_settings.push_back( SpeedAttribute::generate(
                "Approaching goal", SpeedAttribute::app_goal,
                speed_now_, slow_speed_, pos_speed, norm_neg_speed) );

    // Smooth deceleration profile that will make the car stop at the goal
    double d = move_status_.dist_to_goal - ppc_stop_dist_;
    sc.goal_rec = d<0 ? 0 : sqrt(2*dec_station_*d);
    speed_settings.push_back( SpeedAttribute::generate(
            "Goal", SpeedAttribute::goal,
            speed_now_, sc.goal_rec, pos_speed, norm_neg_speed) );


    //
    // Decision making: go for the lowest speed
    //

    geometry_msgs::Twist move_speed;
    SpeedAttribute & sa = speed_settings.find_min_speed();
    move_speed.linear.x = speed_now_ = sa.final_speed_;
    move_speed.angular.z = move_status_.steer_angle;

    //it was found that, in simulation with stage, although commanded to
    //travel at 2 m/s, it is actually travelling at 3.33x faster,
    //compensation is needed
//    if( use_sim_time_ ) move_speed.linear.x *= 0.3;

    recommend_speed_pub_.publish(move_speed);


    sc.element = sa.element_;
    sc.description = sa.description_;
    sc.speed_now = speed_now_;
    sc.dist_goal = move_status_.dist_to_goal;
    //determine if goal has reached
    if( element_now_!=sc.element )
    {
        element_pre_ = element_now_;
        element_now_ = (SpeedAttribute::SpeedAttributeDescription) sc.element;
    }
    if( (sc.element == SpeedAttribute::goal || element_pre_ == SpeedAttribute::goal)
            && sc.speed_now == 0 && sc.dist_goal < 7 )
        sc.goal = true;

    speed_contribute_pub_.publish(sc);
}

void SpeedAdvisor::moveSpeedCallback(pnc_msgs::move_status status)
{
    last_update_ = ros::Time::now();
    move_status_ = status;

    //only change the button when the speed element is not from intersection
    // BRICE: Why? removing for now !
    //if( element_now_==SpeedAttribute::norm_zone
    //        || element_now_ == SpeedAttribute::slow_zone )
    //{
    int_h_.update(status);
    //}


    if( status.dist_to_sig < ppc_stop_dist_ && status.dist_to_sig > 0 )
    {
        if( status.sig_type != signal_type_ )
        {
            signal_type_ = status.sig_type;
            // 0: left_signal, 1: right_signal, 2: off_signals

            std_msgs::Bool blinker;
            blinker.data = (signal_type_==0);
            left_blinker_pub_.publish(blinker);
            blinker.data = (signal_type_==1);
            right_blinker_pub_.publish(blinker);
        }
    }
}

bool SpeedAdvisor::getRobotGlobalPose(tf::Stamped<tf::Pose>& odom_pose) const
{
    tf::Stamped<tf::Pose> robot_pose;
    robot_pose.setIdentity();
    robot_pose.frame_id_ =  base_link_;
    robot_pose.stamp_ = ros::Time(); // get the latest pose information
                                     // rather than try to match with current time.

    try {
        tf_.transformPose(map_id_, robot_pose, odom_pose);
    }
    catch(tf::LookupException& ex) {
        ROS_ERROR("No Transform available Error: %s\n", ex.what());
        return false;
    }
    catch(tf::ConnectivityException& ex) {
        ROS_ERROR("Connectivity Error: %s\n", ex.what());
        return false;
    }
    catch(tf::ExtrapolationException& ex) {
        ROS_ERROR("Extrapolation Error: %s\n", ex.what());
        return false;
    }
    return true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "speed_advisor");
    SpeedAdvisor sa;
    ros::spin();
    return 0;
}