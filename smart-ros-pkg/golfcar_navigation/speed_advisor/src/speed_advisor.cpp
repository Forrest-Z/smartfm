#include "speed_advisor.h"

SpeedSettings::SpeedSettings()
: automode_(true), emergency_(false), curr_vel_(0), odo_vel_(0)
{
    automode_sub_ = nh_.subscribe("/button_state_automode", 1, &SpeedSettings::automode_callback, this);
    emergency_sub_ = nh_.subscribe("/button_state_emergency", 1, &SpeedSettings::emergency_callback, this);
    odo_sub_ = nh_.subscribe("/odom", 1, &SpeedSettings::odom_callback, this);
}

void SpeedSettings::automode_callback(const std_msgs::Bool & msg)
{
    button_state(msg.data, emergency_);
}

void SpeedSettings::emergency_callback(const std_msgs::Bool & msg)
{
    button_state(automode_, msg.data);
}

void SpeedSettings::button_state(bool automode, bool emergency)
{
    mutex_.lock();
    if( (automode_==false && automode && !emergency) ||
            (emergency_ && emergency==false && automode) )
    {
        curr_vel_ = odo_vel_;
    }
    automode_ = automode;
    emergency_ = emergency;
    mutex_.unlock();
}

void SpeedSettings::odom_callback(const nav_msgs::Odometry & msg)
{
    odo_vel_ = msg.twist.twist.linear.x;
}

void SpeedSettings::add(const string & description_str,
        SpeedAttribute::Description description,
        double target_speed, double speed_inc, double speed_dec)
{
    mutex_.lock();
    SpeedAttribute attr;
    attr.description_str_ = description_str;
    attr.description_ = description;
    attr.target_speed_ = target_speed;

    if( curr_vel_ > target_speed )
        attr.final_speed_ = fmutil::max(curr_vel_ + speed_dec, target_speed);
    else if( curr_vel_ < target_speed )
        attr.final_speed_ = fmutil::min(curr_vel_ + speed_inc, target_speed);
    else
        attr.final_speed_ = curr_vel_;

    ROS_DEBUG("Adding speed attr: \"%s\", target=%.1f, curr_vel=%g, final=%g",
            description_str.c_str(), target_speed, curr_vel_, attr.final_speed_);
    attrs_.push_back(attr);
    mutex_.unlock();
}

SpeedAttribute SpeedSettings::select_min_speed()
{
    assert( ! attrs_.empty() );
    mutex_.lock();
    std::vector<SpeedAttribute>::iterator min, it;
    for( min=attrs_.begin(), it=attrs_.begin()+1; it!=attrs_.end(); ++it)
        if( it->final_speed_ < min->final_speed_ )
            min = it;
    SpeedAttribute attr = *min;
    attrs_.clear();
    if( automode_ && !emergency_ )
        curr_vel_ = attr.final_speed_;
    ROS_DEBUG("Selecting profile: \"%s\", target=%.1f, curr_vel=%g, final=%g",
            attr.description_str_.c_str(), attr.target_speed_, curr_vel_, attr.final_speed_);
    mutex_.unlock();
    return attr;
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
    nh.param("enterstation_speed",enterstation_speed_, slow_speed_); //by default, enter station speed is the same as slow speed
    nh.param("stationspeed_dist", stationspeed_dist_, 20.0);
    nh.param("kinematic_acceleration", kinematics_acc_, true);

    speed_settings_.max_neg_speed_ = -max_dec_ / frequency_;
    speed_settings_.pos_speed_ = acc_ / frequency_;
    speed_settings_.norm_neg_speed_ = -norm_dec_ / frequency_;

    nh_.param("use_sim_time", use_sim_time_, false);
    ROS_DEBUG_STREAM("Simulated time is "<<use_sim_time_);

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

/** Checks a list of conditions and generate different speed profiles for each.
 * Finally choose the slowest profile.
 * TODO add user interface
 */
void SpeedAdvisor::ControlLoop(const ros::TimerEvent& event)
{
    ROS_DEBUG("Control Loop: t=%f", ros::Time::now().toSec());

    // Add a normal speed profile
    speed_settings_.add( "norm zone", SpeedAttribute::norm_zone, high_speed_);

    // Problem with the navigation node (move_base) --> brake
    if( fabs((ros::Time::now()-last_update_).toSec()) > tolerance_ )
        speed_settings_.add_brake("no response from move_base", SpeedAttribute::no_response);

    // PPC couldn't find a path to follow, it may due to localisation error or illegal path received
    if( ! move_status_.path_exist )
        speed_settings_.add_brake("move_base reports no path", SpeedAttribute::path_exist);

    if(move_status_.emergency)
        speed_settings_.add_brake("Emergency!", SpeedAttribute::emergency);

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
            speed_settings_.add( ss.str(), SpeedAttribute::slow_zone, slow_speed_);
        }
    }

    // This message will be published later
    pnc_msgs::speed_contribute sc;

    //
    // Dealing with obstacles
    //
    double obs_dist = move_status_.obstacles_dist;

    // Obstacle is very close (emergency zone)
    if(obs_dist<=e_zone_)
        speed_settings_.add_brake("Obs within emergency zone!", SpeedAttribute::e_zone);

    // Obstacle is not so close (slow zone)
    if( obs_dist < slow_zone_ )
        speed_settings_.add( "Obs within safety zone", SpeedAttribute::warn_brake, slow_speed_);

    // Compute some deceleration value based on the distance to the
    // nearest obstacle

    // Although this is covered by the e_zone and slow_zone,
    // the algorithm below take into account the real kinematics of
    // the vehicle that should ensure that no collision would occur even
    // when slow_zone/e_zone specified is not enough to prevent collision
    // from occurring. The following algorithm also ensure smooth
    // transition of stopping behaviour
    sc.dec_req = pow(speed_settings_.curr_vel(),2)/(2*(obs_dist-e_zone_));
    if(kinematics_acc_)
    {
        if( sc.dec_req > max_dec_ )
            speed_settings_.add_brake("Obs 1st: Max brake", SpeedAttribute::max_brake);
        else if( sc.dec_req >= norm_dec_ )
            speed_settings_.add( "Obs 1st: really need to brake",
                    SpeedAttribute::need_brake,
                    0, speed_settings_.pos_speed_, -sc.dec_req/frequency_);
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
        // inaccurate, causes overshoot due to constraint given by deceleration
        // and the recommended speed is reported as the next point
        // need to add to check that the speed is actually zero
        // there are some fundamental problem here:
        // --------- The inaccurate calculation of int dist ------------------

        float d = int_h_.dist_to_int() - ppc_stop_dist_;
        sc.int_rec = d<=0 ? 0 : sqrt(2 * dec_ints_ * d);

        ROS_DEBUG_STREAM("Intersection not clear. dist=" << int_h_.dist_to_int()
                <<". Recommended speed: " <<sc.int_rec);

        speed_settings_.add("Intersection", SpeedAttribute::intersection, sc.int_rec);
    }


    //
    // Approaching the goal:
    // - when closer than stationspeed_dist_, slow down to slow_speed_
    // - compute a deceleration profile that will make the car stop at the goal.
    //

    // Getting near the goal: slowing down to slow_speed_
    if( move_status_.dist_to_goal < stationspeed_dist_ )
        speed_settings_.add("Approaching goal", SpeedAttribute::app_goal, slow_speed_);

    // Smooth deceleration profile that will make the car stop at the goal
    double d = move_status_.dist_to_goal - ppc_stop_dist_;
    sc.goal_rec = d<0 ? 0 : sqrt(2*dec_station_*d);
    speed_settings_.add("Goal", SpeedAttribute::goal, sc.goal_rec);


    //
    // Decision making: go for the lowest speed
    //
    geometry_msgs::Twist move_speed;
    SpeedAttribute sattr = speed_settings_.select_min_speed();

    move_speed.linear.x = sattr.final_speed_;

    // steer is not set here using "cmd_vel"
    // golfcar_pp set it using "cmd_steer"
    move_speed.angular.z = 0.0;

    // Since the speed controller cannot track very small speed, so if we are
    // not stopped yet (i.e. final_speed>0) and if the target speed is small,
    // impose a minimal velocity.
    static const double minimal_vel = 0.1;
    if( sattr.final_speed_!=0 && sattr.final_speed_<minimal_vel && sattr.target_speed_<minimal_vel )
        move_speed.linear.x = minimal_vel;

    //it was found that, in simulation with stage, although commanded to
    //travel at 2 m/s, it is actually travelling at 3.33x faster,
    //compensation is needed
//    if( use_sim_time_ ) move_speed.linear.x *= 0.3;

    recommend_speed_pub_.publish(move_speed);


    sc.element = sattr.description_;
    sc.description = sattr.description_str_;
    sc.speed_now = move_speed.linear.x;
    sc.dist_goal = move_status_.dist_to_goal;
    //determine if goal has reached
    if( element_now_!=sc.element )
    {
        element_pre_ = element_now_;
        element_now_ = (SpeedAttribute::Description) sc.element;
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

    int_h_.update(status);

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
    SpeedAdvisor speed_advisor;
    ros::spin();
    return 0;
}
