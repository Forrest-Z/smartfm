#include "speed_advisor.h"


//todo: comment and test the automode behavior.

SpeedSettings::SpeedSettings()
: automode_(true), emergency_(false), curr_vel_(0), odo_vel_(0)
{
    automode_sub_ = nh_.subscribe("/button_state_automode", 1, &SpeedSettings::automode_callback, this);
    emergency_sub_ = nh_.subscribe("/button_state_emergency", 1, &SpeedSettings::emergency_callback, this);
    odo_sub_ = nh_.subscribe("/odom", 1, &SpeedSettings::odom_callback, this);

    switching_gear_ = false;
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

    if(switching_gear_ > 0)
    {
        if(fabs(odo_vel_) < 0.05)
            switching_gear_++;
        else
            switching_gear_ = 1;

        if(switching_gear_ > 10)
        {
            switching_gear_ = 0;
            ROS_WARN("Switched gear");
        }
    }
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

    ROS_DEBUG_NAMED("ctrl_loop", "Adding speed attr: \"%s\", target=%.1f, curr_vel=%g, final=%g",
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
    else
    {
        attr.description_ = SpeedAttribute::manual_mode;
        attr.description_str_ = "manual mode";
        attr.final_speed_ = 0;
        attr.target_speed_ = 0;
    }
    ROS_DEBUG_NAMED("ctrl_loop", "Selecting profile: \"%s\", target=%.1f, curr_vel=%g, final=%g",
            attr.description_str_.c_str(), attr.target_speed_, curr_vel_, attr.final_speed_);
    mutex_.unlock();
    return attr;
}



SpeedAdvisor::SpeedAdvisor()
{
    ros::NodeHandle nh("~");
    nh.param("max_speed", high_speed_, 2.0);
    nh.param("bwd_speed", bwd_speed_, 0.5);
    nh.param("min_speed", min_speed_, 0.15);
    // speed_controller doesn't use accel pedal when below 0.1, so put min_speed_ > 0.1
    nh.param("acc", acc_, 0.5);
    nh.param("max_dec", max_dec_, 2.0);
    nh.param("norm_dec", norm_dec_, 0.7);
    nh.param("robot_frame_id", base_link_, string("base_link"));
    nh.param("map_frame_id", map_id_, string("map"));
    nh.param("dec_ints", dec_ints_, 0.5);
    nh.param("dec_station", dec_station_, 0.5);
    nh.param("stop_ints_dist", stop_ints_dist_, 4.0);
    nh.param("ppc_stop_dist", ppc_stop_dist_, 3.0);

    nh.param("frequency", frequency_,20.0);
    nh.param("tolerance", tolerance_, 0.5);
    nh.param("emergency_zone", e_zone_, 2.0);
    nh.param("slow_zone", slow_zone_, 10.0);
    nh.param("slow_speed", slow_speed_, 1.5);
    nh.param("baselink_carfront_length", baselink_carfront_length_, 2.0);
    nh.param("kinematic_acceleration", kinematics_acc_, true);

    speed_settings_.max_neg_speed_ = -max_dec_ / frequency_;
    speed_settings_.pos_speed_ = acc_ / frequency_;
    speed_settings_.norm_neg_speed_ = -norm_dec_ / frequency_;

    nh_.param("use_sim_time", use_sim_time_, false);
    ROS_DEBUG_STREAM("Simulated time is "<<use_sim_time_);
    if(use_sim_time_)
    {
        ppc_stop_dist_ = 1.0; // to make the vehicle stop near the end in simulation
    }
    final_stop_dist_ = pow(min_speed_, 2) / (2.0 * norm_dec_);

    signal_type_ = -1;

    bwd_drive_pub_ = nh_.advertise<std_msgs::Bool>("direction_ctrl", 1);
    recommend_speed_pub_= nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1);
    speed_contribute_pub_ = nh_.advertise<pnc_msgs::speed_contribute>("speed_status",1);
    left_blinker_pub_ = nh_.advertise<std_msgs::Bool>("left_blinker",1);
    right_blinker_pub_ = nh_.advertise<std_msgs::Bool>("right_blinker",1);

    move_base_speed_ = nh_.subscribe("/move_status", 1, &SpeedAdvisor::moveSpeedCallback, this);
    rrts_sub_ = nh_.subscribe("rrts_status", 1, &SpeedAdvisor::rrts_callback, this);
    slowzone_sub_ = nh_.subscribe("slowZone", 1, &SpeedAdvisor::slowZoneCallback, this);
    bwd_drive_ = false;

    timer_ = nh_.createTimer(ros::Duration(1.0/frequency_),
                                &SpeedAdvisor::ControlLoop, this);


    // Init named loggers so that they the log level can be set in rxconsole
    ROS_DEBUG_NAMED("ctrl_loop","");
    ROS_DEBUG_NAMED("intersection","");
    ROS_DEBUG_NAMED("laser_area","");
    ROS_DEBUG_NAMED("infrastructure","");
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
    ROS_DEBUG_NAMED("ctrl_loop", "Control Loop: t=%f", ros::Time::now().toSec());

    // Add a normal speed profile
    if(move_status_.backward_driving)
        speed_settings_.add( "bwd driving", SpeedAttribute::bwd_driving, bwd_speed_);
    else // backward normal speed is different
        speed_settings_.add( "norm zone", SpeedAttribute::norm_zone, high_speed_);

    // switching gear fwd->bwd or bwd->fwd
    if(speed_settings_.is_gear_switching())
        speed_settings_.add_brake("switching gear", SpeedAttribute::switching_gear);

    // RRTS says robot is in collision --> brake
    if(rrts_status_.robot_in_collision)
        speed_settings_.add_brake("rrts says robot is in collision", SpeedAttribute::robot_collision);

    // Problem with the navigation node (move_base) --> brake
    if( fabs((ros::Time::now()-last_update_).toSec()) > tolerance_ )
        speed_settings_.add_brake("no response from move_base", SpeedAttribute::no_response);

    // PPC couldn't find a path to follow, it may due to localisation error or illegal path received
    if( !move_status_.path_exist )
        speed_settings_.add_brake("move_base reports no path", SpeedAttribute::path_noexist);

    // This may be an abuse for emergency, but to enable max deceleration
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

        float d = int_h_.dist_to_int() - stop_ints_dist_;
        sc.int_rec = d<=0 ? 0 : sqrt(2 * dec_ints_ * (d - stop_ints_dist_));

        ROS_DEBUG_STREAM("Intersection not clear. dist=" << int_h_.dist_to_int()
                <<". Recommended speed: " <<sc.int_rec);

        speed_settings_.add("Intersection", SpeedAttribute::intersection, sc.int_rec);
    }

    //
    // stopping at the end
    //
    if(move_status_.dist_to_goal > ppc_stop_dist_)
    {
        sc.goal_rec = sqrt(2*dec_station_*(move_status_.dist_to_goal-ppc_stop_dist_));
        sc.goal_rec = fmutil::max(sc.goal_rec, min_speed_);
    }
    else if(move_status_.dist_to_goal > final_stop_dist_)
    {
        if(move_status_.want_exact_stop)
            sc.goal_rec = min_speed_; // go slowly if we are short of some dist
        else
            sc.goal_rec = 0.0; // stopping early is fine (this is for the final segment)
    }
    else
    {
        if(move_status_.want_exact_stop)
            sc.goal_rec = sqrt(2*norm_dec_*move_status_.dist_to_goal);
        else
            sc.goal_rec = 0.0;
    }
    speed_settings_.add("Goal", SpeedAttribute::goal, sc.goal_rec);


    //
    // Decision making: go for the lowest speed
    //
    geometry_msgs::Twist move_speed;
    SpeedAttribute sattr = speed_settings_.select_min_speed();

    move_speed.linear.x = sattr.final_speed_;
    if(use_sim_time_) // stage model doesn't match well with ours, little tuning
        move_speed.angular.z = 0.8*move_status_.steer_angle;
    else
        move_speed.angular.z = move_status_.steer_angle;

    // Since the speed controller cannot track very small speed, so if we are
    // not stopped yet (i.e. final_speed>0) and if the target speed is small(not zero!),
    // impose a minimal velocity.
    static const double minimal_vel = min_speed_;
    if( sattr.final_speed_>0.0 && sattr.final_speed_<minimal_vel &&
        sattr.target_speed_>0.0 && sattr.target_speed_<minimal_vel )
        move_speed.linear.x = minimal_vel;

    //it was found that, in simulation with stage, although commanded to
    //travel at 2 m/s, it is actually travelling at 3.33x faster,
    //compensation is needed
//    if( use_sim_time_ ) move_speed.linear.x *= 0.3;

    if(move_status_.backward_driving)
        move_speed.linear.x = -move_speed.linear.x;
    recommend_speed_pub_.publish(move_speed);

    std_msgs::Bool backward_driving;
    backward_driving.data = move_status_.backward_driving;
    bwd_drive_pub_.publish(backward_driving);

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

    if(bwd_drive_ != status.backward_driving)
    {
        speed_settings_.switch_gear();
        ROS_WARN("gear will be switched to %s", status.backward_driving == true ? "bwd" : "fwd");
    }
    bwd_drive_ = status.backward_driving;
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

void SpeedAdvisor::rrts_callback(const rrts::rrts_status &msg)
{
    rrts_status_ = msg;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "speed_advisor");
    SpeedAdvisor speed_advisor;
    ros::spin();
    return 0;
}
