#include "speed_advisor.h"

SpeedAdvisor::SpeedAdvisor()
{
    /* the speed_advisor receive message from move_base package and perform neccessary speed profile generation
     * Currently only trapezoidal profile is implemented, and the speed regulation largely seperated into 2 zone:
     * slow-down and stopping zone.
     *
     */
    recommend_speed_= n.advertise<geometry_msgs::Twist>("cmd_vel", 1);
    speed_contribute_ = n.advertise<pnc_msgs::speed_contribute>("speed_status",1);
    left_blinker_pub_ = n.advertise<std_msgs::Bool>("left_blinker",1);
    right_blinker_pub_ = n.advertise<std_msgs::Bool>("right_blinker",1);
    move_base_speed_=n.subscribe("/move_status",1, &SpeedAdvisor::moveSpeedCallback, this);
    slowzone_sub_=n.subscribe("slowZone",1,&SpeedAdvisor::slowZoneCallback,this);
    ros::NodeHandle nh("~");
    nh.param("max_speed", high_speed_, 2.0);
    nh.param("acc", acc_, 0.5);
    nh.param("max_dec", max_dec_, 2.0);
    nh.param("norm_dec", norm_dec_, 0.7);
    nh.param("robot_frame_id", base_link_, string("base_link"));
    nh.param("map_frame_id", map_id_, string("map"));
    nh.param("dec_ints", dec_ints_, 0.5);
    nh.param("dec_station", dec_station_, 0.5);
    nh.param("ppc_stop_dist", ppc_stop_dist_,5.0); //the stopping distance from ppc.
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
    n.param("use_sim_time", use_sim_time_, false);
    ROS_DEBUG_STREAM("Simulated time is "<<use_sim_time_);
    junction_stop_ = false;
    through_ints_ = true;
    ros::Timer timer = n.createTimer(ros::Duration(1.0/frequency_),&SpeedAdvisor::ControlLoop,this);
    speed_now_ = 0;
    signal_type_ = -1;
    //visualization
    interactive_markers::InteractiveMarkerServer server("intersection");
    marker_server_ = &server;
    ros::spin();
}

void SpeedAdvisor::slowZoneCallback(geometry_msgs::PoseArrayConstPtr slowzones)
{
    slowZone_.poses=slowzones->poses;
}

void SpeedAdvisor::ControlLoop(const ros::TimerEvent& event)
{
    //todo: add user interface

    //precompute the amount of change in speed
    double max_neg_speed = -max_dec_ / frequency_;
    double pos_speed = acc_ / frequency_;
    double norm_neg_speed = -norm_dec_ / frequency_;
    vector<double> proposed_speed;
    /*Speed calculation*/

    //different speed is compared with current recommended speed and the lowest
    //speed is selected

    double recommended_speed=0;
    vector<string> speed_contributor;

    SpeedSettings speed_settings;
    SpeedAttribute speed_att(speed_now_);

    if((ros::Time::now()-last_update_).toSec()>tolerance_)
    {
        speed_att.final_speed("no response from move_base",speed_att.no_response,pos_speed, max_neg_speed, 0);
        speed_settings.push_back(speed_att);
    }

    if(!move_status_.acc_dec)
    {
        speed_att.final_speed("deceleration command from move_base",speed_att.movebase_dec,pos_speed, max_neg_speed, 0);
        speed_settings.push_back(speed_att);
    }

    speed_att.final_speed("norm zone",speed_att.norm_zone,pos_speed, norm_neg_speed, high_speed_);
    speed_settings.push_back(speed_att);

    for(unsigned int i=0; i<slowZone_.poses.size();i++)
    {
        tf::Stamped<tf::Pose> robot_pose;
        getRobotGlobalPose(robot_pose);

        double robot_x = robot_pose.getOrigin().x() + baselink_carfront_length_ * cos(robot_pose.getRotation().getAngle());
        double robot_y = robot_pose.getOrigin().y() + baselink_carfront_length_ * sin(robot_pose.getRotation().getAngle());
        double dist=fmutil::distance(robot_x, robot_y, slowZone_.poses[i].position.x, slowZone_.poses[i].position.y);
        if(dist<slowZone_.poses[i].position.z)
        {
            stringstream ss;
            ss<<"Slow zone: "<<i;
            speed_att.final_speed(ss.str(),speed_att.slow_zone,pos_speed, norm_neg_speed, slow_speed_);
            speed_settings.push_back(speed_att);
        }
    }
    pnc_msgs::speed_contribute sc;
    //obstacles
        //There are 2 kind of situations possible when dealing with obstacles
    double obs_dist = move_status_.obstacles_dist;
    double deceleration_required = pow(speed_now_,2)/(2*(obs_dist-e_zone_));
    sc.dec_req = deceleration_required;
    if(move_status_.emergency)
    {
        speed_att.final_speed("Emergency!",speed_att.emergency,pos_speed, max_neg_speed, 0);
        speed_settings.push_back(speed_att);
    }
    if(deceleration_required>=norm_dec_)
    {
        if(deceleration_required>max_dec_)
        {
            speed_att.final_speed("Obs 1st: Max brake",speed_att.max_brake,pos_speed, max_neg_speed, 0);
            speed_settings.push_back(speed_att);
        }
        else
        {
            speed_att.final_speed("Obs 1st: really need to brake",speed_att.need_brake,pos_speed, -deceleration_required/frequency_, 0);
            speed_settings.push_back(speed_att);
        }
    }
    if(obs_dist<=e_zone_)
    {
        speed_att.final_speed("Obs Danger!",speed_att.e_zone,pos_speed, max_neg_speed, 0);
        speed_settings.push_back(speed_att);
    }

    if(obs_dist<slow_zone_)
    {
        speed_att.final_speed("Obs 2nd: Within the safety zone",speed_att.warn_brake,pos_speed, norm_neg_speed, slow_speed_);
        speed_settings.push_back(speed_att);
    }

    double net_dist;
    //only change the dist_to_ints to the next station when through_ints has set to true
    if(!through_ints_)//move_status_.dist_to_ints>0 &&
    {
        net_dist = move_status_.dist_to_ints-ppc_stop_dist_;
        sc.int_rec=recommended_speed = sqrt(2*dec_ints_*net_dist);
        if(net_dist<0 || last_ints_dist_ < ppc_stop_dist_) recommended_speed = 0;
        speed_att.final_speed("Intersection",speed_att.intersection,pos_speed, norm_neg_speed, recommended_speed);
        speed_settings.push_back(speed_att);
    }
    else last_ints_dist_ = move_status_.dist_to_ints;

    if(move_status_.dist_to_ints - last_ints_dist_ < ppc_stop_dist_) last_ints_dist_ = move_status_.dist_to_ints;

    //reset the intersection flag to allow vehicle to stop for next intersection
    //this assumed that the distance between intersections are at least 10 m.
    if(move_status_.dist_to_ints>10) through_ints_ = false;
    if(move_status_.dist_to_goal<stationspeed_dist_)
    {
        speed_att.final_speed("Approaching goal",speed_att.app_goal,pos_speed, norm_neg_speed, slow_speed_);
        speed_settings.push_back(speed_att);
    }

    net_dist = move_status_.dist_to_goal-ppc_stop_dist_;
    sc.goal_rec=recommended_speed = sqrt(2*dec_station_*net_dist);
    if(net_dist<0) recommended_speed = 0;
    speed_att.final_speed("Goal",speed_att.goal,pos_speed, norm_neg_speed, recommended_speed);
    speed_settings.push_back(speed_att);

    //find out whats the min speed is and publish that speed
    SpeedAttribute* sa = speed_settings.find_min_speed();
    speed_now_ = sa->final_speed_;// speed_settings.min_speed;
    sc.element = sa->element_; //speed_settings.min_element;
    sc.description = sa->description_;//speed_settings.description;
    geometry_msgs::Twist move_speed;
    move_speed.angular.z = move_status_.steer_angle;
    double speed_compensation = 1.0;
    //it was found that the although commanded to travel 2 m/s, it is actually travelling at 3.33x faster in simulation with stage, compensation is needed
    if(use_sim_time_) speed_compensation=0.3;
    move_speed.linear.x = speed_now_*speed_compensation;
    recommend_speed_.publish(move_speed);
    sc.speed_now = speed_now_;
    sc.dist_goal = move_status_.dist_to_goal;
    //determine if goal has reached
    if(element_now_!=sc.element)
    {
        element_pre_ = element_now_;
        element_now_ = sc.element;
    }
    if((sc.element == speed_att.goal || element_pre_ == speed_att.goal)
            && sc.speed_now == 0 && sc.dist_goal < 7) sc.goal = true;

    speed_contribute_.publish(sc);
}

void SpeedAdvisor::moveSpeedCallback(pnc_msgs::move_status status)
{
    last_update_ = ros::Time::now();
    move_status_ = status;

    //only change the button when the speed element is not from intersection
    SpeedAttribute sa;
    if(element_now_==sa.norm_zone || element_now_ == sa.slow_zone)
    {
        if(int_point_.x!=status.int_point.x &&int_point_.y!=status.int_point.y)
        {//interactive_markers::InteractiveMarkerServer &server, geometry_msgs::Vector3 scale, std_msgs::ColorRGBA color, geometry_msgs::Pose pose, std::string name, std::string description)
            int_point_ = status.int_point;
            geometry_msgs::Vector3 size; size.x=1.0; size.y=1.0;
            std_msgs::ColorRGBA color; color.a=1;color.r=0.5;
            geometry_msgs::Pose pose; pose.position =int_point_ ;pose.orientation.w=1;
            add_button_marker(*marker_server_,size, color, pose, "Int", "Intersection" );
        }
        marker_server_->applyChanges();
    }

    if(status.dist_to_sig<ppc_stop_dist_ && status.dist_to_sig>0)
    {
        if(status.sig_type!=signal_type_)
        {
            signal_type_ = status.sig_type;
            bool left_blinker(false), right_blinker(false);
            switch (signal_type_)
            {
                /*
                 * 0: left_signal
                 * 1: right_signal
                 * 2: off_signals
                 */
                case 0:
                    left_blinker = true;
                    right_blinker = false;
                    break;
                case 1:
                    left_blinker = false;
                    right_blinker = true;
                    break;
                case 2:
                    left_blinker = right_blinker = false;
                    break;
            }
            std_msgs::Bool blinker;
            blinker.data = left_blinker;
            left_blinker_pub_.publish(blinker);
            blinker.data = right_blinker;
            right_blinker_pub_.publish(blinker);
        }
    }
}

bool SpeedAdvisor::getRobotGlobalPose(tf::Stamped<tf::Pose>& odom_pose) const
{
    odom_pose.setIdentity();
    tf::Stamped<tf::Pose> robot_pose;
    robot_pose.setIdentity();
    robot_pose.frame_id_ =  base_link_;
    robot_pose.stamp_ = ros::Time();
    ros::Time current_time = ros::Time::now(); // save time for checking tf delay later

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

void SpeedAdvisor::add_button_marker(interactive_markers::InteractiveMarkerServer &server, geometry_msgs::Vector3 scale, std_msgs::ColorRGBA color, geometry_msgs::Pose pose, std::string name, std::string description)
{
        // create an interactive marker for our server
        visualization_msgs::InteractiveMarker int_marker;
        int_marker.header.frame_id = "/map";
        int_marker.name = name;
        int_marker.description = description;

        // create a grey box marker
        visualization_msgs::Marker box_marker;
        box_marker.type = visualization_msgs::Marker::CUBE;
        box_marker.scale = scale;
        box_marker.color = color;
        int_marker.pose = pose;
        // create a non-interactive control which contains the box
        visualization_msgs::InteractiveMarkerControl box_control;
        box_control.markers.push_back( box_marker );
        box_control.always_visible = true;
        box_control.interaction_mode = InteractiveMarkerControl::BUTTON;

        // add the control to the interactive marker
        int_marker.controls.push_back( box_control );
        server.insert(int_marker,boost::bind(&SpeedAdvisor::processFeedback, this, _1));
}

void SpeedAdvisor::processFeedback(const InteractiveMarkerFeedbackConstPtr &feedback )
{
        if(feedback->event_type==InteractiveMarkerFeedback::MOUSE_UP)
        {
               through_ints_ = true;
        }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "speed_advisor");
    SpeedAdvisor sa;
    ros::spin();
    return 0;
}
