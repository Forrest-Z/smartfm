#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Point.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>

#include <string>
#include <cmath>

#include <pnc_msgs/move_status.h>
#include <fmutil/fm_math.h>

using namespace std;


class PurePursuit
{
public:
    PurePursuit();

private:
    ros::Subscriber traj_sub_;
    ros::Publisher cmd_steer_pub_;
    ros::Publisher move_status_pub_;
    ros::Subscriber cmd_vel_sub_;
    ros::Publisher cmd_vel_pub_; // only in case of (emergency) && (speed_advisor died)
    ros::Publisher pp_segments_rviz_pub_; // to visualize the circular segments by pp
    ros::Publisher ref_segments_rviz_pub_; // to visualize the reference segments for pp
    ros::Timer timer_;

    string robot_frame_id_, coord_frame_id_;
    double max_timer_;
    double max_timer_complaint_;
    double max_silence_cmd_vel_;
    double neglect_distance_;
    double look_ahead_;
    double max_steering_;
    double car_length_;

    tf::TransformListener tf_;
    nav_msgs::Path trajectory_;
    nav_msgs::Path pp_segments_;
    nav_msgs::Path ref_segments_;
    int last_segment_;
    bool last_bwd_;
    ros::Time last_traj_time_;
    ros::Time last_timer_complaint_;
    ros::Time last_segment_complaint_;
    ros::Time last_time_cmd_vel_;

    void trajCallBack(const nav_msgs::Path::ConstPtr &traj);
    void cmdVelCallBack(const geometry_msgs::Twist &cmd_vel);
    void controlLoop(const ros::TimerEvent &e);

    void set_pp_segments_rviz(double cur_x, double cur_y, double cur_yaw,
                              double gamma, bool backward);
    void set_ref_segments_rviz(int segment);
    bool is_segment_backward(int segment);
    bool need_gear_switch_later(int segment);
    bool getRobotPose(tf::Stamped<tf::Pose> &odom_pose) const;
    double get_distance(double x1, double y1, double x2, double y2);
    bool get_center(double tar_x, double tar_y,
                    double ori_x, double ori_y, double inv_R,
                    double center[2]);
    double get_inv_R(double u);
    bool btwn_points(double tar_x, double tar_y,
                     double ori_x, double ori_y,
                     double inv_R, double x, double y);
    void get_projection(double tar_x, double tar_y,
                        double ori_x, double ori_y,
                        double inv_R, double cur_x, double cur_y,
                        double proj[2]);
    int get_segment(double cur_x, double cur_y);
    int find_lookahead_segment(int segment, double cur_x, double cur_y, double L);
    double get_dist_to_go(int segment, double cur_x, double cur_y);
    double get_steering(int segment, double cur_x, double cur_y, double cur_yaw, bool &emergency);

    bool intersection_circle_line(double tar_x, double tar_y,
                                  double ori_x, double ori_y,
                                  double cen_x, double cen_y, double r);
    bool intersection_circle_arc(double tar_x, double tar_y,
                                 double ori_x, double ori_y,
                                 double cen1_x, double cen1_y, double r1,
                                 double cen2_x, double cen2_y, double r2);
};



PurePursuit::PurePursuit()
{
    ros::NodeHandle n;
    traj_sub_ = n.subscribe("pnc_trajectory", 100, &PurePursuit::trajCallBack, this);
    pp_segments_rviz_pub_ = n.advertise<nav_msgs::Path>("pp_segments",1);
    ref_segments_rviz_pub_ = n.advertise<nav_msgs::Path>("ref_segments",1);
    cmd_steer_pub_ = n.advertise<geometry_msgs::Twist>("cmd_steer",1);
    cmd_vel_pub_ = n.advertise<geometry_msgs::Twist>("cmd_vel",1); // only in case of (emergency) && (speed_advisor died)
    move_status_pub_ = n.advertise<pnc_msgs::move_status>("move_status", 1);
    cmd_vel_sub_ = n.subscribe("cmd_vel", 100, &PurePursuit::cmdVelCallBack, this);
    timer_ = n.createTimer(ros::Duration(0.05), &PurePursuit::controlLoop, this);

    ros::NodeHandle private_nh("~");
    if(!private_nh.getParam("robot_frame_id", robot_frame_id_)) robot_frame_id_ = "/base_link";
    if(!private_nh.getParam("coord_frame_id", coord_frame_id_)) coord_frame_id_ = "/odom";
    if(!private_nh.getParam("max_timer", max_timer_)) max_timer_ = 2.0;
    if(!private_nh.getParam("max_timer_complaint", max_timer_complaint_)) max_timer_complaint_ = 5.0;
    if(!private_nh.getParam("max_silence_cmd_vel", max_silence_cmd_vel_)) max_silence_cmd_vel_ = 0.3;
    if(!private_nh.getParam("neglect_distance", neglect_distance_)) neglect_distance_ = 0.001;
    if(!private_nh.getParam("look_ahead", look_ahead_)) look_ahead_ = 3.0;
    if(!private_nh.getParam("max_steering", max_steering_)) max_steering_ = 0.65;
    if(!private_nh.getParam("car_length", car_length_)) car_length_ = 1.632;

    last_segment_ = 0;
    last_bwd_ = false;
    last_traj_time_ = ros::Time::now();
    last_timer_complaint_ = ros::Time::now();
    last_segment_complaint_ = ros::Time::now();
    last_time_cmd_vel_ = ros::Time::now();

    std::cout<<"robot_frame_id: "<<robot_frame_id_<<"\n";
    std::cout<<"coord_frame_id: "<<coord_frame_id_<<"\n";
    std::cout<<"max_timer: "<<max_timer_<<"\n";
    std::cout<<"max_timer_complaint: "<<max_timer_complaint_<<"\n";
    std::cout<<"max_silence_cmd_vel: "<<max_silence_cmd_vel_<<"\n";
    std::cout<<"neglect_distance: "<<neglect_distance_<<"\n";
    std::cout<<"look_ahead: "<<look_ahead_<<"\n";
    std::cout<<"max_steering: "<<max_steering_<<"\n";
    std::cout<<"car_length: "<<car_length_<<"\n";
}

void PurePursuit::set_pp_segments_rviz(double cur_x, double cur_y,
                                       double cur_yaw, double gamma, bool backward)
{
    ros::Time time_now = ros::Time::now();
    pp_segments_.header.stamp = time_now;
    pp_segments_.header.frame_id = coord_frame_id_;

    if(gamma == 0.0)
        return;

    if(backward)
        gamma = -gamma;
    double R_abs = fabs(1.0/gamma);

    double center_x, center_y;
    if(gamma > 0.0)
    {
        center_x = cur_x + R_abs*cos(cur_yaw+M_PI_2);
        center_y = cur_y + R_abs*sin(cur_yaw+M_PI_2);
    }
    else
    {
        center_x = cur_x + R_abs*cos(cur_yaw-M_PI_2);
        center_y = cur_y + R_abs*sin(cur_yaw-M_PI_2);
    }
    double arg_from = atan2(cur_y - center_y, cur_x - center_x);
    double arg_diff = 2*asin(look_ahead_*gamma/2);
    if(backward)
        arg_diff = -arg_diff;

    geometry_msgs::PoseStamped point;
    point.header.stamp = time_now;
    point.header.frame_id = coord_frame_id_;
    point.pose.position.z = 0.0;

    int arg_step_n = 10;
    double arg_step = arg_diff/arg_step_n;
    for(int i = 0; i <= arg_step_n; i++)
    {
        point.pose.position.x = center_x + R_abs*cos(arg_from + i*arg_step);
        point.pose.position.y = center_y + R_abs*sin(arg_from + i*arg_step);

        pp_segments_.poses.push_back(point);
    }
}

void PurePursuit::set_ref_segments_rviz(int segment)
{
    ros::Time time_now = ros::Time::now();
    ref_segments_.header.stamp = time_now;
    ref_segments_.header.frame_id = coord_frame_id_;

    if(segment < 0)
        return;
    if((int) trajectory_.poses.size() < 2)
        return;
    if((int) trajectory_.poses.size() < segment + 2)
        return;

    double tar_x, tar_y, ori_x, ori_y, inv_R;
    double center[2];

    tar_x = trajectory_.poses[segment+1].pose.position.x;
    tar_y = trajectory_.poses[segment+1].pose.position.y;
    ori_x = trajectory_.poses[segment].pose.position.x;
    ori_y = trajectory_.poses[segment].pose.position.y;
    inv_R = get_inv_R(trajectory_.poses[segment].pose.position.z);
    if(inv_R != 0.0 && !get_center(tar_x, tar_y, ori_x, ori_y, inv_R, center))
        inv_R = 0.0;

    geometry_msgs::PoseStamped point;
    point.header.stamp = time_now;
    point.header.frame_id = coord_frame_id_;
    point.pose.position.z = 0.0;

    if(inv_R == 0.0)
    {
        double dist = get_distance(tar_x, tar_y, ori_x, ori_y);
        if(dist < neglect_distance_)
            return;

        // 1.25 is to visualize extra segments
        point.pose.position.x = ori_x + 1.25*look_ahead_*(ori_x - tar_x)/dist;
        point.pose.position.y = ori_y + 1.25*look_ahead_*(ori_y - tar_y)/dist;

        ref_segments_.poses.push_back(point);

        point.pose.position.x = tar_x + 1.25*look_ahead_*(tar_x - ori_x)/dist;
        point.pose.position.y = tar_y + 1.25*look_ahead_*(tar_y - ori_y)/dist;

        ref_segments_.poses.push_back(point);

        return;
    }

    double R_abs = fabs(1.0/inv_R);

    double arg_from = atan2(ori_y - center[1], ori_x - center[0]);
    double arg_diff = 2.5*asin(look_ahead_*inv_R/2); // 2 is exact, used 2.5 to visualize extra segments

    int arg_step_n = 13;
    double arg_step = arg_diff/arg_step_n;
    for(int i = -arg_step_n; i <= arg_step_n; i++)
    {
        point.pose.position.x = center[0] + R_abs*cos(arg_from + i*arg_step);
        point.pose.position.y = center[1] + R_abs*sin(arg_from + i*arg_step);

        ref_segments_.poses.push_back(point);
    }
}

bool PurePursuit::is_segment_backward(int segment)
{
    return trajectory_.poses[segment].pose.orientation.w < 0.0 ? 1 : 0;
}

bool PurePursuit::need_gear_switch_later(int segment)
{
    if(segment < 0)
        return false;

    bool backward_driving = is_segment_backward(segment);
    int on_segment = segment;

    while((int) trajectory_.poses.size() > on_segment+1)
    {
        bool backward = is_segment_backward(on_segment);
        if(backward != backward_driving)
            return true;
        on_segment++;
    }

    return false;
}

void PurePursuit::trajCallBack(const nav_msgs::Path::ConstPtr &traj)
{
    trajectory_.header.stamp = ros::Time::now(); // this is used to see planner's inactivity
    if(last_traj_time_ == traj->header.stamp) // this is to not re-initialize last_segment_ for same traj
        return;

    last_traj_time_ = traj->header.stamp;
    last_segment_ = 0;
    trajectory_.header.frame_id = coord_frame_id_;

    bool bwd = last_bwd_;
    int gear_change = 0;
    double temp; // for orientation.w, which is a hack to pass fwd/bwd value
    trajectory_.poses.resize(traj->poses.size());
    for(unsigned int i=0; i<traj->poses.size(); i++)
    {
        try {
            temp = traj->poses[i].pose.orientation.w;
            tf_.transformPose(coord_frame_id_, traj->poses[i], trajectory_.poses[i]);
            trajectory_.poses[i].pose.orientation.x = 0.0;
            trajectory_.poses[i].pose.orientation.y = 0.0;
            trajectory_.poses[i].pose.orientation.z = 0.0;
            trajectory_.poses[i].pose.orientation.w = temp;
            if(bwd != is_segment_backward(i))
            {
                bwd = is_segment_backward(i);
                gear_change++;
            }
        }
        catch(tf::LookupException& ex) {
            ROS_ERROR("No Transform available Error: %s", ex.what());
            return;
        }
        catch(tf::ConnectivityException& ex) {
            ROS_ERROR("Connectivity Error: %s", ex.what());
            return;
        }
        catch(tf::ExtrapolationException& ex) {
            ROS_ERROR("Extrapolation Error: %s", ex.what());
            return;
        }
    }

    ROS_INFO("got new traj, size=%d, gear_change=%d from %s",
             (int) traj->poses.size(), gear_change, last_bwd_ ? "bwd" : "fwd");

    // for debuging, given rrt plan seems very weird
    if(trajectory_.poses.size() < 1)
        return;
    double dist = 0.0;
    double x = trajectory_.poses[0].pose.position.x;
    double y = trajectory_.poses[0].pose.position.y;
    double r = trajectory_.poses[0].pose.position.z;
    bwd = is_segment_backward(0);
    ROS_INFO("from (%.2lf,%.2lf), r=%.2lf, %s", x, y, r, bwd ? "bwd" : "fwd");
    for(unsigned int i=1; i<trajectory_.poses.size(); i++)
    {
        dist += get_distance(trajectory_.poses[i-1].pose.position.x,
                             trajectory_.poses[i-1].pose.position.y,
                             trajectory_.poses[i].pose.position.x,
                             trajectory_.poses[i].pose.position.y);
        if((bwd != is_segment_backward(i)) ||
           (r != trajectory_.poses[i].pose.position.z))
        {
            x = trajectory_.poses[i].pose.position.x;
            y = trajectory_.poses[i].pose.position.y;
            bwd = is_segment_backward(i);
            r = trajectory_.poses[i].pose.position.z;
            ROS_INFO("to (%.2lf,%.2lf), dist=%.2lf", x, y, dist);
            ROS_INFO("from (%.2lf,%.2lf), r=%.2lf, %s",
                     x, y, r, bwd ? "bwd" : "fwd");
            dist = 0.0;
        }
    }
    int i = trajectory_.poses.size()-1;
    ROS_INFO("end, to (%.2lf,%.2lf), dist=%.2lf",
             trajectory_.poses[i].pose.position.x,
             trajectory_.poses[i].pose.position.y, dist);
}

void PurePursuit::cmdVelCallBack(const geometry_msgs::Twist &cmd_vel)
{
    last_time_cmd_vel_ = ros::Time::now();
}

void PurePursuit::controlLoop(const ros::TimerEvent &e)
{
    geometry_msgs::Twist cmd_ctrl;
    pnc_msgs::move_status move_status;
    double cmd_steer;
    double goal_dist;

    bool emergency = false;
    bool path_exist = false;
    bool backward_driving = last_bwd_;
    bool want_exact_stop = true;
    ros::Time time_now = ros::Time::now();
    ros::Duration time_diff = time_now - trajectory_.header.stamp;
    double dt = time_diff.toSec();
    tf::Stamped<tf::Pose> pose;
    pp_segments_.poses.clear();
    ref_segments_.poses.clear();

    if(dt > max_timer_)
    {
        ros::Duration time_diff2 = time_now - last_timer_complaint_;
        double dt2 = time_diff2.toSec();
        if(dt2 > max_timer_complaint_)
        {
            ROS_WARN("stopping due to timer, %lf s passed after the last plan!", dt);
            last_timer_complaint_ = time_now;
        }
        cmd_steer = 0.0;
        goal_dist = 0.0;
        emergency = true;
    }
    else if((int) trajectory_.poses.size() > 1 && getRobotPose(pose))
    {
        double cur_x = pose.getOrigin().x();
        double cur_y = pose.getOrigin().y();
        double cur_yaw = tf::getYaw(pose.getRotation());
        int segment = get_segment(cur_x, cur_y);

        goal_dist = get_dist_to_go(segment, cur_x, cur_y);
        want_exact_stop = need_gear_switch_later(segment);
        if(goal_dist == 0.0 && want_exact_stop)
        {
            ROS_WARN("not good, it seems fwd/bwd labeling is wrong");
        }
        ROS_DEBUG("goal_dist=%lf will be passed to speed_advisor", goal_dist);
        cmd_steer = get_steering(segment, cur_x, cur_y, cur_yaw, emergency);
        if(segment > -1)
        {
            path_exist = true;
            backward_driving = is_segment_backward(segment);
            last_bwd_ = backward_driving;
        }
    }
    else
    {
        cmd_steer = 0.0;
        goal_dist = 0.0;
        emergency = true;
    }

    // vel is not set here using "cmd_steer"
    // based on move_status, speed_advisor set it using "cmd_vel"
    cmd_ctrl.linear.x = 0.0;
    cmd_ctrl.angular.z = cmd_steer;
    cmd_steer_pub_.publish(cmd_ctrl);

    move_status.dist_to_goal = goal_dist;
    move_status.dist_to_ints = 99.0;
    move_status.dist_to_sig = 99.0;
    move_status.steer_angle = cmd_steer;
    move_status.obstacles_dist = 99.0;
    move_status.path_exist = path_exist;
    move_status.emergency = emergency;
    move_status.backward_driving = backward_driving;
    move_status.want_exact_stop = want_exact_stop;
    move_status_pub_.publish(move_status);

    if(goal_dist <= 0.0 || emergency || !path_exist)
        pp_segments_.poses.clear();
    pp_segments_rviz_pub_.publish(pp_segments_);
    ref_segments_rviz_pub_.publish(ref_segments_);

    ros::Duration time_diff3 = time_now - last_time_cmd_vel_;
    double dt3 = time_diff3.toSec();
    if(dt3 > max_silence_cmd_vel_)
    {
        ROS_WARN("speed_advisor did not send out cmd_vel for %lf s, I'm commanding 0.0 for safety", dt3);
        cmd_ctrl.linear.x = 0.0;
        cmd_ctrl.angular.z = 0.0;
        cmd_vel_pub_.publish(cmd_ctrl);
    }
}

bool PurePursuit::getRobotPose(tf::Stamped<tf::Pose> &odom_pose) const
{
    odom_pose.setIdentity();
    tf::Stamped<tf::Pose> robot_pose;
    robot_pose.setIdentity();
    robot_pose.frame_id_ = robot_frame_id_;
    robot_pose.stamp_ = ros::Time();
    ros::Time current_time = ros::Time::now(); // save time for checking tf delay later

    try {
        tf_.transformPose(coord_frame_id_, robot_pose, odom_pose);
    }
    catch(tf::LookupException& ex) {
        ROS_ERROR("No Transform available Error: %s", ex.what());
        return false;
    }
    catch(tf::ConnectivityException& ex) {
        ROS_ERROR("Connectivity Error: %s", ex.what());
        return false;
    }
    catch(tf::ExtrapolationException& ex) {
        ROS_ERROR("Extrapolation Error: %s", ex.what());
        return false;
    }
    // check odom_pose timeout
    if (current_time.toSec() - odom_pose.stamp_.toSec() > 0.2) {
        ROS_WARN("PurePursuit transform timeout. Current time: %.4f, pose(%s) stamp: %.4f, tolerance: %.4f",
                 current_time.toSec(), coord_frame_id_.c_str(), odom_pose.stamp_.toSec(), 0.2);
        return false;
    }

    return true;
}

double PurePursuit::get_distance(double x1, double y1, double x2, double y2)
{
    return (sqrt((x2-x1)*(x2-x1) + (y2-y1)*(y2-y1)));
}

bool PurePursuit::get_center(double tar_x, double tar_y,
                             double ori_x, double ori_y, double inv_R,
                             double center[2])
{
    double a, b, x, y, dl, dx, dy, dist1, dist2;
    x = (tar_x + ori_x)/2.0;
    y = (tar_y + ori_y)/2.0;
    dist1 = get_distance(ori_x, ori_y, tar_x, tar_y);
    double sq = 1.0/inv_R/inv_R - dist1*dist1/4.0;
    if(sq < 0.0) {
        ROS_ERROR("R*R-dist*dist/4=%lf < 0, x1=%lf, y1=%lf, x2=%lf, y2=%lf, inv_R=%lf",
                  sq, ori_x, ori_y, tar_x, tar_y, inv_R);
        return false;
    }

    if(inv_R > 0.0)
        dist2 = sqrt(sq);
    else
        dist2 = -sqrt(sq);
    a = tar_y - ori_y;
    b = ori_x - tar_x;

    dl = sqrt(a*a + b*b);
    dx = -a*dist2/dl;
    dy = -b*dist2/dl;

    center[0] = x + dx;
    center[1] = y + dy;

    return true;
}

double PurePursuit::get_inv_R(double u)
{
    double inv_R = 0.0;
    if(abs(u) > 1.0e-6)
        inv_R = 1.0/u;

    return inv_R;
}

bool PurePursuit::btwn_points(double tar_x, double tar_y,
                              double ori_x, double ori_y,
                              double inv_R, double x, double y)
{
    if(inv_R == 0.0)
    {
        double a, b, c;
        a = tar_x - ori_x;
        b = tar_y - ori_y;
        c = (ori_x - tar_x)*x + (ori_y - tar_y)*y;
        if((a*tar_x + b*tar_y + c)*(a*ori_x + b*ori_y + c) > 0.0)
            return false;
        else
            return true;
    }
    else
    {
        double center[2];
        if(!get_center(tar_x, tar_y, ori_x, ori_y, inv_R, center))
            return btwn_points(tar_x, tar_y, ori_x, ori_y, 0.0, x, y);

        double arg1, arg2, arg;
        arg1 = atan2(ori_y - center[1], ori_x - center[0]);
        arg2 = atan2(tar_y - center[1], tar_x - center[0]);
        arg = atan2(y - center[1], x - center[0]);
        if(inv_R > 0.0)
        {
            while(arg2 < arg1)
                arg2 += 2*M_PI;
            while(arg < arg1)
                arg += 2*M_PI;
            if ( arg > arg2 )
                return false;
            else
                return true;
        }
        else
        {
            while(arg2 > arg1)
                arg2 -= 2*M_PI;
            while(arg > arg1)
                arg -= 2*M_PI;
            if ( arg < arg2 )
                return false;
            else
                return true;
        }
    }
}

void PurePursuit::get_projection(double tar_x, double tar_y,
                                 double ori_x, double ori_y,
                                 double inv_R, double cur_x, double cur_y,
                                 double proj[2])
{
    if(inv_R == 0.0)
    {
        double a, b, c, x, dl, dx, dy;
        a = tar_y - ori_y;
        b = ori_x - tar_x;
        c = (tar_x - ori_x)*ori_y + ori_x*(ori_y - tar_y);
        x = (a*cur_x + b*cur_y + c)/sqrt(a*a + b*b);

        dl = sqrt(a*a + b*b);
        dx = -a*x/dl;
        dy = -b*x/dl;

        proj[0] = cur_x + dx;
        proj[1] = cur_y + dy;
    }
    else
    {
        double center[2];
        if(!get_center(tar_x, tar_y, ori_x, ori_y, inv_R, center))
            return get_projection(tar_x, tar_y, ori_x, ori_y, 0.0, cur_x, cur_y, proj);

        double dist = get_distance(center[0], center[1], cur_x, cur_y);

        proj[0] = center[0] + (cur_x - center[0])/dist/abs(inv_R);
        proj[1] = center[1] + (cur_y - center[1])/dist/abs(inv_R);
    }
}

// which segment of trajectory the robot is on
int PurePursuit::get_segment(double cur_x, double cur_y)
{
    double tar_x, tar_y, ori_x, ori_y, inv_R;
    double prj[2], center[2];

    if(last_segment_ < 0)
        return -1;
    if((int) trajectory_.poses.size() < 2)
        return -1;
    if((int) trajectory_.poses.size() < last_segment_+2)
        return -1;

    int segment = last_segment_;
    bool bContinue;
    do
    {
        bContinue = false;

        tar_x = trajectory_.poses[segment+1].pose.position.x;
        tar_y = trajectory_.poses[segment+1].pose.position.y;
        ori_x = trajectory_.poses[segment].pose.position.x;
        ori_y = trajectory_.poses[segment].pose.position.y;

        // sometimes trajectory points can collapse
        if(get_distance(tar_x, tar_y, ori_x, ori_y) < neglect_distance_)
        {
            segment++;
            if(segment+1 < (int) trajectory_.poses.size())
                bContinue = true;
            else
            {
                segment = -1;
                bContinue = false;
            }
            continue;
        }

        inv_R = get_inv_R(trajectory_.poses[segment].pose.position.z);
        if(inv_R != 0.0 && !get_center(tar_x, tar_y, ori_x, ori_y, inv_R, center))
            inv_R = 0.0;

        get_projection(tar_x, tar_y, ori_x, ori_y, inv_R, cur_x, cur_y, prj);
        if(!btwn_points(tar_x, tar_y, ori_x, ori_y, inv_R, prj[0], prj[1]))
        {
            if(get_distance(tar_x, tar_y, prj[0], prj[1]) < get_distance(ori_x, ori_y, prj[0], prj[1]))
            {
                segment++;
                if(segment+1 < (int) trajectory_.poses.size())
                    bContinue = true;
                else
                {
                    segment = -1;
                    bContinue = false;
                }
            }
        }
    } while(bContinue);

    last_segment_ = segment;
    return segment;
}

int PurePursuit::find_lookahead_segment(int segment, double cur_x, double cur_y, double L)
{
    if(segment < 0)
        return -1;
    if((int) trajectory_.poses.size() < 2)
        return -1;
    if((int) trajectory_.poses.size() < segment+2)
        return -1;

    double tar_x, tar_y, ori_x, ori_y, inv_R;
    double center[2];
    int on_segment = segment;
    bool backward_driving = is_segment_backward(on_segment);

    while((int) trajectory_.poses.size() > on_segment+1)
    {
        bool backward = is_segment_backward(on_segment);
        if(backward != backward_driving)
            break;

        tar_x = trajectory_.poses[on_segment+1].pose.position.x;
        tar_y = trajectory_.poses[on_segment+1].pose.position.y;
        ori_x = trajectory_.poses[on_segment].pose.position.x;
        ori_y = trajectory_.poses[on_segment].pose.position.y;

        // sometimes trajectory points can collapse
        if(get_distance(tar_x, tar_y, ori_x, ori_y) < neglect_distance_)
            on_segment++;

        inv_R = get_inv_R(trajectory_.poses[segment].pose.position.z);
        if(inv_R != 0.0 && !get_center(tar_x, tar_y, ori_x, ori_y, inv_R, center))
            inv_R = 0.0;

        // consider the intersection of a circle
        // (centered at current pose, radius is look_ahead_) with ...
        if(inv_R == 0.0) // with a line segment
        {
            if(intersection_circle_line(tar_x, tar_y, ori_x, ori_y, cur_x, cur_y, look_ahead_))
            {
                return on_segment;
            }
        }
        else if(inv_R > 0.0) // with an arc 1/inv_R > 0
        {
            if(intersection_circle_arc(tar_x, tar_y, ori_x, ori_y, center[0], center[1],
                                       1.0/inv_R, cur_x, cur_y, look_ahead_))
            {
                return on_segment;
            }
        }
        else // with an arc 1/inv_R < 0
        {
            if(intersection_circle_arc(ori_x, ori_y, tar_x, tar_y, center[0], center[1],
                                       -1.0/inv_R, cur_x, cur_y, look_ahead_))
            {
                return on_segment;
            }
        }

        on_segment++;
    }

    // if not found, use the current segment
    return segment;
}

// this function assumes that trajectory points are reasonably dense
// if the trajectory points are very sparse like very few points in the arc,
// this function needs to be changed
double PurePursuit::get_dist_to_go(int segment, double cur_x, double cur_y)
{
    if(segment < 0)
        return 0.0;
    if((int) trajectory_.poses.size() < 2)
        return 0.0;
    if((int) trajectory_.poses.size() < segment+2)
        return 0.0;

    double dist_to_go = 0.0;
    int on_segment = segment;
    bool backward_driving = is_segment_backward(on_segment);

    double tar_x = trajectory_.poses[on_segment+1].pose.position.x;
    double tar_y = trajectory_.poses[on_segment+1].pose.position.y;
    double ori_x = trajectory_.poses[on_segment].pose.position.x;
    double ori_y = trajectory_.poses[on_segment].pose.position.y;
    double inv_R = get_inv_R(trajectory_.poses[on_segment].pose.position.z);
    double center[2];
    if(inv_R != 0.0 && !get_center(tar_x, tar_y, ori_x, ori_y, inv_R, center))
        inv_R = 0.0;

    double prj[2];
    get_projection(tar_x, tar_y, ori_x, ori_y, inv_R, cur_x, cur_y, prj);
    if(btwn_points(tar_x, tar_y, ori_x, ori_y, inv_R, prj[0], prj[1]))
        dist_to_go += get_distance(tar_x, tar_y, prj[0], prj[1]);
    on_segment++;

    while((int) trajectory_.poses.size() > on_segment+1)
    {
        bool backward = is_segment_backward(on_segment);
        if(backward != backward_driving)
            break;

        dist_to_go += get_distance(trajectory_.poses[on_segment+1].pose.position.x,
                                   trajectory_.poses[on_segment+1].pose.position.y,
                                   trajectory_.poses[on_segment].pose.position.x,
                                   trajectory_.poses[on_segment].pose.position.y);
        on_segment++;
    }

    return dist_to_go;
}

double PurePursuit::get_steering(int segment, double cur_x, double cur_y,
                                 double cur_yaw, bool &emergency)
{
    ros::Time time_now = ros::Time::now();
    ros::Duration time_diff;
    double dt;

    if(segment < 0)
    {
        time_diff = time_now - last_segment_complaint_;
        dt = time_diff.toSec();
        if(dt > max_timer_complaint_)
        {
            ROS_WARN("cannot determine, segment = -1");
            last_segment_complaint_ = time_now;
        }
        emergency = true;
        return 0.0;
    }
    if((int) trajectory_.poses.size() < segment+1)
    {
        ROS_WARN("trajectory(%d), segment(%d), not possible!", (int) trajectory_.poses.size(), segment);
        emergency = true;
        return 0.0;
    }

    double L = look_ahead_;
    int lookahead_segment = find_lookahead_segment(segment, cur_x, cur_y, L);
    if(lookahead_segment < 0)
    {
        ROS_WARN("cannot determine, lookahead_segment = -1");
        emergency = true;
        return 0.0;
    }

    double tar_x = trajectory_.poses[lookahead_segment+1].pose.position.x;
    double tar_y = trajectory_.poses[lookahead_segment+1].pose.position.y;
    double ori_x = trajectory_.poses[lookahead_segment].pose.position.x;
    double ori_y = trajectory_.poses[lookahead_segment].pose.position.y;
    double inv_R = get_inv_R(trajectory_.poses[lookahead_segment].pose.position.z);
    double center[2];
    if(inv_R != 0.0 && !get_center(tar_x, tar_y, ori_x, ori_y, inv_R, center))
        inv_R = 0.0;

    double a, b, c, x, r, theta, gamma;

    if(inv_R == 0.0)
    {
        a = tar_y - ori_y;
        b = ori_x - tar_x;
        c = (tar_x - ori_x)*ori_y + ori_x*(ori_y - tar_y);
        r = 0.0;
        x = (a*cur_x + b*cur_y + c)/sqrt(a*a + b*b);
        theta = cur_yaw - atan2(tar_y - ori_y, tar_x - ori_x);
    }
    else
    {
        if(inv_R > 0.0)
        {
            r = get_distance(center[0], center[1], cur_x, cur_y) - abs(1.0/inv_R);
            x = (inv_R*(r*r + L*L) + 2.0*r)/(2.0*(1.0 + inv_R*r));
            theta = cur_yaw - atan2(cur_x - center[0], center[1] - cur_y);
        }
        else
        {
            r = -get_distance(center[0], center[1], cur_x, cur_y) + abs(1.0/inv_R);
            x = (inv_R*(r*r + L*L) + 2.0*r)/(2.0*(1.0 + inv_R*r));
            theta = cur_yaw - atan2(-cur_x + center[0], cur_y - center[1]);
        }
    }

    if(is_segment_backward(segment))
        theta += M_PI;

    while(theta > M_PI)
        theta -= 2*M_PI;
    while(theta < -M_PI)
        theta += 2*M_PI;

    if(L < abs(x))
    {
        ROS_WARN("too off from traj, L=%lf < abs(x=%lf)", L, x);
        emergency = true;
        return 0.0;
    }

    // for all the calculations, refer IROS'95 by Ollero and Heredia
    gamma = 2.0/(L*L)*(x*cos(theta) - sqrt(L*L - x*x)*sin(theta));
    double steering;
    if(is_segment_backward(segment))
        steering = -atan(gamma * car_length_);
    else
        steering = atan(gamma * car_length_);
    ROS_DEBUG("%ssteering(%s), on_segment=%d lookahead_seg=%d:(%lf,%lf)->(%lf,%lf) at x=%lf y=%lf yaw=%lf",
              emergency ? "emergency! " : "",
              is_segment_backward(segment) ? "bwd" : "fwd", segment, lookahead_segment,
              ori_x, ori_y, tar_x, tar_y, cur_x, cur_y, cur_yaw);
    ROS_DEBUG("inv_R=%lf L=%lf r=%lf x=%lf theta=%lf gamma=%lf steering=%lf",
              inv_R, L, r, x, theta, gamma, steering);

    // the circular segments by pure pursuit
    set_pp_segments_rviz(cur_x, cur_y, cur_yaw, gamma, is_segment_backward(segment));
    // the ref segments (lookahead segment) for pure pursuit
    set_ref_segments_rviz(lookahead_segment);

    if(isnan(steering))
    {
        ROS_WARN("steering isnan, so commanding just 0!");
        emergency = true;
        steering = 0.0;
    }
    else if(steering > max_steering_)
        steering = max_steering_;
    else if(steering < -max_steering_)
        steering = -max_steering_;

    return steering;
}

// check whether the line segment contains the lookahead point
// a circle centered at current (x, y) with the radius = look_ahead_
bool PurePursuit::intersection_circle_line(double tar_x, double tar_y,
                                           double ori_x, double ori_y,
                                           double cen_x, double cen_y, double r)
{
    // http://stackoverflow.com/questions/1073336/circle-line-collision-detection

    double dx = tar_x - ori_x;
    double dy = tar_y - ori_y;
    double fx = ori_x - cen_x;
    double fy = ori_y - cen_y;

    double a = dx*dx + dy*dy;
    double b = 2.0*(fx*dx + fy*dy);
    double c = fx*fx + fy*fy - r*r;

    double discriminant = b*b - 4.0*a*c;
    if(discriminant < 0.0)
    {
/*
        ROS_DEBUG("No intersection, D(%lf)<0, ori:x=%lf y=%lf, tar:x=%lf y=%lf, current:x=%lf y=%lf",
                  discriminant, ori_x, ori_y, tar_x, tar_y, cen_x, cen_y);
*/
        return false;
    }

    double t1 = (-b + sqrt(discriminant)) / (2.0*a);
    if(t1 >= 0.0 && t1 <= 1.0)
    {
        return true;
    }
    double t2 = (-b - sqrt(discriminant)) / (2.0*a);
    if(t2 >= 0.0 && t2 <= 1.0)
    {
        return true;
    }

/*
    ROS_DEBUG("No intersection, t1=%lf, t2=%lf, ori:x=%lf y=%lf, tar:x=%lf y=%lf, current:x=%lf y=%lf",
              t1, t2, ori_x, ori_y, tar_x, tar_y, cen_x, cen_y);
*/
    return false;
}

// on circle1, (ori_x, ori_y) to (tar_x, tar_y) defines the circular trajectory
// circle2 is centered at current (x, y) with the radius = look_ahead_
// if any of the intersection points between two circles are on the circular trajectory,
// that is the lookahead point we use in pure pursuit strategy
bool PurePursuit::intersection_circle_arc(double tar_x, double tar_y,
                                          double ori_x, double ori_y,
                                          double cen1_x, double cen1_y, double r1,
                                          double cen2_x, double cen2_y, double r2)
{
    // http://paulbourke.net/geometry/2circle

    double a, d, dx, dy, h, hx, hy;
    double x2, y2, xi1, yi1, xi2, yi2;

    // dx and dy are distances between the circle centers
    dx = cen2_x - cen1_x;
    dy = cen2_y - cen1_y;
    d = sqrt(dx*dx + dy*dy);

    // check for solvability
    if(d >= r1 + r2)
    {
        // circles do not intersect, far each other
        // '=' case not considered as an intersection since we want an 'arc'
        return false;
    }
    if(d <= fabs(r1 - r2))
    {
        // one circle is contained in the other
        // '=' case not considered as an intersection since we want an 'arc'
        if(r1 == r2) // circles overlap, practically not happening
            return true;
        else
            return false;
    }

    // (x2, y2) is the point where
    // the line (through the circle intersection points)
    // crosses the line (between the circle centers)
    a = (r1*r1 - r2*r2 + d*d) / (2.0*d); // the distance from circle1 to (x2, y2)
    x2 = cen1_x + dx*a/d;
    y2 = cen1_y + dy*a/d;

    // the distance from (x2, y2) to either of the intersection points
    h = sqrt(r1*r1 - a*a);

    // the offsets of the intersection points from (x2, y2)
    hx = -dy*h/d;
    hy = dx*h/d;

    // intersection points
    xi1 = x2 + hx; xi2 = x2 - hx;
    yi1 = y2 + hy; yi2 = y2 - hy;

    // determine the trajectory arc
    double arg_from = atan2(ori_y - cen1_y, ori_x - cen1_x);
    double arg_to = atan2(tar_y - cen1_y, tar_x - cen1_x);
    while(arg_to < arg_from)
        arg_to += 2*M_PI;

    // determine the args from intersection points
    // check whether intersection points are on the trajectory arc
    double arg1 = atan2(yi1 - cen1_y, xi1 - cen1_x);
    while(arg1 < arg_from)
        arg1 += 2*M_PI;
    if(arg1 < arg_to)
        return true;
    double arg2 = atan2(yi2 - cen1_y, xi2 - cen1_x);
    while(arg2 < arg_from)
        arg2 += 2*M_PI;
    if(arg2 < arg_to)
        return true;

/*
    ROS_DEBUG("No intersection, r=%lf ori:x=%lf y=%lf, tar:x=%lf y=%lf, current:x=%lf y=%lf lookahead=%lf",
              r1, ori_x, ori_y, tar_x, tar_y, cen2_x, cen2_y, r2);
*/
    return false;
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "golfcar_pp");
    PurePursuit pp;
    ros::spin();
    return 0;
}
