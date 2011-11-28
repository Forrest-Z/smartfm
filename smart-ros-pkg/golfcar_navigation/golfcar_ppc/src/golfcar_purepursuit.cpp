#include "golfcar_purepursuit.h"
using namespace std;
namespace golfcar_purepursuit
{

PurePursuit::PurePursuit()
{
    Lfw_ = 3;
    lfw_ = 1;
    car_length = 1.632;
    nextPathThres_ = 5;
    dist_to_final_point = 100;
    initialized_=false;
    path_n_ = 0;
    nextPathCount_ = 0;
    ros::NodeHandle n;
    pp_vis_ = n.advertise<geometry_msgs::PolygonStamped>("pp_vis", 1);
}


bool PurePursuit::steering_control(double& wheel_angle, double &dist_to_goal)
{
    geometry_msgs::Point pt;
    if(!initialized_)
    {
        current_point_ = vehicle_base_.position;
        next_point_ = path_.poses[0].pose.position;
        initialized_=true;
    }
    double heading_lh=0;

    if(heading_lookahead(heading_lh,dist_to_goal))
    {
        wheel_angle = atan((car_length * sin(heading_lh))/(Lfw_/2+lfw_*cos(heading_lh)));
        if(wheel_angle > 0.65) wheel_angle = 0.65;
        else if(wheel_angle < -0.65) wheel_angle = -0.65;
        ROS_DEBUG("pursuing point: %lf %lf, %lf %lf", current_point_.x, current_point_.y, next_point_.x, next_point_.y);
        return true;
    }
    else
        return false;
}


bool PurePursuit::heading_lookahead(double &heading_la, double &dist_to_goal)
{
    double vehicle_heading = tf::getYaw(vehicle_base_.orientation);
    geometry_msgs::Point anchor_pt;
    anchor_pt.x = vehicle_base_.position.x + lfw_ * cos(vehicle_heading);
    anchor_pt.y = vehicle_base_.position.y + lfw_ * sin(vehicle_heading);

    geometry_msgs::Point collided_pt;

    //search through all the path segments
    ros::Time t1 = ros::Time::now();
    path_n_=0;
    current_point_ = path_.poses[path_n_].pose.position;
    next_point_ = path_.poses[path_n_+1].pose.position;
    //std::cout<<current_point_<<next_point_<<std::endl;
    dist_to_goal=0;
    while(!circle_line_collision(anchor_pt,collided_pt))
    {
        path_n_++;

        if(path_n_+1<path_.poses.size()){
            current_point_ = path_.poses[path_n_].pose.position;
            next_point_ = path_.poses[path_n_+1].pose.position;
            //std::cout<<current_point_<<next_point_<<std::endl;
            //ROS_DEBUG("Updating points");
            dist_to_goal=sqrt_distance(vehicle_base_.position, path_.poses[path_n_+1].pose.position);
            for(int i=path_.poses.size()-1;i>path_n_+1;i--)
            {
                dist_to_goal+=sqrt_distance(path_.poses[i].pose.position,path_.poses[i-1].pose.position);
            }
        }
        else
        {
            dist_to_goal=sqrt_distance(vehicle_base_.position, path_.poses[path_.poses.size()-1].pose.position);
            return false;
        }
    }
    ros::Time t2 = ros::Time::now();
    //ROS_INFO("Search time %d", (t2-t1).toNSec());
    //std::cout<<"Search time "<< (t2-t1).toNSec()<<std::endl;
    //heading_la = -1 indicate the controller couldn't find a path to follow, it has to be handle by trajectory_planner
    heading_la = atan2(collided_pt.y-anchor_pt.y, collided_pt.x-anchor_pt.x) - vehicle_heading;
    return true;
}


double PurePursuit::sqrt_distance(geometry_msgs::Point wp_a, geometry_msgs::Point wp_b)
{
    return sqrt((wp_a.x - wp_b.x)*(wp_a.x - wp_b.x)+(wp_a.y - wp_b.y)*(wp_a.y - wp_b.y));
}


bool PurePursuit::circle_line_collision(geometry_msgs::Point& anchor_point,
                                        geometry_msgs::Point& intersect_point){
    //http://stackoverflow.com/questions/1073336/circle-line-collision-detection
    double Ex = current_point_.x;
    double Ey = current_point_.y;
    double Lx = next_point_.x;
    double Ly = next_point_.y;
    double Cx = anchor_point.x;
    double Cy = anchor_point.y;
    double r = Lfw_;
    double dx = Lx - Ex; double dy = Ly - Ey;
    double fx = Ex - Cx; double fy = Ey - Cy;

    float a = dx * dx + dy * dy;
    float b = 2 * (fx * dx + fy * dy);
    float c = (fx * fx + fy * fy) - (r * r);

    float discriminant = b*b-4*a*c;

    geometry_msgs::Point32 p;
    geometry_msgs::PolygonStamped polyStamped;
    polyStamped.header.frame_id = "/map";
    polyStamped.header.stamp = ros::Time::now();
    //ROS_INFO("cur:x=%lf y%lf, next:x=%lf y=%lf, anchor_point:x=%lf y=%lf r=%lf", Ex,Ey,Lx,Ly,Cx,Cy,r);
    if(discriminant < 0)
    {
        ROS_DEBUG("No intersection, cur:x=%lf y%lf, next:x=%lf y=%lf, anchor_point:x=%lf y=%lf", Ex,Ey,Lx,Ly,Cx,Cy);
        return false;
    }
    else
    {
        discriminant = sqrt(discriminant);
        float t1 = (-b + discriminant)/(2*a);
        float t2 = (-b - discriminant)/(2*a);
        geometry_msgs::Point intersect_point2;
        p.x = intersect_point2.x = Ex+t2*dx;
        p.y = intersect_point2.y = Ey+t2*dy;

        if(t1 >=0 && t1 <=1)
        {

            p.x = intersect_point.x = Ex+t1*dx;
            p.y = intersect_point.y = Ey+t1*dy;
            polyStamped.polygon.points.push_back(p);
            p.x = Cx; p.y=Cy;
            polyStamped.polygon.points.push_back(p);
            pp_vis_.publish(polyStamped);
            return true;
        }
        else
        {
            ROS_DEBUG("No solution, cur:x=%lf y%lf, next:x=%lf y=%lf", Ex,Ey,Lx,Ly);
            return false;
        }

    }
}

} //namespace golfcar_purepursuit
