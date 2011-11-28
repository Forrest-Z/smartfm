#include <math.h>

#include <string>
#include <cmath>

using namespace std;

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PolygonStamped.h>
#include <std_msgs/Bool.h>


class StopJunction
{
public:
    StopJunction();

    ros::NodeHandle n;
    tf::TransformListener tf_;
    ros::Publisher junction_pub;
    ros::Subscriber global_plan_;

private:
    int lastStop_;
    bool getRobotGlobalPose(tf::Stamped<tf::Pose>& odom_pose) const;
    void globalPlanCallback(nav_msgs::Path path);
    double sqrtDistance(double x1, double y1, double x2, double y2);
    double stopping_distance_;
    void UILoop();
    vector<geometry_msgs::Point> stoppingPoint_;
};


StopJunction::StopJunction()
{
    junction_pub= n.advertise<std_msgs::Bool>("/nav_junction", 1);
    global_plan_=n.subscribe("/global_plan",1,&StopJunction::globalPlanCallback,this);

    ros::NodeHandle nh("~");
    nh.param("stopping_distance", stopping_distance_, 8.0);

    lastStop_ = -1;

    geometry_msgs::Point p;
    p.x = 322; p.y = 2300; stoppingPoint_.push_back(p);
    p.x = 636; p.y = 538; stoppingPoint_.push_back(p);
    StopJunction::UILoop();

}

void StopJunction::UILoop()
{
    ros::Rate loop(10);
    double res = 0.1;
    int y_pixels = 3536;
    while(ros::ok())
    {
        tf::Stamped<tf::Pose> robot_pose;
        getRobotGlobalPose(robot_pose);
        double robot_x = robot_pose.getOrigin().x();
        double robot_y = robot_pose.getOrigin().y();
        std_msgs::Bool junction;
        for(int i=0; i<stoppingPoint_.size();i++)
        {
            double sp_x = stoppingPoint_[i].x *res;
            double sp_y = (y_pixels-stoppingPoint_[i].y) *res;
            if(sqrtDistance(robot_x,robot_y,sp_x, sp_y)<=stopping_distance_ && lastStop_!=i)
            {
                lastStop_ = i;

                junction.data = true;
                junction_pub.publish(junction);
                ros::spinOnce();
                string temp = "";
                cout<<"Clear to go?"<<endl;
                getline(cin, temp);
                cout<<"Continue"<<endl;
            }
        }
        junction.data=false;
        junction_pub.publish(junction);
        ros::spinOnce();
        loop.sleep();
    }
}

void StopJunction::globalPlanCallback(nav_msgs::Path path)
{
    //listen to any new path is given, if it is, simply reset the last stopping record so that all stops can be reevaluate again
    lastStop_=-1;
}

double StopJunction::sqrtDistance(double x1, double y1, double x2, double y2)
{
    return sqrt((x1-x2)*(x1-x2)+(y1-y2)*(y1-y2));
}

bool StopJunction::getRobotGlobalPose(tf::Stamped<tf::Pose>& odom_pose) const
{
    odom_pose.setIdentity();
    tf::Stamped<tf::Pose> robot_pose;
    robot_pose.setIdentity();
    robot_pose.frame_id_ = "/base_link";
    robot_pose.stamp_ = ros::Time();
    ros::Time current_time = ros::Time::now(); // save time for checking tf delay later

    try {
        tf_.transformPose("/map", robot_pose, odom_pose);
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
    ros::init(argc, argv, "stop_junction");
    StopJunction sa;
    ros::spin();
    return 0;
}
