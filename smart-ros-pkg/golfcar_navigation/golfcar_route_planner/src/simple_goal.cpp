#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>

using namespace std;
class SimpleGoal
{
public:
    SimpleGoal();
    void goalCallback(geometry_msgs::PoseStamped map_pose);
    void on_goal_timer(const ros::TimerEvent& e);

    ros::NodeHandle n;
    ros::Subscriber sub;
    ros::Publisher waypoint_pub_;
    ros::Timer goal_timer;
    tf::TransformListener listener;
    
    double current_point;
};



SimpleGoal::SimpleGoal()
{
    waypoint_pub_ = n.advertise<geometry_msgs::PoseStamped>("move_base_simple/goal", 100);
    goal_timer = n.createTimer(ros::Duration(0.5), &SimpleGoal::on_goal_timer, this);
    current_point = -1;
}

void SimpleGoal::on_goal_timer(const ros::TimerEvent& e)
{
    geometry_msgs::PoseStamped goal;
    double n;
    if(current_point==-1) 
    {
        cout<<"Current station: ";
        cin>>current_point;
        
        cout<<"Current station is "<<current_point<<endl;
    }
    goal.header.stamp = ros::Time::now();
    goal.header.frame_id = "/map";
    
    goal.pose.position.x = current_point;
    cout<<"Next station: ";
    cin>>n;
    cout<<"Next station is "<<n<<endl;
    goal.pose.position.y = n;
    current_point = n;
    goal.pose.orientation.z = 1.0;
    ROS_INFO("send goal: [%f %f]", goal.pose.position.x, goal.pose.position.y);
    waypoint_pub_.publish(goal);
    //(int)goal.pose.position.y;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "goal_talker");
    SimpleGoal sg;
    ros::spin();
    return 0;
}
