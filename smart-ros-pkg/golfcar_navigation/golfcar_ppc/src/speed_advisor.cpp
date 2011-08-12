#include <golfcar_ppc/speed_advisor.h>

using namespace std;
namespace speed_advisor {


SpeedAdvisor::SpeedAdvisor(){

	ros::NodeHandle n;
	recommend_speed_= n.advertise<geometry_msgs::Twist>("cmd_vel", 1);
	move_base_speed_=n.subscribe("/move_vel",1, &SpeedAdvisor::moveSpeedCallback, this);
	global_plan_=n.subscribe("/global_plan",1,&SpeedAdvisor::globalPlanCallback,this);
	vector<geometry_msgs::Point> stoppingPoint;
	geometry_msgs::Point p;
	p.x = 322; p.y = 2300; stoppingPoint.push_back(p);
	p.x = 636; p.y = 538; stoppingPoint.push_back(p);
	double res = 0.1;
	int y_pixels = 3536;
	double stoppingDistance = 8;
	lastStop_ = -1;
	//to do, simply subscribe to cmd_vel from move_base then republish it, cmd_vel from move_base must renamed
	while(ros::ok())
	{
		tf::Stamped<tf::Pose> robot_pose;
		getRobotGlobalPose(robot_pose);
		double robot_x = robot_pose.getOrigin().x();
		double robot_y = robot_pose.getOrigin().y();

		//loop through all the points for stopping
		for(int i=0; i<stoppingPoint.size();i++)
		{
			double sp_x = stoppingPoint[i].x *res;
			double sp_y = (y_pixels-stoppingPoint[i].y) *res;
			if(sqrtDistance(robot_x,robot_y,sp_x, sp_y)<=stoppingDistance && lastStop_!=i)
			{
				move_speed_.linear.x = 0;
				lastStop_ = i;
				recommend_speed_.publish(move_speed_);
				ros::Duration(0.1).sleep();
				ros::spinOnce();
				string temp = "";
				cout<<"Clear to go?"<<endl;
				getline(cin, temp);
				cout<<"Continue"<<endl;
			}
		}
		recommend_speed_.publish(move_speed_);
		ros::Duration(0.1).sleep();
		ros::spinOnce();


	}
}

void SpeedAdvisor::globalPlanCallback(nav_msgs::Path path)
{
	//listen to any new path is given, if it is, simply reset the last stopping record so that all stops can be reevaluate again
	lastStop_=-1;
}
double SpeedAdvisor::sqrtDistance(double x1, double y1, double x2, double y2)
{
	return sqrt((x1-x2)*(x1-x2)+(y1-y2)*(y1-y2));
}
void SpeedAdvisor::moveSpeedCallback(geometry_msgs::Twist cmd_vel)
{
	move_speed_ = cmd_vel;
}

bool SpeedAdvisor::getRobotGlobalPose(tf::Stamped<tf::Pose>& odom_pose) const
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
};

int main(int argc, char **argv)
{
	ros::init(argc, argv, "speed_advisor");
	ros::NodeHandle nh_;
	speed_advisor::SpeedAdvisor *sa = new speed_advisor::SpeedAdvisor();


	ros::spin();

	return 0;
}
