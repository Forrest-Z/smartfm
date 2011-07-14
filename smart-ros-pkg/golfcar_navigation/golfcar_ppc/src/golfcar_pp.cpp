#include <golfcar_ppc/golfcar_pp.h>


namespace golfcar_purepursuit {
	PurePursuit::PurePursuit()
	{
		traj_sub_ = n_.subscribe("pnc/trajectory", 1, &PurePursuit::trajCallBack, this);
		cmd_pub_ = n_.advertise<geometry_msgs::Twist>("cmd_vel",1);
		freq_ = 100;
		PurePursuit::controlLoop();
	}

	PurePursuit::~PurePursuit()
	{
	}


	void PurePursuit::trajCallBack(const nav_msgs::Path::ConstPtr &traj)
	{
		//trajectory given by planner is in base_link frame, convert everything to map frame
		//loop through every points in the path
		trajectory_.header.frame_id = "map";
		trajectory_.header.stamp = ros::Time::now();
		trajectory_.poses.resize(traj->poses.size());
		for(unsigned int i=0; i<traj->poses.size(); i++)
		{
			 try{
				tf_.transformPose("map", traj->poses[i], trajectory_.poses[i]);
			}
			catch(tf::LookupException& ex) {
				ROS_ERROR("No Transform available Error: %s\n", ex.what());
				return;
			}
				catch(tf::ConnectivityException& ex) {
				ROS_ERROR("Connectivity Error: %s\n", ex.what());
				return;
			}
				catch(tf::ExtrapolationException& ex) {
				ROS_ERROR("Extrapolation Error: %s\n", ex.what());
				return;
			}
		}

		//re-initialise control loop

	
	}

	void PurePursuit::controlLoop()
	{
		ros::Rate control_freq(freq_);

		while(ros::ok())
		{
			cout<<"Control loop in action"<<endl;
			ros::spinOnce();
			control_freq.sleep();
		}
	
	}
	bool PurePursuit::getRobotPose(tf::Stamped<tf::Pose>& global_pose) const 
	{
	    global_pose.setIdentity();
	    tf::Stamped<tf::Pose> robot_pose;
	    robot_pose.setIdentity();
	    robot_pose.frame_id_ = "base_link";
	    robot_pose.stamp_ = ros::Time();
	    ros::Time current_time = ros::Time::now(); // save time for checking tf delay later

	    //get the global pose of the robot
	    try{
	      tf_.transformPose("map", robot_pose, global_pose);
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
	    // check global_pose timeout
	    if (current_time.toSec() - global_pose.stamp_.toSec() > 0.1) {
	      ROS_WARN("PurePursuit transform timeout. Current time: %.4f, global_pose stamp: %.4f, tolerance: %.4f",
		  current_time.toSec() ,global_pose.stamp_.toSec() ,"0.1");
	      return false;
	    }

	    return true;
	  }
};	

int main(int argc, char **argv)
{
    ros::init(argc, argv, "golfcar_pp");
    ros::NodeHandle n;
    golfcar_purepursuit::PurePursuit pp;

    ros::spin();
	return 0;
}	
