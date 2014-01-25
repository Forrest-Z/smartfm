#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <csignal>

const double freq = 20;
const double acceleration0 = 0.6;
const double acceleration1 = 0.7;

const double alpha0 = 0.5;
const double alpha1 = 0.7;

sig_atomic_t emergency_break = 0;

void sig_break(int param) {
    emergency_break = 1;
    std::cerr << "Emergency break!" << std::endl;
}

class VelPublisher {
public:
    VelPublisher(): curr_vel(0), target_vel(0) {
    }

    void spin() {
        ros::NodeHandle nh;
        vel_sub = nh.subscribe("cmd_vel_pomdp", 1, &VelPublisher::velCallBack, this);
        ros::Timer timer = nh.createTimer(ros::Duration(1 / freq), &VelPublisher::publishSpeed, this);
        cmd_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel",1);
        ros::spin();
    }

    virtual void velCallBack(geometry_msgs::TwistConstPtr pomdp_vel) = 0;

    virtual void publishSpeed(const ros::TimerEvent& event) = 0;

    void _publishSpeed()
    {
        geometry_msgs::Twist cmd;
        cmd.angular.z = 0;
        cmd.linear.x = emergency_break? 0 : curr_vel;
        cmd_pub.publish(cmd);
    }

    double curr_vel, target_vel;
    ros::Subscriber vel_sub;
    ros::Publisher cmd_pub;
};

class VelPublisher1 : public VelPublisher {
public:

    VelPublisher1(): VelPublisher() {}

    void velCallBack(geometry_msgs::TwistConstPtr pomdp_vel) {
        target_vel = pomdp_vel->linear.x;
        if(target_vel > curr_vel)
            curr_vel = curr_vel*(1-alpha0)+target_vel*alpha0;
        else
            curr_vel = curr_vel*(1-alpha1)+target_vel*alpha1;
    }

    void publishSpeed(const ros::TimerEvent& event) {
        _publishSpeed();
    }
};

class VelPublisher2 : public VelPublisher {
    void velCallBack(geometry_msgs::TwistConstPtr pomdp_vel) {
		if(pomdp_vel->linear.x==-1)  {
			curr_vel=0.0;	
			target_vel=0.0;
			return;
		}
        target_vel = pomdp_vel->linear.x;
		if(0.2<target_vel && target_vel < 0.5) {
			target_vel = 0.6;
		}

    }

    void publishSpeed(const ros::TimerEvent& event) {
        double delta = acceleration0 / freq;
        if(target_vel > curr_vel + delta) {
			double delta = acceleration0 / freq;
            curr_vel += delta;
		} else if(target_vel < curr_vel - delta) {
			double delta = acceleration1 / freq;
            curr_vel -= delta;
		} else
			curr_vel = target_vel;

        _publishSpeed();
    }

};

int main(int argc,char**argv)
{
	ros::init(argc,argv,"vel_publisher");
    signal(SIGUSR1, sig_break);
    VelPublisher2 velpub;
    velpub.spin();
	return 0;
}
