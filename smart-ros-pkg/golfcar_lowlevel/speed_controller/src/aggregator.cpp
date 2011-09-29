#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <golfcar_halsampler/odo.h>
#include <lse_xsens_mti/imu_rpy.h>
#include <golfcar_halstreamer/throttle.h>
#include <golfcar_halstreamer/brakepedal.h>
#include <speed_controller/agg.h>
#include <golfcar_odom/gcSpeedFilter.h>

class Speed_agg
{
    public:
        Speed_agg(ros::NodeHandle);

    private:
        ros::NodeHandle n;
        ros::Subscriber cmdVelSub, odoSub, rpySub, throttleSub, brakePedalSub;
        ros::Publisher aggPub;

        void cmdVelCallBack(geometry_msgs::Twist);
        void rpyCallBack(lse_xsens_mti::imu_rpy);
        //void samplerCallBack(golfcar_halsampler::odo);
        void samplerCallBack(golfcar_odom::gcSpeedFilter);
        void throttleCallBack(golfcar_halstreamer::throttle);
        void brakeCallBack(golfcar_halstreamer::brakepedal);

        speed_controller::agg aggMsg;
        int counter;
};



Speed_agg::Speed_agg(ros::NodeHandle nh) : n(nh)
{
    cmdVelSub= n.subscribe("cmd_vel", 1, &Speed_agg::cmdVelCallBack, this);
    //odoSub= n.subscribe("golfcar_sampler", 1, &Speed_agg::samplerCallBack, this);
    odoSub= n.subscribe("speed_filter", 1, &Speed_agg::samplerCallBack, this);
    rpySub = n.subscribe("ms/imu/rpy", 1, &Speed_agg::rpyCallBack, this);
    throttleSub = n.subscribe("golfcar_speed", 1, &Speed_agg::throttleCallBack, this);
    brakePedalSub = n.subscribe("golfcar_brake", 1, &Speed_agg::brakeCallBack, this);
    aggPub = n.advertise<speed_controller::agg>("speed_agg", 1);
}


void Speed_agg::cmdVelCallBack(geometry_msgs::Twist cmd_vel)
{
    aggMsg.cmd = cmd_vel.linear.x;
}


void Speed_agg::rpyCallBack(lse_xsens_mti::imu_rpy rpy)
{
    aggMsg.pitch = rpy.pitch;
}


void Speed_agg::throttleCallBack(golfcar_halstreamer::throttle th)
{
    aggMsg.th = th.volt;
}


void Speed_agg::brakeCallBack(golfcar_halstreamer::brakepedal bp)
{
    aggMsg.brake = bp.angle;
}

//void Speed_agg::samplerCallBack(golfcar_halsampler::odo sampler)
void Speed_agg::samplerCallBack(golfcar_odom::gcSpeedFilter gcsf)
{
    //aggMsg.speed = sampler.vel;
    aggMsg.speed = gcsf.speed;
    aggMsg.speed_f = gcsf.speedFiltered;
    aggMsg.acc = gcsf.acc;
    aggMsg.acc_f = gcsf.accFiltered;
    //if(counter%10 ==0)
        aggPub.publish(aggMsg);
    counter++;
}


int main(int argc, char**argv)
{
    ros::init(argc, argv, "speed_agg");
    ros::NodeHandle nh;
    Speed_agg pidc(nh);
    ros::spin();
    return 0;
}
