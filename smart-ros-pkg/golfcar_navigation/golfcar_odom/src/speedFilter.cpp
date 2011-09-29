/** Filters the speed and acceleration.
 *
 * Gets the current speed information from odometry, filters it, computes
 * acceleration and filters it too. Publish the result to topic speed_filter.
 */


#include <ros/ros.h>
#include <sensor_msgs/Imu.h>

#include <filters/filter_chain.h>

#include <golfcar_odom/gcSpeedFilter.h> //The output message
#include <golfcar_halsampler/odo.h>     //The input message


class SpeedFilter
{
    public:
        SpeedFilter(ros::NodeHandle nh_);

    private:
        void samplerCallBack(golfcar_halsampler::odo sampler);

        ros::NodeHandle n;
        ros::Subscriber sub;
        ros::Publisher filter_pub;

        filters::MultiChannelFilterChain<double> *speedFilter, *accFilter;
        std::vector<double> speedvec, accvec;

        double time_pre, speed_pre;
};



SpeedFilter::SpeedFilter(ros::NodeHandle nh_) : n(nh_)
{
    time_pre = -1; // means that the filter has not been called yet

    sub = n.subscribe("golfcar_sampler", 1000, &SpeedFilter::samplerCallBack, this);
    filter_pub = n.advertise<golfcar_odom::gcSpeedFilter>("speed_filter",1);

    speedFilter = new filters::MultiChannelFilterChain<double>("double");
    accFilter = new filters::MultiChannelFilterChain<double>("double");

    if (!speedFilter->configure(1, "velocity_filter"))
        ROS_ERROR("Velocity filter not configured");

    if (!accFilter->configure(1, "acc_filter"))
        ROS_ERROR("Acceleration filter not configured");


    accvec.push_back(0);
    speedvec.push_back(0);
}


void SpeedFilter::samplerCallBack(golfcar_halsampler::odo sampler)
{
    double now = ros::Time::now().toSec();

    if( time_pre<0 )
    {
        // First time init
        time_pre = now;
        speed_pre = sampler.vel;
        return;
    }

    // Filter velocity
    speedvec[0] = sampler.vel;
    speedFilter->update(speedvec, speedvec);

    // Compute acc and filter it
    accvec[0] = (speedvec[0] - speed_pre) * (now - time_pre);
    accFilter->update(accvec, accvec);
    speed_pre = speedvec[0];
    time_pre = now;

    // Create message and publish it
    golfcar_odom::gcSpeedFilter gcsf;
    gcsf.speed = sampler.vel;
    gcsf.speedFiltered = speedvec[0];
    gcsf.acc = accvec[0];
    gcsf.accFiltered = accvec[0];
    filter_pub.publish(gcsf);
}





int main(int argc, char **argv)
{
    ros::init(argc, argv, "golfcar_odom");
    ros::NodeHandle nh_;
    SpeedFilter filter(nh_);
    ros::spin();
    return 0;
}
