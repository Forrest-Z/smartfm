/** Generate sensor and ground truth data for a simulated vehicle following a
 * given speed (or rather) acceleration profile.
 * - publish ground truth as an Odometry message (on "ground_truth"), a TF, and a
 *   SimpleOdo message (on "ground_truth_linear")
 * - publish simulated encoder data as lowlevel/Encoders messages on "encoders"
 * - publish simulated IMU data as sensor_msgs/Imu messages on "ms/imu/data"
 */

#include <math.h>

#include <iostream>
#include <vector>
#include <string>
#include <algorithm>
using std::cout;
using std::endl;
using std::vector;
using std::string;


#include <ros/ros.h>
#include <rosbag/bag.h>

#include <rosgraph_msgs/Clock.h>
#include <geometry_msgs/Quaternion.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>

#include <lowlevel/Encoders.h>

#include <boost/random.hpp>
#include <boost/random/normal_distribution.hpp>


/*
 * Design: StampedData is a base class to hold data that can have 3 different
 * subtypes (State, OdoData or ImuData). The base class holds the time of the
 * data and a type indicator. The advantage of this design is that all the data
 * can be generated and stored in a common container that can later be easily
 * sorted using the std::sort function.
 */

const double DEFAULT_NOISE_LEVEL = 0.05;
const double DEFAULT_PERIOD = 0.01;

enum StampedDataType {state_t, odo_t, imu_t};
string type2str(StampedDataType t)
{
    string s;
    switch( t )
    {
        case state_t:
            s = "state";
            break;
        case odo_t:
            s = "odo";
            break;
        case imu_t:
            s = "imu";
            break;
    }
    return s;
}

class StampedData
{
public:
    double time;
    StampedDataType type;
    StampedData(StampedDataType t) : time(0), type(t) { }
    string typestr() const { return type2str(type); }
};


/// Class State holds the ground truth
class State : public StampedData
{
public:
    double l, dl; //total distance travelled
    double x, y, th; //pose
    double v, w; //vel
    double a, dw; //acc
    double dt; //period

    State() : StampedData(state_t),
    l(0.0), x(0.0), y(0.0), th(0.0), v(0.0), w(0.0), a(0.0), dw(0.0)
    {

    }

    /// Integrate given a time step, an acceleration and an angular acceleration.
    double integrate(double dt, double a, double dw)
    {
        time += dt;
        this->dt = dt;
        this->a = a;
        this->dw = dw;
        v += a*dt;
        w += dw*dt;
        th += w*dt;
        dl = v*dt;
        l += dl;
        x += dl*cos(th);
        y += dl*sin(th);
        return time;
    }

    nav_msgs::Odometry odometryMsg() const
    {
        nav_msgs::Odometry odomsg;
        odomsg.header.stamp = ros::Time(time);
        odomsg.header.frame_id = "/ground_truth";
        odomsg.child_frame_id = "/base_link";
        odomsg.pose.pose.position.x = x;
        odomsg.pose.pose.position.y = y;
        odomsg.pose.pose.orientation = tf::createQuaternionMsgFromYaw(th);
        odomsg.twist.twist.linear.x = v;
        odomsg.twist.twist.angular.z = w;
        return odomsg;
    }

    geometry_msgs::TransformStamped tf() const
    {
        geometry_msgs::TransformStamped odom_trans;
        odom_trans.header.stamp = ros::Time(time);
        odom_trans.header.frame_id = "/ground_truth";
        odom_trans.child_frame_id = "/base_link";

        odom_trans.transform.translation.x = x;
        odom_trans.transform.translation.y = y;
        odom_trans.transform.rotation = tf::createQuaternionMsgFromYaw(th);

        return odom_trans;
    }

};


/// Search the state vector to find the state that corresponds to a give time.
unsigned find(const vector<State> & states, unsigned start, double time)
{
    unsigned i = start;
    for( ; i<states.size(); i++ )
        if( states[i].time==time || (i+1<states.size() && states[i+1].time>time ) )
            break;
    return i;
}


class NoiseGenerator
{
    boost::mt19937 rng;
    boost::normal_distribution<> nd;
    boost::variate_generator<boost::mt19937&, boost::normal_distribution<> > var_nor;

public:
    NoiseGenerator(double noiseLevel) : nd(0.0, noiseLevel), var_nor(rng, nd) { }
    double operator()() { return (nd.sigma()==0.0 ? 0.0 : var_nor()); }
};



class ImuData : public StampedData
{
public:
    sensor_msgs::Imu imumsg;

    ImuData() : StampedData(imu_t) { }

    ImuData(double t, const string & frame_id, const State & s, NoiseGenerator & ng)
        : StampedData(imu_t)
    {
        time = t;
        imumsg.header.stamp = ros::Time(t);
        imumsg.header.frame_id = frame_id;
        imumsg.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, 0, s.th);
        imumsg.angular_velocity.z = s.w + ng();
        imumsg.linear_acceleration.x = s.a + ng();
        imumsg.linear_acceleration.y = s.v * s.w + ng();
    }

    static vector<ImuData> generate(const vector<State> & states)
    {
        ros::NodeHandle nh("~");

        double noiseLevel;
        nh.param<double>("noiseLevel", noiseLevel, DEFAULT_NOISE_LEVEL);
        nh.param<double>("imuNoiseLevel", noiseLevel, noiseLevel);
        NoiseGenerator ng(noiseLevel);

        double startTime, period, imuPeriod;
        nh.param<double>("period", period, DEFAULT_PERIOD);
        nh.param<double>("imuPeriod", imuPeriod, period);
        if( imuPeriod<period )
            ROS_WARN("Imu period < period. This is not recommended.");
        period = imuPeriod;
        nh.param<double>("imuStartTime", startTime, period);
        double t = startTime==0.0 ? period : startTime;

        string frameID = "/base_link";
        vector<ImuData> data;
        unsigned i=0;
        while( true ) {
            i = find(states, i, t);
            if( i==states.size() )
                break;
            data.push_back( ImuData(t, frameID, states[i], ng) );
            //cout <<"Generating: " <<data.size() <<"(" <<i <<"/" <<states.size() <<")" <<endl;
            t += period;
        }
        return data;
    }
};


class OdoData : public StampedData
{
public:
    lowlevel::Encoders encodermsg;

    OdoData() : StampedData(odo_t) { }

    OdoData(double t, const State & s, NoiseGenerator & ng, double period)
        : StampedData(odo_t)
    {
        time = t;
        double v = s.v + ng();
        double w = s.w + ng();

        encodermsg.stamp = ros::Time(t);
        encodermsg.dt = period;
        encodermsg.v = v;
        encodermsg.w = w;
        encodermsg.d_dist = v*period;
        encodermsg.d_th = w*period;
    }

    static vector<OdoData> generate(const vector<State> & states)
    {
        ros::NodeHandle nh("~");

        double noiseLevel;
        nh.param<double>("noiseLevel", noiseLevel, DEFAULT_NOISE_LEVEL);
        nh.param<double>("encNoiseLevel", noiseLevel, noiseLevel);
        NoiseGenerator ng(noiseLevel);

        double startTime, period, encPeriod;
        nh.param<double>("period", period, DEFAULT_PERIOD);
        nh.param<double>("encPeriod", encPeriod, period);
        if( encPeriod<period )
            ROS_WARN("Encoders period < period. This is not recommended.");
        period = encPeriod;
        nh.param<double>("encStartTime", startTime, period);
        double t = startTime==0.0 ? period : startTime;

        vector<OdoData> data;
        unsigned i=0;
        while( true ) {
            i = find(states, i, t);
            if( i==states.size() )
                break;
            data.push_back( OdoData(t, states[i], ng, period) );
            //cout <<"Generating: " <<data.size() <<"(" <<i <<"/" <<states.size() <<")" <<endl;
            t += period;
        }
        return data;
    }
};


class ProfileStep
{
public:
    double duration;
    double a;
    double dw;
    ProfileStep(double _duration, double _a, double _dw) : duration(_duration), a(_a), dw(_dw) { }
};

class Profile
{
    ///This contains the definition of the profile (see add() )
    vector<ProfileStep> steps;

    /// This will contain the corresponding states after generateData() is called.
    vector<State> states;

    /// This will contain the IMU data after generateData() is called.
    vector<ImuData> imudata;

    /// This will contain the odo data (simulated encoders) after generateData() is called.
    vector<OdoData> ododata;

    /// This will contain pointers to all the data, sorted by time, after
    /// generateData() is called.
    vector<StampedData *> data;

    /// A helper function to compute the states from the profile. Read all the
    /// profile steps and integrate the accelerations.
    void computeStates()
    {
        ros::NodeHandle nh("~");
        double period;
        nh.param<double>("period", period, DEFAULT_PERIOD);

        unsigned i = 0; //current profile step
        double t = period; //time counter
        double T = steps[0].duration;

        states.clear();
        State s;

        while( true ) {
            while( t<T ) {
                //cout <<"t=" <<t <<", i=" <<i <<endl;
                t = s.integrate(period, steps[i].a, steps[i].dw);
                states.push_back(s);
            }
            if( ++i < steps.size() )
                T += steps[i].duration;
            else
                break;
        }
    }

    static bool comp (const StampedData *a, const StampedData *b)
    {
        return a->time < b->time;
    }


public:
    /// This function can be used to create the profile, by adding acceleration
    /// profiles bits by bits (duration, acceleration, angular acceleration).
    void add(double duration, double acc, double dw)
    {
        steps.push_back( ProfileStep(duration,acc,dw) );
    }

    /// Generate all the data, sort it, and return it.
    const vector<StampedData *> & generateData()
    {
        data.clear();

        computeStates();
        //cout <<"Computed " <<states.size() <<" states" <<endl;
        for( unsigned i=0; i<states.size(); i++ ) data.push_back( &states[i] );

        imudata = ImuData::generate(states);
        //cout <<"Computed " <<imudata.size() <<" imu data" <<endl;
        for( unsigned i=0; i<imudata.size(); i++ ) data.push_back( &imudata[i] );

        ododata = OdoData::generate(states);
        //cout <<"Computed " <<ododata.size() <<" odo data" <<endl;
        for( unsigned i=0; i<ododata.size(); i++ ) data.push_back( &ododata[i] );

        std::sort(data.begin(), data.end(), comp );

        return data;
    }
};


Profile makeLoopProfile()
{
    Profile profile;
    double dw = M_PI/60;

    profile.add(60,0,0); //stand still for the first 60 secs
    // start first loop
    profile.add(2,0.5,0); //acceleration for 2 secs --> 0 to 1 m/s in 1 m
    profile.add(29,0,0); //maintain for 29 secs --> 29 m
    profile.add(2,0,dw); //start turning to the left
    profile.add(28,0,0); //maintain for 28 secs
    profile.add(2,0,-dw); //stop turning
    profile.add(60,0,0); //maintain for 60 secs
    profile.add(2,0,dw); //start turning to the left
    profile.add(28,0,0); //maintain for 28 secs
    profile.add(2,0,-dw); //stop turning
    profile.add(30,0,0); //maintain for 30 secs
    // one loop
    profile.add(30,0,0); //maintain for 30 secs
    profile.add(2,0,dw); //start turning to the left
    profile.add(28,0,0); //maintain for 28 secs
    profile.add(2,0,-dw); //stop turning
    profile.add(60,0,0); //maintain for 60 secs
    profile.add(2,0,dw); //start turning to the left
    profile.add(28,0,0); //maintain for 28 secs
    profile.add(2,0,-dw); //stop turning
    profile.add(30,0,0); //maintain for 30 secs
    // one loop
    profile.add(30,0,0); //maintain for 30 secs
    profile.add(2,0,dw); //start turning to the left
    profile.add(28,0,0); //maintain for 28 secs
    profile.add(2,0,-dw); //stop turning
    profile.add(60,0,0); //maintain for 60 secs
    profile.add(2,0,dw); //start turning to the left
    profile.add(28,0,0); //maintain for 28 secs
    profile.add(2,0,-dw); //stop turning
    profile.add(29,0,0); //maintain for 29 secs
    // stop
    profile.add(2,-0.5,0);
    profile.add(10,0,0); //stand still for 10 secs

    return profile;
}


class SensorPublisher
{
private:
    ros::NodeHandle n;
    ros::Publisher imuPub;
    ros::Publisher clockPub;
    ros::Publisher encodersPub;
    ros::Publisher groundTruthOdometryPub;
    tf::TransformBroadcaster groundTruthTfBroadcaster;
    rosbag::Bag bag;
    bool realtime;
    double period;

    void publish(const State *s)
    {
        if( realtime )
        {
            groundTruthOdometryPub.publish( s->odometryMsg() );
            groundTruthTfBroadcaster.sendTransform( s->tf() );
            ros::WallDuration(0.01).sleep();
        }
        else
        {
            bag.write("/ground_truth", ros::Time(s->time), s->odometryMsg() );
        }
    }

    void publish(const ImuData *d)
    {
        if( realtime )
            imuPub.publish( d->imumsg );
        else
            bag.write("/ms/imu/data", ros::Time(d->time), d->imumsg);
    }

    void publish(const OdoData *d)
    {
        if( realtime )
            encodersPub.publish( d->encodermsg );
        else
            bag.write("/encoders", ros::Time(d->time), d->encodermsg);
    }

public:
    SensorPublisher()
    {
        // Create the ROS topics
        imuPub = n.advertise<sensor_msgs::Imu>("/ms/imu/data", 0);
        clockPub = n.advertise<rosgraph_msgs::Clock>("/clock", 0);
        encodersPub = n.advertise<lowlevel::Encoders>("/encoders", 0);
        groundTruthOdometryPub= n.advertise<nav_msgs::Odometry>("/ground_truth", 0);
        realtime = true;
        n.param<double>("period", period, DEFAULT_PERIOD);
    }

    SensorPublisher(string name)
    {
        bag.open(name, rosbag::bagmode::Write);
        bag.setCompression(rosbag::compression::BZ2);
        realtime = false;
    }

    ~SensorPublisher() { if( !realtime ) bag.close(); }

    void publish(const vector<StampedData *> & data)
    {
        for( unsigned i=0; i<data.size() && ros::ok(); i++ )
        {
            //cout <<"i="<<i <<", time=" <<data[i]->time <<", type=" <<data[i]->typestr() <<endl;
            if( realtime )
            {
                rosgraph_msgs::Clock clockmsg;
                clockmsg.clock = ros::Time(data[i]->time);
                clockPub.publish( clockmsg );
            }

            if( data[i]->type == state_t )
                publish( static_cast<State *>(data[i]) );
            else if( data[i]->type == imu_t )
                publish( static_cast<ImuData *>(data[i]) );
            else if( data[i]->type == odo_t )
                publish( static_cast<OdoData *>(data[i]) );
        }
    }
};


void printHelp()
{
    cout <<"sensorSimulation: publish fake sensor data and ground truth" <<endl;
    cout <<"Options:" <<endl;
    cout <<"\t --help, -h: prints this help message and exit" <<endl;
    cout <<"\t -O bagname: sets to bag mode and the output to the given bag file." <<endl;
    cout <<"\t     In this mode, data is written to a bag instead of being published" <<endl;
    cout <<"\t     in realtime. As a result it goes very fast, but no data is published on any topic." <<endl;
    cout <<endl;
    cout <<"Parameters:" <<endl;
    cout <<"\t period: period of the sensors. Default value is " <<DEFAULT_PERIOD <<'.' <<endl;
    cout <<"\t encPeriod: period for the encoders." <<endl;
    cout <<"\t imuPeriod: period for the IMU." <<endl;
    cout <<endl;
    cout <<"\t encStartTime: starting time for the encoders. Default is 0." <<endl;
    cout <<"\t imuStartTime: starting time for the IMU. Default is 0." <<endl;
    cout <<endl;
    cout <<"\t noiseLevel: default noise level. Default value is " <<DEFAULT_NOISE_LEVEL <<'.' <<endl;
    cout <<"\t encNoiseLevel: noise level for the encoders." <<endl;
    cout <<"\t imuNoiseLevel: noise level for the IMU." <<endl;
    cout <<endl;
    cout <<"Example:" <<endl;
    cout <<"rosrun lowlevel sensorSimulation -O ~/sensorSimulation.bag _period:=0.001 _imuPeriod:=0.005 _encPeriod:=0.004 _imuNoiseLevel:=0.01 _encNoiseLevel:=0.01" <<endl;

}



int main(int argc, char **argv)
{
    // Init ROS
    ros::init(argc, argv, "sensorSimulator");

    // Parse command line arguments
    bool realtime = true;
    string bagname;

    for( int i=1; i<argc; i++ )
    {
        if( strcmp(argv[i],"-O")==0 && i+1<argc ) {
            realtime = false;
            bagname = argv[++i];
        }
        else if( strcmp(argv[i],"--help")==0 || strcmp(argv[i],"-h")==0) {
            printHelp();
            return 0;
        }
    }

    // Generate the data
    Profile profile = makeLoopProfile();
    const vector<StampedData *> & data = profile.generateData();
    ROS_INFO("Generated the data");

    SensorPublisher *publisher;
    if( realtime ) publisher = new SensorPublisher();
    else publisher = new SensorPublisher(bagname);

    // Publish the data
    ROS_INFO("Publishing the data");
    publisher->publish(data);

    delete publisher;

    return 0;
}
