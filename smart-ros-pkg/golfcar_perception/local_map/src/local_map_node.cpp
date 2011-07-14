#include <local_map/local_map.h>
#include <local_map/local_map_msg.h>

#include <sensor_msgs/LaserScan.h>
#include <laser_geometry/laser_geometry.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf/message_filter.h>
#include <sensor_msgs/PointCloud.h>
#include <message_filters/subscriber.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/GridCells.h>

using namespace std;

class local_map_node
{		
    public:
        local_map_node();
        ~local_map_node();

    private:
        Local_map lmap;

        ros::NodeHandle n_;
        ros::Subscriber odom_sub_;
        int odom_skip_;
        int odom_count_;
        Pose odom_pre_;
        void odomCallBack(const nav_msgs::Odometry::ConstPtr &odom_in);
        void on_sick(const sensor_msgs::LaserScan::ConstPtr &msg);
        tf::TransformListener tf_;
        tf::MessageFilter<sensor_msgs::LaserScan> *tf_filter_;
        message_filters::Subscriber<sensor_msgs::LaserScan> laser_scan_sub_;

        ros::Publisher map_pub_;
        ros::Publisher map_points_pub_;
};

local_map_node::local_map_node()
{
    Local_map lmap;

    odom_sub_ = n_.subscribe("odom",1, &local_map_node::odomCallBack, this);
    map_points_pub_ = n_.advertise<sensor_msgs::PointCloud>("localPoints",1);
    map_pub_ = n_.advertise<local_map::local_map_msg>("localCells",1);
    
    laser_scan_sub_.subscribe(n_, "/sick_scan", 1);
    tf_filter_ = new tf::MessageFilter<sensor_msgs::LaserScan>(laser_scan_sub_, tf_, "base_link", 10);
    tf_filter_->registerCallback(boost::bind(&local_map_node::on_sick, this, _1));
    tf_filter_->setTolerance(ros::Duration(0.05));
    odom_skip_ = 1;
    odom_count_ = 0;
}

local_map_node::~local_map_node()
{
}

void local_map_node::odomCallBack(const nav_msgs::Odometry::ConstPtr &odom_in)
{

    Pose odom_now;

    odom_now.x[0] = odom_in->pose.pose.position.x;
    odom_now.x[1] = odom_in->pose.pose.position.y;
    odom_now.x[2] = tf::getYaw(odom_in->pose.pose.orientation);

    //call function to update the local map
    lmap.transform_map(odom_pre_, odom_now);

    //then
    odom_pre_ = odom_now;

    // publish point cloud
    sensor_msgs::PointCloud gc;
    gc.header.stamp = ros::Time::now();
    gc.header.frame_id = "base_link";

    for(unsigned int i=0; i< lmap.map_points.size(); i++)
    {
        for(unsigned int j=0; j< lmap.map_points[i].size(); j++)
        {
            geometry_msgs::Point32 p;
            p.x = lmap.map_points[i][j].x[0];
            p.y = lmap.map_points[i][j].x[1];
            p.z = lmap.map_points[i][j].x[2];

            gc.points.push_back(p);
        }

    }
    map_points_pub_.publish(gc);
   
    // publish local_map
    local_map::local_map_msg mtmp;
    mtmp.header.stamp = ros::Time::now();
    mtmp.header.frame_id = "base_link";
    mtmp.res = lmap.res;
    mtmp.width = lmap.width;
    mtmp.height = lmap.height;
    mtmp.xsize = lmap.xsize;
    mtmp.ysize = lmap.ysize;
    mtmp.xorigin = lmap.xorigin;
    mtmp.yorigin = lmap.yorigin;

    for(int i=0; i < lmap.xsize; i++)
    {
        for(int j=0; j< lmap.ysize; j++)
        {
            mtmp.vals.push_back(lmap.map[ i + j*lmap.xsize]);
        }
    }
    map_pub_.publish(mtmp);
}

void local_map_node::on_sick(const sensor_msgs::LaserScan::ConstPtr &scan_in)
{
    sensor_msgs::PointCloud cloud;
    laser_geometry::LaserProjection projector;

    try
    {
        projector.transformLaserScanToPointCloud("base_link",*scan_in, cloud,tf_);
    }
    catch (tf::TransformException& e)
    {
        std::cout << e.what();
        return;

    }
    vector<Pose> points;
    points.resize(cloud.points.size());
    for(unsigned int i=0; i<cloud.points.size(); i++)
    {
        points[i].x[0]=cloud.points[i].x;
        points[i].x[1]=cloud.points[i].y;
        points[i].x[2]=cloud.points[i].z;
    }
    lmap.process_points(points);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "local_map_node");
    ros::NodeHandle n;
    local_map_node lmn;

    ros::spin();

}

