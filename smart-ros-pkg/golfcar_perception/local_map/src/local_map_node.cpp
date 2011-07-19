#include <local_map/local_map.h>
#include <local_map/local_map_msg.h>

#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <sensor_msgs/LaserScan.h>
#include <laser_geometry/laser_geometry.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf/message_filter.h>
#include <sensor_msgs/PointCloud.h>
#include <message_filters/subscriber.h>
#include <nav_msgs/Odometry.h>

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

        int laser_skip_;
        int laser_count_;
        Pose odom_now_;
        
        void publish_msg();
        void odomCallBack(const nav_msgs::Odometry::ConstPtr &odom_in);
        void on_sick(const sensor_msgs::LaserScan::ConstPtr &msg);
        tf::TransformListener tf_;
        tf::MessageFilter<sensor_msgs::LaserScan> *tf_filter_;
        message_filters::Subscriber<sensor_msgs::LaserScan> laser_scan_sub_;

        ros::Publisher map_pub_;
        ros::Publisher map_points_pub_;
        ros::Publisher grid_pub;
};

local_map_node::local_map_node()
{
    odom_sub_ = n_.subscribe("odom",1, &local_map_node::odomCallBack, this);
    map_points_pub_ = n_.advertise<sensor_msgs::PointCloud>("localPoints",1);
    map_pub_ = n_.advertise<local_map::local_map_msg>("localCells",1);
    grid_pub = n_.advertise<sensor_msgs::PointCloud>("localgridsent", 1);

    laser_scan_sub_.subscribe(n_, "/sick_scan", 1);
    tf_filter_ = new tf::MessageFilter<sensor_msgs::LaserScan>(laser_scan_sub_, tf_, "odom", 5);
    tf_filter_->registerCallback(boost::bind(&local_map_node::on_sick, this, _1));
    tf_filter_->setTolerance(ros::Duration(0.05));
    
    laser_skip_ = 10;
    laser_count_ = 0;
}

local_map_node::~local_map_node()
{
}

void local_map_node::odomCallBack(const nav_msgs::Odometry::ConstPtr &odom_in)
{
    laser_count_++;

    odom_now_.position = odom_in->pose.pose.position;
    odom_now_.orientation = odom_in->pose.pose.orientation;
    
    // copy current pose in lmap to process points
    lmap.pose = odom_now_;
}

void local_map_node::on_sick(const sensor_msgs::LaserScan::ConstPtr &scan_in)
{
    laser_count_++;

    sensor_msgs::PointCloud cloud;
    laser_geometry::LaserProjection projector;

    try
    {
        projector.transformLaserScanToPointCloud("odom", *scan_in, cloud,tf_);
    }
    catch (tf::TransformException& e)
    {
        std::cout << e.what();
        return;

    }
    
    // process points
    vector<Point> points;
    points.resize(cloud.points.size());
    for(unsigned int i=0; i<cloud.points.size(); i++)
    {
        points[i].x=cloud.points[i].x;
        points[i].y=cloud.points[i].y;
        points[i].z=cloud.points[i].z;
    }
    lmap.process_points(points);
    
    if(laser_count_ % laser_skip_ == 0)
    {
        laser_count_ = 0;
        
        //call function to update the local map
        lmap.create_map();
        
        // publish all msgs
        publish_msg();
    }
}

void local_map_node::publish_msg()
{
    // publish point cloud around car
    sensor_msgs::PointCloud pc;
    pc.header.stamp = ros::Time::now();
    pc.header.frame_id = "odom";

    for(unsigned int i=0; i< lmap.map_points.size(); i++)
    {
        for(unsigned int j=0; j< lmap.map_points[i].size(); j++)
        {
            geometry_msgs::Point32 p;
            p.x = lmap.map_points[i][j].x;
            p.y = lmap.map_points[i][j].y;
            p.z = lmap.map_points[i][j].z;

            pc.points.push_back(p);
        }
    }
    map_points_pub_.publish(pc);

    // publish local grid map
    local_map::local_map_msg mtmp;
    mtmp.header.stamp = ros::Time::now();
    mtmp.header.frame_id = "odom";
    mtmp.res = lmap.res;
    mtmp.width = lmap.width;
    mtmp.height = lmap.height;
    mtmp.xsize = lmap.xsize;
    mtmp.ysize = lmap.ysize;
    mtmp.xorigin = lmap.xorigin;
    mtmp.yorigin = lmap.yorigin;
    mtmp.origin.x = lmap.pose.position.x;
    mtmp.origin.y = lmap.pose.position.y;
    mtmp.origin.z = tf::getYaw(lmap.pose.orientation);          // Note .z sends in yaw to the planner
    //cout<<"wrote origin: "<< mtmp.origin.x<<" "<< mtmp.origin.y<<" "<< mtmp.origin.z<< endl;
    for(int i=0; i < lmap.xsize; i++)
    {
        for(int j=0; j< lmap.ysize; j++)
        {
            float tmp = (float)lmap.map[ i + j*lmap.xsize];
            
            //if(tmp > 50.0)
            //    cout<<"1: "<<i<<" "<<j << endl;
            mtmp.vals.push_back(tmp);
        }
    }
    map_pub_.publish(mtmp);

    // publish grid cloud to visualizer
    sensor_msgs::PointCloud gc;
    gc.header.stamp = ros::Time::now();
    gc.header.frame_id = "odom";
    for(int i=0; i < lmap.xsize; i++)
    {
        for(int j=0; j< lmap.ysize; j++)
        {
            if( lmap.map[i + j*lmap.xsize] > 50)
            {
                //cout<<"2: "<<i<<" "<<j << endl;
                geometry_msgs::Point32 p;

                float tmpx = (i - lmap.xorigin)*lmap.res + lmap.pose.position.x;
                float tmpy = (j - lmap.yorigin)*lmap.res + lmap.pose.position.y;
                float tmpz = lmap.pose.position.z;

                //cout<<"tmpx: "<< tmpx<<" "<<tmpy<<endl;
                p.x = tmpx;
                p.y = tmpy;
                p.z = tmpz;

                gc.points.push_back(p);
            }
        }
    }
    //cout << gc.cells.size() << " " << lmap.xsize*lmap.ysize << endl;
    grid_pub.publish(gc);
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "local_map_node");
    ros::NodeHandle n;
    local_map_node lmn;

    ros::spin();

}
