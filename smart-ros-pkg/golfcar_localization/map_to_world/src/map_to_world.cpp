#include "ros/ros.h"
#include <fmutil/UtmToLatLon.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <map_to_world/coordinate.h>

class MapToWorld
{
public:
    MapToWorld();

private:
    tf::TransformListener tf_;
    ros::ServiceClient client_;
    ros::Publisher coordinate_pub_;
    void UpdateLoop(const ros::TimerEvent& event);
    bool getRobotGlobalPose(tf::Stamped<tf::Pose>& odom_pose) const;
    std::string zone_;
    double offset_x_, offset_y_;
    bool publish_tf_;
};

MapToWorld::MapToWorld()
{
    ros::NodeHandle nh("~");
    double frequency;

    nh.param("update_frequency", frequency, 20.0);
    nh.param("zone", zone_, std::string("48N"));
    nh.param("offset_x", offset_x_, 363133.0);
    nh.param("offset_y", offset_y_, 143485.0);
    nh.param("publish_tf", publish_tf_, true);
    client_ = nh.serviceClient<fmutil::UtmToLatLon>("/utm_to_latlon");
    coordinate_pub_ = nh.advertise<map_to_world::coordinate>("/world_utm_latlon", 1);
    ros::Timer timer = nh.createTimer(ros::Duration(1.0/frequency),&MapToWorld::UpdateLoop,this);
    ros::spin();
}

void MapToWorld::UpdateLoop(const ros::TimerEvent& event)
{
    tf::Stamped<tf::Pose> robot_pose;
    if(getRobotGlobalPose(robot_pose))
    {
        fmutil::UtmToLatLon utmLL;
        utmLL.request.zone = zone_;
        utmLL.request.easting = offset_x_ + robot_pose.getOrigin().x();
        utmLL.request.northing = offset_y_ + robot_pose.getOrigin().y();

        if(client_.call(utmLL))
        {
            map_to_world::coordinate cd;
            cd.zone = zone_;
            cd.easting = utmLL.request.easting;
            cd.northing = utmLL.request.northing;
            cd.latitude = utmLL.response.latitude;
            cd.longtitude = utmLL.response.longitude;
            coordinate_pub_.publish(cd);

            if(publish_tf_)
            {
                static tf::TransformBroadcaster br;
                tf::Transform transform;
                transform.setOrigin( tf::Vector3(cd.easting, cd.northing, 0.0) );
                transform.setRotation( robot_pose.getRotation() );
                br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "map"));
            }
        }
        else
        {
            ROS_ERROR("Failed to call service utm_to_latlon");
        }
    }

}

bool MapToWorld::getRobotGlobalPose(tf::Stamped<tf::Pose>& odom_pose) const
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
  ros::init(argc, argv, "map_to_world");
  ros::NodeHandle n;

  MapToWorld mw;
  ros::spin();

  return 0;
}
