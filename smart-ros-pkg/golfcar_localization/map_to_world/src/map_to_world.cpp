/** A node that publishes the robot's position in UTM and lat,lon coordinates.
 *
 * The robot position is first retrieved in the /map frame (via tf). An offset
 * is then applied to obtain the UTM coordinates. The /utm_to_latlon service
 * is used to retrieve latitude and longitude. The result is published on topic
 * world_utm_latlon (type map_to_world/coordinate).
 *
 * The node also broadcasts the transform between the map frame and the world
 * frame. This is a constant transform, defined by the offset.
 *
 * parameters:
 *  - update_frequency (default 20)
 *  - zone (default 48N)
 *  - offset_x (default 363133)
 *  - offset_y (default 143485)
 *  - publish_tf (default true): whether to publish the transform from map to
 *    world
 *
 */


#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

#include <map_to_world/UtmToLatLon.h>
#include <map_to_world/coordinate.h>

class MapToWorld
{
public:
    MapToWorld();

private:
    tf::TransformListener tf_;
    ros::ServiceClient client_;
    ros::Publisher coordinate_pub_;
    ros::Timer timer_;

    std::string zone_;
    double offset_x_, offset_y_;
    bool publish_tf_;

    void timerCallback(const ros::TimerEvent& event);
    bool getRobotGlobalPose(tf::Stamped<tf::Pose>& odom_pose) const;
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

    client_ = nh.serviceClient<map_to_world::UtmToLatLon>("/utm_to_latlon");
    coordinate_pub_ = nh.advertise<map_to_world::coordinate>("/world_utm_latlon", 1);
    timer_ = nh.createTimer(ros::Duration(1.0/frequency),
                            &MapToWorld::timerCallback, this);
}

void MapToWorld::timerCallback(const ros::TimerEvent& event)
{
    //ROS_DEBUG("timer callback");
    tf::Stamped<tf::Pose> robot_pose;
    if(getRobotGlobalPose(robot_pose))
    {
        //ROS_DEBUG("global pose: %f, %f", robot_pose.getOrigin().x(), robot_pose.getOrigin().y());
        map_to_world::UtmToLatLon utmLL;
        utmLL.request.zone = zone_;
        utmLL.request.easting = offset_x_ + robot_pose.getOrigin().x();
        utmLL.request.northing = offset_y_ + robot_pose.getOrigin().y();

        if(client_.call(utmLL))
        {
            //ROS_DEBUG("world coordinates: utm=(%f,%f), geo=(%f,%f)",
            //          utmLL.request.easting, utmLL.request.northing,
            //          utmLL.response.latitude, utmLL.response.longitude);
            map_to_world::coordinate cd;
            cd.zone = zone_;
            cd.easting = utmLL.request.easting;
            cd.northing = utmLL.request.northing;
            cd.latitude = utmLL.response.latitude;
            cd.longitude = utmLL.response.longitude;
            coordinate_pub_.publish(cd);

        }
        else
        {
            ROS_ERROR("Failed to call service utm_to_latlon");
        }
    }

    if(publish_tf_)
    {
        static tf::TransformBroadcaster br;
        tf::Transform transform;
        transform.setOrigin( tf::Vector3(offset_x_, offset_y_, 0.0) );
        transform.setRotation( tf::Quaternion(0, 0, 0, 1));
        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", "world"));
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
    //ros::Time current_time = ros::Time(); //get the latest available transform without checking for the time

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
