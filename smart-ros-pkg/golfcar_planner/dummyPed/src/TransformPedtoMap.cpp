
#include <ros/ros.h>
#include <sensing_on_road/pedestrian_laser_batch.h>
#include <tf/transform_listener.h>

class transform_ped {
public:
    transform_ped();
private:
    void PedPoseCallback(sensing_on_road::pedestrian_laser_batch ped);
    void transformPoint(geometry_msgs::PointStamped& in, geometry_msgs::PointStamped& out);
    ros::Publisher transformed_ped_pub_;
    ros::Subscriber ped_sub_;
    tf::TransformListener tf_listener_;

};



transform_ped::transform_ped()
{
    ros::NodeHandle nh;
    transformed_ped_pub_ = nh.advertise<sensing_on_road::pedestrian_laser_batch>("ped_map_laser_batch", 2);
    ped_sub_ = nh.subscribe("ped_laser_batch", 1, &transform_ped::PedPoseCallback, this);
    ros::spin();
}

void transform_ped::transformPoint(geometry_msgs::PointStamped& in, geometry_msgs::PointStamped& out)
{
    try{
        tf_listener_.transformPoint("map", in, out);
    }
    catch(tf::TransformException& ex){
        ROS_ERROR("Received an exception trying to transform a point from \"ldmrs\" to \"map\": %s", ex.what());
    }
}
void transform_ped::PedPoseCallback(sensing_on_road::pedestrian_laser_batch ped)
{
    sensing_on_road::pedestrian_laser_batch ped_map_laser_batch;
    ped_map_laser_batch = ped;
    for(size_t i=0; i<ped.pedestrian_laser_features.size();i++)
    {
        transformPoint(ped.pedestrian_laser_features[i].pedestrian_laser, ped_map_laser_batch.pedestrian_laser_features[i].pedestrian_laser);
    }
    transformed_ped_pub_.publish(ped_map_laser_batch);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "transform_ped");

    transform_ped transform_ped_node;


}

