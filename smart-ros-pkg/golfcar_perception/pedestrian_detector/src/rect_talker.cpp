#include <ros/ros.h>

#include <sensing_on_road/pedestrian_vision_batch.h>
#include <sensing_on_road/pedestrian_vision.h>
#include <sensor_msgs/PointCloud.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "talker");

    ros::NodeHandle n;

    ros::Publisher chatter_pub = n.advertise<sensing_on_road::pedestrian_vision_batch>("pd_vision_batch", 2);
    ros::Publisher pc_pub = n.advertise<sensor_msgs::PointCloud>("points",1);
    ros::Rate loop_rate(40);
    sensor_msgs::PointCloud pc;
    pc.channels.resize(1);
    pc.channels[0].name = "rgb";
    pc.header.frame_id = "map";
    pc.header.stamp = ros::Time::now();
    geometry_msgs::Point32 p;
    pc.points.push_back(p);

    int rgb = 255<<16|255<<8|0;//(right_image.data[row*320+col+2] << 16) | (right_image.data[row*320+col+1] << 8) | right_image.data[row*320+col+0];
    float float_rgb = *(float*)&rgb;
    pc.channels[0].values.push_back(float_rgb);
    while (ros::ok())
    {

        pc_pub.publish(pc);
        sensing_on_road::pedestrian_vision pr;
        pr.x = 0;
        pr.y = 0;
        pr.width = 640;
        pr.height = 360;
        sensing_on_road::pedestrian_vision_batch prs;
        prs.pd_vector.push_back(pr);

        chatter_pub.publish(prs);

        ros::spinOnce();

        loop_rate.sleep();

    }


    return 0;
}

