/** Filters out blobs that are too small.
 *
 * Subscribes to topic "in" with type vision_motion_detection/Blobs,
 * and republishes valid blobs on topic "out".
 *
 * Parameter "area" controls the size threshold. Its default value is 500.
 */

#include <ros/ros.h>
#include <vision_motion_detection/Blobs.h>

#include <dynamic_reconfigure/server.h>
#include <vision_motion_detection/BlobAreaFilterConfig.h>

#include <vision_motion_detection/BlobFilter.h>
#include <vision_motion_detection/data_types.h>

BlobFilterArea blobFilter;

ros::Publisher publisher;

void blobCallback(const vision_motion_detection::Blobs & msg)
{
    publisher.publish( blobFilter.filter(msg) );
}

void configCallback(vision_motion_detection::BlobAreaFilterConfig & config, uint32_t level)
{
    blobFilter.threshold_ = config.area_threshold;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "blob_area_filter");
    ros::NodeHandle nh;

    publisher = nh.advertise<vision_motion_detection::Blobs>("out", 10);
    ros::Subscriber sub = nh.subscribe("in", 10, &blobCallback);

    dynamic_reconfigure::Server<vision_motion_detection::BlobAreaFilterConfig> server;
    server.setCallback(&configCallback);

    ros::spin();
    return 0;
}
