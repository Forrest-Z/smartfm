/** Filters out blobs that are too small.
 *
 * Subscribes to topic "in" with type infrastructure_road_monitoring/Blobs,
 * and republishes valid blobs on topic "out".
 *
 * Parameter "area" controls the size threshold. Its default value is 500.
 */

#include <ros/ros.h>
#include <infrastructure_road_monitoring/Blobs.h>

#include <dynamic_reconfigure/server.h>
#include <infrastructure_road_monitoring/BlobAreaFilterConfig.h>

#include "BlobFilter.h"
#include "data_msg_conv.h"

BlobFilterArea blobFilter;

ros::Publisher publisher;

void blobCallback(const infrastructure_road_monitoring::Blobs & msg)
{
    infrastructure_road_monitoring::Blobs outmsg;
    outmsg.header = msg.header;
    for(unsigned i=0; i<msg.blobs.size(); i++)
    {
        Blob blob = blobMsgToData(msg.blobs[i], msg.header.stamp.toSec());
        if( blobFilter.check(blob) )
            outmsg.blobs.push_back( blobDataToMsg(blob) );
    }
    publisher.publish(outmsg);
}

void configCallback(infrastructure_road_monitoring::BlobAreaFilterConfig & config, uint32_t level)
{
    blobFilter.threshold_ = config.area_threshold;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "blob_area_filter");
    ros::NodeHandle nh;

    publisher = nh.advertise<infrastructure_road_monitoring::Blobs>("out", 10);
    ros::Subscriber sub = nh.subscribe("in", 10, &blobCallback);

    dynamic_reconfigure::Server<infrastructure_road_monitoring::BlobAreaFilterConfig> server;
    server.setCallback(&configCallback);

    ros::spin();
    return 0;
}
