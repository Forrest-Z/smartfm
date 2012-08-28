/** Filters out blobs that are outside a given polygon.
 *
 * Subscribes to topic "in" with type vision_motion_detection/Blobs,
 * and republishes valid blobs on topic "out".
 *
 * The polygon is defined by the parameter "poly". If undefined (i.e. null
 * polygon), then accept all blobs.
 */

#include <ros/ros.h>
#include <vision_motion_detection/Blobs.h>
#include <vision_motion_detection/BlobFilter.h>
#include <vision_motion_detection/data_types.h>

BlobFilterPosition blobFilter;
ros::Publisher publisher;

void blobCallback(const vision_motion_detection::Blobs & msg)
{
    XmlRpc::XmlRpcValue xmlrpc;
    if( ros::param::getCached("~poly", xmlrpc) )
    {
        blobFilter.area_.from_XmlRpc(xmlrpc);
        ROS_INFO_STREAM("New poly " <<blobFilter.area_.to_str());
    }
    publisher.publish( blobFilter.filter(msg) );
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "blob_area_filter");
    ros::NodeHandle nh;

    publisher = nh.advertise<vision_motion_detection::Blobs>("out", 10);
    ros::Subscriber sub = nh.subscribe("in", 10, &blobCallback);

    ros::spin();
    return 0;
}
