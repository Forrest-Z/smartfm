#include "data_msg_conv.h"

infrastructure_road_monitoring::Blob blobDataToMsg(const Blob & data)
{
    infrastructure_road_monitoring::Blob msg;
    blobDataToMsg(&msg, data);
    return msg;
}

void blobDataToMsg(infrastructure_road_monitoring::Blob * msg, const Blob & data)
{
    msg->contour.clear();
    for(unsigned i=0; i<data.contour.size(); i++)
    {
        geometry_msgs::Point p;
        p.x = data.contour[i].x;
        p.y = data.contour[i].y;
        msg->contour.push_back(p);
    }
    msg->centroid.x = data.centroid.x;
    msg->centroid.y = data.centroid.y;
    msg->centroid.z = 0;
}

Blob blobMsgToData(const infrastructure_road_monitoring::Blob & msg, double time)
{
    Blob data;
    blobMsgToData(&data, msg, time);
    return data;
}

void blobMsgToData(Blob * data, const infrastructure_road_monitoring::Blob & msg, double time)
{
    data->contour.clear();
    for(unsigned i=0; i<msg.contour.size(); i++)
    {
        cv::Point p;
        p.x = (int) msg.contour[i].x;
        p.y = (int) msg.contour[i].y;
        data->contour.push_back(p);
    }
    data->centroid.x = (int) msg.centroid.x;
    data->centroid.y = (int) msg.centroid.y;
    data->timestamp = time;
}




infrastructure_road_monitoring::Track trackDataToMsg(const Track & data)
{
    infrastructure_road_monitoring::Track msg;
    trackDataToMsg(&msg, data);
    return msg;
}

void trackDataToMsg(infrastructure_road_monitoring::Track *msg, const Track & data)
{
    msg->id = data.id;
    try { msg->xvel = data.vel_x.value(); } catch(std::runtime_error & e) { msg->xvel=0; }
    try { msg->yvel = data.vel_y.value(); } catch(std::runtime_error & e) { msg->yvel=0; }
    blobDataToMsg(&(msg->blob), *(data.latestObserved()));
}

Track trackMsgToData(const infrastructure_road_monitoring::Track & msg)
{
    Track data;
    trackMsgToData(&data, msg);
    return data;
}

void trackMsgToData(Track * data, const infrastructure_road_monitoring::Track & msg)
{
    data->id = msg.id;
    data->vel_x.value(msg.xvel);
    data->vel_y.value(msg.yvel);
    data->observations.push_back( blobMsgToData(msg.blob, 0) );
}
