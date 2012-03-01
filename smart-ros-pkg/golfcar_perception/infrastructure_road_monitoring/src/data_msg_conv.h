#ifndef DATA_MSG_CONV_H_
#define DATA_MSG_CONV_H_

#include <infrastructure_road_monitoring/data_def.h>
#include <infrastructure_road_monitoring/Blobs.h>
#include <infrastructure_road_monitoring/Tracks.h>


infrastructure_road_monitoring::Blob blobDataToMsg(const Blob &);
void blobDataToMsg(infrastructure_road_monitoring::Blob *, const Blob &);

Blob blobMsgToData(const infrastructure_road_monitoring::Blob &, double time=0.0);
void blobMsgToData(Blob *, const infrastructure_road_monitoring::Blob &, double time=0.0);



infrastructure_road_monitoring::Track trackDataToMsg(const Track &);
void trackDataToMsg(infrastructure_road_monitoring::Track *, const Track &);

Track trackMsgToData(const infrastructure_road_monitoring::Track &);
void trackMsgToData(Track *, const infrastructure_road_monitoring::Track &);


#endif /* DATA_MSG_CONV_H_ */
