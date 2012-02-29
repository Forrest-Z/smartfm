#ifndef DATA_MSG_CONV_H_
#define DATA_MSG_CONV_H_

#include "Blob.h"

#include <infrastructure_road_monitoring/Blobs.h>


infrastructure_road_monitoring::Blob blobDataToMsg(const Blob &);
void blobDataToMsg(infrastructure_road_monitoring::Blob *, const Blob &);

Blob blobMsgToData(const infrastructure_road_monitoring::Blob &, double time=0.0);
void blobMsgToData(Blob *, const infrastructure_road_monitoring::Blob &, double time=0.0);

#endif /* DATA_MSG_CONV_H_ */
