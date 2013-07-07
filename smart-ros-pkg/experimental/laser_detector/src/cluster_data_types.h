/*
 * cluster_data_types.h
 *
 *  Created on: Jul 5, 2013
 *      Author: Shen Xiaotong
 */

#ifndef CLUSTER_DATA_TYPES_H_
#define CLUSTER_DATA_TYPES_H_

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

struct ClusterPoint
{
	PCL_ADD_POINT4D;

	int cluster_id;
	int group_id;

	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

}EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT(ClusterPoint,
		(float, x, x)
		(float, y, y)
		(float, z, z)
		(int, cluster_id, cluster_id)
		(int, group_id, group_id)
)


#endif /* CLUSTER_DATA_TYPES_H_ */
