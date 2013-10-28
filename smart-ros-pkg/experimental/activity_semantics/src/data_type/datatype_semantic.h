/*
 * track_data
 * author: Baoxing
 * date:   2013/04/16
 */

#ifndef DATATYPE_SEMANTIC_H
#define DATATYPE_SEMANTIC_H

#define PIC_GXWX(pic, x, scale) (floor( x / scale + 0.5))
#define PIC_GYWY(pic, y, scale) (pic->height -1- floor( y / scale + 0.5))
#define PIC_VALID(pic, i, j) ((i >= 0) && (i < pic->width) && (j >= 0) && (j < pic->height))

#include <ros/ros.h>
#include <vector>

namespace golfcar_semantics{

typedef enum
{
  MOVING,
  STATIC,
  NOISE
} track_type;

//this element is a node on track, which has not only position information, but also speed and moving direction; plus object size information;
class track_element
{
	public:

	//original information recorded directly from pedestrian detection;
	double time;
	double x, y;
	double local_x, local_y;

	//processed information by utilizing original information between nodes;
	double speed;
	double thetha;

	//size information;
	double width, depth;
};

class track_common
{
	public:
	track_type ped_activity;
	double confidence;
	size_t track_length;
	std::vector <track_element> elements;
	//for moving_track, and it should only be "1" or 2";
	int cluster_label;

	//for training purposes;
	int track_label;
};

class pd_track_container
{
	public:
	//more information to add;
	std::vector<track_common> tracks;
};

};

#endif
