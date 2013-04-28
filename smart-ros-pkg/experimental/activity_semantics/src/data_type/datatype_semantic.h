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

class track_element
{
	public:
	double time;
	double x, y;
	double width, depth;
	double local_x, local_y;
};

class track_common
{
	public:
	track_type ped_activity;
	double confidence;
	size_t track_length;
	std::vector <track_element> elements;
};

class pd_track_container
{
	public:
	//more information to add;
	std::vector<track_common> tracks;
};

};

#endif
