#ifndef GOLFCART_SEMANTICS_ACTIVITY_MAP_H
#define GOLFCART_SEMANTICS_ACTIVITY_MAP_H

#include <vector>
#include <stdint.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>

#include "datatype_semantic.h"

/**************************************************************************
 * Map manipulation macros
 **************************************************************************/

// Convert from map index to world coords
#define MAP_WXGX(map, i) (map->origin_x + (i) * map->scale)
#define MAP_WYGY(map, j) (map->origin_y + (j) * map->scale)

// Convert from world coords to map coords
#define MAP_GXWX(map, x) (floor((x - map->origin_x) / map->scale + 0.5))
#define MAP_GYWY(map, y) (floor((y - map->origin_y) / map->scale + 0.5))

// Test to see if the given map coords lie within the absolute map bounds.
#define MAP_VALID(map, i, j) ((i >= 0) && (i < map->size_x) && (j >= 0) && (j < map->size_y))

// Compute the cell index for the given map coords.
#define MAP_INDEX(map, i, j) ((i) + (j) * map->size_x)

// Description for a single map cell.

namespace golfcar_semantics
{
	using namespace cv;

	typedef struct
	{
		int track_label;
		double speed, thetha;
		double width, depth;
	} moving_activity;

	typedef struct
	{
		double dwell_time;
		double width, depth;
	} static_activity;

	typedef struct
	{
		bool road_flag;
		bool pedPath_flag;
		bool pedSourceSink_flag;

		//associated tracks, indexed by their serials;
		std::vector<size_t> track_serials;
		//for moving tracks;
		std::vector<moving_activity> moving_activities;
		double avg_speed;
		Vec2d direction_gaussion;
		//std::vector<Vec2d> thetha_Gaussians;

		//for static tracks;
		std::vector<static_activity> static_activities;

		//results from GP;
		Vec2d gp_estimation;

		//distance to the nearest obstacle;
		double obs_dist;

		//direction of its nearest skeleton edge;
		double skel_dist;
		double skel_angle;
		int nearest_edge_ID;

		//weighted times of pedestrian activity;
		double activity_intensity;


		double probe_direction;
		double probe_distance1;
		double probe_distance2;

		double path_score;
		double EE_score;
		double sidewalk_score;
		double crossing_score;

	} activity_grid;


	// Description for a map
	typedef struct
	{
	  // Map origin; the map is a viewport onto a conceptual larger map.
	  double origin_x, origin_y, yaw;
	  // Map scale (m/cell)
	  double scale;
	  // Map dimensions (number of cells)
	  int size_x, size_y;
	  // The map data, stored as a grid
	  activity_grid *cells;

	  Mat* road_image;
	  pd_track_container* pd_container_pointer;

	} activity_map;


	/**************************************************************************
	 * Basic map functions
	 **************************************************************************/
	// Create a new map
	activity_map *map_alloc(void)
	{
		activity_map *map;

	  map = (activity_map*) malloc(sizeof(activity_map));

	  // Assume we start at (0, 0)
	  map->origin_x = 0;
	  map->origin_y = 0;
	  // Make the size odd
	  map->size_x = 0;
	  map->size_y = 0;
	  map->scale = 0;

	  // Allocate storage for main map
	  map->cells = (activity_grid*) NULL;

	  return map;
	}

	// Destroy a map
	void map_free(activity_map *map)
	{
	  free(map->cells);
	  free(map);
	  return;
	}

	// Get the cell at the given point
	activity_grid *map_get_cell(activity_map *map, double ox, double oy, double oa)
	{
	  int i, j;
	  activity_grid *cell;

	  i = MAP_GXWX(map, ox);
	  j = MAP_GYWY(map, oy);

	  if (!MAP_VALID(map, i, j))
		return NULL;

	  cell = map->cells + MAP_INDEX(map, i, j);
	  return cell;
	}

};

#endif
