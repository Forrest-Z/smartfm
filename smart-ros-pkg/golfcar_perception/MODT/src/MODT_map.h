#ifndef GOLFCART_MODT_MAP_H
#define GOLFCART_MODT_MAP_H

#include <vector>
#include <stdint.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>

#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/PointStamped.h>

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

namespace golfcar_perception
{
	using namespace cv;

	typedef struct
	{
		double free_probability;
		double last_update_time;
	} modt_grid;

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
	  modt_grid *cells;

	} modt_map;


	/**************************************************************************
	 * Basic map functions
	 **************************************************************************/
	// Create a new map
	modt_map *map_alloc(void)
	{
	  modt_map *map;
	  map = (modt_map*) malloc(sizeof(modt_map));
	  // Assume we start at (0, 0)
	  map->origin_x = 0;
	  map->origin_y = 0;
	  // Make the size odd
	  map->size_x = 0;
	  map->size_y = 0;
	  map->scale = 0;
	  // Allocate storage for main map
	  map->cells = (modt_grid*) NULL;
	  return map;
	}

	// Destroy a map
	void map_free(modt_map *map)
	{
	  free(map->cells);
	  free(map);
	  return;
	}

	// Get the cell at the given point
	modt_map *map_get_cell(modt_map *map, double ox, double oy, double oa)
	{
	  int i, j;
	  modt_grid *cell;

	  i = MAP_GXWX(map, ox);
	  j = MAP_GYWY(map, oy);

	  if (!MAP_VALID(map, i, j))
		return NULL;

	  cell = map->cells + MAP_INDEX(map, i, j);
	  return cell;
	}

	// Extract a single range reading from the map.  Unknown cells and/or
	// out-of-bound cells are treated as occupied, which makes it easy to
	// use Stage bitmap files.
	double map_calc_range(modt_map *map, double ox, double oy, double oa, double max_range)
	{
	  // Bresenham raytracing
	  int x0,x1,y0,y1;
	  int x,y;
	  int xstep, ystep;
	  char steep;
	  int tmp;
	  int deltax, deltay, error, deltaerr;

	  x0 = MAP_GXWX(map,ox);
	  y0 = MAP_GYWY(map,oy);

	  x1 = MAP_GXWX(map, ox + max_range * cos(oa));
	  y1 = MAP_GYWY(map, oy + max_range * sin(oa));

	  if(abs(y1-y0) > abs(x1-x0))
	    steep = 1;
	  else
	    steep = 0;

	  if(steep)
	  {
	    tmp = x0;
	    x0 = y0;
	    y0 = tmp;

	    tmp = x1;
	    x1 = y1;
	    y1 = tmp;
	  }

	  deltax = abs(x1-x0);
	  deltay = abs(y1-y0);
	  error = 0;
	  deltaerr = deltay;

	  x = x0;
	  y = y0;

	  if(x0 < x1)
	    xstep = 1;
	  else
	    xstep = -1;
	  if(y0 < y1)
	    ystep = 1;
	  else
	    ystep = -1;

	  if(steep)
	  {
	    if(!MAP_VALID(map,y,x) || map->cells[MAP_INDEX(map,y,x)].occ_state > -1)
	      return sqrt((x-x0)*(x-x0) + (y-y0)*(y-y0)) * map->scale;
	  }
	  else
	  {
	    if(!MAP_VALID(map,x,y) || map->cells[MAP_INDEX(map,x,y)].occ_state > -1)
	      return sqrt((x-x0)*(x-x0) + (y-y0)*(y-y0)) * map->scale;
	  }

	  while(x != (x1 + xstep * 1))
	  {
	    x += xstep;
	    error += deltaerr;
	    if(2*error >= deltax)
	    {
	      y += ystep;
	      error -= deltax;
	    }

	    if(steep)
	    {
	      if(!MAP_VALID(map,y,x) || map->cells[MAP_INDEX(map,y,x)].occ_state > -1)
	        return sqrt((x-x0)*(x-x0) + (y-y0)*(y-y0)) * map->scale;
	    }
	    else
	    {
	      if(!MAP_VALID(map,x,y) || map->cells[MAP_INDEX(map,x,y)].occ_state > -1)
	        return sqrt((x-x0)*(x-x0) + (y-y0)*(y-y0)) * map->scale;
	    }
	  }
	  return max_range;
	}
};

#endif
