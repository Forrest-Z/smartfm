#ifndef GOLFCART_PERCEPTION_OGMAPPING_MAP_H
#define GOLFCART_PERCEPTION_OGMAPPING_MAP_H

#include <stdint.h>

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
typedef struct
{
  int update_times;
  double occupy_probility;

} map_cell_t;


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
  map_cell_t *cells;

  // Max distance at which we care about obstacles, for constructing
  // likelihood field
  double max_occ_dist;

} map_t;


/**************************************************************************
 * Basic map functions
 **************************************************************************/

// Create a new map
map_t *map_alloc(void)
{
  map_t *map;

  map = (map_t*) malloc(sizeof(map_t));

  // Assume we start at (0, 0)
  map->origin_x = 0;
  map->origin_y = 0;

  // Make the size odd
  map->size_x = 0;
  map->size_y = 0;
  map->scale = 0;

  // Allocate storage for main map
  map->cells = (map_cell_t*) NULL;

  return map;
}

// Destroy a map
void map_free(map_t *map)
{
  free(map->cells);
  free(map);
  return;
}


// Get the cell at the given point
map_cell_t *map_get_cell(map_t *map, double ox, double oy, double oa)
{
  int i, j;
  map_cell_t *cell;

  i = MAP_GXWX(map, ox);
  j = MAP_GYWY(map, oy);

  if (!MAP_VALID(map, i, j))
    return NULL;

  cell = map->cells + MAP_INDEX(map, i, j);
  return cell;
}

#endif
