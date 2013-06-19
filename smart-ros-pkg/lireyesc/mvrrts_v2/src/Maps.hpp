/*! 
 * @file 
 * @brief Maps of the world model. 
 */

#ifndef MAPS_HPP
#define	MAPS_HPP

// Header files: Standard C++
#include <vector>

// Header files: Custom [Luis]
#include "Defs_Rules_of_Road.hpp"
#include "Paths_and_Regions.hpp"

// Header files: ROS
#include <geometry_msgs/Point.h>
#include <nav_msgs/OccupancyGrid.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>

// Header files: ROS custom-made [SMART-FM]
#include <pnc_msgs/local_map.h>

/** 
 * Global or local map generated from an occupancy grid. 
 */

class Map {
    
protected:
    
    const format_t                          car_length, car_width;
    
    const format_t                          car_rear_axis_offset;
    
    const format_t                          car_safety_padding;
    
    const format_t                          carFrame_front_guard;
    
    const format_t                          carFrame_sideLeft;
    
    format_t                                map_origin [3];
    
    format_t                                map_origin_cos_th, map_origin_sin_th;
    
    unsigned int                            grid_width, grid_height;
    
    float                                   grid_resolution;
    
    nav_msgs::OccupancyGrid::_data_type     grid_data;
    
public:
    
    Map();
    
    virtual ~Map();
    
public:
    
    // --------------------------------------------------------------------------------------------
    // PUBLIC MEMBERS
    // --------------------------------------------------------------------------------------------
    
    format_t area() const;
    
    int get_cell_index( const format_t x_coord, const format_t y_coord) const;
    
    bool get_xy_coords( const unsigned int index, 
    
                        format_t& x_coord, 
                        
                        format_t& y_coord) const;
    
    void read_map( const nav_msgs::OccupancyGrid& grid);
    
    void transform_global_to_local( const format_t * state_global, 
    
                                    format_t * state_local) const;
    
    void transform_local_to_global( const format_t * state_local, 
    
                                    format_t * state_global) const;
    
    // --------------------------------------------------------------------------------------------
    // PUBLIC MEMBERS - VIRTUAL AND PURELY VIRTUAL
    // --------------------------------------------------------------------------------------------
    
    virtual bool is_in_collision( const format_t * state, 
    
                                  Subset_of_Sigma * state_label = NULL) const;
    
    virtual void sample_free_space( format_t * state, 
    
                                    const format_t heuristic_rate = 0.0) const = 0;
    
    // --------------------------------------------------------------------------------------------
    // PUBLIC MEMBERS - FOR DEBUGGING PURPOSES
    // --------------------------------------------------------------------------------------------
    
    void DEBUG_PRINT_CAR_FOOTPRINT( const format_t * state) const;
    
    void DEBUG_PRINT_MAP() const;
    
};

class Map_Global : public Map {
    
protected:
    
    DubRegion_Cylindrical                   heuristic_sampling_region;
    
public:
    
    Map_Global();
    
    ~Map_Global();
    
    void sample_free_space( format_t * state, const format_t heuristic_rate = 0.0) const;
    
    void setup_heuristic_sampling_region( const format_t * initial_state, 
    
                                          const DubRegion_Cylindrical& goal_region);
    
};

class Map_Local : public Map {
    
protected:
    
    pnc_msgs::local_map::_free_cells_type   free_cells;
    
public:
    
    Map_Local();
    
    ~Map_Local();
    
public:
    
    bool is_in_collision( const format_t * state, 
            
                          Subset_of_Sigma * state_label = NULL) const;
    
    void read_map( const pnc_msgs::local_map& map);
    
    void sample_free_space( format_t * state, 
    
                            const format_t heuristic_rate = 0.0) const;
    
};

#endif	/* MAPS_HPP */
