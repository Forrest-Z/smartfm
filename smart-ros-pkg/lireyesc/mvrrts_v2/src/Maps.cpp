// Header files: Standard C++
#include <iomanip>

// Header files: Custom [Luis]
#include "Maps.hpp"
#include "Defs_Utilities.hpp"

// ------------------------------------------------------------------------------------------------
// MAP : CONSTRUCTOR AND DESTRUCTOR
// ------------------------------------------------------------------------------------------------

Map::Map() : 

    // Defines the physical characteristics of the car
            
    car_length (2.28),
        
    car_width (1.20),
        
    car_rear_axis_offset (0.45),
        
    car_safety_padding (0.20), 
        
    carFrame_front_guard ( car_length - car_rear_axis_offset + car_safety_padding ), 
        
    carFrame_sideLeft ( (car_width / 2.0) + car_safety_padding ), 

    map_origin { 0, 0, 0}, 
        
    map_origin_cos_th (1.0), map_origin_sin_th (0.0), 
        
    grid_width (0), grid_height (0), 
            
    grid_resolution (0.0), 
            
    grid_data () {}
    
Map::~Map() {}

// ------------------------------------------------------------------------------------------------
// MAP : PUBLIC METHODS
// ------------------------------------------------------------------------------------------------

format_t Map::area() const {
    
    if ( grid_width && grid_height && grid_resolution ) {
        
        return grid_width * grid_height * grid_resolution * grid_resolution;
        
    }
    
    else { return 0.0; }
    
}

int Map::get_cell_index( const format_t x_coord, const format_t y_coord) const {
    
    if ( (x_coord >= 0) && (y_coord >= 0) ) {
        
        unsigned int cell_x = x_coord / grid_resolution;
        
        unsigned int cell_y = y_coord / grid_resolution;
        
        if ( (cell_x < grid_width) && (cell_y < grid_height) ) {
            
            return cell_x + (cell_y * grid_width);
            
        }
        
    }
    
    return -1;
    
}

bool Map::get_xy_coords( const unsigned int index, format_t& x_coord, format_t& y_coord) const {
    
    if ( index < grid_width * grid_height ) {
        
        x_coord = ( index % grid_width ) * grid_resolution;
        
        y_coord = ( index / grid_width ) * grid_resolution;
        
        return true;
        
    }
    
    return false;
    
}

void Map::read_map( const nav_msgs::OccupancyGrid& grid) {
    
    grid_width      = grid.info.width;
    
    grid_height     = grid.info.height;
    
    grid_resolution = grid.info.resolution;
    
    grid_data       = grid.data;
    
}

void Map::transform_global_to_local( const format_t * state_global, 
        
                                     format_t * state_local) const {
    
    state_local[0] =   (state_global[0] - map_origin[0]) * map_origin_cos_th 
            
                     + (state_global[1] - map_origin[1]) * map_origin_sin_th;
    
    state_local[1] = - (state_global[0] - map_origin[0]) * map_origin_sin_th 
            
                     + (state_global[1] - map_origin[1]) * map_origin_cos_th;
    
    state_local[2] = unwrap( state_global[2] - map_origin[2] );
    
}

void Map::transform_local_to_global( const format_t * state_local, 
        
                                     format_t * state_global) const {
    
    state_global[0] = map_origin[0] 
            
                    + ( state_local[0] * map_origin_cos_th ) - ( state_local[1] * map_origin_sin_th );
    
    state_global[1] = map_origin[1] 
            
                    + ( state_local[0] * map_origin_sin_th ) + ( state_local[1] * map_origin_cos_th );
    
    state_global[2] = unwrap( map_origin[2] + state_local[2] );
    
}

// ------------------------------------------------------------------------------------------------
// MAP : PUBLIC METHODS - VIRTUAL AND PURELY VIRTUAL
// ------------------------------------------------------------------------------------------------

bool Map::is_in_collision( const format_t * state, Subset_of_Sigma * state_label) const {
    
    // Declares and computes unit vector along direction of angular coordinate of state
    
    format_t cos_th = cos( state[2] ), sin_th = sin( state[2] );
    
    // Declares temp variables
    
    format_t map_frame_x, map_frame_y; int map_cell_index;
    
    bool driving_in_correct_direction = true;
    
    // Iterates through the points in the grid formed by the discretization of 
    // the footprint of the car in the frame of the current state
    
    for ( float stateFrame_x = -(car_rear_axis_offset + car_safety_padding); 
            
            stateFrame_x < carFrame_front_guard; stateFrame_x += grid_resolution ) {
        
        for ( float stateFrame_y = -carFrame_sideLeft; 
                
                stateFrame_y < carFrame_sideLeft; stateFrame_y += grid_resolution ) {
            
            // Transforms coordinates of grid point from base link frame (a.k.a. car frame) 
            // to local map frame
            
            map_frame_x = state[0] + (stateFrame_x * cos_th) + (stateFrame_y * sin_th);
            
            map_frame_y = state[1] - (stateFrame_x * sin_th) + (stateFrame_y * cos_th);
            
            // Checks whether there exists a point in the local map that is adjacent 
            // (in the sense of the local map grid) to the current grid point
            
            map_cell_index = get_cell_index( map_frame_x, map_frame_y);
            
            // If such a point exists then check it for collisions and retrieve its label; otherwise 
            // (i.e. if point lies outside map) then assume state is not in collision
            
            if ( map_cell_index >= 0 ) {
                
                // If the point is not in collision (color zero, a.k.a. pure black) then checks 
                // that color of point matches the direction of the state
                
                if ( grid_data[map_cell_index] ) {
                    
                    driving_in_correct_direction = driving_in_correct_direction && true;
                    
                }
                
                else { return true; }
                
            }
            
        }
        
    }
    
    // If this line is reached then the state is not in collision
    
    if ( state_label && driving_in_correct_direction ) { state_label -> insert_AP(LANE_LEFT); }
    
    return false;
    
}

// ------------------------------------------------------------------------------------------------
// MAP : PUBLIC METHODS - FOR DEBUGGING PURPOSES
// ------------------------------------------------------------------------------------------------

void Map::DEBUG_PRINT_CAR_FOOTPRINT( const format_t* state) const {
    
    // Declares and computes unit vector along direction of angular coordinate of state
    
    format_t cos_th = cos( state[2] ), sin_th = sin( state[2] );
    
    // Declares temp variables
    
    format_t map_frame_x, map_frame_y; int map_cell_index;
    
    std::cout << "------------------------------------------------" << std::endl;
    
    // Iterates through the points in the grid formed by the discretization of 
    // the footprint of the car in the frame of the current state
    
    for ( float stateFrame_x = -(car_rear_axis_offset + car_safety_padding); 
            
            stateFrame_x < carFrame_front_guard; stateFrame_x += grid_resolution ) {
        
        for ( float stateFrame_y = -carFrame_sideLeft; 
                
                stateFrame_y < carFrame_sideLeft; stateFrame_y += grid_resolution ) {
            
            // Transforms coordinates of grid point from base link frame (a.k.a. car frame) 
            // to local map frame
            
            map_frame_x = state[0] + (stateFrame_x * cos_th) + (stateFrame_y * sin_th);
            
            map_frame_y = state[1] - (stateFrame_x * sin_th) + (stateFrame_y * cos_th);
            
            // Checks whether there exists a point in the local map that is adjacent 
            // (in the sense of the local map grid) to the current grid point
            
            map_cell_index = get_cell_index( map_frame_x, map_frame_y);
            
            // If such a point exists
            
            if ( map_cell_index >= 0 ) {
                
                std::cout << std::setw(3);
                
                std::cout << (int) grid_data[map_cell_index];
                
            }
            
        }
        
        std::cout << std::endl;
        
    }
    
    std::cout << "------------------------------------------------" << std::endl;
    
}

void Map::DEBUG_PRINT_MAP() const {
    
    std::cout << "------------------------------------------------" << std::endl;
    
    for ( unsigned int k = 0; k < grid_data.size(); k++) {
        
        switch ( grid_data[k] ) {
            
            case (0) :
                
                std::cout << "X"; break;
            
            default : 
                
                std::cout << std::cout << " "; break;
            
        }
        
        if ( (k+1) % grid_width == 0 ) { std::cout << std::endl; }
        
    }
    
    std::cout << "------------------------------------------------" << std::endl;
    
}

// ------------------------------------------------------------------------------------------------
// MAP_GLOBAL
// ------------------------------------------------------------------------------------------------

Map_Global::Map_Global() : Map() {}

Map_Global::~Map_Global() {}

void Map_Global::sample_free_space( format_t* state, const format_t heuristic_rate) const {
    
    if ( uniform() < heuristic_rate ) {
        
        // Iterates until a collision-free state is found
        
        do {
            
            // Samples uniformly from a circle of radius equal to that of 
            // the heuristic sampling region
            
            format_t rand_radius = heuristic_sampling_region.radius_xy * sqrt( uniform() );
            
            format_t rand_angle = unwrap( 4.0 * M_PI * (uniform() - 0.5) );
            
            // Shifts the samples to the center of the heuristic sampling region
            
            state[0] = heuristic_sampling_region.center[0] + rand_radius * cos(rand_angle);

            state[1] = heuristic_sampling_region.center[1] + rand_radius * sin(rand_angle);
            
            // Samples angular coordinate uniformly at random
            
            state[2] = unwrap( 4.0 * M_PI * (uniform() - 0.5) );
            
        }

        while ( is_in_collision(state) );
        
    }
    
    else {
        
        // Iterates until a collision-free state is found

        do {

            state[0] = uniform() * grid_width * grid_resolution;

            state[1] = uniform() * grid_height * grid_resolution;

            state[2] = unwrap( 4.0 * M_PI * (uniform() - 0.5) );

        }

        while ( is_in_collision(state) );
        
    }
    
}

void Map_Global::setup_heuristic_sampling_region( const format_t * initial_state, 
        
                                                  const DubRegion_Cylindrical& goal_region) {
    
    // Sets up the center of the region to the midpoint between the initial and goal states
    
    heuristic_sampling_region.center[0] = 
            
            initial_state[0] + 0.5 * ( goal_region.center[0] - initial_state[0] );
    
    heuristic_sampling_region.center[1] = 
            
            initial_state[1] + 0.5 * ( goal_region.center[1] - initial_state[1] );
    
    heuristic_sampling_region.center[2] = 0.0;
    
    // Sets up the radii of the region
    
    heuristic_sampling_region.radius_xy = 
            
            vec2d_norm( initial_state, goal_region.center) + goal_region.radius_xy;
    
    heuristic_sampling_region.radius_th = M_PI;
    
}

// ------------------------------------------------------------------------------------------------
// MAP_LOCAL
// ------------------------------------------------------------------------------------------------

Map_Local::Map_Local() : 

    // Calls constructor of parent and initializes all other members to trivial values

    Map(), free_cells(), center_line(), center_line_theta() {}

Map_Local::~Map_Local() {}

bool Map_Local::is_in_collision( const format_t* state, Subset_of_Sigma* state_label) const {
    
    return false;
    
}

void Map_Local::read_map( const pnc_msgs::local_map& map) {
    
    // Copies planar coordinates of origin
    
    map_origin[0] = (float) map.occupancy.info.origin.position.x;
    
    map_origin[1] = (float) map.occupancy.info.origin.position.y;
    
    // Computes angular coordinate of origin
    
    tf::Quaternion q;
    
    tf::quaternionMsgToTF( map.occupancy.info.origin.orientation, q);
    
    double roll = 0, pitch = 0, yaw = 0;
    
    ( tf::Matrix3x3(q) ).getRPY( roll, pitch, yaw); 
    
    map_origin[2] = (float) yaw;
    
    // Computes unit vector along direction of angular coordinate of origin
    
    map_origin_cos_th = cos( map_origin[2] );
    
    map_origin_sin_th = sin( map_origin[2] );
    
    // Copies local map parameters and data
    
    grid_width          = map.occupancy.info.width;
    
    grid_height         = map.occupancy.info.height;
    
    grid_resolution     = map.occupancy.info.resolution;
    
    grid_data           = map.occupancy.data;
    
    free_cells          = map.free_cells;
    
    center_line         = map.centerline;
    
    // Declares points along center line
    
    format_t center_line_pt [2], center_line_pt_next [2];
    
    // Computes center line directions (i.e. angular coordinates of centerline states) 
    // from the differences in the centerline point positions
    
    center_line_theta.clear();
    
    for ( unsigned int index = 0; index < center_line.size() - 1; index++) {
        
        if ( get_xy_coords( center_line[index + 0], 
                            center_line_pt[0], center_line_pt[1]) && 
                
             get_xy_coords( center_line[index + 1], 
                            center_line_pt_next[0], center_line_pt_next[1]) ) {            
            
            center_line_theta.push_back( atan2( center_line_pt_next[1] - center_line_pt[1], 
                                                center_line_pt_next[0] - center_line_pt[0] ) );
            
        }
        
        else {
            
            std::cerr << "In line " <<  __LINE__ << " of " << __FILE__ << ": ";
            std::cerr << "INVALID index value for centerline point" << std::endl;
            exit(EXIT_FAILURE);
            
        }
        
    }
    
}

void Map_Local::sample_free_space( format_t * state, const format_t heuristic_rate) const {
    
    // Declares temp variables
    
    unsigned int center_line_index, local_map_cell_index;
    
    // Chooses whether or not to use heuristics, depending on the heuristic sampling rate
    
    if ( uniform() < heuristic_rate ) {
        
        // Iterates until a collision-free state is found
        
        do {
            
            // Samples a local map cell along the center line
            
            center_line_index = (unsigned int) ( uniform() * center_line.size() );
            
            local_map_cell_index = center_line[ center_line_index ];
            
            // Computes planar coordinates of cell and adds noise
            
            get_xy_coords( local_map_cell_index, state[0], state[1]);
            
            state[0] += (uniform() - 0.5) * grid_resolution;

            state[1] += (uniform() - 0.5) * grid_resolution;
            
            // Samples angular coordinate according to a Gaussian distribution
            
            state[2] = unwrap( gaussian( center_line_theta[center_line_index], M_PI_4) );
            
        }
        
        while ( is_in_collision(state) );
        
    }
    
    else {
        
        // Iterates until a collision-free state is found
        
        do {
            
            // Samples a local map cell in the free space
            
            local_map_cell_index = free_cells[ (unsigned int) ( uniform() * free_cells.size() ) ];
            
            // Computes planar coordinates of cell and adds noise
            
            get_xy_coords( local_map_cell_index, state[0], state[1]);
            
            state[0] += (uniform() - 0.5) * grid_resolution;

            state[1] += (uniform() - 0.5) * grid_resolution;
            
            // Samples angular coordinate uniformly at random
            
            state[2] = unwrap( 4.0 * M_PI * (uniform() - 0.5) );
            
        }
        
        while ( is_in_collision(state) );
        
    }
    
}
