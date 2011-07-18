#include <iostream>
#include <stdlib.h>
#include <vector>
#include <math.h>
#include <string.h>
#include <inttypes.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>

using namespace std;
using namespace geometry_msgs;

#define CELL_LIN(x, y)      ((x) + (y)*xsize)

#define REDUCE_BELIEF_DELTA     (1)
#define INCREASE_BELIEF_DELTA   (10)

class Local_map
{
    private:

    public:
        float res;              // resolution, width, height of grid map
        float width;
        float height;
    
        Pose pose;

        int xsize;              // size in meters
        int ysize;
        
        int xorigin;            // origin of the vehicle in grid coords
        int yorigin;            

        Local_map();
        ~Local_map();

        // map of obstacle beliefs
        unsigned char *map;
        vector <vector<Point> > map_points;

        void reinit_map();

        Point get_cell_coor(int x, int y);
        int get_cell_num(Point p, int &x, int &y);
        void process_points( vector<Point> points);
        void create_map();
};

