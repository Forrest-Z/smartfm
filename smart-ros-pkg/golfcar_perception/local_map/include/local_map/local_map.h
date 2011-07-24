#include <iostream>
#include <stdlib.h>
#include <vector>
#include <math.h>
#include <string.h>
#include <inttypes.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <sensor_msgs/PointCloud.h>

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
        float curb_height;
        float curb_dist;

        int xsize;              // size in meters
        int ysize;
        
        int xorigin;            // origin of the vehicle in grid coords
        int yorigin;            

        Local_map();
        ~Local_map();

        // map of obstacle beliefs
        unsigned char *map;
        vector <vector<Point> > map_points;
        vector<sensor_msgs::PointCloud> left_curb_points;
        vector<sensor_msgs::PointCloud> right_curb_points;

        void reinit_map();

        Point get_cell_coor(int x, int y);
        int get_cell_num(Point p, int &x, int &y);
        void process_points( vector<Point>& points);
        void create_map();
};

// returns 1 if pose outside the local map
inline int Local_map::get_cell_num(Point p, int &x, int &y)
{
    float px = p.x - pose.position.x;
    float py = p.y - pose.position.y;
    x = px/res + xorigin;
    y = py/res + yorigin;

    if( ( (x >= xsize) || (x < 0) ) || ( (y >= ysize) || (y < 0) ) )
        return 1;
    else
        return 0;
}

inline Point Local_map::get_cell_coor(int x, int y)
{
    Point pret;
    pret.x = (x - xorigin)*res + pose.position.x;
    pret.y = (y - yorigin)*res + pose.position.y;

    return pret;
}

