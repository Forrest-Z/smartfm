#include <iostream>
#include <stdlib.h>
#include <vector>
#include <math.h>
#include <string.h>
using namespace std;

// x is forward, y is left
#define MAP_RES     (0.01)
#define MAP_WIDTH   (20.0)
#define MAP_HEIGHT  (10.0)

#define XORIGIN (xsize/2.0)
#define YORIGIN (6*ysize/10.0)

#define CELL_LIN(x, y)      ((x) + (y)*xsize)

#define REDUCE_BELIEF_DELTA     (1)
#define INCREASE_BELIEF_DELTA   (10)

class Pose
{
    public:
        float x[3];
        Pose(){};
        Pose(float xin, float yin, float thin)
        {
            x[0] = xin;
            x[1] = yin;
            x[2] = thin;
        }
};

class Local_map
{
    private:

    public:
        int xsize;
        int ysize;
        
        Local_map();
        ~Local_map();

        // map of obstacle beliefs
        unsigned char *map;
        vector <vector<Pose> > map_points;

        void reinit_map();

        Pose get_cell_coor(int x, int y);
        inline int get_cell_num(Pose p, int &x, int &y);
        void reduce_belief();
        void process_points( vector<Pose> points);
        inline Pose transform_pose(Pose src, Pose trans);
        void transform_map(Pose prev, Pose curr);
};

