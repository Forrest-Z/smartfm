#include <iostream>
#include <stdlib.h>
#include <vector>
#include <math.h>
#include <string.h>
using namespace std;

#define MAP_RES     (0.1)
#define MAP_WIDTH   (5.0)
#define MAP_HEIGHT  (10.0)

#define XORIGIN (xsize/2.0)
#define YORIGIN (ysize/4.0)

#define CELL_LIN(x, y)      ((x) + (y)*ysize)

// essentially says p(obs|laser) / p(not obs|laser) = 10
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
        int xsize;
        int ysize;

    public:
        
        Local_map();
        ~Local_map();

        // map of obstacle beliefs
        unsigned char *map;
        unsigned char *map_copy;

        void reinit_map();

        inline int get_cell_num(Pose p, int &x, int &y);
        inline Pose get_cell_coor(int x, int y);
        void reduce_belief();
        void process_points( vector<Pose> points);
        inline Pose transform_pose(Pose src, Pose trans);
        void transform_map(Pose prev, Pose curr);
};
