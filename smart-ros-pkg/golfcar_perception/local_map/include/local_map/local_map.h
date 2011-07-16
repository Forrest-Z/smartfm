#include <iostream>
#include <stdlib.h>
#include <vector>
#include <math.h>
#include <string.h>
#include <inttypes.h>
using namespace std;

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
        float res;
        float width;
        float height;
    
        int xsize;
        int ysize;
        
        int xorigin;
        int yorigin;

        Local_map();
        ~Local_map();

        // map of obstacle beliefs
        unsigned char *map;
        vector <vector<Pose> > map_points;

        void reinit_map();

        Pose get_cell_coor(int x, int y);
        int get_cell_num(Pose p, int &x, int &y);
        void process_points( vector<Pose> points);
        inline Pose transform_pose(Pose src, Pose trans);
        void transform_map(Pose prev, Pose curr);
};

