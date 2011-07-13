#include <iostream>
#include <stdlib.h>
#include <vector>
using namespace std;

#define MAP_RES     (0.05)
#define MAP_WIDTH   (5.0)
#define MAP_HEIGHT  (5.0)

#define CELL_LIN(x, y)      ((x) + (y)*ysize)

#define REDUCE_BELIEF_DELTA     (3)
#define INCREASE_BELIEF_DELTA   (3)

struct Point
{
    float x[3];
};
typedef struct Point Point;

class local_map
{
    private:
        int xsize;
        int ysize;

    public:
        
        local_map();
        ~local_map(){};

        unsigned char *map;
        void map_init();
        void map_free();
       
        void reduce_belief();
        void process_points( vector<Point> points);

};
