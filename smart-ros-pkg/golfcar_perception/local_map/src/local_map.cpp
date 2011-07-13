#include <local_map.h>


local_map::local_map()
{
    xsize = MAP_WIDTH/MAP_RES;
    ysize = MAP_HEIGHT/MAP_RES;
}

// returns 1 if point outside the local map
inline int get_cell_num(Point p, int &x, int &y)
{
    float px = Point.x[0];
    float py = Point.x[1];
    x = px/MAP_RES + xsize/2.0;
    y = py/MAP_RES + ysize/2.0;

    if( ( (x > xsize) || (x < 0) ) || ( (y > ysize) || (y < 0) ) )
        return 1;
    else
        return 0;
}

void local_map::map_init()
{
    map = (unsigned char *) calloc(sizeof(unsigned char), (int)(xsize*ysize));
}

void local_map::reduce_belief()
{
    for(int i=0; i < xsize; i++)
    {
        for(int j=0; j< ysize; j++)
        {
            unsigned char val = map[ CELL_LIN(i, j) ];
            map[ CELL_LIN(i, j) ] = MAX(0, val- REDUCE_BELIEF_DELTA);
        }
    }
}

void local_map::process_points(vector<Points> points)
{
    reduce_belief();

    float xcell, ycell;
    for(int i=0; i< points.size(); i++)
    {

        int res = get_cell_num(points[i], xcell, ycell);
        int map_loc = CELL_LIN(xcell, ycell);
        if(res == 0)
        {
            unsigned char val = map[map_loc  ];
            map[ map_loc ] = MIN(val+INCREASE_BELIEF_DELTA, 255);
        }
    }
};

