#include <local_map.h>

Local_map::Local_map()
{
    xsize = MAP_WIDTH/MAP_RES;
    ysize = MAP_HEIGHT/MAP_RES;
    
    map = new unsigned char[ xsize*ysize];
    map_copy = new unsigned char[ xsize*ysize];
}

Local_map::~Local_map()
{
    delete[] map;
    delete[] map_copy;
}

void Local_map::reinit_map()
{
    memset(map, 0, xsize*ysize);
    memset(map_copy, 0, xsize*ysize);
}

// returns 1 if pose outside the local map
inline int Local_map::get_cell_num(Pose p, int &x, int &y)
{
    float px = p.x[0];
    float py = p.x[1];
    x = px/MAP_RES + XORIGIN;
    y = py/MAP_RES + YORIGIN;

    if( ( (x > xsize) || (x < 0) ) || ( (y > ysize) || (y < 0) ) )
        return 1;
    else
        return 0;
}

inline Pose Local_map::get_cell_coor(int x, int y)
{
    Pose pret;
    pret.x[0] = (x - XORIGIN)*MAP_RES;
    pret.x[1] = (y - YORIGIN)*MAP_RES;

    return pret;
}

void Local_map::reduce_belief()
{
    for(int i=0; i < xsize; i++)
    {
        for(int j=0; j< ysize; j++)
        {
            unsigned char val = map[ CELL_LIN(i, j) ];
            map[ CELL_LIN(i, j) ] = max(0, val- REDUCE_BELIEF_DELTA);
        }
    }
}

void Local_map::process_points(vector<Pose> points)
{
    reduce_belief();

    int xcell, ycell;
    for(unsigned int i=0; i< points.size(); i++)
    {

        int res = get_cell_num(points[i], xcell, ycell);
        int map_loc = CELL_LIN(xcell, ycell);
        if(res == 0)
        {
            unsigned char val = map[map_loc  ];
            map[ map_loc ] = min(val+INCREASE_BELIEF_DELTA, 255);
        }
    }
};

inline Pose Local_map::transform_pose(Pose src, Pose trans)
{
    Pose dst;
    float cth = cos(trans.x[2]);
    float sth = sin(trans.x[2]);

    dst.x[0] = src.x[0]*cth - src.x[1]*sth + trans.x[0];
    dst.x[1] = src.x[0]*sth + src.x[1]*cth + trans.x[1];

    return dst;
}

void Local_map::transform_map(Pose prev, Pose curr)
{
    float delx = curr.x[0] - prev.x[0];
    float dely = curr.x[1] - prev.x[1];
    float delth= curr.x[2] - prev.x[2];
    
    memcpy(map_copy, map, xsize*ysize);

    for(int i=0; i < xsize; i++)
    {
        for(int j=0; j< ysize; j++)
        {
            Pose curr_point = get_cell_coor(i, j);
            Pose new_point = transform_pose(curr_point, Pose(delx, dely, delth) );
            
            int newx, newy;
            int res = get_cell_num(new_point, newx, newy);
            if(res == 0)
            {
                // copy old val into new map
                map[ CELL_LIN(newx, newy)] = map_copy[ CELL_LIN(i, j) ];     
            }
        }
    }
    memcpy(map_copy, map, xsize*ysize);
}

int main()
{
    Local_map lmap;
    Pose p1(10, 10, 0);
    Pose trans(1, 1, 0.2);

    Pose newp = lmap.transform_pose(p1, trans);
    cout<<"newp: "<< newp.x[0] <<" " << newp.x[1] <<" "<< newp.x[2] << endl;

    return 0;
}

