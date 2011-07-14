#include <local_map/local_map.h>

Local_map::Local_map()
{

    // x is forward, y is left
    res = 0.05;
    width = 20.0;
    height = 10.0;
    
    xsize = height/res;
    ysize = width/res;
    
    xorigin = xsize/2.0;
    yorigin = 6*ysize/10.0;

    map = new unsigned char[ xsize*ysize];
}

Local_map::~Local_map()
{
    delete[] map;
}

void Local_map::reinit_map()
{
    memset(map, 0, sizeof(unsigned char)*xsize*ysize);
}

// returns 1 if pose outside the local map
inline int Local_map::get_cell_num(Pose p, int &x, int &y)
{
    float px = p.x[0];
    float py = p.x[1];
    x = px/res + xorigin;
    y = py/res + yorigin;

    if( ( (x > xsize) || (x < 0) ) || ( (y > ysize) || (y < 0) ) )
        return 1;
    else
        return 0;
}

Pose Local_map::get_cell_coor(int x, int y)
{
    Pose pret;
    pret.x[0] = (x - xorigin)*res;
    pret.x[1] = (y - yorigin)*res;

    return pret;
}

void Local_map::reduce_belief()
{
    cout << "reduce belief called" << endl;
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
    for(unsigned int i=0; i< points.size(); i++)
    {
        if( (points[i].x[1] > 5) || (points[i].x[1] < -15) )
        {
            points.erase(points.begin() + i);
        }
        else if(points[i].x[2] < 0.3)
        {
            points.erase(points.begin() + i);
        }
    }
    map_points.push_back(points);
    if(map_points.size() > 100)
    {
        map_points.erase(map_points.begin() );
    }
    
};

// transforms the map forward
inline Pose Local_map::transform_pose(Pose src, Pose trans)
{
    Pose dst;
    
    // rest are zero
    dst.x[0] = src.x[0] - trans.x[0];
    dst.x[1] = src.x[1];
    dst.x[2] = src.x[2];

    return dst;
}

void Local_map::transform_map(Pose prev, Pose curr)
{
    float delx = curr.x[0] - prev.x[0];
    float dely = curr.x[1] - prev.x[1];
    float delth= curr.x[2] - prev.x[2];
    
    // reset local map
    //memset(map, 0, sizeof(unsigned char)*xsize*ysize);
    for(unsigned int i=0; i< map_points.size(); i++)
    {
        for(unsigned int j=0; j< map_points[i].size(); j++)
        {
            map_points[i][j].x[0] = map_points[i][j].x[0] - delx;
            
            Pose ptmp(map_points[i][j].x[0], map_points[i][j].x[1], map_points[i][j].x[2]);
            int xnum, ynum;
            int res = get_cell_num(ptmp, xnum, ynum);
            if(res == 0)
            {
                int map_loc = CELL_LIN(xnum, ynum);
                map[map_loc] = (uint8_t) (min(255.0, (map_points[i][j].x[2]*255.0)));
            }
        }
    }
}

/*
int main()
{
    Local_map lmap;
    Pose p1(10, 10, 0);
    Pose trans(1, 1, 0.2);
    
    //Pose newp = lmap.transform_pose(p1, trans);
    Pose newp = lmap.get_cell_coor(50, 100);
    cout<<"newp: "<< newp.x[0] <<" " << newp.x[1] <<" "<< newp.x[2] << endl;
    
    return 0;
}
*/
