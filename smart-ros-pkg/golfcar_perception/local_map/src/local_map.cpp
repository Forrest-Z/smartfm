#include <local_map.h>

Local_map::Local_map()
{
    xsize = MAP_HEIGHT/MAP_RES;
    ysize = MAP_WIDTH/MAP_RES;
    
    map = new float[ xsize*ysize];
    map_copy = new float[ xsize*ysize];
}

Local_map::~Local_map()
{
    delete[] map;
    delete[] map_copy;
}

void Local_map::reinit_map()
{
    memset(map, 0, sizeof(float)*xsize*ysize);
    memset(map_copy, 0, sizeof(float)*xsize*ysize);
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

Pose Local_map::get_cell_coor(int x, int y)
{
    Pose pret;
    pret.x[0] = (x - XORIGIN)*MAP_RES;
    pret.x[1] = (y - YORIGIN)*MAP_RES;

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
    /*
    //reduce_belief();
    
    int xcell, ycell;
    for(unsigned int i=0; i< points.size(); i++)
    {

        int res = get_cell_num(points[i], xcell, ycell);
        cout<<"poit rec: " << points[i].x[0] <<" "<<points[i].x[1]<<" "<<points[i].x[2] << " " << xcell<<" "<<ycell<<endl;
        
        int map_loc = CELL_LIN(xcell, ycell);
        if(res == 0)
        {
            unsigned char val = map[map_loc ];
            if(points[i].x[2] > 0.5)
                map[map_loc] = points[i].x[2];
            else
                map[map_loc] = 0;

        }
    }
    */
    
    for(int i=0; i< points.size(); i++)
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
    
    /*
    memcpy(map_copy, map, sizeof(float)*xsize*ysize);

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
                //cout<<"copied: " << i<<" " << j<<" to: "<< newx <<" " << newy << " val: " << map_copy[ CELL_LIN(i, j) ]<<endl;
                map[ CELL_LIN(newx, newy)] = map_copy[ CELL_LIN(i, j) ];     
            }
        }
    }
    memcpy(map_copy, map, sizeof(float)*xsize*ysize);
    */

    for(unsigned int i=0; i< map_points.size(); i++)
    {
        for(unsigned int j=0; j< map_points[i].size(); j++)
        {
            map_points[i][j].x[0] = map_points[i][j].x[0] - delx; 
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
