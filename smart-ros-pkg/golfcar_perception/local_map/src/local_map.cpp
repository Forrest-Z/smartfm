#include <local_map/local_map.h>

Local_map::Local_map()
{

    // x is forward, y is left
    res = 0.15;
    width = 15.0;
    height = 15.0;
    
    xsize = height/res;
    ysize = width/res;
    
    xorigin = xsize/2.0;
    yorigin = ysize/2.0;

    // init pose in odom frame to (0,0)
    pose.position.x = 0; 
    pose.position.y = 0; 
    pose.position.z = 0; 
    
    //cout<<"xsize: "<< xsize <<" ysize: "<< ysize << " xorigin: "<< xorigin<<" yorigin: "<< yorigin<<endl;

    map = new unsigned char[ xsize*ysize];
}

Local_map::~Local_map()
{
    //delete[] map;
}

void Local_map::reinit_map()
{
    ;
}

// returns 1 if pose outside the local map
int Local_map::get_cell_num(Point p, int &x, int &y)
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

Point Local_map::get_cell_coor(int x, int y)
{
    Point pret;
    pret.x = (x - xorigin)*res + pose.position.x;
    pret.y = (y - yorigin)*res + pose.position.y;

    return pret;
}

void Local_map::process_points(vector<Point> points)
{
    vector<Point> toput;
    for(unsigned int i=0; i< points.size(); i++)
    {
        float xtmp = points[i].x;
        float ytmp = points[i].y;
        if( sqrt((xtmp- pose.position.x)*(xtmp- pose.position.x) + (ytmp- pose.position.y)*(ytmp- pose.position.y)) > 15)
        {}
        else if( points[i].z - pose.position.z < 0.6)
        {
        }
        else
            toput.push_back(points[i]);
    }
    map_points.push_back(toput);
    if(map_points.size() > 100)
    {
        map_points.erase(map_points.begin());
    }
};

void Local_map::create_map()
{
    // reset local map
    for(int i=0; i< xsize; i++)
    {
        for(int j=0; j< ysize; j++)
            map[i + j*xsize] = 0;
    }

    for(unsigned int i=0; i< map_points.size(); i++)
    {
        for(unsigned int j=0; j< map_points[i].size(); j++)
        {
            // put points in gridmap
            Point ptmp;
            ptmp.x= map_points[i][j].x;
            ptmp.y= map_points[i][j].y;
            ptmp.z= map_points[i][j].z;
            
            int xnum, ynum;
            int res = get_cell_num(ptmp, xnum, ynum);
            if(res == 0)
            {
                //cout<<"xnum: "<< xnum<<" "<<ynum<<endl;
                int map_loc = CELL_LIN(xnum, ynum);
                //cout<<"map_loc: "<< map_loc << " "<<xsize*ysize << endl;
                map[map_loc] = 250;
                //cout<<"accessed map array"<<endl;
            }
        }
    }
}


