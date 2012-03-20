#include <local_map/local_map.h>

Local_map::Local_map()
{

    // x is forward, y is left
    res = 0.25;
    width = 30.0;
    height = 30.0;
    
    xsize = (int)height/res;
    ysize = (int)width/res;
    
    xorigin = xsize/2;
    yorigin = ysize/2;

    // init pose in odom frame to (0,0)
    pose.position.x = 0; 
    pose.position.y = 0; 
    pose.position.z = 0; 
    curb_height = 0;
    curb_dist = 0;

    //cout<<"xsize: "<< xsize <<" ysize: "<< ysize << " xorigin: "<< xorigin<<" yorigin: "<< yorigin<<endl;

    points_per_laser = 180;
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

void Local_map::clear_inside_points(float px, float py)
{
    float dist = sqrt((px- pose.position.x)*(px- pose.position.x) + \
            (py - pose.position.y)*(py- pose.position.y));
    int num_steps = dist/res;
    
    Point ptmp;
    int xnum, ynum;
    for(int i=0; i < num_steps; i++)
    {
        ptmp.x = pose.position.x + (px-pose.position.x)*i/(double)num_steps;
        ptmp.y = pose.position.y + (py-pose.position.y)*i/(double)num_steps;
        
        if(get_cell_num(ptmp, xnum, ynum) == 0)
        {
            map[CELL_LIN(xnum, ynum)] = 0;
        }
        else
            break;
    }
}

void Local_map::put_laser(vector<Point> &points)
{
    points_per_laser = points.size();
    map_points.insert(map_points.end(), points.begin(), points.end());
    
    if(map_points.size() > (unsigned int)10*points_per_laser)
    {
        map_points.erase(map_points.begin(), map_points.begin()+points_per_laser);
    }
}

void Local_map::create_map()
{
    // set everything to zero
    memset(map, 0, xsize*ysize);

    ROS_DEBUG("map_size: %d", map_points.size());
    for(unsigned int i=0; i< map_points.size(); i++)
    {
        // put points in gridmap
        Point ptmp;
        ptmp.x= map_points[i].x;
        ptmp.y= map_points[i].y;
        ptmp.z= map_points[i].z;
        //clear_inside_points(ptmp.x, ptmp.y);

        int xnum, ynum;
        int res = get_cell_num(ptmp, xnum, ynum);
        if(res == 0)
        {
            if( (map_points[i].z - pose.position.z) > -0.4 )
            {
                int map_loc = CELL_LIN(xnum, ynum);
                map[map_loc] = 250;
            }
        }
    }
    
    /*
    for(unsigned int i=0; i< left_curb_points.size(); i++)
    {
        for(unsigned int j=0; j< left_curb_points[i].points.size(); j++)
        {
            Point ptmp;
            ptmp.x = left_curb_points[i].points[j].x;
            ptmp.y = left_curb_points[i].points[j].y;
            ptmp.z = left_curb_points[i].points[j].z;
            
            int xnum, ynum;
            int res = get_cell_num( ptmp, xnum, ynum);
            if(res == 0)
            {
                int map_loc = CELL_LIN(xnum, ynum);
                map[map_loc] = 250;
            }
        }
    }
    for(unsigned int i=0; i< right_curb_points.size(); i++)
    {
        for(unsigned int j=0; j< right_curb_points[i].points.size(); j++)
        {
            Point ptmp;
            ptmp.x = right_curb_points[i].points[j].x;
            ptmp.y = right_curb_points[i].points[j].y;
            ptmp.z = right_curb_points[i].points[j].z;
            
            int xnum, ynum;
            int res = get_cell_num( ptmp, xnum, ynum);
            if(res == 0)
            {
                int map_loc = CELL_LIN(xnum, ynum);
                map[map_loc] = 250;
            }
        }
    }
    */
}



void Local_map::put_curbs()
{

}


