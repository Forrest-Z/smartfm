#include <local_map/local_map.h>

Local_map::Local_map()
{

    // x is forward, y is left
    res = 0.25;
    width = 20.0;
    height = 20.0;
    
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

void Local_map::process_points(vector<Point>& points)
{
    /*
    vector<Point> toput;
    for(unsigned int i=0; i< points.size(); i++)
    {
        float xtmp = points[i].x;
        float ytmp = points[i].y;
        if( sqrt((xtmp- pose.position.x)*(xtmp- pose.position.x) + (ytmp- pose.position.y)*(ytmp- pose.position.y)) > 15)
        {}
        else
            toput.push_back(points[i]);
        else if( points[i].z - pose.position.z < 0.6)
        {
        }
    }
    */
    map_points.clear();
    map_points.push_back(points);
    /*
    if(map_points.size() > 1)
    {
        map_points.erase(map_points.begin());
    }
    */
};

void Local_map::create_map()
{

    // set everything to zero
    memset(map, 0, xsize*ysize);

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
    for(unsigned int i=0; i< curb_points.size(); i++)
    {
        for(unsigned int j=0; j< curb_points[i].points.size(); j++)
        {
            Point ptmp;
            ptmp.x = curb_points[i].points[j].x;
            ptmp.y = curb_points[i].points[j].y;
            ptmp.z = curb_points[i].points[j].z;
            
            int xnum, ynum;
            int res = get_cell_num( ptmp, xnum, ynum);
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


