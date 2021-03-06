/** Paths between the different stations are encoded here as lists of waypoints. */

#include <stdlib.h>
#include <math.h>

#include <vector>
#include <iostream>
#include <sstream>

#include "StationPath.h"


using namespace std;


#define VERBOSE 0 //whether to print path coordinates and info about paths (for testing)



double PathPoint::distance(const PathPoint &p1, const PathPoint &p2)
{
    return sqrt( pow(p1.x_-p2.x_,2) + pow(p1.y_-p2.y_,2) );
}



double StationPath::recomputeLength()
{
    length_ = 0.0;
    for( unsigned i=1; i<size(); i++ )
        length_ += PathPoint::distance(this->at(i-1), this->at(i));
    return length_;
}


#define /*7*/DCC_MCD {1046,2220},{1066,2271},{1448,3116},{1575, 3153}, {1600,3218}, {1566, 3278}, {1496,3266}//{1471,3094},{1526,3108},{1590,3130},{1608,3185},{1574,3230},{1526,3229}
#define /*8*/MCD_DCC/* {1526,3229},{1490,3177},{1430,3062},{1118,2406},*/{1464,3215}, {1447, 3116}, {1382,3029}, {1169,2556},{1104,2348},{1066,2271},{1046,2220}

#define /*33*/DCC_EA {1014,2178},{970,2109},{930,2060},{882,2024},{828,2004},{768,2000},{717,2012},{675,2033},{631,2068},{601,2100},{566,2147},{536,2192},{517,2239},{507,2271},{469,2318},{412,2334},{363,2347},{324,2331},{302,2285},{300,2216},{346,1722},{374,1224},{386,1165},{489,856},{563,672},{597,588},{641,493},{724,343},{781,245},{801,221},{832,206},{894,217},{1171,316}
#define /*33*/EA_DCC {1008, 276}, {1006,284},{960,266},{900,248},{853,233},{817,244},{794,266},{776,294},{746,353},{675,481},{639,561},{588,683},{497,912},{410,1175},{399,1229},{374,1721},{325,2217},{334,2270},{368,2296},{420,2293},{472,2250},{522,2186},{558,2131},{588,2084},{622,2047},{663,2010},{720,1986},{776,1980},{833,1982},{892,2008},{943,2044},{983,2099},{1009,2130}
#define /*6*/E3A_EA {1833,650},{1800,610},{1789,573},{1740,545},{1415,434},{1298,400}
#define /*8*/EA_E3A {1423,415},{1856,563},{1938,596},{1966,619},{1976,654},{1948,682},/*{1946,676},*/{1909,682},{1868,670}

int dcc_mcd[][2] = {DCC_MCD};
int mcd_dcc[][2] = {MCD_DCC};
int dcc_ea[][2] = {DCC_EA};
int ea_dcc[][2] = {EA_DCC};
int e3a_ea[][2] = {E3A_EA};
int ea_e3a[][2] = {EA_E3A};
// main route
//dcc to mcd: {1046, 2220}, {1066, 2271}, {1446,3054}, {1526,3108}, {1596, 3130}, {1608, 3185}, {1573, 3221}, {1526, 3220}


int path_points_01[][2] = {{1063, 2175}, DCC_MCD};
int path_points_02[][2] = {{1063, 2175}, DCC_EA, {1223, 339}, {1236, 389}, {1206, 442}, {1170, 432}, {1116, 412}};
int path_points_03[][2] = {{1063, 2175}, DCC_EA, EA_E3A};
int path_points_10[][2] = { MCD_DCC , {1014, 2178}, {1031, 2141}, {1064, 2134}, {1077, 2159}};
int path_points_12[][2] = { MCD_DCC, DCC_EA, {1223, 339}, {1236, 389}, {1206, 442}, {1170, 432}, {1116, 412}};
int path_points_13[][2] = { MCD_DCC, DCC_EA, EA_E3A };
int path_points_20[][2] = {{1079, 394}, {1054, 348}, {1036, 304}, EA_DCC, {1053, 2122}, {1077, 2159}};
int path_points_21[][2] = {{1079, 394}, {1054, 348}, {1036, 304}, EA_DCC, DCC_MCD};
int path_points_23[][2] = {{1079, 394}, {1054, 348}, {1090, 301}, EA_E3A };
int path_points_30[][2] = { E3A_EA, EA_DCC, {1053, 2122}, {1077, 2159}};
int path_points_31[][2] = { E3A_EA, EA_DCC, DCC_MCD};
int path_points_32[][2] = {E3A_EA, {1206, 442}, {1172, 432}, {1116, 412}};

#define NPTS(v) sizeof(v)/sizeof(v[0])


///Stores an array of points into the given path. Point coordinates are
///converted from pixel number to map coordinates.
void storeIntoStationPaths(int path_points[][2], StationPath *path, unsigned size, bool pixel=false)
{
    double res = 0.1;
    int y_pixels = 3536;
    if( VERBOSE ) cout <<"Path size: " <<size <<". ";
    for(unsigned i=0; i<size; i++)
    {
        PathPoint p;
        if(!pixel)
        {
            p.x_ = path_points[i][0]*res;
            p.y_ = (y_pixels - path_points[i][1])*res;
        }
        else
        {
            p.x_ = path_points[i][0];
            p.y_ = path_points[i][1];
        }
        path->push_back(p);
    }
    if( VERBOSE ) cout <<"Length= " <<path->recomputeLength() <<endl;
}


void printInterPointsDistance(bool PixelUsed_=false)
{
    vector< vector<StationPath> > main_paths;
    main_paths.resize(1);
    main_paths[0].resize(6);

    storeIntoStationPaths(dcc_mcd, &main_paths[0][0], NPTS(dcc_mcd), PixelUsed_);
    storeIntoStationPaths(mcd_dcc, &main_paths[0][1], NPTS(mcd_dcc), PixelUsed_);
    storeIntoStationPaths(dcc_ea, &main_paths[0][2], NPTS(dcc_ea), PixelUsed_);
    storeIntoStationPaths(ea_dcc, &main_paths[0][3], NPTS(ea_dcc), PixelUsed_);
    storeIntoStationPaths(e3a_ea, &main_paths[0][4], NPTS(e3a_ea), PixelUsed_);
    storeIntoStationPaths(ea_e3a, &main_paths[0][5], NPTS(ea_e3a), PixelUsed_);

    for(unsigned i=0; i<6; i++)
    {
        cout <<endl;
        for(unsigned j=1; j<main_paths[0][i].size(); j++) {
            double d = PathPoint::distance(main_paths[0][i][j-1], main_paths[0][i][j]);
            cout <<d <<endl;
        }
        cout <<endl;
    }
}


void printPath(const StationPath &p)
{
    for(unsigned i=0; i<p.size(); i++)
        cout <<p[i].x_ <<',' <<p[i].y_ <<'\t';
    cout <<endl;
}


void addPointsInPath(StationPath *p)
{
    const double max_straight = 5;
    StationPath &path(*p);

    if(path.size()<2)
        return;

    for(StationPath::iterator it = path.begin(); it<path.end()-1; it++)
    {
        double dist = PathPoint::distance(*it, *(it+1));
        //cout<<dist<<endl;
        if(dist>max_straight)
        {
            StationPath::iterator startAddPoint = it;
            //cout<<endl<<"Start x y "<< it->x<<' '<<it->y<<" End x y "<< (it+1)->x<<' '<<(it+1)->y<<":"<<endl;
            double inc = max_straight/dist;
            //cout<<"adding segment with inc "<<inc<<" with distance of "<<dist<<"path size "<<path.size()<<endl;
            double x_1 = it->x_, y_1 = it->y_;
            double x_2 = (it+1)->x_, y_2 = (it+1)->y_;
            for(double t=inc; t<1; t+=inc)
            {
                it++;
                PathPoint p;
                p.x_ = x_1 + t * (x_2 - x_1);
                p.y_ = y_1 + t * (y_2 - y_1);
                it = path.insert(it, p);

                //cout<<p.x<<' '<<p.y<<'\t';
            }
            //cout<<endl;
            while(startAddPoint<=it+1)
            {
                //cout<<(*startAddPoint).x <<' '<<(*startAddPoint).y<<endl;
                startAddPoint++;
            }
        }
    }
    path.recomputeLength();
}

StationPaths::StationPaths()
 {
        PixelUsed_=false;
        IniPaths();
 }
 
 StationPaths::StationPaths(bool PixelUsed)
 {
        PixelUsed_=PixelUsed;
        IniPaths();
 }
 
void StationPaths::IniPaths()
{
    const unsigned NSTATIONS = knownStations_.size();
    stationPaths_.resize(NSTATIONS);
    for(unsigned i=0; i<NSTATIONS; i++)
        stationPaths_[i].resize(NSTATIONS);
    
    storeIntoStationPaths(path_points_01, &stationPaths_[0][1], NPTS(path_points_01), PixelUsed_);
    storeIntoStationPaths(path_points_02, &stationPaths_[0][2], NPTS(path_points_02), PixelUsed_);
    storeIntoStationPaths(path_points_03, &stationPaths_[0][3], NPTS(path_points_03), PixelUsed_);
    storeIntoStationPaths(path_points_10, &stationPaths_[1][0], NPTS(path_points_10), PixelUsed_);
    storeIntoStationPaths(path_points_12, &stationPaths_[1][2], NPTS(path_points_12), PixelUsed_);
    storeIntoStationPaths(path_points_13, &stationPaths_[1][3], NPTS(path_points_13), PixelUsed_);
    storeIntoStationPaths(path_points_20, &stationPaths_[2][0], NPTS(path_points_20), PixelUsed_);
    storeIntoStationPaths(path_points_21, &stationPaths_[2][1], NPTS(path_points_21), PixelUsed_);
    storeIntoStationPaths(path_points_23, &stationPaths_[2][3], NPTS(path_points_23), PixelUsed_);
    storeIntoStationPaths(path_points_30, &stationPaths_[3][0], NPTS(path_points_30), PixelUsed_);
    storeIntoStationPaths(path_points_31, &stationPaths_[3][1], NPTS(path_points_31), PixelUsed_);
    storeIntoStationPaths(path_points_32, &stationPaths_[3][2], NPTS(path_points_32), PixelUsed_);


    if( VERBOSE ) {
        printInterPointsDistance(PixelUsed_);
        printPath(stationPaths_[0][2]);
    }

    for(unsigned i=0; i<NSTATIONS;i++)
        for(unsigned j=0; j<NSTATIONS; j++)
            addPointsInPath( & stationPaths_[i][j] );

    if( VERBOSE ) printPath(stationPaths_[0][2]);
}

const StationPath & StationPaths::getPath(const Station & pickup, const Station & dropoff) const
{
    if( ! knownStations_.exists(pickup) )
        throw StationDoesNotExistException("StationPaths::getPath: Invalid pickup station \"" + pickup.str() + "\"");
    if( ! knownStations_.exists(dropoff) )
        throw StationDoesNotExistException("StationPaths::getPath: Invalid dropoff station \"" + dropoff.str() + "\"");
    return stationPaths_[pickup.number()][dropoff.number()];
}
