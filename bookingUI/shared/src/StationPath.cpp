/** Paths between the different stations are encoded here as lists of waypoints. */

#include <stdlib.h>
#include <math.h>

#include <vector>
#include <iostream>
#include <sstream>

#include "StationPath.h"
#include "SvgPath.h"

using namespace std;


#define VERBOSE 0 //whether to print path coordinates and info about paths (for testing)

//definition of the svg file
//(BRICE) using a more flexible path definition so that it can work on several machines.
//   --> Use a symlink
// TODO: find a more flexible method. For instance, this could be passed at construcion
// time, or rely on an environment variable, etc.
#define SVG_FILE "~/SvgPath/ALL_1215.svg"

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

#include <wordexp.h>
string tilde_expand(const char *path)
{
	wordexp_t p;
	wordexp(path, &p, 0);
	char **w = p.we_wordv;
	assert(p.we_wordc==1);
	string res = w[0];
	wordfree(&p);
	return res;
}

///Stores an array of points into the given path. Point coordinates are
///converted from pixel number to map coordinates.
void storeIntoStationPaths(StationPath *stationpath, const char* id)
{
    StationPath path;
    PathPoint height_width;
    
    try
    {

        SvgPath svg(tilde_expand(SVG_FILE).c_str(), &path, &height_width, id);
        svg.convert_to_meter(&path, height_width, 0.1);
        if( VERBOSE ) cout <<"Path size: " <<path.size() <<". ";
        stationpath->insert(stationpath->end(), path.begin(), path.end());
        if( VERBOSE ) cout <<"Length= " <<stationpath->recomputeLength() <<endl;
    }
    catch (string error)
    {
        cout<<"Error: "<<error<<endl;
    }
}


void printInterPointsDistance()
{
    vector< vector<StationPath> > main_paths;
    main_paths.resize(1);
    main_paths[0].resize(6);

    storeIntoStationPaths(&main_paths[0][0], "DCC_MCD");
    storeIntoStationPaths(&main_paths[0][1], "MCD_DCC");
    storeIntoStationPaths(&main_paths[0][2], "DCC_EA");
    storeIntoStationPaths(&main_paths[0][3], "EA_DCC");
    storeIntoStationPaths(&main_paths[0][4], "E3A_EA");
    storeIntoStationPaths(&main_paths[0][5], "EA_E3A");

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


void storeFromMultipleSVGs(StationPath *stationpath, const char *svg_file1, const char *svg_file2)
{
    storeIntoStationPaths(stationpath, svg_file1);
    storeIntoStationPaths(stationpath, svg_file2);
}

void storeFromMultipleSVGs(StationPath *stationpath, const char *svg_file1, const char *svg_file2, const char *svg_file3)
{
    storeFromMultipleSVGs(stationpath, svg_file1, svg_file2);
    storeIntoStationPaths(stationpath, svg_file3);
}

void storeFromMultipleSVGs(StationPath *stationpath, const char *svg_file1, const char *svg_file2, const char *svg_file3, const char *svg_file4)
{
    storeFromMultipleSVGs(stationpath, svg_file1, svg_file2, svg_file3);
    storeIntoStationPaths(stationpath, svg_file4);
}

StationPaths::StationPaths()
{
    const unsigned NSTATIONS = knownStations_.size();
    stationPaths_.resize(NSTATIONS);
    for(unsigned i=0; i<NSTATIONS; i++)
        stationPaths_[i].resize(NSTATIONS);

    storeFromMultipleSVGs(&stationPaths_[0][1], "DCC_START", "DCC_MCD");
    storeFromMultipleSVGs(&stationPaths_[0][2], "DCC_START2", "DCC_EA", "EA_END3","EA_END2");
    storeFromMultipleSVGs(&stationPaths_[0][3], "DCC_START2", "DCC_EA", "EA_E3A");
    
    storeFromMultipleSVGs(&stationPaths_[1][0], "MCD_DCC", "DCC_END");
    storeFromMultipleSVGs(&stationPaths_[1][2], "MCD_DCC", "DCC_EA", "EA_END3", "EA_END2");
    storeFromMultipleSVGs(&stationPaths_[1][3], "MCD_DCC", "DCC_EA", "EA_E3A");
    
    storeFromMultipleSVGs(&stationPaths_[2][0], "EA_START", "EA_DCC", "DCC_END2");
    storeFromMultipleSVGs(&stationPaths_[2][1], "EA_START", "EA_DCC", "DCC_MCD");
    storeFromMultipleSVGs(&stationPaths_[2][3], "EA_START2", "EA_E3A");

    storeFromMultipleSVGs(&stationPaths_[3][0], "E3A_EA", "EA_DCC", "DCC_END2");
    storeFromMultipleSVGs(&stationPaths_[3][1], "E3A_EA", "EA_DCC", "DCC_MCD");
    storeFromMultipleSVGs(&stationPaths_[3][2], "E3A_EA", "EA_END", "EA_END2");
    
    if( VERBOSE ) {
        printInterPointsDistance();
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
