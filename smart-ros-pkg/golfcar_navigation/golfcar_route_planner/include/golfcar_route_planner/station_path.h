/*
 * station_path.h
 *
 *  Created on: Jul 21, 2011
 *      Author: golfcar
 */

#ifndef STATION_PATH_H_
#define STATION_PATH_H_


#endif /* STATION_PATH_H_ */

#define /*7*/DCC_MCD {1046,2220},{1066,2271},{1448,3116},{1575, 3153}, {1600,3218}, {1566, 3278}, {1496,3266}//{1471,3094},{1526,3108},{1590,3130},{1608,3185},{1574,3230},{1526,3229}
#define /*8*/MCD_DCC/* {1526,3229},{1490,3177},{1430,3062},{1118,2406},*/{1464,3215}, {1447, 3116}, {1382,3029}, {1169,2556},{1104,2348},{1066,2271},{1046,2220}

#define /*33*/DCC_EA {1014,2178},{970,2109},{930,2060},{882,2024},{828,2004},{768,2000},{717,2012},{675,2033},{631,2068},{601,2100},{566,2147},{536,2192},{517,2239},{507,2271},{469,2318},{412,2334},{363,2347},{324,2331},{302,2285},{300,2216},{346,1722},{374,1224},{386,1165},{489,856},{563,672},{597,588},{641,493},{724,343},{781,245},{801,221},{832,206},{894,217},{1171,316}
#define /*33*/EA_DCC {1008, 276}, {1006,284},{960,266},{900,248},{853,233},{817,244},{794,266},{776,294},{746,353},{675,481},{639,561},{588,683},{497,912},{410,1175},{399,1229},{374,1721},{325,2217},{334,2270},{368,2296},{420,2293},{472,2250},{522,2186},{558,2131},{588,2084},{622,2047},{663,2010},{720,1986},{776,1980},{833,1982},{892,2008},{943,2044},{983,2099},{1009,2130}
#define /*6*/E3A_EA {1833,650},{1800,610},{1789,573},{1740,545},{1415,434},{1298,400}
#define /*8*/EA_E3A {1423,415},{1856,563},{1938,596},{1966,619},{1976,654},{1948,682},/*{1946,676},*/{1909,682},{1868,670}
#include <geometry_msgs/Point.h>

using namespace std;

namespace station_path {

int dcc_mcd[][2]{DCC_MCD};
int mcd_dcc[][2]{MCD_DCC};
int dcc_ea[][2]{DCC_EA};
int ea_dcc[][2]{EA_DCC};
int e3a_ea[][2]{E3A_EA};
int ea_e3a[][2]{EA_E3A};
// main route
//dcc to mcd: {1046, 2220}, {1066, 2271}, {1446,3054}, {1526,3108}, {1596, 3130}, {1608, 3185}, {1573, 3221}, {1526, 3220}

int path_points_01[][2] {{1063, 2175}, DCC_MCD};
int path_points_02[][2] {{1063, 2175}, DCC_EA, {1223, 339}, {1236, 389}, {1206, 442}, {1170, 432}, {1116, 412}};
int path_points_03[][2] {{1063, 2175}, DCC_EA, EA_E3A};
int path_points_10[][2] { MCD_DCC , {1014, 2178}, {1031, 2141}, {1064, 2134}, {1077, 2159}};
int path_points_12[][2] { MCD_DCC, DCC_EA, {1223, 339}, {1236, 389}, {1206, 442}, {1170, 432}, {1116, 412}};
int path_points_13[][2] { MCD_DCC, DCC_EA, EA_E3A };
int path_points_20[][2] {{1079, 394}, {1054, 348}, {1036, 304}, EA_DCC, {1053, 2122}, {1077, 2159}};
int path_points_21[][2] {{1079, 394}, {1054, 348}, {1036, 304}, EA_DCC, DCC_MCD};
int path_points_23[][2] {{1079, 394}, {1054, 348}, {1090, 301}, EA_E3A };
int path_points_30[][2] { E3A_EA, EA_DCC, {1053, 2122}, {1077, 2159}};
int path_points_31[][2] { E3A_EA, EA_DCC, DCC_MCD};
int path_points_32[][2] {E3A_EA, {1206, 442}, {1172, 432}, {1116, 412}};

class stationPath
{
public:
    stationPath();
    double square_distance(geometry_msgs::Point p1, geometry_msgs::Point p2);
    void getPath(int stationPickUp, int stationDropOff, vector<geometry_msgs::Point> &path);
    vector<vector<vector<geometry_msgs::Point> > > station_paths;

private:
    void storeIntoStationPaths(int path_points[][2], vector<geometry_msgs::Point> &station_paths, int size);
};



stationPath::stationPath()
{
    const int station_number = 4;
    const int max_straight = 5;
    station_paths.resize(station_number);
    for(int i=0; i<station_number;i++)
        station_paths[i].resize(station_number);

    storeIntoStationPaths(path_points_01, station_paths[0][1],sizeof(path_points_01)/sizeof(path_points_01[0]));
    storeIntoStationPaths(path_points_02, station_paths[0][2],sizeof(path_points_02)/sizeof(path_points_02[0]));
    storeIntoStationPaths(path_points_03, station_paths[0][3],sizeof(path_points_03)/sizeof(path_points_03[0]));
    storeIntoStationPaths(path_points_10, station_paths[1][0],sizeof(path_points_10)/sizeof(path_points_10[0]));
    storeIntoStationPaths(path_points_12, station_paths[1][2],sizeof(path_points_12)/sizeof(path_points_12[0]));
    storeIntoStationPaths(path_points_13, station_paths[1][3],sizeof(path_points_13)/sizeof(path_points_13[0]));
    storeIntoStationPaths(path_points_20, station_paths[2][0],sizeof(path_points_20)/sizeof(path_points_20[0]));
    storeIntoStationPaths(path_points_21, station_paths[2][1],sizeof(path_points_21)/sizeof(path_points_21[0]));
    storeIntoStationPaths(path_points_23, station_paths[2][3],sizeof(path_points_23)/sizeof(path_points_23[0]));
    storeIntoStationPaths(path_points_30, station_paths[3][0],sizeof(path_points_30)/sizeof(path_points_30[0]));
    storeIntoStationPaths(path_points_31, station_paths[3][1],sizeof(path_points_31)/sizeof(path_points_31[0]));
    storeIntoStationPaths(path_points_32, station_paths[3][2],sizeof(path_points_32)/sizeof(path_points_32[0]));

    vector<vector<vector<geometry_msgs::Point> > > main_paths;
    main_paths.resize(1);
    main_paths[0].resize(6);

    storeIntoStationPaths(dcc_mcd, main_paths[0][0],sizeof(dcc_mcd)/sizeof(dcc_mcd[0]));
    storeIntoStationPaths(mcd_dcc, main_paths[0][1],sizeof(mcd_dcc)/sizeof(mcd_dcc[0]));
    storeIntoStationPaths(dcc_ea, main_paths[0][2],sizeof(dcc_ea)/sizeof(dcc_ea[0]));
    storeIntoStationPaths(ea_dcc, main_paths[0][3],sizeof(ea_dcc)/sizeof(ea_dcc[0]));
    storeIntoStationPaths(e3a_ea, main_paths[0][4],sizeof(e3a_ea)/sizeof(e3a_ea[0]));
    storeIntoStationPaths(ea_e3a, main_paths[0][5],sizeof(ea_e3a)/sizeof(ea_e3a[0]));

    for(int i=0; i<6; i++)
    {
        cout <<endl;
        for(int j=0; j<main_paths[0][i].size()-1; j++)
            cout <<square_distance(main_paths[0][i][j],main_paths[0][i][j+1]) <<endl;
        cout <<endl;
    }
    for(int i=0; i<station_paths[0][2].size();i++)
        cout<<station_paths[0][2][i].x<<','<<station_paths[0][2][i].y<<'\t';
    cout<<endl;


    for(int i=0; i<station_number;i++)
    {
        for(int j=0; j<station_number; j++)
        {
            if(station_paths[i][j].size()<2)
                continue;

            int k=0;
            for(vector<geometry_msgs::Point>::iterator it = station_paths[i][j].begin(); it<station_paths[i][j].end()-1; it++)
            {
                //cout<<k<<' '<<station_paths[i][j].size()<<*(it+1)<<endl;
                double dist = square_distance(*it, *(it+1));
                //cout<<dist<<endl;
                if(dist>max_straight)
                {
                    vector<geometry_msgs::Point>::iterator startAddPoint = it;
                    //cout<<endl<<"Start x y "<< it->x<<' '<<it->y<<" End x y "<< (it+1)->x<<' '<<(it+1)->y<<":"<<endl;
                    double inc = max_straight/dist;
                    //cout<<"adding segment with inc "<<inc<<" with distance of "<<dist<<"path size "<<station_paths[i][j].size()<<endl;
                    double x_1 = it->x, y_1 = it->y;
                    double x_2 = (it+1)->x, y_2 = (it+1)->y;
                    for(double t=inc; t<1; t+=inc)
                    {
                        it++;
                        geometry_msgs::Point p;
                        p.x = x_1 + t * (x_2 - x_1);
                        p.y = y_1 + t * (y_2 - y_1);
                        it=station_paths[i][j].insert(it, p);

                        //cout<<p.x<<' '<<p.y<<'\t';
                    }
                    //cout<<endl;
                    while(startAddPoint<=it+1)
                    {
                        //cout<<(*startAddPoint).x <<' '<<(*startAddPoint).y<<endl;
                        startAddPoint++;
                    }

                }
                k++;
            }
        }
    }
    for(int i=0; i<station_paths[0][2].size();i++)
        cout <<station_paths[0][2][i].x <<',' <<station_paths[0][2][i].y <<'\t';
    cout <<endl;

}


double stationPath::square_distance(geometry_msgs::Point p1, geometry_msgs::Point p2)
{
    return sqrt((p1.x-p2.x)*(p1.x-p2.x)+(p1.y-p2.y)*(p1.y-p2.y));
}


void stationPath::getPath(int stationPickUp, int stationDropOff, vector<geometry_msgs::Point> &path)
{
    path = station_paths[stationPickUp][stationDropOff];
}


void stationPath::storeIntoStationPaths(int path_points[][2], vector<geometry_msgs::Point> &station_paths, int size)
{
    //this function will convert the points from pixel number to points in the map frame
    double res = 0.1;
    int y_pixels = 3536;
    double distance=0;
    cout <<"Path size: " <<size <<". ";
    for(unsigned int i=0; i<size;i++)
    {
        geometry_msgs::Point p;
        p.x = path_points[i][0]*res;
        p.y = (y_pixels - path_points[i][1])*res;
        station_paths.push_back(p);

        if(i>0)
        {
            double x_cur = station_paths[i].x, x_pre = station_paths[i-1].x;
            double y_cur = station_paths[i].y, y_pre = station_paths[i-1].y;
            distance+=sqrt((x_cur-x_pre)*(x_cur-x_pre)+(y_cur-y_pre)*(y_cur-y_pre));
        }
    }
    cout <<"Distance= " <<distance <<endl;
}


}; //namespace station_path
