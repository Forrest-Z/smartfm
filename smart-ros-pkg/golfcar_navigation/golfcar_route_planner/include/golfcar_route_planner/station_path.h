/*
 * station_path.h
 *
 *  Created on: Jul 21, 2011
 *      Author: golfcar
 */

#ifndef STATION_PATH_H_
#define STATION_PATH_H_


#endif /* STATION_PATH_H_ */

#define /*7*/DCC_MCD {1046, 2220}, {1066, 2271}, {1446,3054}, {1526,3108}, {1596, 3130}, {1608, 3185}, {1577, 3236}
#define /*6*/MCD_DCC {1526, 3229}, {1490, 3177}, {1430, 3062}, {1092, 2349}, {1066, 2271}, {1046, 2220}
#define /*29*/DCC_EA {1014, 2178}, {970,2109}, {932,2056}, {887,2020}, {830,2001}, {769,1998}, {728,2006}, {673,2032}, {632,2064}, {598,2100}, {566,2147}, {536,2192}, {517,2239}, {507,2271}, {469,2318}, {412,2334}, {363,2347}, {324, 2331}, {302, 2285}, {300, 2216}, {346, 1722}, {374, 1224}, {386, 1165}, {563, 672}, {726, 344}, {781, 245}, {837, 213}, {894, 217}, {1171, 316}
#define /*31*/EA_DCC {1008, 276}, {956,254}, {888, 242}, {843, 229},{813,240}, {793, 263}, {774, 285}, {746, 353}, {687, 458}, {588, 683}, {497, 912}, {410, 1175}, {399, 1229}, {374, 1721}, {325, 2217}, {334, 2270}, {360, 2306}, {440, 2288}, {481, 2249}, {522, 2186}, {558, 2131}, {588, 2084}, {622, 2047}, {663, 2010}, {720, 1986}, {768, 1985}, {833, 1982}, {892, 2008}, {943, 2044}, {983, 2099}, {1009, 2130}
#define /*6*/E3A_EA {1833, 650}, {1800, 610}, {1789, 573}, {1740, 545}, {1415, 434}, {1298, 400}
#define /*7*/EA_E3A {1423, 415}, {1856, 563}, {1938, 596}, {1985, 659}, {1946, 676}, {1909, 682}, {1868, 670}
#include <geometry_msgs/Point.h>

using namespace std;

namespace station_path {

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

	void storeIntoStationPaths(int path_points[][2], vector<geometry_msgs::Point> &station_paths, int size);

public:
	stationPath();
	~stationPath();

	void getPath(int stationPickUp, int stationDropOff, vector<geometry_msgs::Point> &path);
	vector<vector<vector<geometry_msgs::Point> > > station_paths;
};
};

namespace station_path{

stationPath::stationPath()
{
	const int station_number = 4;

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
}

stationPath::~stationPath()
{

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
	cout<<"Path size: "<<size<<endl;
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
	cout<<"Distance= " <<distance<<endl;
}




};
