#ifndef STATION_PATH_H_
#define STATION_PATH_H_

#include <vector>

#include <geometry_msgs/Point.h>


namespace station_path {

class stationPath
{
public:
    stationPath();
    double square_distance(geometry_msgs::Point p1, geometry_msgs::Point p2);
    void getPath(int stationPickUp, int stationDropOff, std::vector<geometry_msgs::Point> &path);
    std::vector<std::vector<std::vector<geometry_msgs::Point> > > station_paths;

private:
    void storeIntoStationPaths(int path_points[][2], std::vector<geometry_msgs::Point> &station_paths, int size);
};

}; //namespace station_path


#endif /* STATION_PATH_H_ */
