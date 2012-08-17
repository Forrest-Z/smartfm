#include <iostream>
#include <fstream>

#include "Path.h"

double waypoints[][3] = {
    {0,  0,   0},
    {10, 20,  50},
    {5,  30,  50},
    {10, 50,  25},
    {5,  100, 50},
    {10, 50,  100},
    {10, 25,  75},
    {3,  15,  85},
    {5,  50,  150},
    {15, 150, 100}
};

int main()
{
    Path path(waypoints, sizeof(waypoints)/sizeof(double)/3);
    std::cout <<"Created the path." <<std::endl;

    std::ofstream file("trajectory.dat");
    file <<"#m=0,S=2" <<std::endl;
    path.print_waypoints_xy(file);
    file <<"#m=1,S=0" <<std::endl;
    path.print_path_xy(file);

    return 0;
}
