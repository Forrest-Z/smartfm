#include <unistd.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <stdio.h>

#include "StationNetwork.h"

using namespace std;

const int StationNetwork::STATION_NETWORK[][4] = {
    {WORKSHOP, MCDONALD, 126, 126},
    {WORKSHOP, EA, 385, 385},
    {WORKSHOP, E3A, 460, 460},
    {MCDONALD, WORKSHOP, 127, 127},
    {MCDONALD, EA, 497, 497},
    {MCDONALD, E3A, 572, 572},
    {EA, WORKSHOP, 345, 345},
    {EA, MCDONALD, 467, 467},
    {EA, E3A, 121, 121},
    {E3A, WORKSHOP, 424, 424},
    {E3A, MCDONALD, 546, 546},
    {E3A, EA, 81, 81}
};

StationNetwork::StationNetwork()
{
    this->numPairs = sizeof(STATION_NETWORK)/sizeof(STATION_NETWORK[0]);
}

int StationNetwork::travelTime(StationID stationID1, StationID stationID2)
{
    for (unsigned i = 0; i < this->numPairs; i++)
        if (STATION_NETWORK[i][0] == stationID1 && STATION_NETWORK[i][1] == stationID2)
            return STATION_NETWORK[i][3];

        return -1;
}
