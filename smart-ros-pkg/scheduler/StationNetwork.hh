#ifndef STATION_NETWORK_HH_
#define STATION_NETWORK_HH_

#include <unistd.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <stdio.h>

class StationNetwork {

 private:
  int numStations;
  int *stList;
  int **stNetwork;

 public:
  // Default Constructor
  StationNetwork();

  // Default destructor
  virtual ~StationNetwork();

  // Check whether a given station exists in the network
  bool exists(int stationID);

  // Return the travel time between 2 given stations.
  // Return -1 if either station doesn't exist or there is no path between them.
  int travelTime(int stationID1, int stationID2);
};

#endif /*STATION_NETWORK_HH_*/
