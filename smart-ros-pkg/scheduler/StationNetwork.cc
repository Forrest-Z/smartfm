#include "StationNetwork.hh"
#include "Stations.h"

using namespace std;

StationNetwork::StationNetwork()
{
  this->numStations = sizeof(stations::STATION_LIST);

  this->stList = new int[this->numStations];
  for (int i = 0; i < numStations; i++)
    this->stList[i] = stations::STATION_LIST[i];

  this->stNetwork = new int*[sizeof(stations::STATION_NETWORK)];
  for (int i = 0; i < sizeof(this->stNetwork); i++) {
    stNetwork[i] = new int[4];
    for (int j = 0; j < 4; j++)
      this->stNetwork[i][j] = stations::STATION_NETWORK[i][j];
  }
}

StationNetwork::~StationNetwork()
{
  delete [] this->stList;
  for (int i = 0; i < numStations; i++)
    delete [] this->stNetwork[i];
}

bool StationNetwork::exists(int stationID)
{
  for (int i = 0; i < this->numStations; i++) {
    if (this->stList[i] == stationID) {
      return true;
    }
  }
  return false;
}

int StationNetwork::travelTime(int stationID1, int stationID2)
{
  for (int i = 0; i < sizeof(this->stNetwork); i++)
    if (this->stNetwork[i][0] == stationID1 && this->stNetwork[i][1] == stationID2)
      return this->stNetwork[i][3];

  return -1;
}
