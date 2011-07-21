#include "RoutePlanner.hh"
#include <iostream>
#include <string>

int main ( int argc, int argv[] )
{
  RoutePlanner rp("localhost", 4444);
  rp.sendStatus(0, VEHICLE_AVAILABLE, 0);

  int usrID, pickup, dropoff;

  while (1) {
    rp.getNewTask(usrID, pickup, dropoff);
    sleep(5);
    rp.sendStatus(0, VEHICLE_AVAILABLE, 0);
  }
  return 0;
}
