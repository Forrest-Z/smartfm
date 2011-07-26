#include "RoutePlanner.hh"
#include <iostream>
#include <string>

int main ( int argc, char **argv )
{
  //RoutePlanner rp("172.29.147.5", 8888);
  RoutePlanner rp("localhost", 8888, 4440);
  //RoutePlanner rp("172.29.146.10", 8888, "172.29.146.10", 4440);
  rp.sendStatus(0, VEHICLE_AVAILABLE, 0);

  int usrID, pickup, dropoff;

  while (1) {
    rp.getNewTask(usrID, pickup, dropoff);
    std::cout << "Receive ;" << usrID << ":" << pickup << ":" << dropoff << std::endl;
    rp.sendStatus(0, VEHICLE_POB, 10);
    rp.sendLocation(1.298718, 103.770778);
    sleep(1);
    rp.sendStatus(0, VEHICLE_POB, 9);
    rp.sendLocation(1.299048, 103.770778);
    sleep(1);
    rp.sendStatus(0, VEHICLE_POB, 8);
    rp.sendLocation(1.299048, 103.770617);
    sleep(1);
    rp.sendStatus(0, VEHICLE_POB, 7);
    rp.sendLocation(1.299070, 103.770118);
    sleep(1);
    rp.sendStatus(0, VEHICLE_POB, 6);
    rp.sendLocation(1.299352, 103.769957);
    sleep(1);
    rp.sendStatus(0, VEHICLE_POB, 5);
    rp.sendLocation(1.299352, 103.769957);
    sleep(1);
    rp.sendStatus(0, VEHICLE_POB, 4);
    rp.sendLocation(1.299352, 103.769957);
    sleep(1);
    rp.sendStatus(0, VEHICLE_POB, -1246543);
    rp.sendLocation(1.299352, 103.769957);
    sleep(1);
    rp.sendStatus(0, VEHICLE_AVAILABLE, 0);
    rp.sendLocation(1.299352, 103.769957);
    //std::cout << "Status sent" << std::endl;
  }
  return 0;
}
