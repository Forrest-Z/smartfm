#include "RoutePlanner.hh"
#include <iostream>
#include <string>

int main ( int argc, char **argv )
{
  //RoutePlanner rp("172.29.147.5", 8888);
  //RoutePlanner rp("localhost", 8888, 4440);
  RoutePlanner rp("localhost", 8888, "172.17.7.9", 4440);
  rp.sendStatus(0, VEHICLE_AVAILABLE, 0);

  int usrID, pickup, dropoff;

  while (1) {
    rp.getNewTask(usrID, pickup, dropoff);
    std::cout << "Receive ;" << usrID << ":" << pickup << ":" << dropoff << std::endl;
    rp.sendStatus(0, VEHICLE_POB, 10);
    rp.sendLocation(10.5, 11.6);
    sleep(1);
    rp.sendStatus(0, VEHICLE_POB, 9);
    rp.sendLocation(10.6, 11.6);
    sleep(1);
    rp.sendStatus(0, VEHICLE_POB, 8);
    rp.sendLocation(10.7, 11.6);
    sleep(1);
    rp.sendStatus(0, VEHICLE_POB, 7);
    rp.sendLocation(10.8, 11.6);
    sleep(1);
    rp.sendStatus(0, VEHICLE_POB, 6);
    rp.sendLocation(10.9, 11.6);
    sleep(1);
    rp.sendStatus(0, VEHICLE_POB, 5);
    rp.sendLocation(11, 11.6);
    sleep(1);
    rp.sendStatus(0, VEHICLE_POB, 4);
    rp.sendLocation(11.1, 11.6);
    sleep(1);
    rp.sendStatus(0, VEHICLE_POB, -1246543);
    rp.sendLocation(11.2, 11.6);
    sleep(1);
    rp.sendStatus(0, VEHICLE_AVAILABLE, 0);
    rp.sendLocation(11.5, 11.6);
    //std::cout << "Status sent" << std::endl;
  }
  return 0;
}
