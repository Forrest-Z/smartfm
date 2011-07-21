#ifndef ROUTEPLANNER_HH_
#define ROUTEPLANNER_HH_

#include "socket_handler/ClientSocket.hh"
#include "socket_handler/SocketException.hh"

enum VehicleStatus
  {
    VEHICLE_NOT_AVAILABLE = 0,
    VEHICLE_ON_CALL = 1,
    VEHICLE_POB = 2,
    VEHICLE_BUSY = 3, // either not available, on call, or pob
    VEHICLE_AVAILABLE = 4
  };

class RoutePlanner
{
public:
  // Default Constructor
  RoutePlanner(std::string host, int port);

  // Default destructor
  virtual ~RoutePlanner() {};

  // Send new status to the server
  bool sendStatus(int vehicleID, VehicleStatus vehStatus, int tremain);
//available should be sent only once
//update pob, on call just keep sending it

  // Get a new task
  bool getNewTask(int &usrID, int &pickup, int &dropoff);

private:
  ClientSocket m_socket;
  bool m_isconnected;
  std::string msg;
};

#endif // ROUTEPLANNER_HH_
