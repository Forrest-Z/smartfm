#ifndef ROUTEPLANNER_HH_
#define ROUTEPLANNER_HH_

#include "ClientSocket.hh"
#include "SocketException.hh"

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
  // Constructors
  // Only communication with the scheduler. Do not report current location.
  RoutePlanner(std::string schHost, int schPort); 
  // Use the same host to communicate with the scheduler and to report current location.
  RoutePlanner(std::string host, int schPort, int locPort);
  RoutePlanner(std::string schHost, int schPort, std::string locHost, int locPort);

  // Default destructor
  virtual ~RoutePlanner() {};

  // Communication with the scheduler
  // Send new status to the server
  bool sendStatus(int vehicleID, VehicleStatus vehStatus, int tremain);
  // Get a new task
  bool getNewTask(int &usrID, int &pickup, int &dropoff);

  // Report current location
  bool sendLocation(double lat, double lon);

private:
  void initSchComm(std::string host, int port);
  void initLocComm(std::string host, int port);

  ClientSocket m_schSocket, m_locSocket;
  bool m_schConnected, m_locConnected;
  std::string msg;
};

#endif // ROUTEPLANNER_HH_
