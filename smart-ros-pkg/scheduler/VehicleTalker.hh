#ifndef VEHICLETALKER_HH_
#define VEHICLETALKER_HH_

#include <pthread.h>
#include <stdio.h>
#include "Scheduler.hh"
#include "ServerSocket.hh"
#include "SocketException.hh"

struct VehicleInfo
{
  // ID of the vehicle
  int vehicleID;

  // Status of the vehicle
  std::VehicleStatus status;

  // Whether this is a new status
  bool isNew;

  // Time to dropoff (if status is POB) or to pickup (if status is ON_CALL)
  int tremain;
};

class VehicleTalker
{
public:
  // Default Constructor
  VehicleTalker(int port, int verbosity_level);

  // Default destructor
  virtual ~VehicleTalker();

  // Send new task to the vehicle (server)
  bool sendNewTask(int customerID, int pickup, int dropoff);

  // Check whether there is new info
  bool checkNewInfo();

  // Return the latest vehicle info.
  VehicleInfo getVehicleInfo();

  // An infinite loop that keep listening for new vehicle info
  void runVehicleReceiver();

  // Stop running talker
  bool quit();

private:
  VehicleInfo m_vehInfo;
  pthread_mutex_t m_statusMutex;
  ServerSocket m_server, m_socket;
  bool m_quit, m_isconnected;
  bool m_newStatusRecv;
  int m_verbosity;
  FILE * logFile;
};

#endif // VEHICLETALKER_HH_
