#ifndef __VEHICLE_TALKER_H__
#define __VEHICLE_TALKER_H__

#include <pthread.h>
#include <stdio.h>

#include "Scheduler.h"
#include "socket_handler.h"


struct VehicleInfo
{
    // ID of the vehicle
    unsigned vehicleID;

    // Status of the vehicle
    VehicleStatus status;

    // Whether this is a new status
    bool isNew;

    // Time to dropoff (if status is POB) or to pickup (if status is ON_CALL)
    Duration tremain;
};


class VehicleTalker
{
public:
    // Default Constructor
    VehicleTalker(unsigned port, int verbosity_level);

    // Default destructor
    virtual ~VehicleTalker();

    // Send new task to the vehicle (server)
    bool sendNewTask(std::string customerID, const Station & pickup, const Station & dropoff);

    // Check whether there is new info
    bool checkNewInfo();

    // Return the latest vehicle info.
    VehicleInfo getVehicleInfo();

    // An infinite loop that keep listening for new vehicle info
    void runVehicleReceiver();

    // Stop running talker
    void quit();

private:
    VehicleInfo m_vehInfo;
    pthread_mutex_t m_statusMutex;
    ServerSocket m_server;
    Socket m_socket;
    bool m_quit, m_isconnected;
    bool m_newStatusRecv;
    int m_verbosity;
    FILE * logFile;
};

#endif // VEHICLETALKER_HH_
