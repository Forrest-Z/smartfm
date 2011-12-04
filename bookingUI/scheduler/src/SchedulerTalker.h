#ifndef __SCHEDULER_TALKER_H__
#define __SCHEDULER_TALKER_H__

#include <stdio.h>
#include <string>

#include <station_path.h>
#include "socket_handler.h"
#include "Scheduler.h"

class SchedulerTalker
{
public:
    // Default Constructor
    SchedulerTalker(std::string host, unsigned port, int verbosity_level);

    // Default destructor
    virtual ~SchedulerTalker();

    // Send task status to the server
    bool sendTaskStatus(unsigned taskID, Duration twait, unsigned vehicleID);

    // Check whether there is a new task
    bool checkTask();

    // Receive the latest task. Return true if this message is new. Otherwise, return false
    bool recvTask(std::string &usrID, Station &pickup, Station &dropoff);

    // An infinite loop that keep listening for new task
    void runMobileReceiver();

    // Stop running talker
    void quit();

private:
    StationList m_stationList;
    ClientSocket m_socket;
    bool m_quit, m_isconnected;
    bool m_newTaskRecv;
    std::string m_usrID;
    Station m_pickup, m_dropoff;
    int m_verbosity;
    FILE * logFile;
};

#endif // __SCHEDULER_TALKER_H__
