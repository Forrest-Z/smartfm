#ifndef __DB_TALKER_H__
#define __DB_TALKER_H__


#include <string>


/* MySQL Connector/C++ specific headers */
#include <cppconn/driver.h>
#include <cppconn/connection.h>
#include <cppconn/statement.h>
#include <cppconn/prepared_statement.h>
#include <cppconn/resultset.h>
#include <cppconn/metadata.h>
#include <cppconn/resultset_metadata.h>
#include <cppconn/exception.h>
#include <cppconn/warning.h>

#include "Scheduler.h"



/*
 * Necessary actions (i.e. interface)
 * - check DB for new task to schedule (REQUESTED), schedule them and update
 * their status to ACKNOWLEDGED, or PROCESSING.
 * - check info of vehicles (i.e. status, time to arrival)
 *
 * Mode of operation:
 * The main will poll the DB for new tasks, and check status of scheduled tasks and
 * vehicles, then take actions
 */



class DBTalker
{
private:
    sql::Driver *driver;
    sql::Connection *con;
    std::string hostname, username, passwd;

public:
    DBTalker(std::string hostname, std::string username, std::string passwd);
    ~DBTalker();

    /// Check the DB for bookings that haven't been scheduled yet
    std::vector<Task> getRequestedBookings();

    /// Update task status (from scheduler to DB)
    void updateTaskStatus(Task);

    /// Update task status (from DB to scheduler)
    Task getTaskStatus(unsigned taskID);

    /// Update vehicle status
    std::pair<VehicleStatus, Duration> getVehicleStatus(unsigned vehicleID);
};



#endif //__DB_TALKER_H__
