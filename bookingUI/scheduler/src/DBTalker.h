#ifndef __DB_TALKER_H__
#define __DB_TALKER_H__

#include <string>

/* MySQL Connector/C++ specific headers */
#include <cppconn/driver.h>
#include <cppconn/connection.h>

#include "SchedulerTypes.h"

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
    std::string hostname, username, passwd, dbname;
    StationList stationList;
    unsigned verbosity_level;
    FILE * logFile;

public:

    DBTalker(std::string hostname, std::string username, std::string passwd,
            std::string dbname);
    ~DBTalker();

    void setLogFile(FILE *);
    void setVerbosityLevel(unsigned);

    /// Check the DB for bookings that haven't been scheduled yet
    std::vector<SchedulerTypes::Task> getRequestedBookings();

    /// Update task status (from scheduler to DB)
    void updateTaskStatus(unsigned taskID, std::string status);

    /// Update task ETA (from scheduler to DB)
    void updateTime(unsigned taskID, SchedulerTypes::Duration duration);

    /// Set the task status as 'Acknowledged' in the DB
    void acknowledgeTask(SchedulerTypes::Task task);

    /// Set the task status as 'Confirmed' in the DB
    void confirmTask(SchedulerTypes::Task task);

    /// Update task status (from DB to scheduler)
    SchedulerTypes::TaskStatus getTaskStatus(unsigned taskID);

    /// Update vehicle status
    SchedulerTypes::VehicleInfo getVehicleInfo(std::string vehicleID);

    /// Get vehicles from DB
    std::vector<SchedulerTypes::VehicleInfo> getVehiclesInfo(
            std::string vehicleID = "");

    // Functions to create tasks and vehicle in the database for simulated mode
public:
    /// Make a booking: add an entry in the database. Returns the task ID.
    unsigned makeBooking(std::string customerID, Station pickup,
            Station dropoff);

    /// Requests for the task to be cancelled.
    void custCancel(unsigned taskID);

private:
    void connect();
};

#endif //__DB_TALKER_H__
