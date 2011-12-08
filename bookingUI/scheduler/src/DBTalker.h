#ifndef __DB_TALKER_H__
#define __DB_TALKER_H__


#include <string>


/* MySQL Connector/C++ specific headers */
#include <cppconn/driver.h>
#include <cppconn/connection.h>

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
    std::string hostname, username, passwd, dbname;
    StationList stationList;
    unsigned verbosity_level;
    FILE * logFile;


public:
    struct TaskStatus
    {
        Task task;
        std::string status;
    };

    DBTalker(std::string hostname, std::string username, std::string passwd, std::string dbname);
    ~DBTalker();

    void setLogFile(FILE *);
    void setVerbosityLevel(unsigned);

    /// Check the DB for bookings that haven't been scheduled yet
    std::vector<Task> getRequestedBookings();

    /// Update task ETA (from scheduler to DB)
    void updateTime(unsigned taskID, Duration duration);

    /// Set the task status as 'Acknowledged' in the DB
    void acknowledgeTask(Task task);

    /// Set the task status as 'Confirmed' in the DB
	void confirmTask(Task task);

    /// Update task status (from DB to scheduler)
    TaskStatus getTaskStatus(unsigned taskID);

    /// Update vehicle status
    std::pair<VehicleStatus, Duration> getVehicleStatus(std::string vehicleID);

    /// Make a booking: add an entry in the database. Returns the task ID.
    /// Useful for testing without mobile phone.
    unsigned makeBooking(std::string customerID, Station pickup, Station dropoff);

    /// Inserts a vehicle entry in the database. Useful for testing in simulated
    /// mode.
    void createVehicleEntry(std::string VehicleID);

private:
    void connect();
};



#endif //__DB_TALKER_H__
