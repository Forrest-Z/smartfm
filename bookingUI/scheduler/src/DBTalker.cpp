#include <stdio.h>

#include <string>
#include <sstream>
#include <vector>
#include <list>
#include <exception>

#include <cppconn/statement.h>
#include <cppconn/prepared_statement.h>
#include <cppconn/resultset.h>

#include "DBTalker.h"


#define DEBUG_LOGFILE logFile
#define DEBUG_VERBOSITY_VAR verbosity_level
#include "debug_macros.h"

using namespace std;
using namespace sql;


DBTalker::DBTalker(string hostname, string username, string passwd, string dbname)
: verbosity_level(0), logFile(NULL)
{
    this->hostname = hostname;
    this->username = username;
    this->passwd = passwd;
    this->dbname = dbname;

    driver = get_driver_instance();
    con = NULL;
    this->connect();
}

DBTalker::~DBTalker()
{
    delete con;
}

void DBTalker::setLogFile(FILE *f)
{
    this->logFile = f;
}

void DBTalker::setVerbosityLevel(unsigned lvl)
{
    this->verbosity_level = lvl;
}

void DBTalker::connect()
{
    con = driver->connect(hostname, username, passwd);
    con->setSchema(dbname);
}

vector<Task> DBTalker::getRequestedBookings()
{
    vector<Task> tasks;
    Statement *stmt = con->createStatement();
    ResultSet *res = stmt->executeQuery("SELECT requestID, customerID, pickupLocation, dropoffLocation FROM requests WHERE status='Requested'");

    while (res->next())
        tasks.push_back(
            Task(
                res->getInt("requestID"),
                res->getString("customerID"),
                "", //vehicle ID (dummy value, will be assigned by scheduler)
                stationList(res->getString("pickupLocation")),
                stationList(res->getString("dropoffLocation"))
            )
        );

    if( !tasks.empty() )
        MSGLOG(2, "Retrieved %u requested bookings.", (unsigned)tasks.size());

    delete res;
    delete stmt;
    return tasks;
}

void DBTalker::acknowledgeTask(Task task)
{
	PreparedStatement *stmt = con->prepareStatement("UPDATE requests SET Status='Acknowledged', VehicleID=? WHERE RequestID=?");
	stmt->setString(1, task.vehicleID);
	stmt->setInt(2, task.taskID);
	stmt->execute();
	delete stmt;
}

void DBTalker::confirmTask(Task task)
{
	PreparedStatement *stmt = con->prepareStatement("UPDATE requests SET Status='Confirmed', VehicleID=? WHERE RequestID=?");
	stmt->setString(1, task.vehicleID);
	stmt->setInt(2, task.taskID);
	stmt->execute();
	delete stmt;
}

void DBTalker::updateTime(unsigned taskID, Duration duration)
{
	return; //TODO: table requests needs an ETA field
    PreparedStatement *stmt = con->prepareStatement("UPDATE requests SET eta=? WHERE RequestID=?");
	stmt->setInt(1, duration);
	stmt->setInt(2, taskID);
	stmt->execute();
    delete stmt;
}

DBTalker::TaskStatus DBTalker::getTaskStatus(unsigned taskID)
{
    PreparedStatement *stmt = con->prepareStatement("SELECT customerID, status, vehicleID, pickupLocation, dropoffLocation FROM requests WHERE requestID=?");
    stmt->setInt(1, taskID);
    ResultSet *res = stmt->executeQuery();

    TaskStatus info;
    while (res->next())
    {
        info.task = Task(
            taskID,
            res->getString("customerID"),
            res->getString("vehicleID"),
            stationList(res->getString("pickupLocation")),
            stationList(res->getString("dropoffLocation"))
        );
        info.status = res->getString("status");
        break;
    }
    delete res;
    delete stmt;
    return info;
}

pair<VehicleStatus, Duration> DBTalker::getVehicleStatus(string vehicleID)
{
    pair<VehicleStatus, Duration> info;

    PreparedStatement *stmt = con->prepareStatement("SELECT status, eta FROM vehicles WHERE vehicleID=?");
    stmt->setString(1, vehicleID);
    ResultSet *res = stmt->executeQuery();

    while (res->next())
    {
        string status = res->getString("status");
        if( status=="WaitingForAMission" )
            info.first = VEHICLE_AVAILABLE;
        else if( status=="GoingToPickupLocation" )
            info.first = VEHICLE_ON_CALL;
        else if( status=="GoingToDropoffLocation" )
            info.first = VEHICLE_POB;
        else if( status=="AtPickupLocation" )
            info.first = VEHICLE_ON_CALL;
        else if( status=="NotAvailable" )
            info.first = VEHICLE_NOT_AVAILABLE;

        info.second = res->getInt("eta");
        break;
    }
    delete res;
    delete stmt;
    return info;
}

unsigned DBTalker::makeBooking(string customerID, Station pickup, Station dropoff)
{
    PreparedStatement *pstmt = con->prepareStatement(
        "INSERT INTO requests "
        "(status, customerID, pickupLocation, dropoffLocation) "
        "VALUES ('Requested', ?, ?, ?)"
    );
    pstmt->setString(1, customerID);
    pstmt->setString(2, pickup.str());
    pstmt->setString(3, dropoff.str());
    pstmt->execute();
    delete pstmt;

    unsigned i=0;
    Statement *stmt = con->createStatement();
    ResultSet *res = stmt->executeQuery("SELECT LAST_INSERT_ID()");
    while (res->next()) i = res->getInt("LAST_INSERT_ID()");

    MSGLOG(2, "Added task <%u,%s,%s,%s>", i, customerID.c_str(), pickup.c_str(), dropoff.c_str());

    delete res;
    delete stmt;
    return i;
}

void DBTalker::createVehicleEntry(std::string VehicleID)
{
    PreparedStatement *pstmt = con->prepareStatement(
        "INSERT INTO vehicles "
        "(vehicleID, status) "
        "VALUES (?, 'WaitingForAMission')"
    );
    pstmt->setString(1, VehicleID);
    pstmt->execute();
    delete pstmt;
}
