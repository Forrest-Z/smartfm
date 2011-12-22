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
using SchedulerTypes::Task;


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
	MSGLOG(2,"Acknowledged request");
}

void DBTalker::confirmTask(Task task)
{
	PreparedStatement *stmt = con->prepareStatement("UPDATE requests SET Status='Confirmed', VehicleID=? WHERE RequestID=?");
	stmt->setString(1, task.vehicleID);
	stmt->setInt(2, task.taskID);
	stmt->execute();
	delete stmt;
	MSGLOG(2,"Confirmed request");
}

void DBTalker::updateTaskStatus(unsigned taskID, string status)
{
	PreparedStatement *stmt = con->prepareStatement("UPDATE requests SET status=? WHERE RequestID=?");
	stmt->setString(1, status);
	stmt->setInt(2, taskID);
	stmt->execute();
	delete stmt;
	MSGLOG(2, "Updated task status: task %d, status %s", taskID, status.c_str());
}

void DBTalker::updateTime(unsigned taskID, SchedulerTypes::Duration duration)
{
    PreparedStatement *stmt = con->prepareStatement("UPDATE requests SET eta=? WHERE RequestID=?");
	stmt->setInt(1, duration);
	stmt->setInt(2, taskID);
	stmt->execute();
    delete stmt;
	MSGLOG(2, "Updated task ETA: task %d, ETA %u", taskID, duration);
}

SchedulerTypes::TaskStatus DBTalker::getTaskStatus(unsigned taskID)
{
    PreparedStatement *stmt = con->prepareStatement(
    		"SELECT customerID, status, vehicleID, pickupLocation, dropoffLocation, "
    		"custCancelled, vehicleAcknowledgedCancel FROM requests WHERE requestID=?");
    stmt->setInt(1, taskID);
    ResultSet *res = stmt->executeQuery();

    SchedulerTypes::TaskStatus info;
    while (res->next())
    {
        info.status = res->getString("status");
        info.custCancelled = res->getBoolean("custCancelled");
        info.vehAcknowledgedCancel = res->getString("vehicleAcknowledgedCancel");
        break;
    }
    delete res;
    delete stmt;
    MSGLOG(2,"Got task status: task %u, status=%s", taskID, info.status.c_str());
    return info;
}

SchedulerTypes::VehicleInfo DBTalker::getVehicleInfo(string vehicleID)
{
	SchedulerTypes::VehicleInfo info;

    PreparedStatement *stmt = con->prepareStatement("SELECT status, eta, currentLocation, requestID FROM vehicles WHERE vehicleID=?");
    stmt->setString(1, vehicleID);
    ResultSet *res = stmt->executeQuery();
    string status = "";

    while (res->next())
    {
        status = res->getString("status");
		info.status = SchedulerTypes::vehicleStatusFromStr(status);
        info.eta = res->getInt("eta");
        info.currentLocation = res->getString("currentLocation");
        info.requestID = res->getInt("requestID");
        break;
    }
    delete res;
    delete stmt;
    MSGLOG(2,"Got vehicle info: vehicle=%s, status=%s, eta=%u, currentLoc=%s, requestID=%u",
    		vehicleID.c_str(), status.c_str(), info.eta, info.currentLocation.c_str(), info.requestID);
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

void DBTalker::custCancel(unsigned taskID)
{
	//TODO: implement custCancel
}
