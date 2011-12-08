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

using namespace std;
using namespace sql;


//------------------------------------------------------------------------------
// Macros declarations


#define LOG(fmt, ...) do { \
        if( logFile ) { \
            fprintf(logFile, "%s#%d, time %u, " fmt "\n", \
                    __func__, __LINE__, (unsigned)time(NULL), ##__VA_ARGS__); \
            fflush(logFile); \
        } \
    } while(0)

#define MSG(lvl, fmt, ...) do { \
        if (verbosity_level >= lvl) { \
            fprintf(stderr, "%s#%d: " fmt "\n", __FUNCTION__, __LINE__, ##__VA_ARGS__); \
            fflush(stderr); \
            }\
    } while(0)

#define MSGLOG(lvl, fmt, ...) do { MSG(lvl, fmt,##__VA_ARGS__); LOG(fmt,##__VA_ARGS__); } while(0)

#define ERROR(fmt, ...) do { \
        fprintf(stderr, "ERROR %s:%d: " fmt "\n", __FUNCTION__, __LINE__, ##__VA_ARGS__); \
        fflush(stderr); \
        LOG("\nERROR: " fmt "\n", ##__VA_ARGS__); \
    } while(0)


//------------------------------------------------------------------------------


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

// Check the DB for bookings that haven't been scheduled yet
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

// Update task status (from scheduler to DB)
void DBTalker::update(const vector<Vehicle> & vehicles)
{
    PreparedStatement *stmt = con->prepareStatement("UPDATE requests SET Status=?, VehicleID=? WHERE RequestID=?");

    vector<Vehicle>::const_iterator vit = vehicles.begin();
    for( ; vit != vehicles.end(); ++vit )
    {
        list<Task>::const_iterator tit = vit->tasks.begin();
        for( ; tit != vit->tasks.end(); ++tit )
        {
            string status;
            //One of 'Requested', 'Acknowledged', 'Confirmed', 'Processing',
            //'Completed', 'Cancelled'
            //vit->status is one of VEHICLE_NOT_AVAILABLE, VEHICLE_ON_CALL,
            //VEHICLE_POB, VEHICLE_AVAILABLE

            if( tit==vit->tasks.begin() )
            { //current task
                if( vit->status==VEHICLE_ON_CALL )
                    status = "Confirmed";
                else if( vit->status==VEHICLE_POB )
                    status = "Processing";
                else if( vit->status==VEHICLE_AVAILABLE )
                    status = "Completed";
                else {
                    stringstream ss;
                    ss <<"Unexpected status (current task): " <<vit->status;
                    throw logic_error(ss.str());
                }
            }
            else
            {
                status = "Acknowledged";
            }

            stmt->setString(1, status);
            stmt->setString(2, vit->id);
            stmt->setInt(3, tit->taskID);
            stmt->execute();
        }
    }

    delete stmt;
}

// Update task status (from DB to scheduler)
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

// Update vehicle status
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

// Make a booking: add an entry in the database. Returns the task ID.
// Useful for testing without mobile phone.
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
