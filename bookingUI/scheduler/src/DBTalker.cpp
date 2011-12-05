#include <sstream>

#include <cppconn/statement.h>
#include <cppconn/prepared_statement.h>
#include <cppconn/resultset.h>

#include "DBTalker.h"

using namespace std;
using namespace sql;

DBTalker::DBTalker(string hostname, string username, string passwd, string dbname)
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
                0, //vehicle ID (dummy value, will be assigned by scheduler)
                stationList(res->getString("pickupLocation")),
                stationList(res->getString("dropoffLocation"))
            )
        );

    delete res;
    delete stmt;
    cout <<"Retrieved " <<tasks.size() <<" requested bookings." <<endl;
    return tasks;
}

// Update task status (from scheduler to DB)
void DBTalker::update(const vector<Vehicle> & vehicles)
{

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
            0, //TODO: this should be a string. res->getString("vehicleID"),
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
//TODO: vehicleID should be a string rather than an int
pair<VehicleStatus, Duration> DBTalker::getVehicleStatus(unsigned vehicleID)
{
    pair<VehicleStatus, Duration> info;

    PreparedStatement *stmt = con->prepareStatement("SELECT status, eta FROM vehicles WHERE vehicleID=?");
    stmt->setString(1,"golfcart1");
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
    PreparedStatement *stmt = con->prepareStatement("INSERT INTO requests (status, customerID, pickupLocation, dropoffLocation) VALUES ('Requested',?,?,?)");
    stmt->setString(1, customerID);
    stmt->setString(2, pickup.str());
    stmt->setString(3, dropoff.str());
    unsigned i = stmt->executeUpdate();
    cout <<"DBTalker::makeBooking: Added task " <<i <<endl;
    delete stmt;
    return i;
}
