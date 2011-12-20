#ifndef __DB_INTERFACE_H__
#define __DB_INTERFACE_H__

#include <string>
#include <vector>

#include <tinyxml.h>

#include "HTTPClient.h"
#include "DBInterfaceException.h"

/** A class to allow the vehicle to interact with the database.
 *
 * Access to the database is through the PHP layer (see HTTPClient).
 * XML parsing is handled by TinyXML.
 */
class DBInterface
{
public:
    /// A Request entry in the database
    class Task
    {
    public:
        unsigned id;
        std::string pickup;
        std::string dropoff;
        std::string customerID;
        std::string status;
        std::string vehicleID;
        bool custCancelled;

        Task();
        static Task fromXML(TiXmlElement *);
    };

    /// A Vehicle entry in the database
    class Vehicle
    {
    public:
        std::string vehicleID;
        std::string status;
        unsigned requestID;
        std::string currentLocation;
        double latitude;
        double longitude;
        int eta;

        Vehicle();
        static Vehicle fromXML(TiXmlElement *);
    };

private:
    std::string url;
    std::string vehicleID;

    /// Returns a client with the url and vehicleID set.
    HTTPClient getHTTPClient(std::string php);

    /// Call the client connect function, parse the result, and throw an exception
    /// if there was an error. Returns the document.
    TiXmlDocument rpc(HTTPClient & client);

// Some helper functions (made public for testing purpose).
public:
    /// Returns the vehicle entry.
    Vehicle getVehicleEntry();
    
    /// Returns all tasks for this vehicle.
    std::vector<DBInterface::Task> getAllTasks();

    /// Returns the task entry.
    Task getTaskEntry(unsigned id);
    
public:
    /// Creates an interface
    ///@arg url is the URL of the server with the path to the PHP scripts
    ///@arg vehicleID is the vehicle ID
    DBInterface(std::string url, std::string vehicleID);

// Some function related to vehicle's status
public:
    /// Adds this vehicle to the database.
    void identify();

    /// Removes this vehicle from the database.
    void deleteVehicle();
    
    /// Sets the current location.
    void setCurrentLocation(std::string loc);

    /// Sets the vehicle's status.
    void setVehicleStatus(std::string status);
    
    /// Sets the GPS coordinates.
    void setGeoLocation(float lat, float lon);
    
    /// Sets the estimated time of arrival
    void setETA(float eta);

    
// Some functions related to the requests.
public:
    /// Returns true if the mission has been cancelled.
    bool checkMissionCancelled(unsigned id);

    void acceptMissionCancel(unsigned id, bool accept);

    /// Checks whether a new mission is available, in which case it is copied
    /// in the task pointer passed as argument.
    bool checkForNewMission(Task *t);

    /// Waits for a new mission and returns it.
    /// @arg period: check period in seconds (defaults to 1).
    /// @return only the id, pickup and dropoff fields are filled in.
    Task waitForNewMission(float period = 1);

    void setMissionStatus(unsigned id, std::string status);
};


#endif
