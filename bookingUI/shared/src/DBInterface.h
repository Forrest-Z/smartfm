#ifndef __DB_INTERFACE_H__
#define __DB_INTERFACE_H__

#include <string>

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

public:
    DBInterface(std::string url, std::string vehicleID);

    /// Adds this vehicle to the database.
    void identify();

    /// Sets the current location.
    void setCurrentLocation(std::string loc);

    /// Removes this vehicle from the database.
    void deleteVehicle();

    /// Returns true if the mission has been cancelled.
    bool checkMissionCancelled(unsigned id);

    /// Returns NULL if no new mission, Task pointer otherwise.
    /// @return only the id, pickup and dropoff fields are filled in.
    Task * checkForNewMission();

    /// Waits for a new mission and returns it.
    /// @arg period: check period in seconds (defaults to 1).
    /// @return only the id, pickup and dropoff fields are filled in.
    Task waitForNewMission(float period = 1);

    /// Sets the GPS coordinates.
    void setGeoLocation(float lat, float lon);

    /// Sets the estimated time of arrival
    void setETA(float eta);

    void setMissionStatus(std::string status);

    void setVehicleStatus(std::string status);

    /// Returns the vehicle entry
    Vehicle getVehicleEntry();

    /// Returns the task entry
    Task getTaskEntry(unsigned id);
};


#endif
