#ifndef __DB_INTERFACE_H__
#define __DB_INTERFACE_H__

#include <string>


/** A class to allow the vehicle to interact with the database.
 *
 * Access to the database is through the PHP layer (see HTTPClient).
 * XML parsing is handled by TinyXML.
 */

class DBInterface
{
public:
    struct Task {
        unsigned id;
        std::string pickup;
        std::string dropoff;
    };

private:
    std::string url;
    std::string vehicleID;

public:
    DBInterface(std::string url, std::string vehicleID);

    /// Adds this vehicle to the database.
    void identify(std::string vehicleName);

    /// Sets the current location.
    void setCurrentLocation(std::string loc);

    /// Removes this vehicle from the database.
    void deleteVehicle();

    /// Returns true if the mission has been cancelled.
    bool checkMissionCancelled(unsigned id);

    /// Returns NULL if no new mission, Task pointer otherwise.
    Task * checkForNewMission();

    /// Waits for a new mission and returns it.
    /// @arg period: check period in seconds (defaults to 1).
    Task waitForNewMission(float period = 1);

    /// Sets the GPS coordinates.
    void setGeoLocation(float lat, float lon);

    /// Sets the estimated time of arrival
    void setETA(float eta);

    void setMissionStatus(std::string status);

    void setVehicleStatus(std::string status);
};


#endif
