#include "DBInterface.h"
#include "HTTPClient.h"


/*
struct Task {
    unsigned id;
    std::string pickup;
    std::string dropoff;
};

std::string url;
std::string vehicleID;
*/


DBInterface::DBInterface(std::string url, std::string vehicleID)
{
    this.url = url;
    this.vehicleID = vehicleID;
}

// Adds this vehicle to the database.
void DBInterface::identify(std::string vehicleName)
{

}

// Sets the current location.
void DBInterface::setCurrentLocation(std::string loc)
{

}

// Removes this vehicle from the database.
void DBInterface::deleteVehicle()
{

}

// Returns true if the mission has been cancelled.
bool DBInterface::checkMissionCancelled(unsigned id)
{
    return false;
}

// Returns NULL if no new mission, Task pointer otherwise.
DBInterface::Task * DBInterface::checkForNewMission()
{
    return 0;
}

// Waits for a new mission and returns it.
// @arg period: check period in seconds (defaults to 1).
DBInterface::Task waitForNewMission(float period)
{
    return Task();
}

// Sets the GPS coordinates.
void DBInterface::setGeoLocation(float lat, float lon)
{

}

// Sets the estimated time of arrival
void DBInterface::setETA(float eta)
{

}

void DBInterface::setMissionStatus(std::string status)
{

}

void DBInterface::setVehicleStatus(std::string status)
{

}
