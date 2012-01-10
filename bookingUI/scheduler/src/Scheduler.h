#ifndef __SCHEDULER_H__
#define __SCHEDULER_H__

#include <string>
#include <list>
#include <map>
#include <exception>

#include <StationPath.h>
#include <DebugLogger.h>

#include "SchedulerTypes.h"
#include "DBTalker.h"
#include "Vehicle.h"


class Scheduler : public DebugLogger
{
public:
    /// Vehicle's nominal velocity used to compute task time
    static const float NOMINAL_VEL;

private:
    typedef std::vector<Vehicle>::iterator VIT;

    /// The pool of vehicles. Each entry has its own list of tasks.
    std::vector<Vehicle> vehicles;

    DBTalker & dbTalker;


public:
    const StationPaths stationPaths;

    typedef std::vector<Vehicle>::const_iterator CVIT;

    /// Default Constructor
    Scheduler(DBTalker & dbTalker);

    void setLogFile(FILE *);
    void setVerbosityLevel(unsigned);

    /// Get new requests from DB and process them, update ETAs, publish changes.
    void update();


// A set of public const functions for observation
public:

    /// Get all the vehicles
    const std::vector<Vehicle> & getVehicles() const { return vehicles; }

    CVIT checkVehicleExist(std::string vehicleID) const;

    /// Method to get a specified task
    const SchedulerTypes::Task & getTask(unsigned taskID) const;

    /// Method to print all the tasks
    std::string toString() const;


private:
    /// Add the task to one of the vehicles' queue. Update the status of the task
    /// in the DB (Acknowledged or Confirmed).
    void addTask(SchedulerTypes::Task);

    /// Method to remove a task. Throws a SchedulerException if the task does
    /// not exist or cannot be cancelled.
    void removeTask(unsigned taskID);

    /// Sets the next pending task as the current task and returns it. Throws a
    /// SchedulerException if there is no pending task or if the vehicle does not
    /// exist.
    SchedulerTypes::Task vehicleSwitchToNextTask(std::string vehicleID);

    /// Method to get a specified task
    SchedulerTypes::Task & getTask(unsigned taskID);

    /// Returns the vehicle, throws a SchedulerException if it does not exist.
    VIT checkVehicleExist(std::string vehicleID);

    /// Returns travel time. Throws SchedulerException if stations or route cannot be found.
    unsigned travelTime(Station pickup, Station dropoff);

    /// Returns the first available vehicle.
    VIT checkVehicleAvailable();

    /// Check the list of vehicles in the DB and update the internal pool of vehicles.
    void updateVehicleList();
};


#endif //__SCHEDULER_H__
