#ifndef __SCHEDULER_H__
#define __SCHEDULER_H__

#include <string>
#include <list>
#include <map>
#include <exception>

#include <golfcar_route_planner/station_path.h>


typedef unsigned Duration;


class Task
{
public:

    bool valid;

    /// ID of this object. This is internal to the scheduler.
    unsigned taskID;

    /// ID of customer
    std::string customerID;

    /// ID of vehicle
    std::string vehicleID;

    /// ID of the pick-up (origin) station
    Station pickup;

    /// ID of the drop-off (destination) station
    Station dropoff;

    /// Time from this pickup to dropoff
    Duration ttask;

    /// Time from previous dropoff to this pickup
    Duration tpickup;

    /// Waiting time until pickup
    Duration twait;


public:
    Task() : valid(false) { }

    Task(unsigned taskID, std::string customerID, std::string vehicleID, Station pickup, Station dropoff);

    std::string toString() const;

    bool isValid() const { return valid; }
};


class SchedulerException : public std::exception
{
public:
    enum SchedulerExceptionTypes
    {
        NO_AVAILABLE_VEHICLE,
        TASK_DOES_NOT_EXIST,
        TASK_CANNOT_BE_CANCELLED,
        INVALID_VEHICLE_ID,
        NO_PENDING_TASKS,
        NO_CURRENT_TASK
    };

private:
    std::string msg;
    SchedulerExceptionTypes type_;

public:
    SchedulerException(SchedulerExceptionTypes t) throw();
    virtual ~SchedulerException() throw() { }
    virtual const char* what() const throw() { return msg.c_str(); }
    SchedulerExceptionTypes type() const throw() { return type_; }
};


enum VehicleStatus
{
    VEHICLE_NOT_AVAILABLE,
    VEHICLE_ON_CALL,
    VEHICLE_POB,
    VEHICLE_AVAILABLE
};

std::string vehicleStatusStr(VehicleStatus vs);


class Vehicle
{
public:
    std::string id;
    VehicleStatus status;
    std::list<Task> tasks;

    Vehicle() : id(0), status(VEHICLE_AVAILABLE) { }
    Vehicle(VehicleStatus s) : id(0), status(s) { }
    Vehicle(std::string i) : id(i), status(VEHICLE_AVAILABLE) { }
    Vehicle(std::string i, VehicleStatus s) : id(i), status(s) { }
};


class Scheduler
{
private:
    /// Level of verbosity
    unsigned verbosity_level;

    FILE * logFile;


public:
    const StationPaths stationPaths;

    /// The pool of vehicles. Each entry has its own list of tasks.
    std::vector<Vehicle> vehicles;

    typedef std::vector<Vehicle>::iterator VIT;

    /// Default Constructor
    Scheduler();

    void setLogFile(FILE *);
    void setVerbosityLevel(unsigned);


public:
    /// Method to add a task. Returns the task. Throws a SchedulerException if
    /// no vehicle is available.
    Task addTask(Task);

    /// Method to remove a task. Throws a SchedulerException if the task does
    /// not exist or cannot be cancelled.
    void removeTask(unsigned taskID);

    /// Sets the next pending task as the current task and returns it. Throws a
    /// SchedulerException if there is no pending task or if the vehicle does not
    /// exist.
    Task & vehicleSwitchToNextTask(std::string vehicleID);


public:
    /// Method to check whether there is a task in the queue
    bool hasPendingTasks(std::string vehicleID);

    /// Method to get the current task for a specified vehicle
    Task & getVehicleCurrentTask(std::string vehicleID);

    /// Method to get all tasks for a specified vehicle
    std::list<Task> & getVehicleTasks(std::string vehicleID);

    /// Method to get the waiting time of the specified task
    Duration getWaitTime(unsigned taskID);

    /// Method to get a specified task
    Task & getTask(unsigned taskID);

    /// Method to get the status of the vehicles
    VehicleStatus getVehicleStatus(std::string vehicleID);

    /// Method to print all the tasks
    std::string toString() const;


public:
    /// Method to update waiting time for a specified vehicle.
    /// timeCurrentTask is the estimated time from current position of the car
    /// to the drop off of current task (i.e., the sum of remaining tpickup and remaining ttask).
    void updateWaitTime(std::string vehicleID);
    void updateWaitTime(std::string vehicleID, Duration timeCurrentTask);

    /// Method to update remaining task time (POB) or pickup time (other status)
    void updateTCurrent(std::string vehicleID, Duration tremain);

    /// Method to update remaining pickup time of current task
    void updateTPickupCurrent(std::string vehicleID, Duration tpickup);

    /// Method to update remaining task time from pickup to drop off of current task
    void updateTTaskCurrent(std::string vehicleID, Duration ttask);

    /// Method to update vehicle status
    void updateVehicleStatus(std::string vehicleID, VehicleStatus status);

private:
    /// Returns the vehicle, throws a SchedulerException if it does not exist.
    VIT checkVehicleExist(std::string vehicleID);

    /// Returns travel time. Throws SchedulerException if stations or route cannot be found.
    unsigned travelTime(Station pickup, Station dropoff);

    /// Returns the first available vehicle.
    VIT checkVehicleAvailable();
};


#endif //__SCHEDULER_H__
