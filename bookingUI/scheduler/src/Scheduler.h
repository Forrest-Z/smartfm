#ifndef __SCHEDULER_H__
#define __SCHEDULER_H__

#include <string>
#include <list>
#include <exception>

#include <station_path.h>

#define NUM_VEHICLES 1


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
    unsigned vehicleID;

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

    Task(unsigned taskID, std::string customerID, unsigned vehicleID, Station pickup, Station dropoff);

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
        NO_PENDING_TASKS
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


class Scheduler
{
public:
    enum VehicleStatus
    {
        VEHICLE_NOT_AVAILABLE,
        VEHICLE_ON_CALL,
        VEHICLE_POB,
        VEHICLE_BUSY, // either not available, on call, or pob
        VEHICLE_AVAILABLE
    };

    const StationPaths stationPaths;


private:
    /// The ordered sequence of tasks
    Task currentTask[NUM_VEHICLES];
    std::list<Task> taskAssignment[NUM_VEHICLES];
    VehicleStatus vehStatus[NUM_VEHICLES];
    unsigned nextTaskID;

    /// Level of verbosity
    unsigned verbosity_level;

public:
    /// Default Constructor
    Scheduler(unsigned verbosity_level);

    /// Method to add a task. Returns the taskid.
    unsigned addTask(std::string customerID, Station pickup, Station dropoff);

    /// Method to remove a task
    void removeTask(unsigned taskID);

    /// Method to check whether there is a task in the queue
    bool hasPendingTasks(unsigned vehicleID);

    /// Method to get the next task for a specified vehicle.
    /// Once this function is called, it's assumed that the current task
    /// is finished.
    Task getVehicleNextTask(unsigned vehicleID);

    /// Method to get the current task for a specified vehicle
    Task getVehicleCurrentTask(unsigned vehicleID);

    /// Method to get all remaining tasks for a specified vehicle
    std::list<Task> getVehicleRemainingTasks(unsigned vehicleID);

    /// Method to get the waiting time of the specified task
    unsigned getWaitTime(unsigned taskID);

    /// Method to get a specified task
    Task getTask(unsigned taskID);

    /// Method to get the status of the vehicles
    VehicleStatus getVehicleStatus(unsigned vehicleID);

    /// Method to update waiting time for a specified vehicle.
    /// timeCurrentTask is the estimated time from current position of the car
    /// to the drop off of current task (i.e., the sum of remaining tpickup and remaining ttask).
    void updateWaitTime(unsigned vehicleID);
    void updateWaitTime(unsigned vehicleID, Duration timeCurrentTask);

    /// Method to update remaining task time (POB) or pickup time (other status)
    void updateTCurrent(unsigned vehicleID, Duration tremain);

    /// Method to update remaining pickup time of current task
    void updateTPickupCurrent(unsigned vehicleID, Duration tpickup);

    /// Method to update remaining task time from pickup to drop off of current task
    void updateTTaskCurrent(unsigned vehicleID, Duration ttask);

    /// Method to update vehicle status
    void updateVehicleStatus(unsigned vehicleID, VehicleStatus status);

    /// Method to print all the tasks
    void printTasks();

private:
    /// Returns travel time. Throws SchedulerException if stations or route cannot be found.
    unsigned travelTime(Station pickup, Station dropoff);
};


#endif //__SCHEDULER_H__
