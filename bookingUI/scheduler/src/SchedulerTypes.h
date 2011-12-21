#ifndef __SCHEDULER_TYPES_H__
#define __SCHEDULER_TYPES_H__

#include <string>

#include <StationPath.h>

namespace SchedulerTypes
{

typedef unsigned Duration;


class Task
{
public:

    bool valid;

    /// Each task is associated an ID by the database.
    unsigned taskID;

    /// ID of customer
    std::string customerID;

    /// ID of vehicle
    std::string vehicleID;

    /// The pick-up (origin) station
    Station pickup;

    /// The drop-off (destination) station
    Station dropoff;

    /// Time from this pickup to dropoff
    /// depends on path length and vehicle speed
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


enum VehicleStatus
{
    VEHICLE_NOT_AVAILABLE,
    VEHICLE_ON_CALL,
    VEHICLE_POB,
    VEHICLE_AVAILABLE
};

std::string vehicleStatusStr(VehicleStatus vs);


struct TaskStatus
{
    std::string status;
    bool custCancelled;
    std::string vehAcknowledgedCancel;
};

struct VehicleInfo
{
	VehicleStatus status;
	Duration eta;
	std::string currentLocation;
};

} //namespace SchedulerTypes

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

#endif // __SCHEDULER_TYPES_H__
