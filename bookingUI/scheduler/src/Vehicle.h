#ifndef __VEHICLE__H__
#define __VEHICLE__H__

#include <string>
#include <list>

#include <DebugLogger.h>

#include "SchedulerTypes.h"
#include "DBTalker.h"

class Vehicle : public DebugLogger
{
    friend class Scheduler;

    DBTalker & dbTalker;
    std::string id;
    SchedulerTypes::VehicleStatus status;
    std::list<SchedulerTypes::Task> tasks;

    SchedulerTypes::Task & getCurrentTask();
    void update();

public:
    /// Extra time allocated between tasks
    static const float INTER_TASK_EXTRA_TIME;

public:
    Vehicle(DBTalker & dbt, std::string name, SchedulerTypes::VehicleStatus s)
        : dbTalker(dbt), id(name), status(s) { }

    Vehicle & operator=( const Vehicle & v);

    const std::list<SchedulerTypes::Task> & getTasks() const { return tasks; }
    const SchedulerTypes::Task & getCurrentTask() const;
    SchedulerTypes::VehicleStatus getStatus() const { return status; }
    std::string getID() const { return id; }

    void switchToNextTask();

    void updateWaitTime();
    void updateTCurrent(SchedulerTypes::Duration tremain);
    void updateStatus(SchedulerTypes::VehicleStatus status);
};


#endif //__VEHICLE__H__
