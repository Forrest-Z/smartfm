#ifndef __VEHICLE__H__
#define __VEHICLE__H__

#include <string>
#include <list>

#include "SchedulerTypes.h"
#include "DBTalker.h"

class Scheduler;

class Vehicle
{
	friend class Scheduler;

	DBTalker & dbTalker;
    std::string id;
    SchedulerTypes::VehicleStatus status;
    std::list<SchedulerTypes::Task> tasks;

    SchedulerTypes::Task & getCurrentTask();
    void update();
    SchedulerTypes::Task switchToNextTask();

public:
    Vehicle(DBTalker & dbt, std::string name, SchedulerTypes::VehicleStatus s)
		: dbTalker(dbt), id(name), status(s) { }

    Vehicle & operator=( const Vehicle & v);

    const std::list<SchedulerTypes::Task> & getTasks() const { return tasks; }
    const SchedulerTypes::Task & getCurrentTask() const;
    SchedulerTypes::VehicleStatus getStatus() const { return status; }
    std::string getID() const { return id; }

    void updateWaitTime();
    void updateWaitTime(SchedulerTypes::Duration timeCurrentTask);
    void updateTCurrent(SchedulerTypes::Duration tremain);
    void updateTPickupCurrent(SchedulerTypes::Duration tpickup);
    void updateTTaskCurrent(SchedulerTypes::Duration ttask);
    void updateStatus(SchedulerTypes::VehicleStatus status);
};


#endif //__VEHICLE__H__
