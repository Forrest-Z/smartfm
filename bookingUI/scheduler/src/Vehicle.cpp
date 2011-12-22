#include <string>
#include <list>

#include "SchedulerTypes.h"
#include "Vehicle.h"

using SchedulerTypes::Task;
using SchedulerTypes::Duration;
using namespace std;


Vehicle & Vehicle::operator=( const Vehicle & v) {
    if( this!=&v ) {
        dbTalker = v.dbTalker;
        id = v.id;
        status = v.status;
        tasks = v.tasks;
    }
    return *this;
}

const Task & Vehicle::getCurrentTask() const
{
    if( tasks.empty() )
        throw SchedulerException(SchedulerException::NO_CURRENT_TASK);
    return tasks.front();
}

Task & Vehicle::getCurrentTask()
{
    if( tasks.empty() )
        throw SchedulerException(SchedulerException::NO_CURRENT_TASK);
    return tasks.front();
}

void Vehicle::updateWaitTime()
{
    Task & t = getCurrentTask();
    updateWaitTime(t.tpickup + t.ttask);
}

void Vehicle::updateWaitTime(Duration timeCurrentTask)
{
    Duration waittime = timeCurrentTask;

    for( list<Task>::iterator it = tasks.begin(); it != tasks.end(); ++it )
    {
        waittime += it->tpickup;
        it->twait = waittime;
        waittime += it->ttask;
    }
}

void Vehicle::updateTCurrent(Duration tremain)
{
    Task & t = getCurrentTask();
    if( status == SchedulerTypes::VEH_STAT_GOING_TO_DROPOFF )
    {
        t.ttask = tremain;
        t.tpickup = 0;
    }
    else
    {
        t.tpickup = tremain;
    }
}

void Vehicle::updateTPickupCurrent(Duration tpickup)
{
    getCurrentTask().tpickup = tpickup;
}

void Vehicle::updateTTaskCurrent(Duration ttask)
{
    getCurrentTask().ttask = ttask;
}

void Vehicle::updateStatus(SchedulerTypes::VehicleStatus status)
{
    this->status = status;
    if( status == SchedulerTypes::VEH_STAT_GOING_TO_DROPOFF )
        getCurrentTask().tpickup = 0;
}

void Vehicle::switchToNextTask()
{
    if( tasks.empty() ) return;
    tasks.pop_front();

    if( tasks.empty() ) return;
    Task task = tasks.front();
    dbTalker.confirmTask(task);
    updateStatus(SchedulerTypes::VEH_STAT_GOING_TO_PICKUP);
    updateWaitTime();
}

void Vehicle::update()
{
    string vehicleID = id;
    SchedulerTypes::VehicleInfo vi = dbTalker.getVehicleInfo(vehicleID);
    updateStatus(vi.status);

    if( tasks.empty() )
        return;

    Task curTask = tasks.front();

    switch(vi.status)
    {
    case SchedulerTypes::VEH_STAT_WAITING:
        break; //nothing to do here

    case SchedulerTypes::VEH_STAT_GOING_TO_PICKUP:
    case SchedulerTypes::VEH_STAT_AT_PICKUP:
    case SchedulerTypes::VEH_STAT_GOING_TO_DROPOFF:
        updateTCurrent(vi.eta);
        updateWaitTime();
        break;

    case SchedulerTypes::VEH_STAT_AT_DROPOFF:
        if( vi.requestID == curTask.taskID )
        {
            dbTalker.updateTime(curTask.taskID, 0);
            dbTalker.updateTaskStatus(curTask.taskID, "Completed");
            switchToNextTask();
        }
    break;

    default:
        throw logic_error(string("Scheduler::update(): vehicle status is ") +
                SchedulerTypes::vehicleStatusStr(vi.status) +
                "This case has not been implemented yet.");
    }

    // Report waiting time
    for( list<Task>::const_iterator tit = tasks.begin(); tit != tasks.end(); ++tit )
        dbTalker.updateTime(tit->taskID, tit->twait);
}
