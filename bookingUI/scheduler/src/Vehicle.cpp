#include <string>
#include <list>

#include "SchedulerTypes.h"
#include "Vehicle.h"

using SchedulerTypes::Task;
using SchedulerTypes::Duration;
using namespace std;

const float Vehicle::INTER_TASK_EXTRA_TIME = 20;

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
    list<Task>::iterator it = tasks.begin();
    Duration waittime = it->tpickup + it->ttask + INTER_TASK_EXTRA_TIME;

    for( ++it; it != tasks.end(); ++it )
    {
        waittime += it->tpickup;
        it->twait = waittime;
        waittime += it->ttask + INTER_TASK_EXTRA_TIME;
    }
}

void Vehicle::updateTCurrent(Duration tremain)
{
    Task & t = getCurrentTask();
    if( status == SchedulerTypes::VEH_STAT_GOING_TO_DROPOFF )
    {
        t.ttask = t.twait = tremain;
        t.tpickup = 0;
    }
    else
    {
        t.tpickup = t.twait = tremain;
    }
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
    if( tasks.empty() )
        return;

    SchedulerTypes::VehicleInfo vi = dbTalker.getVehicleInfo(this->id);
    updateStatus(vi.status);

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
        dbTalker.updateTaskStatus(curTask.taskID, "Processing");
        break;

    case SchedulerTypes::VEH_STAT_AT_DROPOFF:
        if( vi.requestID == curTask.taskID )
        {
            //MSGLOG(1, "Vehicle reached destination. Switching to next task.");
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

    // Check pending tasks whether custommer cancelled.
    list<Task>::iterator tit = tasks.begin();
    ++tit;
    while( tit != tasks.end() ) {
        SchedulerTypes::TaskStatus ts = dbTalker.getTaskStatus(tit->taskID);
        if( ts.custCancelled ) {
            dbTalker.updateTaskStatus(tit->taskID, "Cancelled");
            tit = tasks.erase(tit);
        }
        else
            ++tit;
    }

    // Report waiting time
    for( list<Task>::const_iterator tit = tasks.begin(); tit != tasks.end(); ++tit )
        dbTalker.updateTime(tit->taskID, tit->twait);
}
