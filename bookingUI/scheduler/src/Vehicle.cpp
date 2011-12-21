#include <string>
#include <list>
#include <sstream>

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
    if( status == SchedulerTypes::VEHICLE_POB )
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
    if (status == SchedulerTypes::VEHICLE_POB)
        getCurrentTask().tpickup = 0;
}

Task Vehicle::switchToNextTask()
{
    if( tasks.size()<=1 )
        throw SchedulerException(SchedulerException::NO_PENDING_TASKS);

    tasks.pop_front();
    Task task = tasks.front();
    updateStatus(SchedulerTypes::VEHICLE_ON_CALL);
    updateWaitTime();
    return task;
}

void Vehicle::update()
{
	if( tasks.empty() )
		return;

	string vehicleID = id;
	Task curTask = tasks.front();
	SchedulerTypes::VehicleInfo vi = dbTalker.getVehicleInfo(vehicleID);
	SchedulerTypes::TaskStatus ts = dbTalker.getTaskStatus(curTask.taskID);
	updateStatus(vi.status);

	switch(vi.status)
	{
	case SchedulerTypes::VEHICLE_AVAILABLE:
		dbTalker.updateTime(curTask.taskID, 0);
		dbTalker.updateTaskStatus(curTask.taskID, "Completed");
		if( tasks.size()>1 )
		{
			Task task = switchToNextTask();
			dbTalker.confirmTask(task);
		}
		else
		{
			tasks.clear();
		}
		break;

	case SchedulerTypes::VEHICLE_ON_CALL:
	case SchedulerTypes::VEHICLE_POB:
		updateTCurrent(vi.eta);
		updateWaitTime();
		break;

	default:
		stringstream ss;
		ss <<"Scheduler::update(): task status is " <<ts.status <<". ";
		ss <<"This case has not been implemented yet.";
		throw logic_error(ss.str());
	}

	// Report waiting time
	list<Task>::const_iterator tit = tasks.begin();
	for( ++tit; tit != tasks.end(); ++tit )
		dbTalker.updateTime(tit->taskID, tit->tpickup);
}
