#include <unistd.h>
#include <stdio.h>
#include <assert.h>

#include <iostream>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <string>

#include "Scheduler.h"

#define DEBUG_LOGFILE logFile
#define DEBUG_VERBOSITY_VAR verbosity_level
#include "debug_macros.h"

using namespace std;
using SchedulerTypes::Task;
using SchedulerTypes::Duration;


#define MAX_ADDITIONAL_TIME 5


Scheduler::Scheduler(DBTalker &dbt)
: verbosity_level(0), logFile(NULL), dbTalker(dbt)
{
    // Add a vehicle. TODO: get the list of vehicles from the database.
    vehicles.push_back( Vehicle(dbt, "golfcart1", SchedulerTypes::VEHICLE_AVAILABLE) );
}

void Scheduler::setLogFile(FILE *logfile)
{
    this->logFile = logfile;
}

void Scheduler::setVerbosityLevel(unsigned lvl)
{
    this->verbosity_level = lvl;
}

Scheduler::VIT Scheduler::checkVehicleAvailable()
{
    VIT vit;
    for ( vit = vehicles.begin(); vit != vehicles.end(); ++vit)
        if (vit->status != SchedulerTypes::VEHICLE_NOT_AVAILABLE)
            break;
    return vit;
}

Task Scheduler::addTask(Task task)
{
    // Currently we don't have any procedure to assign a vehicle to the task,
    // so we just assign to the first available vehicle:
    // TODO: support more vehicles

	VIT vit = checkVehicleAvailable();
    if( vit==vehicles.end() )
        throw SchedulerException(SchedulerException::NO_AVAILABLE_VEHICLE);

    task = Task(task.taskID, task.customerID, vit->id, task.pickup, task.dropoff);

    MSGLOG(2, "Adding task <%u,%s,%s,%s> to vehicle %s",
            task.taskID, task.customerID.c_str(),
            task.pickup.c_str(), task.dropoff.c_str(), task.vehicleID.c_str());

    task.ttask = travelTime(task.pickup, task.dropoff);

    if( vit->tasks.empty() )
    {
        MSGLOG(2, "Adding as current.");
        assert( vit->status == SchedulerTypes::VEHICLE_AVAILABLE );
        vit->status = SchedulerTypes::VEHICLE_ON_CALL;
        task.tpickup = 0; //TODO: this should be the time from the current station to the pickup station
        vit->tasks.push_back(task);
        dbTalker.confirmTask(task);
    }
    else
    {
        Station prevDropoff = vit->tasks.front().dropoff;
        list<Task>::iterator it = vit->tasks.begin();
        for ( ++it; it != vit->tasks.end(); ++it )
        {
        	// Time from previous task dropoff to this task pickup
        	unsigned tdp1 = (prevDropoff != task.pickup) ? travelTime(prevDropoff, task.pickup) : 0;
        	// Time from this task dropoff to next task pickup
        	unsigned tdp2 = (task.dropoff != it->pickup) ? travelTime(task.dropoff, it->pickup) : 0;

            MSGLOG(4, "(%u+%u+%u) VS %d\n", tdp1, task.ttask, tdp2, it->tpickup);

            if (tdp1 + task.ttask + tdp2 <= it->tpickup + MAX_ADDITIONAL_TIME) {
                task.tpickup = tdp1;
                it->tpickup = tdp2;
                vit->tasks.insert(it, task);
                vit->updateWaitTime();
                break;
            }
            else
                prevDropoff = it->dropoff;
        }

        if( it == vit->tasks.end() )
        {
            task.tpickup = (prevDropoff != task.pickup) ? travelTime(prevDropoff, task.pickup) : 0;
            vit->tasks.push_back(task);
            vit->updateWaitTime();
        }

        dbTalker.acknowledgeTask(task);
    }
    return task;
}


void Scheduler::removeTask(unsigned taskID)
{
    MSGLOG(2, "Removing task %u", taskID);

    for (VIT vit = vehicles.begin(); vit != vehicles.end(); ++vit)
    {
        list<Task> & tasks = vit->tasks;
        if( tasks.empty() )
            continue;

        if( tasks.front().taskID == taskID )
        {
            //Trying to cancel the current task. This is only possible if the
            //vehicle is currently going to the pickup location, not when going
            //to dropoff.
            //TODO: detect that and update task times, and other appropriate
            //actions. For now: current task cannot be cancelled.

            throw SchedulerException(SchedulerException::TASK_CANNOT_BE_CANCELLED);
        }

        Station prevDropoff = tasks.front().dropoff;
        list<Task>::iterator jt = tasks.begin();
        ++jt;
        for( ; jt != tasks.end(); ++jt )
        {
            if(jt->taskID == taskID)
            {
                jt = tasks.erase(jt);
                if( jt != tasks.end() )
                    jt->tpickup = (prevDropoff != jt->pickup) ? travelTime(prevDropoff, jt->pickup) : 0;
                vit->updateWaitTime();
                return;
            }
            prevDropoff = jt->dropoff;
        }
    }
    ERROR("Task %u does not exist", taskID);
    throw SchedulerException(SchedulerException::TASK_DOES_NOT_EXIST);
}

Scheduler::VIT Scheduler::checkVehicleExist(string vehicleID)
{
    VIT vit;
    for( vit=vehicles.begin(); vit!=vehicles.end(); ++vit )
        if( vit->id == vehicleID )
            break;
    if( vit==vehicles.end() )
        throw SchedulerException(SchedulerException::INVALID_VEHICLE_ID);
    return vit;
}

Scheduler::CVIT Scheduler::checkVehicleExist(std::string vehicleID) const
{
	CVIT vit;
	for( vit=vehicles.begin(); vit!=vehicles.end(); ++vit )
		if( vit->id == vehicleID )
			break;
	if( vit==vehicles.end() )
		throw SchedulerException(SchedulerException::INVALID_VEHICLE_ID);
	return vit;
}

Task & Scheduler::getTask(unsigned taskID)
{
    list<Task>::iterator jt;
    for(VIT vit = vehicles.begin(); vit != vehicles.end(); ++vit)
        for( jt = vit->tasks.begin(); jt != vit->tasks.end(); ++jt )
            if(jt->taskID == taskID)
                return *jt;
    throw SchedulerException(SchedulerException::TASK_DOES_NOT_EXIST);
    return *jt;
}

const Task & Scheduler::getTask(unsigned taskID) const
{
	list<Task>::const_iterator jt;
	for(CVIT vit = vehicles.begin(); vit != vehicles.end(); ++vit)
		for( jt = vit->tasks.begin(); jt != vit->tasks.end(); ++jt )
			if(jt->taskID == taskID)
				return *jt;
	throw SchedulerException(SchedulerException::TASK_DOES_NOT_EXIST);
	return *jt;
}

Duration Scheduler::getWaitTime(unsigned taskID) const
{
    return getTask(taskID).twait;
}

string Scheduler::toString() const
{
    stringstream ss;
    for( vector<Vehicle>::const_iterator vit = vehicles.begin(); vit != vehicles.end(); ++vit )
    {
        ss << "Vehicle " << vit->id << " (" <<vehicleStatusStr(vit->status) << "): ";
        const list<Task> & tasks = vit->tasks;
        if( tasks.empty() )
            ss <<" no tasks." <<endl;
        else
            for ( list<Task>::const_iterator jt=tasks.begin(); jt != tasks.end(); ++jt )
                ss << "  " << jt->toString() << endl;
    }
    return ss.str();
}

Duration Scheduler::travelTime(Station pickup, Station dropoff)
{
    double vel = 1.0;
    double length = stationPaths.getPath(pickup, dropoff).length();
    return (Duration)(length*vel);
}

void Scheduler::update()
{
	vector<Task> newTasks = dbTalker.getRequestedBookings();
	for( vector<Task>::const_iterator tit=newTasks.begin(); tit!=newTasks.end(); ++tit )
		addTask(*tit);

	for( VIT vit=vehicles.begin(); vit!=vehicles.end(); ++vit )
		vit->update();

   	MSGLOG(3, "After updating waiting time...\nTask queue:\n%s", toString().c_str());
}

