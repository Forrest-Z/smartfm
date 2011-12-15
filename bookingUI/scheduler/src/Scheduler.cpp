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



#define MAX_ADDITIONAL_TIME 5


#define __CASE(t) case t: msg=#t; break
SchedulerException::SchedulerException(SchedulerExceptionTypes t) throw()
{
    type_ = t;
    switch(type_)
    {
        __CASE(NO_AVAILABLE_VEHICLE);
        __CASE(TASK_DOES_NOT_EXIST);
        __CASE(TASK_CANNOT_BE_CANCELLED);
        __CASE(INVALID_VEHICLE_ID);
        __CASE(NO_PENDING_TASKS);
        __CASE(NO_CURRENT_TASK);
    }
}


//------------------------------------------------------------------------------


Task::Task(unsigned taskID, string customerID, string vehicleID,
           Station pickup, Station dropoff)
{
    this->taskID = taskID;
    this->customerID = customerID;
    this->vehicleID = vehicleID;
    this->pickup = pickup;
    this->dropoff = dropoff;

    this->ttask = 0;
    this->tpickup = 0;
    this->twait = 0;

    this->valid = true;
}

string Task::toString() const
{
    stringstream s("");
    s << "<taskID:" <<  taskID << ", custID:" << customerID;
    s << ", pickup:" << pickup.str() << ", dropoff:" << dropoff.str();
    s << ", tpickup:" << tpickup << ", ttask:" << ttask << ", twait:" << twait << ">";
    return s.str();
}


//------------------------------------------------------------------------------


Scheduler::Scheduler()
: verbosity_level(0), logFile(NULL)
{
    // Add a vehicle. TODO: get the list of vehicles from the database.
    vehicles.push_back( Vehicle("golfcart1", VEHICLE_AVAILABLE) );
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
        if (vit->status != VEHICLE_NOT_AVAILABLE)
            break;
    return vit;
}

Task Scheduler::addTask(Task task)
{
    VIT vit = checkVehicleAvailable();
    if( vit==vehicles.end() )
        throw SchedulerException(SchedulerException::NO_AVAILABLE_VEHICLE);

    // Currently we don't have any procedure to assign a vehicle to the task,
    // so we just assign to vehicle 0:
    // TODO: support more vehicles
    task = Task(task.taskID, task.customerID, vehicles[0].id, task.pickup, task.dropoff);

    MSGLOG(2, "Adding task <%u,%s,%s,%s> to vehicle %s",
            task.taskID, task.customerID.c_str(),
            task.pickup.c_str(), task.dropoff.c_str(), task.vehicleID.c_str());

    task.ttask = travelTime(task.pickup, task.dropoff);

    list<Task> & tasks = getVehicleTasks(task.vehicleID);

    if( tasks.empty() )
    {
        MSGLOG(2, "Adding as current.");
        assert( vit->status == VEHICLE_AVAILABLE );
        vit->status = VEHICLE_ON_CALL;
        task.tpickup = 0; //TODO: this should be the time from the current station to the pickup station
        tasks.push_back(task);
        updateWaitTime(task.vehicleID);
    }
    else
    {
        Task & curTask = tasks.front();
        // Time from previous task dropoff to this task pickup
        unsigned tdp1 = 0;
        // Time from this task dropoff to next task pickup
        unsigned tdp2 = 0;
        Station prevDropoff = curTask.dropoff;

        list<Task>::iterator it = tasks.begin();
        ++it;
        for ( ; it != tasks.end(); ++it )
        {
            if (prevDropoff != task.pickup)
                tdp1 = travelTime(prevDropoff, task.pickup);
            else
                tdp1 = 0;

            if (task.dropoff != it->pickup)
                tdp2 = travelTime(task.dropoff, it->pickup);
            else
                tdp2 = 0;

            MSGLOG(4, "(%u+%u+%u) VS %d\n", tdp1, task.ttask, tdp2, it->tpickup);

            if (tdp1 + task.ttask + tdp2 <= it->tpickup + MAX_ADDITIONAL_TIME) {
                task.tpickup = tdp1;
                it->tpickup = tdp2;
                tasks.insert(it, task);
                updateWaitTime(task.vehicleID);
                break;
            }
            else
                prevDropoff = it->dropoff;
        }

        if( it == tasks.end() )
        {
            task.tpickup = 0;
            if (prevDropoff != task.pickup)
                task.tpickup = travelTime(prevDropoff, task.pickup);
            tasks.push_back(task);
            updateWaitTime(task.vehicleID);
        }
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
                {
                    jt->tpickup = 0;
                    if (prevDropoff != jt->pickup)
                        jt->tpickup = travelTime(prevDropoff, jt->pickup);
                }
                updateWaitTime(vit->id.c_str());
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

bool Scheduler::hasPendingTasks(string vehicleID)
{
    return getVehicleTasks(vehicleID).size() > 1;
}

Task Scheduler::vehicleSwitchToNextTask(string vehicleID)
{
    VIT vit = checkVehicleExist(vehicleID);
    vit->status = VEHICLE_AVAILABLE;

    if( ! hasPendingTasks(vehicleID) )
    {
        MSGLOG(2, "No more task for vehicle %s\n", vehicleID.c_str());
        throw SchedulerException(SchedulerException::NO_PENDING_TASKS);
    }

    vit->tasks.pop_front();
    Task task = vit->tasks.front();
    vit->status = VEHICLE_ON_CALL;
    updateWaitTime(task.vehicleID);
    MSGLOG(2, "Giving task <%u,%s,%s> to vehicle %s",
           task.taskID, task.pickup.c_str(), task.dropoff.c_str(), task.vehicleID.c_str());
    return task;
}

list<Task> & Scheduler::getVehicleTasks(string vehicleID)
{
    VIT vit = checkVehicleExist(vehicleID);
    return vit->tasks;
}

Task & Scheduler::getVehicleCurrentTask(string vehicleID)
{
    list<Task> & tasks = getVehicleTasks(vehicleID);
    if( tasks.empty() )
        throw SchedulerException(SchedulerException::NO_CURRENT_TASK);
    return tasks.front();
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

Duration Scheduler::getWaitTime(unsigned taskID)
{
    return getTask(taskID).twait;
}

VehicleStatus Scheduler::getVehicleStatus(string vehicleID)
{
    return checkVehicleExist(vehicleID)->status;
}

void Scheduler::updateWaitTime(string vehicleID)
{
    Task & t = getVehicleCurrentTask(vehicleID);
    updateWaitTime(vehicleID, t.tpickup + t.ttask);
}

void Scheduler::updateWaitTime(string vehicleID, Duration timeCurrentTask)
{
    list<Task> & tasks = getVehicleTasks(vehicleID);
    Duration waittime = timeCurrentTask;

	MSGLOG(3, "Updating waiting time for vehicle %s with current task "
			"completion time %u", vehicleID.c_str(), timeCurrentTask);

    for( list<Task>::iterator it = tasks.begin(); it != tasks.end(); ++it )
    {
        waittime += it->tpickup;
        it->twait = waittime;
        waittime += it->ttask;
    }
}

void Scheduler::updateTCurrent(string vehicleID, Duration tremain)
{
    Task & t = getVehicleCurrentTask(vehicleID);
    if (getVehicleStatus(vehicleID) == VEHICLE_POB)
    {
        t.ttask = tremain;
        t.tpickup = 0;
    }
    else
    {
        t.tpickup = tremain;
    }
}

void Scheduler::updateTPickupCurrent(string vehicleID, Duration tpickup)
{
    getVehicleCurrentTask(vehicleID).tpickup = tpickup;
}

void Scheduler::updateTTaskCurrent(string vehicleID, Duration ttask)
{
    getVehicleCurrentTask(vehicleID).ttask = ttask;
}

void Scheduler::updateVehicleStatus(string vehicleID, VehicleStatus status)
{
	VIT vit = checkVehicleExist(vehicleID);
    vit->status = status;
    if (status == VEHICLE_POB)
        getVehicleCurrentTask(vehicleID).tpickup = 0;
}

string vehicleStatusStr(VehicleStatus vs)
{
	switch( vs )
	{
	case VEHICLE_NOT_AVAILABLE:
		return "NOT AVAILABLE";
	case VEHICLE_ON_CALL:
		return "ON CALL";
	case VEHICLE_POB:
		return "POB";
	case VEHICLE_AVAILABLE:
		return "AVAILABLE";
	}
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
