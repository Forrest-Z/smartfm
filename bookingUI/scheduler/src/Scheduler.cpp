#include <unistd.h>
#include <stdio.h>

#include <iostream>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <string>
#include <cassert>
#include <cstring>

#include "Scheduler.h"

using namespace std;
using SchedulerTypes::Task;
using SchedulerTypes::Duration;


#define MAX_ADDITIONAL_TIME 5

const float Scheduler::NOMINAL_VEL = 2.0;


Scheduler::Scheduler(DBTalker &dbt) : dbTalker(dbt)
{
}

Scheduler::VIT Scheduler::checkVehicleAvailable()
{
    VIT vit;
    for ( vit = vehicles.begin(); vit != vehicles.end(); ++vit)
        if ( vit->status != SchedulerTypes::VEH_STAT_NOT_AVAILABLE )
            break;
    return vit;
}

void Scheduler::addTask(Task task)
{
    // Currently we don't have any procedure to assign a vehicle to the task,
    // so we just assign to the first available vehicle:
    // TODO: support more vehicles

    VIT vit = checkVehicleAvailable();
    if( vit==vehicles.end() )
        return; // no vehicle available.

    task = Task(task.taskID, task.customerID, vit->id, task.pickup, task.dropoff);

    stringstream ss;
    ss <<"Adding task <" <<task.taskID <<"," <<task.customerID <<",";
    ss <<task.pickup.str() <<"," <<task.dropoff.str() <<"> to vehicle " <<task.vehicleID;

    task.ttask = travelTime(task.pickup, task.dropoff);

    if( vit->tasks.empty() )
    {
        if( vit->status != SchedulerTypes::VEH_STAT_WAITING ) {
            MSGLOG(1, "%s but it is not ready yet. Skipping.", ss.str().c_str());
            return;
        }
        MSGLOG(1, "%s as current task.", ss.str().c_str());
        vit->status = SchedulerTypes::VEH_STAT_GOING_TO_PICKUP;
        //TODO: this should be the time from the current station to the pickup station.
        MSGLOG(0, "WARNING: using station 0 (DCC) as current station for now.");
        task.tpickup = travelTime(stationPaths.knownStations().get(0) ,task.pickup);
        vit->tasks.push_back(task);
        dbTalker.confirmTask(task);
    }
    else
    {
        MSGLOG(1, "%s. Adding it as a pending task.", ss.str().c_str());
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
}


void Scheduler::removeTask(unsigned taskID)
{
    MSGLOG(1, "Removing task %u", taskID);

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
    double vel = Scheduler::NOMINAL_VEL;
    double length = stationPaths.getPath(pickup, dropoff).length();
    return (Duration)(length/vel);
}

void Scheduler::updateVehicleList()
{
    vector<SchedulerTypes::VehicleInfo> infos = dbTalker.getVehiclesInfo();
    vector<SchedulerTypes::VehicleInfo>::const_iterator it;

    // check if there is any new vehicle
    for( it=infos.begin(); it!=infos.end(); ++it )
    {
        CVIT vit = vehicles.begin();
        for( ; vit!=vehicles.end(); ++vit )
            if( strcmp(vit->id.c_str(), it->id.c_str())==0 )
                break;
        if( vit==vehicles.end() ) {
            // This is a new vehicle
            MSGLOG(1, "Found new vehicle %s.", it->id.c_str());
            assert( it->status==SchedulerTypes::VEH_STAT_WAITING );
            vehicles.push_back( Vehicle(dbTalker, it->id, SchedulerTypes::VEH_STAT_WAITING) );
        }
    }

    //check if any of the known vehicle disapeared
    // use a list for efficiency
    list<Vehicle> vehiclelist = list<Vehicle>(vehicles.begin(), vehicles.end());
    list<Vehicle>::iterator vit = vehiclelist.begin();
    while( vit!=vehiclelist.end() )
    {
        for( it=infos.begin(); it!=infos.end(); ++it )
            if( strcmp(vit->id.c_str(), it->id.c_str())==0 )
                break;
        if( it==infos.end() ) {
            // The vehicle disappeared
            // TODO: reschedule its tasks
            MSGLOG(1, "Vehicle %s is no longer present in the DB.", vit->id.c_str());
            vit = vehiclelist.erase(vit);
        }
        else
            ++vit;
    }
    vehicles = vector<Vehicle>(vehiclelist.begin(), vehiclelist.end());
}

void Scheduler::update()
{
    updateVehicleList();

    vector<Task> newTasks = dbTalker.getRequestedBookings();
    for( vector<Task>::const_iterator tit=newTasks.begin(); tit!=newTasks.end(); ++tit )
        addTask(*tit);

    for( VIT vit=vehicles.begin(); vit!=vehicles.end(); ++vit )
        vit->update();

    MSGLOG(2, "After updating...\nTask queue:\n%s", toString().c_str());
}

