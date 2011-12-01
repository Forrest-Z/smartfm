#include "Scheduler.h"

#include <unistd.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <stdio.h>
#include <string>

using namespace std;

// Error handling
#define MSG(fmt, ...) \
  (fprintf(stderr, fmt "\n", ##__VA_ARGS__) ? 0 : 0)
#define ERROR(fmt, ...) \
  (fprintf(stderr, "ERROR %s:%d: " fmt "\n", __FILE__, __LINE__, ##__VA_ARGS__) ? -1 : 0)
/*
#define MSG(fmt, ...) \
  (fprintf(stderr, "\033[0;32m" fmt "\033[0m\n", ##__VA_ARGS__) ? 0 : 0)
#define ERROR(fmt, ...) \
  (fprintf(stderr, "\033[0;31mERROR %s:%d: " fmt "\033[0m\n", __FILE__, __LINE__, ##__VA_ARGS__) ? -1 : 0)
*/

#define MAX_ADDITIONAL_TIME 5
#define INF_TIME 10000


#define __CASE(t) case t: msg=#t; break
SchedulerException::SchedulerException(SchedulerExceptionTypes t) throw()
{
    type_ = t;
    switch(type_)
    {
        __CASE(NO_AVAILABLE_VEHICLE);
        __CASE(TASK_NOT_EXIST);
        __CASE(INVALID_VEHICLE_ID);
        __CASE(NO_PENDING_TASKS);
    }
}



Task::Task(unsigned id, unsigned customerID, unsigned taskID, unsigned vehicleID,
           Station pickup, Station dropoff)
{
    this->id = id;
    this->customerID = customerID;
    this->taskID = taskID;
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
    s << "<" <<  id << "," << customerID << "," << taskID << "," << pickup.str() << "," << dropoff.str() << ","
    << tpickup << "," << ttask << "," << twait << ">";
    return s.str();
}



Scheduler::Scheduler(unsigned verbosity_level)
{
    this->verbosity_level = verbosity_level;
    for (unsigned i=0; i<NUM_VEHICLES; i++)
        this->vehStatus[i] = VEHICLE_NOT_AVAILABLE;
    this->nextTaskID = 1;
}

void Scheduler::addTask(unsigned customerID, unsigned taskID, Station pickup, Station dropoff)
{
    /*
    if(pickup == -1 || dropoff == -1)
    {
        bool taskRemoved = this->removeTask(customerID, taskID);
        if (!taskRemoved)
            return TASK_NOT_EXIST;
        else
            return ADD_TASK_NO_ERROR;
    }
    */
    bool vehicleAvailable = false;
    for (unsigned i = 0; i < NUM_VEHICLES && !vehicleAvailable; i++)
        if (this->vehStatus[i] != VEHICLE_NOT_AVAILABLE)
            vehicleAvailable = true;

    if (!vehicleAvailable) {
        ERROR("Cannot add task. No vehicle available!");
        throw SchedulerException(SchedulerException::NO_AVAILABLE_VEHICLE);
    }

    // Currently we don't have any procedure to assign a vehile to the task,
    // so we just assign to this vehicle:
    unsigned DEFAULT_VEHICLE_ID = 0;
    Task task(this->nextTaskID++, customerID, taskID, DEFAULT_VEHICLE_ID, pickup, dropoff);

    if (this->verbosity_level > 0)
        MSG("Adding task <%u,%u,%s,%s>", task.id, task.customerID, task.pickup.c_str(), task.dropoff.c_str());

    task.ttask = travelTime(task.pickup, task.dropoff);

    if (this->taskAssignment[task.vehicleID].empty())
    {
        task.tpickup = 0;
        if (this->currentTask[task.vehicleID].dropoff != task.pickup)
            task.tpickup = travelTime(this->currentTask[task.vehicleID].dropoff, task.pickup);
        this->taskAssignment[task.vehicleID].push_back(task);
        updateWaitTime(task.vehicleID);
    }
    else
    {
        // Time from previous task dropoff to this task pickup
        unsigned tdp1 = 0;
        // Time from this task dropoff to next task pickup
        unsigned tdp2 = 0;
        Station prevDropoff = this->currentTask[task.vehicleID].dropoff;

        list<Task>::iterator it = this->taskAssignment[task.vehicleID].begin();
        for ( ; it != this->taskAssignment[task.vehicleID].end(); ++it )
        {
            if (prevDropoff != task.pickup)
                tdp1 = travelTime(prevDropoff, task.pickup);
            else
                tdp1 = 0;

            if (task.dropoff != it->pickup)
                tdp2 = travelTime(task.dropoff, it->pickup);
            else
                tdp2 = 0;

            if (this->verbosity_level > 3)
                MSG("(%u+%u+%u) VS %d\n", tdp1, task.ttask, tdp2, it->tpickup);

            if (tdp1 + task.ttask + tdp2 <= it->tpickup + MAX_ADDITIONAL_TIME) {
                task.tpickup = tdp1;
                it->tpickup = tdp2;
                this->taskAssignment[task.vehicleID].insert(it, task);
                updateWaitTime(task.vehicleID);
                break;
            }
            else
                prevDropoff = it->dropoff;
        }

        if( it == this->taskAssignment[task.vehicleID].end() )
        {
            task.tpickup = 0;
            if (prevDropoff != task.pickup)
                task.tpickup = travelTime(prevDropoff, task.pickup);
            this->taskAssignment[task.vehicleID].push_back(task);
            updateWaitTime(task.vehicleID);
        }
    }
}

void Scheduler::removeTask(unsigned id)
{
    if (this->verbosity_level > 0)
        MSG("Removing task %d", id);

    for (unsigned i = 0; i < NUM_VEHICLES; i++)
    {
        Station prevDropoff = this->currentTask[i].dropoff;
        list<Task>::iterator it = this->taskAssignment[i].begin();
        for ( ; it != this->taskAssignment[i].end(); it++ )
        {
            if(it->id == id)
            {
                it = this->taskAssignment[i].erase(it);
                it->tpickup = 0;
                if (prevDropoff != it->pickup)
                    it->tpickup = travelTime(prevDropoff, it->pickup);
                updateWaitTime(i);
                return;
            }
            prevDropoff = it->dropoff;
        }
    }
    ERROR("Task %d does not exist", id);
    throw SchedulerException(SchedulerException::TASK_NOT_EXIST);
}

void Scheduler::removeTask(unsigned customerID, unsigned taskID)
{
    if (this->verbosity_level > 0)
        MSG("Removing task from customer %d:%d", customerID, taskID);

    for (unsigned i = 0; i < NUM_VEHICLES; i++)
    {
        Station prevDropoff = this->currentTask[i].dropoff;
        list<Task>::iterator it = this->taskAssignment[i].begin();
        for ( ; it != this->taskAssignment[i].end(); it++ )
        {
            if(it->customerID == customerID && it->taskID == taskID)
            {
                it = this->taskAssignment[i].erase(it);
                it->tpickup = 0;
                if (prevDropoff != it->pickup)
                    it->tpickup = travelTime(prevDropoff, it->pickup);
                updateWaitTime(i);
                return;
            }
            prevDropoff = it->dropoff;
        }
    }
    ERROR("Task from customer %d:%d does not exist", customerID, taskID);
    throw SchedulerException(SchedulerException::TASK_NOT_EXIST);
}

bool Scheduler::hasPendingTasks(unsigned vehicleID)
{
    if (vehicleID >= NUM_VEHICLES) {
        ERROR("Invalid vehicle");
        throw SchedulerException(SchedulerException::INVALID_VEHICLE_ID);
    }
    return ! this->taskAssignment[vehicleID].empty();
}

Task Scheduler::getVehicleNextTask(unsigned vehicleID)
{
    if (vehicleID >= NUM_VEHICLES) {
        ERROR("Invalid vehicle");
        throw SchedulerException(SchedulerException::INVALID_VEHICLE_ID);
    }

    this->vehStatus[vehicleID] = VEHICLE_AVAILABLE;

    if( this->taskAssignment[vehicleID].empty() ) {
        MSG("No more task for vehicle %u\n", vehicleID);
        throw SchedulerException(SchedulerException::NO_PENDING_TASKS);
    }

    Task task( this->taskAssignment[vehicleID].front() );
    this->currentTask[vehicleID] = task;
    this->taskAssignment[vehicleID].pop_front();
    this->vehStatus[vehicleID] = VEHICLE_ON_CALL;
    updateWaitTime(vehicleID);
    if (this->verbosity_level > 0)
        MSG("Giving task <%u,%s,%s> to vehicle %u", task.id, task.pickup.c_str(), task.dropoff.c_str(), vehicleID);
    return task;
}

Task Scheduler::getVehicleCurrentTask(unsigned vehicleID)
{
    if (vehicleID >= NUM_VEHICLES){
        ERROR("Invalid vehicle");
        throw SchedulerException(SchedulerException::INVALID_VEHICLE_ID);
    }
    return currentTask[vehicleID];
}

list<Task> Scheduler::getVehicleRemainingTasks(unsigned vehicleID)
{
    if (vehicleID >= NUM_VEHICLES){
        ERROR("Invalid vehicle");
        throw SchedulerException(SchedulerException::INVALID_VEHICLE_ID);
    }
    return this->taskAssignment[vehicleID];
}

Duration Scheduler::getWaitTime(unsigned taskID)
{
    for (unsigned i = 0; i < NUM_VEHICLES; i++) {
        list<Task>::iterator it = this->taskAssignment[i].begin();
        for ( ; it != this->taskAssignment[i].end(); it++ )
            if(it->id == taskID)
                return it->twait;
    }
    throw SchedulerException(SchedulerException::TASK_NOT_EXIST);
    return 0;
}

Task Scheduler::getTask(unsigned taskID)
{
    for(unsigned i = 0; i < NUM_VEHICLES; i++)
    {
        if (currentTask[i].id == taskID)
            return currentTask[i];
        list<Task>::iterator it = this->taskAssignment[i].begin();
        for( ; it != this->taskAssignment[i].end(); it++ )
            if(it->id == taskID)
                return *it;
    }
    throw SchedulerException(SchedulerException::TASK_NOT_EXIST);
    return Task();
}

Scheduler::VehicleStatus Scheduler::getVehicleStatus(unsigned vehicleID)
{
    if (vehicleID >= NUM_VEHICLES) {
        ERROR("Invalid vehicle");
        throw SchedulerException(SchedulerException::INVALID_VEHICLE_ID);
    }
    return this->vehStatus[vehicleID];
}

void Scheduler::updateWaitTime(unsigned vehicleID)
{
    if (vehicleID >= NUM_VEHICLES) {
        ERROR("Invalid vehicle");
        throw SchedulerException(SchedulerException::INVALID_VEHICLE_ID);
    }
    Duration d = this->currentTask[vehicleID].tpickup + this->currentTask[vehicleID].ttask;
    this->updateWaitTime(vehicleID, d);
}

void Scheduler::updateWaitTime(unsigned vehicleID, Duration timeCurrentTask)
{
    if (vehicleID >= NUM_VEHICLES) {
        ERROR("Invalid vehicle");
        throw SchedulerException(SchedulerException::INVALID_VEHICLE_ID);
    }

    if (this->verbosity_level > 1)
        MSG("Updating waiting time for vehicle %d with current task completion time %d",
            vehicleID, timeCurrentTask);

    Duration waittime = timeCurrentTask;
    list<Task>::iterator it = this->taskAssignment[vehicleID].begin();
    for( ; it != this->taskAssignment[vehicleID].end(); ++it )
    {
        waittime += it->tpickup;
        it->twait = waittime;
        waittime += it->ttask;
    }
}

void Scheduler::updateTCurrent(unsigned vehicleID, Duration tremain)
{
    if (vehicleID >= NUM_VEHICLES) {
        ERROR("Invalid vehicle");
        throw SchedulerException(SchedulerException::INVALID_VEHICLE_ID);
    }
    if (vehStatus[vehicleID] == VEHICLE_POB) {
        this->currentTask[vehicleID].ttask = tremain;
        this->currentTask[vehicleID].tpickup = 0;
    }
    else
        this->currentTask[vehicleID].tpickup = tremain;
}

void Scheduler::updateTPickupCurrent(unsigned vehicleID, Duration tpickup)
{
    if (vehicleID >= NUM_VEHICLES) {
        ERROR("Invalid vehicle");
        throw SchedulerException(SchedulerException::INVALID_VEHICLE_ID);
    }
    this->currentTask[vehicleID].tpickup = tpickup;
}

void Scheduler::updateTTaskCurrent(unsigned vehicleID, Duration ttask)
{
    if (vehicleID >= NUM_VEHICLES) {
        ERROR("Invalid vehicle");
        throw SchedulerException(SchedulerException::INVALID_VEHICLE_ID);
    }
    this->currentTask[vehicleID].ttask = ttask;
}

void Scheduler::updateVehicleStatus(unsigned vehicleID, VehicleStatus status)
{
    if (vehicleID >= NUM_VEHICLES) {
        ERROR("Invalid vehicle");
        throw SchedulerException(SchedulerException::INVALID_VEHICLE_ID);
    }
    this->vehStatus[vehicleID] = status;
    if (status == VEHICLE_POB)
        this->currentTask[vehicleID].tpickup = 0;
}

void Scheduler::printTasks()
{
    for (unsigned i = 0; i < NUM_VEHICLES; i++) {
        cout << "Vehicle " << i << " (";

        switch  (this->vehStatus[i]) {
            case VEHICLE_NOT_AVAILABLE:
                cout << "NOT AVAILABLE";
                break;
            case VEHICLE_ON_CALL:
                cout << "ON CALL";
                break;
            case VEHICLE_POB:
                cout << "POB";
                break;
            case VEHICLE_BUSY:
                cout << "BUSY";
                break;
            case VEHICLE_AVAILABLE:
                cout << "AVAILABLE";
        }

        cout << "): " << this->currentTask[i].toString() << endl;
        list<Task>::iterator it;
        for ( it=this->taskAssignment[i].begin() ; it != this->taskAssignment[i].end(); it++ )
            cout << "  " << it->toString() << endl;
    }
}

Duration Scheduler::travelTime(Station pickup, Station dropoff)
{
    double vel = 1.0;
    double length = this->stationPaths.getPath(pickup, dropoff).length();
    return (Duration)(length*vel);
}
