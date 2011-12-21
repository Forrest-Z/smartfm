#include <string>
#include <sstream>

#include "SchedulerTypes.h"

using namespace std;

namespace SchedulerTypes
{

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

} //namespace SchedulerTypes


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


