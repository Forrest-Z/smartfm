#include <cassert>
#include <string>
#include <sstream>
#include <cstring>
#include <stdexcept>

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

map<VehicleStatus,string> * vehicleStatusStrMap__ = 0;

string vehicleStatusStr(VehicleStatus vs)
{
	const map<VehicleStatus,string> & m = getVehicleStatusStrMap();
	map<VehicleStatus,string>::const_iterator mit = m.find(vs);
	assert(mit!=m.end());
	return mit->second;
}

const map<VehicleStatus,string> & getVehicleStatusStrMap()
{
	if( vehicleStatusStrMap__==0 )
	{
		vehicleStatusStrMap__ = new map<VehicleStatus,string>();
		map<VehicleStatus,string> & m = *vehicleStatusStrMap__;
		m[VEH_STAT_WAITING] = "WaitingForAMission";
		m[VEH_STAT_GOING_TO_PICKUP] = "GoingToPickupLocation";
		m[VEH_STAT_AT_PICKUP] = "AtPickupLocation";
		m[VEH_STAT_GOING_TO_DROPOFF] = "GoingToDropoffLocation";
		m[VEH_STAT_AT_DROPOFF] = "AtDropoffLocation";
		m[VEH_STAT_NOT_AVAILABLE] = "NotAvailable";
	}
	return *vehicleStatusStrMap__;
}

VehicleStatus vehicleStatusFromStr(std::string s)
{
	const map<VehicleStatus,string> & m = getVehicleStatusStrMap();
	map<VehicleStatus,string>::const_iterator mit;
	for( mit = m.begin(); mit!=m.end(); ++mit )
		if( strcasecmp(mit->second.c_str(), s.c_str())==0 )
			return mit->first;
	throw runtime_error(string("VehicleStatus ")+s+" is invalid.");
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


