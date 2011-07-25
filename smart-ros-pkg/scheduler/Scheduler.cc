#include "Scheduler.hh"

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

Scheduler::Scheduler(int verbosity_level)
{
  this->verbosity_level = verbosity_level;
  for (int i=0; i<NUM_VEHICLES; i++) {
    this->currentTask[i].id = -1;
    this->currentTask[i].customerID = -1;
    this->currentTask[i].vehicleID = 1;
    this->currentTask[i].pickup = 1;
    this->currentTask[i].dropoff = 1;
    this->currentTask[i].ttask = 0;
    this->currentTask[i].tpickup = 0;
    this->currentTask[i].twait = 0;

    this->vehStatus[i] = VEHICLE_NOT_AVAILABLE;
  }
  this->nextTaskID = 1;
  this->stNetwork = new StationNetwork();
}

Scheduler::~Scheduler()
{
  for (int i = 0; i < NUM_VEHICLES; i++)
    this->taskAssignment[i].clear();
  delete this->stNetwork;
}

int Scheduler::addTask(int customerID, int taskID, int pickup, int dropoff)
{
  if (pickup != -1 && dropoff != -1) {
    Task task;
    task.id = this->nextTaskID++;
    task.customerID = customerID;
    task.taskID = taskID;
    task.vehicleID = DEFAULT_VEHICLE_ID;
    task.pickup = pickup;
    task.dropoff = dropoff;
    int ret = this->addTask(task);
    if (ret == ADD_TASK_NO_ERROR)
      return task.id;
    else
      return ret;
  }
  else {
    bool taskRemoved = this->removeTask(customerID, taskID);
    if (!taskRemoved)
      return TASK_NOT_EXIST;
    else
      return ADD_TASK_NO_ERROR;
  }
}

int Scheduler::addTask(Task task)
{
  if (this->verbosity_level > 0)
    MSG("Adding task <%d,%d,%d,%d>", task.id, task.customerID, task.pickup, task.dropoff);

  bool vehicleAvailable = false;
  for (int i = 0; i < NUM_VEHICLES; i++) {
    if (this->vehStatus[i] != VEHICLE_NOT_AVAILABLE) {
      vehicleAvailable = true;
      break;
    }
  }
  if (!vehicleAvailable) {
    ERROR("Cannot add task. No vehicle available!");
    return VEHICLE_NOT_AVAILABLE;
  }

  int taskTime = validateTask(task);
  if(taskTime < 0)
    return taskTime;

  task.ttask = taskTime;
  if (this->taskAssignment[DEFAULT_VEHICLE_ID].empty()) {
    task.tpickup = 0;
    if (this->currentTask[DEFAULT_VEHICLE_ID].dropoff != task.pickup)
      task.tpickup = this->stNetwork->travelTime(this->currentTask[DEFAULT_VEHICLE_ID].dropoff, task.pickup);
    this->taskAssignment[DEFAULT_VEHICLE_ID].push_back(task);
    updateWaitTime();
    return ADD_TASK_NO_ERROR;  
  }
  else {
    // Time from previous task dropoff to this task pickup
    int tdp1 = 0;
    // Time from this task dropoff to next task pickup
    int tdp2 = 0;
    int prevDropoff = this->currentTask[DEFAULT_VEHICLE_ID].dropoff;
    
    list<Task>::iterator it;
    for ( it=this->taskAssignment[DEFAULT_VEHICLE_ID].begin(); it != this->taskAssignment[DEFAULT_VEHICLE_ID].end(); it++ ) {
      if (prevDropoff != task.pickup)
	tdp1 = this->stNetwork->travelTime(prevDropoff, task.pickup);
      else
	tdp1 = 0;
      if (task.dropoff != it->pickup)
	tdp2 = this->stNetwork->travelTime(task.dropoff, it->pickup);
      else
	tdp2 = 0;
      
      if (this->verbosity_level > 3)
	MSG("(%d+%d+%d) VS %d\n", tdp1, task.ttask, tdp2, it->tpickup);
      
      if (tdp1 + task.ttask + tdp2 <= it->tpickup + MAX_ADDITIONAL_TIME) {
	task.tpickup = tdp1;
	it->tpickup = tdp2;
	this->taskAssignment[DEFAULT_VEHICLE_ID].insert(it, task);
	updateWaitTime();
	return ADD_TASK_NO_ERROR;
      }
      prevDropoff = it->dropoff;
    }
    task.tpickup = 0;
    if (prevDropoff != task.pickup)
      task.tpickup = this->stNetwork->travelTime(prevDropoff, task.pickup);
    this->taskAssignment[DEFAULT_VEHICLE_ID].push_back(task);
    updateWaitTime();
    return ADD_TASK_NO_ERROR;  
  }
}

bool Scheduler::removeTask(int id)
{
  if (this->verbosity_level > 0)
    MSG("Removing task %d", id);

  for (int i = 0; i < NUM_VEHICLES; i++) {
    list<Task>::iterator it;
    int prevDropoff = this->currentTask[i].dropoff;
    for ( it=this->taskAssignment[i].begin() ; it != this->taskAssignment[i].end(); it++ ) {
      if(it->id == id) {
	it = this->taskAssignment[i].erase(it);
	it->tpickup = 0;
	if (prevDropoff != it->pickup)
	  it->tpickup = this->stNetwork->travelTime(prevDropoff, it->pickup);
	updateWaitTime(i);
	return true;
      }
      prevDropoff = it->dropoff;
    }
  }
  ERROR("Task %d does not exist", id);
  return false;
}

bool Scheduler::removeTask(int customerID, int taskID)
{
  if (this->verbosity_level > 0)
    MSG("Removing task from customer %d:%d", customerID, taskID);

  for (int i = 0; i < NUM_VEHICLES; i++) {
    list<Task>::iterator it;
    int prevDropoff = this->currentTask[i].dropoff;
    for ( it=this->taskAssignment[i].begin() ; it != this->taskAssignment[i].end(); it++ ) {
      if(it->customerID == customerID && it->taskID == taskID) {
	it = this->taskAssignment[i].erase(it);
	it->tpickup = 0;
	if (prevDropoff != it->pickup)
	  it->tpickup = this->stNetwork->travelTime(prevDropoff, it->pickup);
	updateWaitTime(i);
	return true;
      }
      prevDropoff = it->dropoff;
    }
  }
  ERROR("Task from customer %d:%d does not exist", customerID, taskID);
  return false;
}

bool Scheduler::checkTask(int vehicleID)
{
  if (vehicleID < NUM_VEHICLES) {
    return !this->taskAssignment[vehicleID].empty();
  }
  else
    ERROR("Invalid vehicle");
  return false;
}

Task Scheduler::getVehicleNextTask(int vehicleID)
{
  Task task;
  task.id = -1;
  task.pickup = -1;
  task.dropoff = -1;
  task.ttask = 0;
  task.tpickup = 0;
  task.twait = 0;
  if (vehicleID < NUM_VEHICLES) {
    this->vehStatus[vehicleID] = VEHICLE_AVAILABLE;
    if (this->taskAssignment[vehicleID].size() > 0) {
      task = this->taskAssignment[vehicleID].front();
      this->currentTask[vehicleID] = task;
      this->taskAssignment[vehicleID].pop_front();
      this->vehStatus[vehicleID] = VEHICLE_ON_CALL;
      updateWaitTime(vehicleID);
      if (this->verbosity_level > 0)
	MSG("Giving task <%d,%d,%d> to vehicle %d", task.id, task.pickup, task.dropoff, vehicleID);
    }
    else if(this->verbosity_level > 0)
      MSG("No more task for vehicle %d\n", vehicleID);
  }
  else
    ERROR("Invalid vehicle");
  return task;
}

Task Scheduler::getVehicleCurrentTask(int vehicleID)
{
  if (vehicleID < NUM_VEHICLES)
    return currentTask[vehicleID];

  ERROR("Invalid vehicle");
  Task task;
  task.id = -1;
  task.pickup = -1;
  task.dropoff = -1;
  task.ttask = 0;
  task.tpickup = 0;
  task.twait = 0;
    
  return task;
}

list<Task> Scheduler::getVehicleRemainingTasks(int vehicleID)
{
  if (vehicleID < NUM_VEHICLES)
    return this->taskAssignment[vehicleID];

  ERROR("Invalid vehicle");
  list<Task> tasks;
  return tasks;
}

int Scheduler::getWaitTime(int taskID)
{
  for (int i = 0; i < NUM_VEHICLES; i++) {
    list<Task>::iterator it;
    for ( it=this->taskAssignment[i].begin() ; it != this->taskAssignment[i].end(); it++ )
      if(it->id == taskID)
	return it->twait;
  }
  return -1;
}

Task Scheduler::getTask(int taskID)
{
  Task task;
  task.id = -1;
  for (int i = 0; i < NUM_VEHICLES; i++) {
    if (currentTask[i].id == taskID)
      return currentTask[i];
    list<Task>::iterator it;
    for ( it=this->taskAssignment[i].begin() ; it != this->taskAssignment[i].end(); it++ )
      if(it->id == taskID) {
	task = *it;
	return task;
      }
  }
  return task;
}

VehicleStatus Scheduler::getVehicleStatus(int vehicleID)
{
  if (vehicleID < NUM_VEHICLES)
    return this->vehStatus[vehicleID];
  else
    return VEHICLE_NOT_AVAILABLE;
}

void Scheduler::updateWaitTime(int vehicleID)
{
  if (vehicleID < NUM_VEHICLES)
    this->updateWaitTime(vehicleID, this->currentTask[vehicleID].tpickup + this->currentTask[vehicleID].ttask);
  else
    ERROR("Invalid vehicle");
}

void Scheduler::updateWaitTime(int vehicleID, int timeCurrentTask)
{
  if (this->verbosity_level > 1)
    MSG("Updating waiting time for vehicle %d with current task completion time %d", 
	vehicleID, timeCurrentTask);

  if (vehicleID < NUM_VEHICLES) {
    int waittime = timeCurrentTask;
    list<Task>::iterator it;
    for ( it=this->taskAssignment[vehicleID].begin(); it != this->taskAssignment[vehicleID].end(); it++ ) {
      waittime += it->tpickup;
      it->twait = waittime;
      waittime += it->ttask;
    }
  }
  else
    ERROR("Invalid vehicle");
}

void Scheduler::updateTCurrent(int tremain)
{
  this->updateTCurrent(DEFAULT_VEHICLE_ID, tremain);
}

void Scheduler::updateTCurrent(int vehicleID, int tremain)
{
  if (vehicleID < NUM_VEHICLES) {
    if (vehStatus[vehicleID] == VEHICLE_POB) {
      this->currentTask[vehicleID].ttask = tremain;
      this->currentTask[vehicleID].tpickup = 0;
    }
    else
      this->currentTask[vehicleID].tpickup = tremain;
  }
  else
    ERROR("Invalid vehicle");
}

void Scheduler::updateTPickupCurrent(int tpickup)
{
  this->updateTPickupCurrent(DEFAULT_VEHICLE_ID, tpickup);
}

void Scheduler::updateTPickupCurrent(int vehicleID, int tpickup)
{
  if (vehicleID < NUM_VEHICLES)
    this->currentTask[vehicleID].tpickup = tpickup;
  else
    ERROR("Invalid vehicle");
}

void Scheduler::updateTTaskCurrent(int ttask)
{
  this->updateTTaskCurrent(DEFAULT_VEHICLE_ID, ttask);
}

void Scheduler::updateTTaskCurrent(int vehicleID, int ttask)
{
  if (vehicleID < NUM_VEHICLES)
    this->currentTask[vehicleID].ttask = ttask;
  else
    ERROR("Invalid vehicle");
}

void Scheduler::updateVehicleStatus(VehicleStatus status)
{
  this->updateVehicleStatus(DEFAULT_VEHICLE_ID, status);
}

void Scheduler::updateVehicleStatus(int vehicleID, VehicleStatus status)
{
  if (vehicleID < NUM_VEHICLES) {
    this->vehStatus[vehicleID] = status;
    if (status == VEHICLE_POB)
      this->currentTask[vehicleID].tpickup = 0;
  }
  else
    ERROR("Invalid vehicle");
}

void Scheduler::printTasks()
{
  for (int i = 0; i < NUM_VEHICLES; i++) {
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

int Scheduler::validateTask(Task task)
{
  if (!this->stNetwork->exists(task.pickup)) {
    ERROR("Invalid pickup station %d", task.pickup);
    return INVALID_PICKUP;
  }
  if (!this->stNetwork->exists(task.dropoff)) {
    ERROR("Invalid dropoff station %d", task.dropoff);
    return INVALID_DROPOFF;
  }  
  int taskTime = this->stNetwork->travelTime(task.pickup, task.dropoff);
  if (taskTime < 0) {
    ERROR("Invalid pickup-dropoff pair (%d, %d)", task.pickup , task.dropoff);
    return INVALID_PICKUP_DROPOFF_PAIR; 
  }
  return taskTime;
}
