#ifndef SCHEDULER_HH_
#define SCHEDULER_HH_

#include <unistd.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <stdio.h>
#include <string>
#include <list>
#include "StationNetwork.hh"

namespace std
{
  const int NUM_VEHICLES = 1;
  const int DEFAULT_VEHICLE_ID = 0;

  // Default customer ID used when operator add task
  const int OPERATOR_ID = -1;
  const int OPERATOR_TASK_ID = -1;

  // Enumerated list of error types when adding task.
  enum
    {
      ADD_TASK_NO_ERROR = 0,
      INVALID_PICKUP = -1,
      INVALID_DROPOFF = -2,
      INVALID_PICKUP_DROPOFF_PAIR = -3,
      NO_AVAILABLE_VEHICLE = -4,
      TASK_NOT_EXIST = -5,
      TASK_REMOVED = -6
    };
  
  enum VehicleStatus
    {
      VEHICLE_NOT_AVAILABLE = 0,
      VEHICLE_ON_CALL = 1,
      VEHICLE_POB = 2,
      VEHICLE_BUSY = 3, // either not available, on call, or pob
      VEHICLE_AVAILABLE = 4
    };

  struct Task
  {
    // ID of this object. This is internal to the scheduler.
    int id;

    // ID of customer
    int customerID;

    // ID of customer task
    int taskID;

    // ID of vehicle
    int vehicleID;
    
    // ID of the pick-up (origin) station
    int pickup;
    
    // ID of the drop-off (destination) station
    int dropoff;

    /*
    // Whether the task is cancelled
    bool cancelled;
    */

    // Time from this pickup to dropoff
    int ttask;

    // Time from previous dropoff to this pickup
    int tpickup;

    // Waiting time until pickup
    int twait;

    Task() {
      id = -1;
      customerID = -2011;
      taskID = -2011;
      vehicleID = -2011;
      pickup = -2011;
      dropoff = -2011;
      ttask = -2011;
      tpickup = -2011;
      twait = -2011;
    }

    string toString() const {
      stringstream s("");
      s << "<" <<  id << "," << customerID << "," << taskID << "," << pickup << "," << dropoff << ","
	<< tpickup << "," << ttask << "," << twait << ">";
      return s.str();
    }
  };


  class Scheduler {
    // The ordered sequence of tasks
    Task currentTask[NUM_VEHICLES];
    list<Task> taskAssignment[NUM_VEHICLES];
    StationNetwork *stNetwork;
    VehicleStatus vehStatus[NUM_VEHICLES];
    int nextTaskID;
    
    // Level of verbosity
    int verbosity_level;
    
  public:
    // Default Constructor
    Scheduler(int verbosity_level);
    
    // Default destructor
    virtual ~Scheduler();

    // Method to add a task
    int addTask(int customerID, int taskID, int pickup, int dropoff);

    // Method to remove a task
    bool removeTask(int id);
    bool removeTask(int customerID, int taskID);

    // Method to check whether there is a task in the queue
    bool checkTask(int vehicleID = DEFAULT_VEHICLE_ID);
    
    // Method to get the next task for a specified vehicle.
    // Once this function is called, it's assumed that the current task
    // is finished.
    Task getVehicleNextTask(int vehicleID = DEFAULT_VEHICLE_ID);

    // Method to get the current task for a specified vehicle
    Task getVehicleCurrentTask(int vehicleID = DEFAULT_VEHICLE_ID);

    // Method to get all remaining tasks for a specified vehicle
    list<Task> getVehicleRemainingTasks(int vehicleID = DEFAULT_VEHICLE_ID);

    // Method to get the waiting time of the specified task
    int getWaitTime(int taskID);

    // Method to get a specified task
    Task getTask(int taskID);

    // Method to get the status of the vehicles
    VehicleStatus getVehicleStatus(int vehicleID = DEFAULT_VEHICLE_ID);

    // Method to update waiting time for a specified vehicle. 
    // timeCurrentTask is the estimated time from current position of the car
    // to the drop off of current task (i.e., the sum of remaining tpickup and remaining ttask).
    void updateWaitTime(int vehicleID = DEFAULT_VEHICLE_ID);
    void updateWaitTime(int vehicleID, int timeCurrentTask);

    // Method to update remaining task time (POB) or pickup time (other status)
    void updateTCurrent(int tremain);
    void updateTCurrent(int vehicleID, int tremain);

    // Method to update remaining pickup time of current task
    void updateTPickupCurrent(int tpickup);
    void updateTPickupCurrent(int vehicleID, int tpickup);

    // Method to update remaining task time from pickup to drop off of current task
    void updateTTaskCurrent(int ttask);
    void updateTTaskCurrent(int vehicleID, int ttask);

    // Method to update vehicle status
    void updateVehicleStatus(VehicleStatus status);
    void updateVehicleStatus(int vehicleID, VehicleStatus status);

    // Method to print all the tasks
    void printTasks();

  private:
    // Method to add a task
    int addTask(Task task);

    // Check whether a given task is valid
    int validateTask(Task task);
  };

} // namespace std
#endif /*SCHEDULER_HH_*/
