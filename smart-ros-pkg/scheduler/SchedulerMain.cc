#include <getopt.h>
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <signal.h>
#include <pthread.h>
#include <time.h>
#include <string.h>
#include "Scheduler.hh"
#include "SchedulerUI.hh"
#include "SchedulerTalker.hh"
#include "VehicleTalker.hh"

using namespace std;

/*
enum {
  OPT_NONE,
  OPT_HELP,
  OPT_HOST,
  OPT_PORT,
  NUM_OPTS
};
*/

enum OperatorOption
    {
      OPERATOR_OPTION_FIRST = 0,
      OPERATOR_ADD_TASK = 1,
      OPERATOR_REMOVE_TASK = 2,
      OPERATOR_VIEW_TASK_LIST = 3,
      OPERATOR_UPDATE = 4,
      OPERATOR_QUIT = 5,
      OPERATOR_OPTION_LAST = 6
    };


// Default options
int VERBOSITY_LEVEL = 0;
int NOGUI = 0;
int NOMOBILE = 0;
int SIMULATE_MOBILE = 0;
int SIMULATE_VEHICLE = 0;
int SIMULATE_TIME = 0;
int NOOP = 0;
char* HOST_NAME = "localhost";
int MOBILE_PORT = 4440;
int VEHICLE_PORT = 8888;

// The name of this program
const char * PROGRAM_NAME;

// Number of station
int NUM_STATIONS = 3;

// Number of seconds in minutes (for testing purpose)
int NUM_SECONDS_IN_MIN = 1;

// Number of seconds between two successive task info
int NUM_SEC_TASK_INFO_SENT = 10;

// Whether the operator has quit
//bool QUIT = 0;

// Scheduler and talkers
Scheduler* scheduler = NULL;
SchedulerTalker* schedulerTalker = NULL;
VehicleTalker* vehicleTalker = NULL;

// Current task
Task currentTask;
time_t previousTime;

// Log file
FILE* logFile;

// Whether the vehicle is still available
bool vehicleIsAvailable = false;

// Error handling
#define MSG(fmt, ...) \
  (fprintf(stderr, fmt "\n", ##__VA_ARGS__) ? 0 : 0)
#define ERROR(fmt, ...) \
  (fprintf(stderr, "ERROR %s:%d: " fmt "\n", __FILE__, __LINE__, ##__VA_ARGS__) ? -1 : 0)

volatile sig_atomic_t QUIT = 0;
void sigintHandler(int /*sig*/)
{
  // if the user presses CTRL-C three or more times, and the program still
  // doesn't terminate, abort
  if (QUIT > 1) {
    abort();
  }
  QUIT++;
}


/* Prints usage information for this program to STREAM (typically stdout 
   or stderr), and exit the program with EXIT_CODE. Does not return. */
void print_usage (FILE* stream, int exit_code)
{
  fprintf( stream, "Usage:  %s [options]\n", PROGRAM_NAME );
  fprintf( stream, "  --host hostname   Specify the IP address of the server for talking to mobile phone. The default is localhost.\n");  
  fprintf( stream, "  --mport mport     Specify the port for talking to mobile phones (any number between 2000 and 65535). The default is 4440.\n");  
  fprintf( stream, "  --vport vport     Specify the port for talking to the vehicle (any number between 2000 and 65535). The default is 4444.\n");  
  fprintf( stream, "  --verbose         See the status of the program at every step.\n");    
  fprintf( stream, "  --nogui           Run in plain text mode.\n");    
  fprintf( stream, "  --nomobile        Do not receive request from mobile phones.\n");  
  fprintf( stream, "  --simmobile       Simulate mobile phone users.\n");  
  fprintf( stream, "  --simvehicle      Simulate vehicles' status.\n");   
  fprintf( stream, "  --simtime         Simulate remaining time of current task.\n"); 
  fprintf( stream, "  --noop            Do not interact with operator when --nogui option is given.\n");   
  fprintf( stream, "  --help            Display this message.\n");
  exit(exit_code);
}


/* TODO: Report invalid task to the mobile customer */
void reportInvalidMobileTask(Task task, int errorCode);

/* Send task status, e.g. waiting time to mobile phones */
void sendMobileTaskStatus();

/* Check whether there is a new task from the mobile customer */
bool checkMobileTask();

/* Get a new task from mobile customer */
Task getMobileTask();

/* TODO: Get the status of the vehicle */
VehicleInfo getVehicleInfo(int vehicleID);

/* Send a new task to the vehicle */
void sendTask2Vehicle(Task task);

/* Add new task to scheduler */
int addTask(Task task);

/* Update all the information */
void update();

/* Get option from the operator. */
OperatorOption getOpOption();

/* Obtain task info if the operator wants to add task. */
Task getOpTaskInfo();

/* Obtain the id of the task to be removed */
int getOpRemoveTaskID();

/* Remove task from scheduler by the operator */
bool opRemoveTask(int taskID);

/* A thread that get command from operator*/
void *operatorLoop(void*);

template<class T, void(T::*mem_fn)()>
void* thunk(void* p)
{
  (static_cast<T*>(p)->*mem_fn)();
  return 0;
}

int main(int argc, char **argv)
{
  /*
  // catch CTRL-C and exit cleanly, if possible
  signal(SIGINT, sigintHandler);
  // and SIGTERM, i.e. when someone types "kill <pid>" or the like
  signal(SIGTERM, sigintHandler);
  signal(SIGHUP, sigintHandler);
  */

  int ch;
  int option_index = 0;
  const char* const short_options = "hp:H:";

  /* An array describing valid long options. */
  static struct option long_options[] = 
  {
    // first: long option (--option) string
    // second: 0 = no_argument, 1 = required_argument, 2 = optional_argument
    // third: if pointer, set variable to value of fourth argument
    //        if NULL, getopt_long returns fourth argument
    {"verbose",     no_argument,       &VERBOSITY_LEVEL,        1},
    {"nogui",       no_argument,       &NOGUI,                  1},
    {"nomobile",    no_argument,       &NOMOBILE,               1},
    {"simmobile",   no_argument,       &SIMULATE_MOBILE,        1},
    {"simvehicle",  no_argument,       &SIMULATE_VEHICLE,       1},
    {"simtime",     no_argument,       &SIMULATE_TIME,          1},
    {"noop",        no_argument,       &NOOP,                   1},
    {"host",        required_argument, 0,                       'H'},
    {"mport",       required_argument, 0,                       'm'},
    {"vport",       required_argument, 0,                       'v'},
    {"help",        no_argument,       0,                       'h'},
    {0,0,0,0}
  };

  /* Remember the name of the program, to incorporate in messages.
     The name is stored in argv[0]. */
  PROGRAM_NAME = argv[0];

  // Loop through and process all of the command-line input options.
  while((ch = getopt_long(argc, argv, short_options, long_options, NULL)) != -1)
    //while((ch = getopt_long_only(argc, argv, "", long_options, &option_index)) != -1)
  {
    switch(ch)
    {
    case 'h':
      /* User has requested usage information. Print it to standard
	 output, and exit with exit code zero (normal 
	 termination). */
      print_usage(stdout, 0);
      break;
      
    case '?': /* The user specified an invalid option. */
      /* Print usage information to standard error, and exit with exit
	 code one (indicating abnormal termination). */
      print_usage(stderr, 1);
      break;
      
    case 'H':
      HOST_NAME = optarg;
      break;

    case 'm':
      MOBILE_PORT = atoi(optarg);
      break;

    case 'v':
      VEHICLE_PORT = atoi(optarg);
      break;

    case -1: /* Done with options. */
      break;
    }
  }

  //cout << "Connecting to " << HOST_NAME << ":" << MOBILE_PORT << endl;

  scheduler = new Scheduler(VERBOSITY_LEVEL);
  currentTask = scheduler->getVehicleCurrentTask();
  SchedulerUI* schedulerUI = NULL;
  logFile = fopen ("log_scheduler.txt","w");
  pthread_t operatorThread, talkerThread, vehicleThread;
  pthread_mutex_t schedulerMutex;

  if (!NOOP && NOGUI) {
    printf("Usage:\n");
    printf ("  (%d) Add task\n", OPERATOR_ADD_TASK);
    printf ("  (%d) Cancel task\n", OPERATOR_REMOVE_TASK);
    printf ("  (%d) View task list\n", OPERATOR_VIEW_TASK_LIST);
    printf ("  (%d) Update\n", OPERATOR_UPDATE);
    printf ("  (%d) Quit\n", OPERATOR_QUIT);

    pthread_mutex_init(&schedulerMutex, NULL);
    int ret = pthread_create(&operatorThread, NULL, &operatorLoop, &schedulerMutex);
    if (ret != 0)
      ERROR("return code from pthread_create(): %d", ret);
  }
  else if (!NOGUI) {
    schedulerUI = new SchedulerUI(scheduler);
    schedulerUI->initConsole();
    schedulerUI->updateConsole();
  }

  if (!NOMOBILE && !SIMULATE_MOBILE) {
    ostringstream ss;
    ss << HOST_NAME;
    schedulerTalker = new SchedulerTalker(ss.str(), MOBILE_PORT, VERBOSITY_LEVEL);
    pthread_attr_t attr;
    pthread_attr_init(&attr);
    pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_DETACHED);
    int ret = pthread_create(&talkerThread, &attr, 
			     thunk<SchedulerTalker, &SchedulerTalker::runMobileReceiver>,
			     schedulerTalker);
    if (ret != 0)
      ERROR("return code from pthread_create(): %d", ret);
  }

  if (!SIMULATE_VEHICLE) {
    MSG("Waiting for connection from the vehicle.");
    if (!NOGUI)
      schedulerUI->updateConsole();
    vehicleTalker = new VehicleTalker(VEHICLE_PORT, VERBOSITY_LEVEL);
    pthread_attr_t attr;
    pthread_attr_init(&attr);
    pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_DETACHED);
    int ret = pthread_create(&vehicleThread, &attr, 
			     thunk<VehicleTalker, &VehicleTalker::runVehicleReceiver>,
			     vehicleTalker);
    MSG("Vehicle ready.");
  }

  time_t prevTimeTaskInfo = time(NULL);
  previousTime = time(NULL);

  while(QUIT == 0)
  {   
    if (!NOOP && NOGUI)
      pthread_mutex_lock(&schedulerMutex);
    else if (!NOGUI)
      schedulerUI->updateConsole();

    if (!NOMOBILE && checkMobileTask()) {
      // Get a new task from mobile phone
      Task task = getMobileTask();
      // Add a new task to the scheduler
      int ret = addTask(task);
      // Confirm task if the task is valid
      if (ret >= 0 && task.pickup != -1 && task.dropoff != -1 && !SIMULATE_MOBILE) {
	task = scheduler->getTask(ret);
	schedulerTalker->sendTaskStatus(task.customerID, task.taskID, task.twait, task.vehicleID);
      }
      else if (ret >= 0 && task.pickup == -1 && task.dropoff == -1 && !SIMULATE_MOBILE)
	schedulerTalker->sendTaskStatus(task.customerID, task.taskID, -1, TASK_REMOVED);
      // Otherwise, report invalid task to the mobile customer
      else if (ret < 0)
	reportInvalidMobileTask(task, ret);
    }

    // Update vehicle status, waiting time and send new task to vehicle if it has completes the previous task.
    update();
  
    // Report waiting time
    if (time(NULL) - prevTimeTaskInfo > NUM_SEC_TASK_INFO_SENT) {
      prevTimeTaskInfo = time(NULL);
      sendMobileTaskStatus();
    }

    if (!NOOP && NOGUI)
      pthread_mutex_unlock(&schedulerMutex);

    if (!NOGUI && schedulerUI->getSchedulerStatus() == SCHEDULER_QUIT) {
      QUIT = 1;
      if (!NOMOBILE && !SIMULATE_MOBILE)
	schedulerTalker->quit();
      if (!SIMULATE_VEHICLE)
	vehicleTalker->quit();
    }
    usleep(100000);
  }

  fclose (logFile);

  if (!NOOP && NOGUI) {
    pthread_join(operatorThread, 0);
    pthread_mutex_destroy(&schedulerMutex);
  }
  else if (!NOGUI) {
    schedulerUI->finishConsole();
    delete schedulerUI;
  }
  if (!NOMOBILE && !SIMULATE_MOBILE) {
    pthread_join(talkerThread, 0);
    delete schedulerTalker;
  }
  if (!SIMULATE_VEHICLE) {
    pthread_join(vehicleThread, 0);
    delete vehicleTalker;
  }
  delete scheduler;
}





/* TODO: Report invalid task to the customer */
void reportInvalidMobileTask(Task task, int errorCode)
{
  if (!NOMOBILE && !SIMULATE_MOBILE) {
    schedulerTalker->sendTaskStatus(task.customerID, task.taskID, 
				    -1, errorCode);
  }
  else {
    char errMessage[1024];
    if (errorCode == INVALID_PICKUP)
      strncpy(errMessage, "Invalid pickup", sizeof(errMessage) - 1);
    else if (errorCode == INVALID_DROPOFF)
      strncpy(errMessage, "Invalid dropoff", sizeof(errMessage) - 1);
    else if (errorCode == INVALID_PICKUP_DROPOFF_PAIR)
      strncpy(errMessage, "Invalid Invalid pickup and dropoff pair", sizeof(errMessage) - 1);
    else if (errorCode == NO_AVAILABLE_VEHICLE)
      strncpy(errMessage, "Vehicles not available", sizeof(errMessage) - 1);
    else
      strncpy(errMessage, "Task not exist", sizeof(errMessage) - 1);
    
    ERROR("Task %d <%d,%d> from customer %d is invalid: %s", task.id, task.pickup, task.dropoff, task.customerID, errMessage);
  }
}

/* Send task status, e.g. waiting time */
void sendMobileTaskStatus()
{
  if (!NOMOBILE && !SIMULATE_MOBILE) {
    VehicleStatus vehStatus = scheduler->getVehicleStatus();
    list<Task> tasks = scheduler->getVehicleRemainingTasks();

    if (vehStatus == VEHICLE_ON_CALL && currentTask.customerID != OPERATOR_ID)
      schedulerTalker->sendTaskStatus(currentTask.customerID, currentTask.taskID, 
				      currentTask.tpickup, currentTask.vehicleID);

    list<Task>::iterator it;
    for ( it=tasks.begin(); it != tasks.end(); it++ ) {
      if (it->customerID != OPERATOR_ID)
	schedulerTalker->sendTaskStatus(it->customerID, it->taskID, it->twait, it->vehicleID);
    }
  }
}

/* Check whether there is a new task from the customer */
bool checkMobileTask()
{
  if (!SIMULATE_MOBILE)
    return schedulerTalker->checkTask();
  else {
    srand ( time(NULL) );
    if (rand() % 45 != 0)
      return false;
    else
      return true;
  }
}

/* Get a new task from mobile customer */
Task getMobileTask()
{
  Task task;
  task.id = 0;
  if (VERBOSITY_LEVEL > 0)
    MSG("\nGetting task from mobile phone...");

  if (!SIMULATE_MOBILE) {
    int usrID, taskID, pickup, dropoff;
    schedulerTalker->recvTask(usrID, taskID, pickup, dropoff);
    task.customerID = usrID;
    task.taskID = taskID;
    task.pickup = pickup;
    task.dropoff = dropoff;
  }
  else {
    //srand ( time(NULL) );
    task.customerID = 1234;
    task.taskID = 1;
    task.pickup = rand() % NUM_STATIONS + 1;
    task.dropoff = rand() % NUM_STATIONS + 1;

    while (task.dropoff == task.pickup)
      task.dropoff = rand() % NUM_STATIONS + 1;
  }

  MSG("Got task <%d,%d,%d,%d> from mobile phone", task.customerID, task.taskID, task.pickup, task.dropoff);

  return task;
}

/* TODO: Get the status of the vehicle */
VehicleInfo getVehicleInfo(int vehicleID)
{
  if (VERBOSITY_LEVEL > 0 && NOGUI)
    MSG("Getting vehicle information...");

  VehicleInfo info;
  info.vehicleID = DEFAULT_VEHICLE_ID;
  time_t currentTime = time(NULL);
  time_t timeDiff = (currentTime - previousTime)/NUM_SECONDS_IN_MIN;
  
  if (timeDiff > 0)
    previousTime = currentTime;

  if (!SIMULATE_VEHICLE) {
    info = vehicleTalker->getVehicleInfo();
    if (SIMULATE_TIME) {
      if (info.status == VEHICLE_AVAILABLE)
	info.tremain = 0;
      else if (info.status == VEHICLE_ON_CALL) {
	info.tremain = currentTask.tpickup - timeDiff;
	if (info.tremain < 0)
	  info.tremain = 0;
      }
      else if (info.status == VEHICLE_POB) {
	info.tremain = currentTask.ttask - timeDiff;
	if (info.tremain < 0)
	  info.tremain = 0;
      }
      else if (info.status == VEHICLE_BUSY) {
	if (currentTask.tpickup > 0) {
	  info.status = VEHICLE_ON_CALL;
	  info.tremain = currentTask.tpickup - timeDiff;
	}
	else {
	  info.status = VEHICLE_POB;
	  info.tremain = currentTask.ttask - timeDiff;
	}
	if (info.tremain < 0)
	  info.tremain = 0;
      }
    }
  }
  else {
    info.isNew = true;
    if (currentTask.id < 0 || currentTask.ttask <= 0) {
      info.status = VEHICLE_AVAILABLE;
      info.tremain = 0;
    }
    else if (currentTask.tpickup > 0) {
      info.status = VEHICLE_ON_CALL;
      info.tremain = currentTask.tpickup - timeDiff;
      if (info.tremain < 0)
	info.tremain = 0;
    }
    else {
      info.status = VEHICLE_POB;
      info.tremain = currentTask.ttask - timeDiff;
      if (info.tremain < 0)
	info.tremain = 0;
    }
  }

  return info;
}

/* Send a new task to the vehicle */
void sendTask2Vehicle(Task task)
{
  MSG("Sending task %d: <%d,%d> from customer %d to vehicle", task.id, task.pickup, task.dropoff, task.customerID);
  fprintf (logFile, "\n%d: Sending task %d: <%d,%d> from customer %d to vehicle", (int) time(NULL), task.id, task.pickup, task.dropoff, task.customerID);
  if (!SIMULATE_VEHICLE) {
    vehicleTalker->sendNewTask(task.customerID, task.pickup, task.dropoff);
  }
  fprintf (logFile, "\n%d: Task sent", (int) time(NULL));
}

int addTask(Task task)
{
  if (VERBOSITY_LEVEL > 0 && NOGUI)
    cout << endl;
  
  int ret = scheduler->addTask(task.customerID, task.taskID, task.pickup, task.dropoff);
  
  if (VERBOSITY_LEVEL > 0 && NOGUI) {
    cout << "After adding task..." << endl;
    cout << "Task queue:" << endl;
    scheduler->printTasks();
  }

  return ret;
}

/* Update all the information */
void update()
{
  // Check the vehicle status
  if (VERBOSITY_LEVEL > 0 && NOGUI)
    cout << endl;

  VehicleInfo vehInfo = getVehicleInfo(DEFAULT_VEHICLE_ID);
  VehicleStatus vehStatus = vehInfo.status;

  fprintf (logFile, "\n%d: got status: %d:%d:%d", (int) time(NULL), vehInfo.status, vehInfo.tremain, vehInfo.isNew);

  // Get the next task and send to the vehicle if the vehicle is available
  if (vehStatus == VEHICLE_AVAILABLE && (vehicleIsAvailable || vehInfo.isNew)) {
    scheduler->updateVehicleStatus(vehStatus);
    if (scheduler->checkTask()) {
      if (VERBOSITY_LEVEL > 0 && NOGUI)
	cout << endl;
      
      currentTask = scheduler->getVehicleNextTask();
      sendTask2Vehicle(currentTask);
      vehicleIsAvailable = false;
      
      if (VERBOSITY_LEVEL > 0 && NOGUI) {
	cout << endl;
	cout << "After task sent..." << endl;
	cout << "Task queue:" << endl;
	scheduler->printTasks();
      }
    }
    else {
      scheduler->updateTPickupCurrent(0);
      scheduler->updateTTaskCurrent(0);
      vehicleIsAvailable = true;
    }
  }
  // Otherwise, update the remaining task time (POB) or pickup time (other status)
  else if (vehStatus != VEHICLE_AVAILABLE) {
    scheduler->updateVehicleStatus(vehStatus);
    scheduler->updateTCurrent(vehInfo.tremain);
    currentTask = scheduler->getVehicleCurrentTask();
    
    if (VERBOSITY_LEVEL > 0 && NOGUI) {
      cout << endl;
      cout << "Updated remaining time to " << vehInfo.tremain << endl;
    }
  }
  
  // Update waiting time
  if (VERBOSITY_LEVEL > 0 && NOGUI)
    cout << endl;
  
  scheduler->updateWaitTime();
  
  if (VERBOSITY_LEVEL > 0 && NOGUI) {
    cout << "After updating waiting time..." << endl;
    cout << "Task queue:" << endl;
    scheduler->printTasks();
  }
}

/* Get option from the operator. */
OperatorOption getOpOption()
{
  printf("\n");
  int option = -1;

  while (option <= OPERATOR_OPTION_FIRST || option >= OPERATOR_OPTION_LAST) {
    scanf ("%d", &option);
  }

  return (OperatorOption) option;
}

/* Obtain task info if the operator wants to add task. */
Task getOpTaskInfo()
{
  Task task;
  int pickup, dropoff;
  task.id = 0;
  printf ("  Enter the pick-up location: "); 
  scanf ("%d",&pickup); 
  printf ("  Enter the drop-off location: "); 
  scanf ("%d",&dropoff);
  task.pickup = pickup;
  task.dropoff = dropoff;
  return task;
}

/* Obtain the id of the task to be removed */
int getOpRemoveTaskID()
{
  int id;
  printf ("  Enter id of task to be removed: ");
  scanf ("%d", &id); 
  return id;
}

/* Remove task by the operator */
bool opRemoveTask(int taskID)
{
  if (VERBOSITY_LEVEL > 0 && NOGUI)
    cout << endl;
  
  bool taskRemoved = scheduler->removeTask(taskID);
  
  if (VERBOSITY_LEVEL > 0 && NOGUI) {
    cout << "After removing task..." << endl;
    cout << "Task queue:" << endl;
    scheduler->printTasks();
  }
  return taskRemoved;
}

void *operatorLoop(void* arg)
{
  pthread_mutex_t* schedulerMutex = static_cast<pthread_mutex_t*> (arg);
  Task task;
  int taskID;
  while (QUIT == 0) {
    // Options for operator
    if (NOGUI && !NOOP) {
      OperatorOption opOption = getOpOption();

      pthread_mutex_lock(schedulerMutex);
      if (opOption == OPERATOR_ADD_TASK) {
	task = getOpTaskInfo();
	task.customerID = OPERATOR_ID;
	task.taskID = 1;
	addTask(task);
      }
      else if (opOption == OPERATOR_REMOVE_TASK) {
	taskID = getOpRemoveTaskID();
	opRemoveTask(taskID);
      }
      else if (opOption == OPERATOR_VIEW_TASK_LIST) {
	cout << endl;
	scheduler->printTasks();
      }
      else if (opOption == OPERATOR_QUIT) {
	QUIT = 1;
	break;
      }
      update();
      pthread_mutex_unlock(schedulerMutex);
    }
  }
  cout << "Operator thread stops\n";
}
