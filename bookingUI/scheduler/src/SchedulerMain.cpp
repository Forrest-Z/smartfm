#include <getopt.h>
#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <pthread.h>
#include <time.h>
#include <sys/stat.h>
#include <string.h>

#include <iostream>
#include <sstream>
#include <string>
using namespace std;

#include "Scheduler.h"
#include "SchedulerUI.h"
#include "SchedulerTalker.h"
#include "VehicleTalker.h"


//------------------------------------------------------------------------------
// Global constants

/// Although the system is intended to support several vehicles, not everything
/// has been implemented, so we will use vehicle 0 only for now.
const unsigned VEHICLE_ID = 0;

/// Number of station
const int NUM_STATIONS = 3;

/// Number of seconds in minutes (for testing purpose)
const int NUM_SECONDS_IN_MIN = 1;

/// Number of seconds between two successive task info
const int NUM_SEC_TASK_INFO_SENT = 10;



//------------------------------------------------------------------------------
// Program options

int optVerbosityLevel = 0;
int optNoGUI = 0;
int optNoMobile = 0;
int optSimulateMobile = 0;
int optSimulateVehicle = 0;
int optSimulateTime = 0;
int optNoOP = 0;
char* optHostName = "localhost";
int optMobilePort = 4440;
int optVehiclePort = 8888;


//------------------------------------------------------------------------------
// Global variables

// Whether the operator has quit (i.e. with Ctrl-C)
volatile sig_atomic_t gQuit = 0;

// Scheduler and talkers
Scheduler* gScheduler = NULL;
SchedulerTalker* gSchedulerTalker = NULL;
VehicleTalker* gVehicleTalker = NULL;
SchedulerUI* gSchedulerUI = NULL;

// Threads and mutex
pthread_t gOperatorThread, gTalkerThread, gVehicleThread;
pthread_mutex_t gSchedulerMutex;

// Current task
Task gCurrentTask;
time_t gPreviousTime;

/// The name of this program
const char * gProgramName;

// Log file
FILE* gLogFile = NULL;


//------------------------------------------------------------------------------
// Function declarations

// Error handling
#define MSG(fmt, ...) \
  (fprintf(stderr, fmt "\n", ##__VA_ARGS__) ? 0 : 0)
#define ERROR(fmt, ...) \
  (fprintf(stderr, "ERROR %s:%d: " fmt "\n", __FILE__, __LINE__, ##__VA_ARGS__) ? -1 : 0)


void sigintHandler(int sig)
{
    // if the user presses CTRL-C three or more times, and the program still
    // doesn't terminate, abort
    if (gQuit++ > 1) abort();
}

/// To be used with pthread_create --> creates the function pointer called by the thread.
template<class T, void(T::*mem_fn)()>
void* thunk(void* p)
{
    (static_cast<T*>(p)->*mem_fn)();
    return 0;
}


/// Prints usage information for this program to STREAM (typically stdout
/// or stderr), and exit the program with EXIT_CODE. Does not return.
void print_usage (FILE* stream, int exit_code);

void parseOptions(int argc, char **argv);
void openLogFile();
void launchThreads();
void terminateThreads();


/// TODO: Report invalid task to the mobile customer
void reportInvalidMobileTask(Task task, int errorCode);

/// Send task status, e.g. waiting time to mobile phones
void sendMobileTaskStatus();

/// Check whether there is a new task from the mobile customer
bool checkMobileTask();

/// Get a new task from mobile customer
Task getMobileTask();

/// TODO: Get the status of the vehicle
VehicleInfo getVehicleInfo(int vehicleID);

/// Send a new task to the vehicle
void sendTask2Vehicle(Task task);

/// Add new task to gScheduler
int addTask(Task task);

/// Update all the information
void update();

/// Get option from the operator.
OperatorOption getOpOption();

/// Obtain task info if the operator wants to add task.
Task getOpTaskInfo();

/// Obtain the id of the task to be removed
int getOpRemoveTaskID();

/// Remove task from gScheduler by the operator
bool opRemoveTask(int taskID);

/// A thread that get command from operator
void *operatorLoop(void*);


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


//------------------------------------------------------------------------------
// Main function

int main(int argc, char **argv)
{
    /*
    // catch CTRL-C and exit cleanly, if possible
    signal(SIGINT, sigintHandler);
    // and SIGTERM, i.e. when someone types "kill <pid>" or the like
    signal(SIGTERM, sigintHandler);
    signal(SIGHUP, sigintHandler);
    */

    parseOptions(int argc, char **argv);

    //cout << "Connecting to " << optHostName << ":" << optMobilePort << endl;

    gScheduler = new Scheduler(optVerbosityLevel);
    gCurrentTask = gScheduler->getVehicleCurrentTask();

    openLogFile();

    launchThreads();

    time_t prevTimeTaskInfo = time(NULL);
    gPreviousTime = time(NULL);

    while(gQuit == 0)
    {
        if (!optNoOP && optNoGUI)
            pthread_mutex_lock(&gSchedulerMutex);
        else if (!optNoGUI)
            gSchedulerUI->updateConsole();

        if (!optNoMobile && checkMobileTask()) {
            // Get a new task from mobile phone
            Task task = getMobileTask();
            // Add a new task to the gScheduler
            int ret = addTask(task);
            // Confirm task if the task is valid
            if (ret >= 0 && task.pickup != -1 && task.dropoff != -1 && !optSimulateMobile) {
                task = gScheduler->getTask(ret);
                gSchedulerTalker->sendTaskStatus(task.customerID, task.taskID, task.twait, task.vehicleID);
                fprintf (gLogFile, "\n%d: Sending status: %d:%d:%d:%d to the server", (int) time(NULL), task.customerID, task.taskID, task.twait, task.vehicleID);
            }
            else if (ret >= 0 && task.pickup == -1 && task.dropoff == -1 && !optSimulateMobile) {
                gSchedulerTalker->sendTaskStatus(task.customerID, task.taskID, -1, TASK_REMOVED);
                fprintf (gLogFile, "\n%d: Sending status: %d:%d:%d:%d to the server", (int) time(NULL), task.customerID, task.taskID, -1, TASK_REMOVED);
            }
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

        if (!optNoOP && optNoGUI)
            pthread_mutex_unlock(&gSchedulerMutex);

        if (!optNoGUI && gSchedulerUI->getSchedulerStatus() == SCHEDULER_gQuit) {
            gQuit = 1;
        if (!optNoMobile && !optSimulateMobile)
            gSchedulerTalker->quit();
        if (!optSimulateVehicle)
            gVehicleTalker->quit();
        }
        usleep(100000);
    }

    fclose (gLogFile);

    terminateThreads();

    delete gScheduler;
} //main()


//------------------------------------------------------------------------------
// Helper functions

void print_usage (FILE* stream, int exit_code)
{
    fprintf( stream, "Usage:  %s [options]\n", gProgramName );
    fprintf( stream, "  --host hostname   Specify the IP address of the server "
            "for talking to mobile phone. The default is localhost.\n");
    fprintf( stream, "  --mport mport     Specify the port for talking to mobile "
            "phones (any number between 2000 and 65535). The default is 4440.\n");
    fprintf( stream, "  --vport vport     Specify the port for talking to the "
            "vehicle (any number between 2000 and 65535). The default is 4444.\n");
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


void parseOptions(int argc, char **argv)
{
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
        {"verbose",     no_argument,       &optVerbosityLevel,        1},
        {"nogui",       no_argument,       &optNoGUI,                  1},
        {"nomobile",    no_argument,       &optNoMobile,               1},
        {"simmobile",   no_argument,       &optSimulateMobile,        1},
        {"simvehicle",  no_argument,       &optSimulateVehicle,       1},
        {"simtime",     no_argument,       &optSimulateTime,          1},
        {"noop",        no_argument,       &optNoOP,                   1},
        {"host",        required_argument, 0,                       'H'},
        {"mport",       required_argument, 0,                       'm'},
        {"vport",       required_argument, 0,                       'v'},
        {"help",        no_argument,       0,                       'h'},
        {0,0,0,0}
    };

    /* Remember the name of the program, to incorporate in messages.
        The name is stored in argv[0]. */
    gProgramName = argv[0];

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

        case '?':
            /* The user specified an invalid option. */
            /* Print usage information to standard error, and exit with exit
            code one (indicating abnormal termination). */
            print_usage(stderr, 1);
            break;

        case 'H':
            optHostName = optarg;
            break;

        case 'm':
            optMobilePort = atoi(optarg);
            break;

        case 'v':
            optVehiclePort = atoi(optarg);
            break;

        case -1: /* Done with options. */
            break;
        }
    }
}


void openLogFile()
{
    char timestr[64];
    time_t t = time(NULL);
    strftime(timestr, sizeof(timestr), "%F-%a-%H-%M", localtime(&t));

    ostringstream oss;
    oss  << "log_scheduler" << "." << timestr << ".log";
    string logFileName = oss.str();

    // if it exists already, append .1, .2, .3 ...
    string suffix = "";
    struct stat st;
    for (int i = 1; stat((logFileName + suffix).c_str(), &st) == 0; i++)
    {
        ostringstream tmp;
        tmp << '.' << i;
        suffix = tmp.str();
    }
    logFileName += suffix;
    gLogFile = fopen(logFileName.c_str(), "w");
}


void launchThreads()
{
    if (!optNoOP && optNoGUI) {
        printf("Usage:\n");
        printf ("  (%d) Add task\n", OPERATOR_ADD_TASK);
        printf ("  (%d) Cancel task\n", OPERATOR_REMOVE_TASK);
        printf ("  (%d) View task list\n", OPERATOR_VIEW_TASK_LIST);
        printf ("  (%d) Update\n", OPERATOR_UPDATE);
        printf ("  (%d) Quit\n", OPERATOR_QUIT);

        pthread_mutex_init(&gSchedulerMutex, NULL);
        int ret = pthread_create(&gOperatorThread, NULL, &operatorLoop, &gSchedulerMutex);
        if (ret != 0)
            ERROR("return code from pthread_create(): %d", ret);
    }
    else if (!optNoGUI) {
        gSchedulerUI = new SchedulerUI(gScheduler);
        gSchedulerUI->initConsole();
        gSchedulerUI->updateConsole();
    }

    if (!optNoMobile && !optSimulateMobile) {
        ostringstream ss;
        ss << optHostName;
        gSchedulerTalker = new SchedulerTalker(ss.str(), optMobilePort, optVerbosityLevel);
        pthread_attr_t attr;
        pthread_attr_init(&attr);
        pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_DETACHED);
        int ret = pthread_create(&gTalkerThread, &attr,
                                thunk<SchedulerTalker, &SchedulerTalker::runMobileReceiver>,
                                gSchedulerTalker);
        if (ret != 0)
        ERROR("return code from pthread_create(): %d", ret);
    }

    if (!optSimulateVehicle) {
        MSG("Waiting for connection from the vehicle.");
        if (!optNoGUI)
            gSchedulerUI->updateConsole();
        gVehicleTalker = new VehicleTalker(optVehiclePort, optVerbosityLevel);
        pthread_attr_t attr;
        pthread_attr_init(&attr);
        pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_DETACHED);
        int ret = pthread_create(&gVehicleThread, &attr,
                                thunk<VehicleTalker, &VehicleTalker::runVehicleReceiver>,
                                gVehicleTalker);
        MSG("Vehicle ready.");
    }
}


void terminateThreads()
{
    if (!optNoOP && optNoGUI) {
        pthread_join(gOperatorThread, 0);
        pthread_mutex_destroy(&gSchedulerMutex);
    }
    else if (!optNoGUI) {
        gSchedulerUI->finishConsole();
        delete gSchedulerUI;
    }
    if (!optNoMobile && !optSimulateMobile) {
        pthread_join(gTalkerThread, 0);
        delete gSchedulerTalker;
    }
    if (!optSimulateVehicle) {
        pthread_join(gVehicleThread, 0);
        delete gVehicleTalker;
    }
}


void reportInvalidMobileTask(Task task, int errorCode)
{
    if (!optNoMobile && !optSimulateMobile)
    {
        gSchedulerTalker->sendTaskStatus(task.customerID, task.taskID,
                                        -1, errorCode);
        fprintf (gLogFile, "\n%d: Sending status: %d:%d:%d:%d to the server",
                 (int) time(NULL), task.customerID, task.taskID, -1, errorCode);
    }
    else
    {
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


void sendMobileTaskStatus()
{
    if (!optNoMobile && !optSimulateMobile)
    {
        VehicleStatus vehStatus = gScheduler->getVehicleStatus();
        list<Task> tasks = gScheduler->getVehicleRemainingTasks();

        if (vehStatus == VEHICLE_ON_CALL && gCurrentTask.customerID != OPERATOR_ID)
        {
            gSchedulerTalker->sendTaskStatus(gCurrentTask.customerID, gCurrentTask.taskID,
                                            gCurrentTask.tpickup, gCurrentTask.vehicleID);
            fprintf (gLogFile, "\n%d: Sending status: %d:%d:%d:%d to the server",
                     (int) time(NULL), gCurrentTask.customerID, gCurrentTask.taskID,
                     gCurrentTask.tpickup, gCurrentTask.vehicleID);
        }

        list<Task>::iterator it;
        for ( it=tasks.begin(); it != tasks.end(); it++ )
        {
            if (it->customerID != OPERATOR_ID)
            {
                gSchedulerTalker->sendTaskStatus(it->customerID, it->taskID, it->twait, it->vehicleID);
                fprintf (gLogFile, "\n%d: Sending status: %d:%d:%d:%d to the server", (int) time(NULL), it->customerID, it->taskID, it->twait, it->vehicleID);
            }
        }
    }
}


bool checkMobileTask()
{
    if (!optSimulateMobile) {
        return gSchedulerTalker->checkTask();
    }
    else {
        srand ( time(NULL) );
        if (rand() % 45 != 0)
            return false;
        else
            return true;
    }
}


Task getMobileTask()
{
    Task task;
    task.id = 0;
    if (optVerbosityLevel > 0)
        MSG("\nGetting task from mobile phone...");

    if (!optSimulateMobile)
    {
        int usrID, taskID, pickup, dropoff;
        gSchedulerTalker->recvTask(usrID, taskID, pickup, dropoff);
        task.customerID = usrID;
        task.taskID = taskID;
        task.pickup = pickup;
        task.dropoff = dropoff;
    }
    else
    {
        //srand ( time(NULL) );
        task.customerID = 1234;
        task.taskID = 1;
        task.pickup = rand() % NUM_STATIONS + 1;
        task.dropoff = rand() % NUM_STATIONS + 1;

        while (task.dropoff == task.pickup)
            task.dropoff = rand() % NUM_STATIONS + 1;
    }

    MSG("Got task <%d,%d,%d,%d> from mobile phone", task.customerID, task.taskID, task.pickup, task.dropoff);
    fprintf(gLogFile, "\n%d: Got task <%d,%d,%d,%d> from mobile phone", (int) time(NULL), task.customerID, task.taskID, task.pickup, task.dropoff);

    return task;
}


VehicleInfo getVehicleInfo(int vehicleID)
{
    if (optVerbosityLevel > 0 && optNoGUI)
        MSG("Getting vehicle information...");

    VehicleInfo info;
    info.vehicleID = DEFAULT_VEHICLE_ID;
    time_t currentTime = time(NULL);
    time_t timeDiff = (currentTime - gPreviousTime)/NUM_SECONDS_IN_MIN;

    if (timeDiff > 0)
        gPreviousTime = currentTime;

    if (!optSimulateVehicle)
    {
        info = gVehicleTalker->getVehicleInfo();
        if (optSimulateTime)
        {
            if (info.status == VEHICLE_AVAILABLE)
            {
                info.tremain = 0;
            }
            else if (info.status == VEHICLE_ON_CALL)
            {
                info.tremain = gCurrentTask.tpickup - timeDiff;
                if (info.tremain < 0)
                    info.tremain = 0;
            }
            else if (info.status == VEHICLE_POB)
            {
                info.tremain = gCurrentTask.ttask - timeDiff;
                if (info.tremain < 0)
                    info.tremain = 0;
            }
            else if (info.status == VEHICLE_BUSY)
            {
                if (gCurrentTask.tpickup > 0) {
                    info.status = VEHICLE_ON_CALL;
                    info.tremain = gCurrentTask.tpickup - timeDiff;
                }
                else {
                    info.status = VEHICLE_POB;
                    info.tremain = gCurrentTask.ttask - timeDiff;
                }
                if (info.tremain < 0)
                    info.tremain = 0;
            }
        }
    }
    else
    {
        info.isNew = true;
        if (gCurrentTask.id < 0 || gCurrentTask.ttask <= 0)
        {
            info.status = VEHICLE_AVAILABLE;
            info.tremain = 0;
        }
        else if (gCurrentTask.tpickup > 0)
        {
            info.status = VEHICLE_ON_CALL;
            info.tremain = gCurrentTask.tpickup - timeDiff;
            if (info.tremain < 0)
                info.tremain = 0;
        }
        else
        {
            info.status = VEHICLE_POB;
            info.tremain = gCurrentTask.ttask - timeDiff;
            if (info.tremain < 0)
                info.tremain = 0;
        }
    }

    return info;
}


void sendTask2Vehicle(Task task)
{
    MSG("Sending task %d: <%d,%d> from customer %d to vehicle",
        task.id, task.pickup, task.dropoff, task.customerID);
    fprintf (gLogFile, "\n%d: Sending task %d: <%d,%d> from customer %d to vehicle",
             (int) time(NULL), task.id, task.pickup, task.dropoff, task.customerID);
    if (!optSimulateVehicle)
        gVehicleTalker->sendNewTask(task.customerID, task.pickup, task.dropoff);
    fprintf (gLogFile, "\n%d: Task sent", (int) time(NULL));
}


int addTask(Task task)
{
    if (optVerbosityLevel > 0 && optNoGUI)
        cout << endl;

    int ret = gScheduler->addTask(task.customerID, task.taskID, task.pickup, task.dropoff);
    fprintf (gLogFile, "\n%d: Added task %d:%d:%d:%d to the gScheduler", (int) time(NULL), task.customerID, task.taskID, task.pickup, task.dropoff);

    if (optVerbosityLevel > 0 && optNoGUI) {
        cout << "After adding task..." << endl;
        cout << "Task queue:" << endl;
        gScheduler->printTasks();
    }

    return ret;
}


void update()
{
    bool vehicleIsAvailable = false;

    // Check the vehicle status
    if (optVerbosityLevel > 0 && optNoGUI)
        cout << endl;

    VehicleInfo vehInfo = getVehicleInfo(DEFAULT_VEHICLE_ID);
    VehicleStatus vehStatus = vehInfo.status;

    fprintf (gLogFile, "\n%d: got status: %d:%d:%d",
             (int) time(NULL), vehInfo.status, vehInfo.tremain, vehInfo.isNew);

    // Get the next task and send to the vehicle if the vehicle is available
    if (vehStatus == VEHICLE_AVAILABLE && (vehicleIsAvailable || vehInfo.isNew))
    {
        gScheduler->updateVehicleStatus(vehStatus);
        if (gScheduler->checkTask())
        {
            if (optVerbosityLevel > 0 && optNoGUI)
                cout << endl;

            gCurrentTask = gScheduler->getVehicleNextTask();
            sendTask2Vehicle(gCurrentTask);
            vehicleIsAvailable = false;

            if (optVerbosityLevel > 0 && optNoGUI) {
                cout << endl;
                cout << "After task sent..." << endl;
                cout << "Task queue:" << endl;
                gScheduler->printTasks();
            }
        }
        else
        {
            gScheduler->updateTPickupCurrent(0);
            gScheduler->updateTTaskCurrent(0);
            vehicleIsAvailable = true;
        }
    }
    // Otherwise, update the remaining task time (POB) or pickup time (other status)
    else if (vehStatus != VEHICLE_AVAILABLE)
    {
        gScheduler->updateVehicleStatus(vehStatus);
        gScheduler->updateTCurrent(vehInfo.tremain);
        gCurrentTask = gScheduler->getVehicleCurrentTask();
        fprintf (gLogFile, "\n%d: Updated vehicle status to %d", (int) time(NULL), vehStatus, vehInfo.tremain);

        if (optVerbosityLevel > 0 && optNoGUI) {
            cout << endl;
            cout << "Updated remaining time to " << vehInfo.tremain << endl;
        }
    }

    // Update waiting time
    if (optVerbosityLevel > 0 && optNoGUI)
        cout << endl;

    gScheduler->updateWaitTime();

    if (optVerbosityLevel > 0 && optNoGUI)
    {
        cout << "After updating waiting time..." << endl;
        cout << "Task queue:" << endl;
        gScheduler->printTasks();
    }
}


OperatorOption getOpOption()
{
    printf("\n");
    int option = -1;

    while (option <= OPERATOR_OPTION_FIRST || option >= OPERATOR_OPTION_LAST)
        scanf ("%d", &option);

    return (OperatorOption) option;
}


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


int getOpRemoveTaskID()
{
    int id;
    printf ("  Enter id of task to be removed: ");
    scanf ("%d", &id);
    return id;
}


bool opRemoveTask(int taskID)
{
    if (optVerbosityLevel > 0 && optNoGUI)
        cout << endl;

    bool taskRemoved = gScheduler->removeTask(taskID);

    if (optVerbosityLevel > 0 && optNoGUI) {
        cout << "After removing task..." << endl;
        cout << "Task queue:" << endl;
        gScheduler->printTasks();
    }
    return taskRemoved;
}


void *operatorLoop(void* arg)
{
    pthread_mutex_t* gSchedulerMutex = static_cast<pthread_mutex_t*> (arg);
    Task task;
    int taskID;
    while (gQuit == 0)
    {
        // Options for operator
        if (optNoGUI && !optNoOP)
        {
            OperatorOption opOption = getOpOption();

            pthread_mutex_lock(gSchedulerMutex);
            if (opOption == OPERATOR_ADD_TASK)
            {
                task = getOpTaskInfo();
                task.customerID = OPERATOR_ID;
                task.taskID = 1;
                addTask(task);
            }
            else if (opOption == OPERATOR_REMOVE_TASK)
            {
                taskID = getOpRemoveTaskID();
                opRemoveTask(taskID);
            }
            else if (opOption == OPERATOR_VIEW_TASK_LIST)
            {
                cout << endl;
                gScheduler->printTasks();
            }
            else if (opOption == OPERATOR_gQuit)
            {
                gQuit = 1;
                break;
            }
            update();
            pthread_mutex_unlock(gSchedulerMutex);
        }
    }
    cout << "Operator thread stops\n";
}
