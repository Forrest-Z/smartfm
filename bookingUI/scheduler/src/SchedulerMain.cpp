#include <stdio.h>
#include <stdlib.h>
#include <limits.h>
#include <getopt.h>
#include <time.h>
#include <string.h>
#include <signal.h>
#include <math.h>

#include <iostream>
#include <sstream>
#include <string>
using namespace std;

#include <StationPath.h>
#include <SimulatedRoutePlanner.h>
#include <MissionComm.h>

#include "Scheduler.h"
#include "SchedulerUI.h"
#include "DBTalker.h"

using SchedulerTypes::Task;
using SchedulerTypes::Duration;


//------------------------------------------------------------------------------
// A vehicle class for simulated vehicles
class SimulatedVehicle {
public:
    SimulatedRoutePlanner rp;
    DBMissionComm comm;
    DummyPassengerComm pc;

    SimulatedVehicle(StationPaths & sp, string vname, float speed, string hostname)
    : rp(sp,speed), comm(rp, pc, hostname+"/booking", vname) { }
};


//------------------------------------------------------------------------------
// Program options

int optVerbosityLevel = 0;
int optNoGUI = 0;
int optSimulateMobile = 0;
int optSimulateVehicle = 0;
int optNoOP = 0;
string optHostName = "localhost";
float optSimVehicleSpeed = Scheduler::NOMINAL_VEL; ///< control the speed of the simulated vehicle
float optNewTaskProba = 0.001; ///< control the probability of a new task in optSimulateMobile mode



//------------------------------------------------------------------------------
// Global variables

// Whether the operator has quit (i.e. with Ctrl-C)
volatile sig_atomic_t gQuit = 0;

// Scheduler and talkers
Scheduler* gScheduler = NULL;
DBTalker* gDBTalker = NULL;
SchedulerUI* gSchedulerUI = NULL;

time_t gPreviousTime = 0;

/// The name of this program
const char * gProgramName;

// logger
DebugLogger logger;

StationPaths gStationPaths;
const StationList & gStationList = gStationPaths.knownStations();

SimulatedVehicle * gSimulatedVehicle = 0;

//------------------------------------------------------------------------------
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
// Function declarations

/// Prints usage information for this program to STREAM (typically stdout
/// or stderr), and exit the program with EXIT_CODE. Does not return.
void print_usage (FILE* stream, int exit_code);

void parseOptions(int argc, char **argv);

/// Create random tasks and make requests
void addMobileTask();

void createObjects();

/// get commands from operator in text mode
void textUI();


void sigintHandler(int sig)
{
    //First time: raise the QUIT flag
    //second time: abort
    if( ++gQuit > 1 )
        abort();
}



//------------------------------------------------------------------------------
// Main function

int main(int argc, char **argv)
{
    int exit_code = 0;

    // catch CTRL-C and exit cleanly, if possible
    signal(SIGINT, sigintHandler);
    // and SIGTERM, i.e. when someone types "kill <pid>" or the like
    signal(SIGTERM, sigintHandler);
    signal(SIGHUP, sigintHandler);

    srand ( time(NULL) );
    parseOptions(argc, argv);

    time_t prevTimeTaskInfo = time(NULL);

    try
    {
        createObjects();

        while(gQuit == 0)
        {
            if( optNoGUI && ! optNoOP )
                textUI();
            else if( !optNoGUI )
                gSchedulerUI->updateConsole();

            if( optSimulateMobile )
                addMobileTask();

            // Update vehicle status, waiting time and send new task to vehicle if
            // it has completes the previous task.
            time_t currentTime = time(NULL);
            if( currentTime - prevTimeTaskInfo >= 3 ) {
                prevTimeTaskInfo = currentTime;
                gScheduler->update();
            }

            if (!optNoGUI && gSchedulerUI->getSchedulerStatus() == SchedulerUI::SCHEDULER_QUIT)
                gQuit = 1;

            usleep( (unsigned)(0.1 *1e6) );

        } //while(gQuit == 0)

    } //try
    catch( StationDoesNotExistException & e ) {
        ERROR_(logger, "Caught StationDoesNotExistException: station %s does not exist.", e.what());
        exit_code = 1;
    }
    catch( SchedulerException & e ) {
        ERROR_(logger, "Caught SchedulerException: %s.", e.what());
        exit_code = 1;
    }
    catch( exception & e )
    {
        ERROR_(logger, "Caught exception: %s", e.what());
        exit_code = 1;
    }

    if (!optNoGUI)
    {
        gSchedulerUI->finishConsole();
        delete gSchedulerUI;
    }

    delete gScheduler;
    delete gDBTalker;
    logger.close();

    return exit_code;
} //main()


//------------------------------------------------------------------------------
// Helper functions

void print_usage (FILE* stream, int exit_code)
{
    stringstream ss;
    ss <<"Usage:  " <<gProgramName <<" [options]" <<endl;
    ss <<"  --host hostname   Specify the IP address of the server. The default is localhost." <<endl;
    ss <<"  --verbose lvl     Set verbosity level to lvl." <<endl;
    ss <<"  --nogui           Run in plain text mode (as opposed to using the NCurse interface)." <<endl;
    ss <<"  --simmobile       Do not intend to communicate with mobile phones. Simulate mobile phone users instead (automatically generate random tasks)." <<endl;
    ss <<"  --taskproba p     Sets the probability of a new task (only used when simmobile is on, default value is " <<optNewTaskProba <<")." <<endl;
    ss <<"  --simvehicle      Simulate vehicles' status." <<endl;
    ss <<"  --simspeed tf     Speed of the vehicle in m/s (default is 1m/s)." <<endl;
    ss <<"  --noop            No interaction with user. (daemon mode). Only active when --nogui is set." <<endl;
    ss <<"  --help            Display this message." <<endl;

    fprintf( stream, "%s", ss.str().c_str());

    exit(exit_code);
}


void parseOptions(int argc, char **argv)
{
    int ch;
    const char* const short_options = "h";

    /* An array describing valid long options. */
    static struct option long_options[] =
    {
        // first: long option (--option) string
        // second: 0 = no_argument, 1 = required_argument, 2 = optional_argument
        // third: if pointer, set variable to value of fourth argument
        //        if NULL, getopt_long returns fourth argument
        {"nogui",       no_argument,       &optNoGUI,                 1},
        {"simmobile",   no_argument,       &optSimulateMobile,        1},
        {"simvehicle",  no_argument,       &optSimulateVehicle,       1},
        {"noop",        no_argument,       &optNoOP,                  1},
        {"verbose",     required_argument, 0,                       'V'},
        {"simspeed",    required_argument, 0,                       'F'},
        {"taskproba",   required_argument, 0,                       'P'},
        {"host",        required_argument, 0,                       'H'},
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

        case 'F':
            optSimVehicleSpeed = atof(optarg);
            break;

        case 'P':
            optNewTaskProba = atof(optarg);
            break;

        case 'V':
            optVerbosityLevel = atoi(optarg);
            break;

        case -1: /* Done with options. */
            break;
        }
    }
}

void createObjects()
{
    // Create a log file name
    char timestr[64];
    time_t now = time(NULL);
    strftime(timestr, sizeof(timestr), "%F-%a-%H-%M-%S", localtime(&now));
    ostringstream oss;
    oss  << "log-" << timestr <<".log";

    // Create the logger object
    logger.setLogFile(oss.str());
    logger.setVerbosityLevel(optVerbosityLevel);
    logger.setConsoleStream(stderr);

    // Create the database talker object
    string username = "fmauto", passwd = "smartfm", dbname = "booking";
    gDBTalker = new DBTalker(optHostName, username, passwd, dbname);
    gDBTalker->copyLoggingSettings(logger);

    // Create the scheduler
    gScheduler = new Scheduler(*gDBTalker);
    gScheduler->copyLoggingSettings(logger);

    if( optSimulateVehicle )
    {
        // Create a simulated vehicle
        gSimulatedVehicle = new SimulatedVehicle(gStationPaths, "golfcart1", optSimVehicleSpeed, optHostName);
        gSimulatedVehicle->comm.startThread();
        gSimulatedVehicle->rp.startThread();

        // Wait for current station input
        while( ! gSimulatedVehicle->rp.getCurrentStation().isValid() )
            sleep(1);
    }

    if (!optNoGUI)
    {
        // Create the NCurse based UI
        gSchedulerUI = new SchedulerUI(*gScheduler, *gDBTalker);
        gSchedulerUI->copyLoggingSettings(logger);
        gSchedulerUI->initConsole();
        gSchedulerUI->updateConsole();
    }
}

double random_(double min, double max)
{
    return (((double)rand())/RAND_MAX) * (max-min) + min;
}

double random_()
{
    return random_(0,1);
}

unsigned randu_(unsigned max)
{
    return (unsigned) random_(0,max);
}

void addMobileTask()
{
    /* Adding some random tasks to the database.
    * The proba of a random task is equal to optNewTaskProba multiplied by
    * optTimeFactor (to scale for fast simulations).
    * Several tasks can be issued. The probability of a new task is half
    * the probability of the previous task.
    * For each task, pickup and dropoff stations are picked at random.
    */
    unsigned i = 0;
    while( random_() < optNewTaskProba*optSimVehicleSpeed/pow(2,i++) )
    {
        unsigned s1 = randu_(gStationList.size());
        unsigned s2;
        do
            s2 = randu_(gStationList.size());
        while (s1==s2);

        string customer = "cust1";
        Station st1( gStationList(s1) );
        Station st2( gStationList(s2) );
        MSGLOG_(logger, 1, "Adding simulated mobile task: %s,%s,%s",
                customer.c_str(), st1.c_str(), st2.c_str());
        gDBTalker->makeBooking(customer, st1, st2);
    }
}


// TODO: Add a mechanism to escape from the loop.
int getNumeric(int min=0, int max=INT_MAX)
{
    int val = -1;

    while ( true )
    {
        if( cin >>val )
        {
            if( val >= min && val <= max )
                break;
            else
                cout <<"Please enter a value between " <<min <<"and " <<max <<".\n";
        }
        else
        {
            cout <<"Please enter a numeric value." <<endl;
            cin.clear();
            cin.ignore(80, '\n');
        }
    }

    return val;
}


void printMenu()
{
    printf("Menu:\n");
    printf ("  (%d) Add task\n", OPERATOR_ADD_TASK);
    printf ("  (%d) Cancel task\n", OPERATOR_REMOVE_TASK);
    printf ("  (%d) View task list\n", OPERATOR_VIEW_TASK_LIST);
    printf ("  (%d) Update\n", OPERATOR_UPDATE);
    printf ("  (%d) Quit\n", OPERATOR_QUIT);
    printf ("Your choice: ");
}


void textUI()
{
    printMenu();
    OperatorOption opOption = (OperatorOption) getNumeric(OPERATOR_OPTION_FIRST+1, OPERATOR_OPTION_LAST-1);

    if (opOption == OPERATOR_ADD_TASK)
    {
        gStationList.print();
        Station pickup = gStationList.prompt("  Enter the pick-up location: ");
        Station dropoff = gStationList.prompt("  Enter the drop-off location: ");
        unsigned id = gDBTalker->makeBooking("cust1", pickup, dropoff);
        MSGLOG_(logger, 1, "Created request %u.", id);
    }
    else if (opOption == OPERATOR_REMOVE_TASK)
    {
        printf ("  Enter id of task to be cancelled: ");
        unsigned id = getNumeric();
        gDBTalker->custCancel(id);
        MSGLOG_(logger, 1, "Cancelled task %u.", id);
    }
    else if (opOption == OPERATOR_VIEW_TASK_LIST)
    {
        cout <<endl << gScheduler->toString() <<endl <<endl;
    }
    else if (opOption == OPERATOR_QUIT)
    {
        gQuit = 1;
        return;
    }
}
