#include <unistd.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <sys/poll.h>
#include <assert.h>

#include "SchedulerUI.h"

#define DEBUG_LOGFILE logFile
#define DEBUG_VERBOSITY_VAR verbosity_level
#include "debug_macros.h"

#define DEFAULT_VEHICLE_ID "golfcart1"

using namespace std;
using SchedulerTypes::Task;


const int CURSOR_ROW = 21;
const int CURSOR_COL = 0;

// Default constructor
SchedulerUI::SchedulerUI(const Scheduler & s, DBTalker & db_talker) : scheduler(s), dbTalker(db_talker)
{
    this->focusButtonInd = NEW_PICKUP;
    this->focusNewPickup = true;
    this->focusNewDropoff = false;
    this->focusTaskID = false;
    this->newPickupTextSize = 0;
    this->newDropoffTextSize = 0;
    this->taskIDTextSize = 0;
    this->schedulerStatus = SCHEDULER_RUNNING;
}

SchedulerUI::~SchedulerUI()
{

}

void SchedulerUI::setLogFile(FILE *logfile)
{
    this->logFile = logfile;
}

void SchedulerUI::setVerbosityLevel(unsigned lvl)
{
    this->verbosity_level = lvl;
}

// Initialize console display
void SchedulerUI::initConsole()
{
    const char *displayTemplate =
    "-----------------------------------------------------------------------------------------------------------------\n"
    " Vehicle Scheduler \n"
    "-----------------------------------------------------------------------------------------------------------------\n"
    " Vehicle:                          Status:                    |                       Tasks                      \n"
    "   Current task                                               |  ID | Customer |  Pick-Up   |  Drop-Off  | tWait \n"
    "     ID:                                                      |                                                  \n"
    "     Pickup station:                                          |                                                  \n"
    "     Dropoff station:                                         |                                                  \n"
    "     Time remaining:                                          |                                                  \n"
    "--------------------------------------------------------------|                                                  \n"
    "                                                              |                                                  \n"
    " Pickup station:          Dropoff station:         [ ADD(A) ] |                                                  \n"
    "                                                              |                                                  \n"
    "                                                              |                                                  \n"
    " Task ID:                           [ VIEW(V) ] [ CANCEL(C) ] |                                                  \n"
    "                                                              |                                                  \n"
    "                                                              |                                                  \n"
    "-----------------------------------------------------------------------------------------------------------------\n"
    "                                                                                                                 \n"
    "                                                                                                                 \n"
    "                                                                                                                 \n"
    "                                                                                                                 \n"
    "                                                                                                                 \n"
    "-----------------------------------------------------------------------------------------------------------------\n"
    "                                                                                                    [ QUIT(Q) ]  \n";
    /*
     *    "-----------------------------------------------------------------------------------------------------------------\n"
     *    " Vehicle Scheduler \n"
     *    "-----------------------------------------------------------------------------------------------------------------\n"
     *    " Vehicle: %%VEHICLE_ID%%           Status: %%VEHICLE_STATUS%% |                  Tasks                           \n"
     *    "   Current task                                               |    ID  Customer  Pick-Up  Drop-Off  Waiting Time \n"
     *    "     ID: %%CURRENT_TASK_ID%%                                  |  %%TASK00%%                                      \n"
     *    "     Pickup station: %%CURRENT_TASK_PICKUP%%                  |  %%TASK01%%                                      \n"
     *    "     Dropoff station: %%CURRENT_TASK_DROPOFF%%                |  %%TASK02%%                                      \n"
     *    "     Time remaining: %%CURRENT_TASK_TIME_REMAINING%%          |  %%TASK03%%                                      \n"
     *    "--------------------------------------------------------------|  %%TASK04%%                                      \n"
     *    "                                                              |  %%TASK05%%                                      \n"
     *    " Pickup station: %%_ap%%  Dropoff station: %%_ad%%    [%ADD%] |  %%TASK06%%                                      \n"
     *    "   %%NEW_TASK_ID%%                                            |  %%TASK07%%                                      \n"
     *    "                                                              |  %%TASK08%%                                      \n"
     *    " Task ID: %%VC_TASK_ID%%                  [%VIEW%] [%CANCEL%] |  %%TASK09%%                                      \n"
     *    "   %%VC_TASK_VEHICLE_ID%%                                     |  %%TASK10%%                                      \n"
     *    "   %%VC_TASK_TIME_WAIT%%                                      |  %%TASK11%%                                      \n"
     *    "-----------------------------------------------------------------------------------------------------------------\n"
     *    "  %stderr%                                                                                                       \n"
     *    "  %stderr%                                                                                                       \n"
     *    "  %stderr%                                                                                                       \n"
     *    "  %stderr%                                                                                                       \n"
     *    "  %stderr%                                                                                                       \n"
     *    "-----------------------------------------------------------------------------------------------------------------\n"
     *    "                                                                                                    [%QUIT(Q)%]  \n";
     */

    this->textFields[VEHICLE_ID].row = 3;
    this->textFields[VEHICLE_ID].col = 10;
    this->textFields[VEHICLE_STATUS].row = this->textFields[VEHICLE_ID].row;
    this->textFields[VEHICLE_STATUS].col = this->textFields[VEHICLE_ID].col + 33;
    this->textFields[CURRENT_TASK_ID].row = this->textFields[VEHICLE_ID].row + 2;
    this->textFields[CURRENT_TASK_ID].col = 22;
    this->textFields[CURRENT_TASK_PICKUP].row = this->textFields[CURRENT_TASK_ID].row + 1;
    this->textFields[CURRENT_TASK_PICKUP].col = 22;
    this->textFields[CURRENT_TASK_DROPOFF].row = this->textFields[CURRENT_TASK_PICKUP].row + 1;
    this->textFields[CURRENT_TASK_DROPOFF].col = 22;
    this->textFields[CURRENT_TASK_TIME_REMAINING].row = this->textFields[CURRENT_TASK_DROPOFF].row + 1;
    this->textFields[CURRENT_TASK_TIME_REMAINING].col = 22;

    this->buttons[NEW_PICKUP].row = this->textFields[CURRENT_TASK_TIME_REMAINING].row + 3;
    this->buttons[NEW_PICKUP].col = 17;
    strncpy(this->buttons[NEW_PICKUP].text, " ", sizeof(this->buttons[NEW_PICKUP].text) - 1);
    this->buttons[NEW_DROPOFF].row = this->buttons[NEW_PICKUP].row;
    this->buttons[NEW_DROPOFF].col = this->buttons[NEW_PICKUP].col + 26;
    strncpy(this->buttons[NEW_DROPOFF].text, " ", sizeof(this->buttons[NEW_DROPOFF].text) - 1);
    this->buttons[VC_TASK_ID].row = this->buttons[NEW_PICKUP].row + 3;
    this->buttons[VC_TASK_ID].col = 10;
    strncpy(this->buttons[VC_TASK_ID].text, " ", sizeof(this->buttons[VC_TASK_ID].text) - 1);

    this->textFields[NEW_TASK_ID].row = this->buttons[NEW_PICKUP].row + 1;
    this->textFields[NEW_TASK_ID].col = 3;
    this->textFields[VC_TASK_VEHICLE_ID].row = this->buttons[VC_TASK_ID].row + 1;
    this->textFields[VC_TASK_VEHICLE_ID].col = 3;
    this->textFields[VC_TASK_TIME_WAIT].row = this->textFields[VC_TASK_VEHICLE_ID].row + 1;
    this->textFields[VC_TASK_TIME_WAIT].col = 3;

    for(unsigned i = 0; i < NUM_TASK_DISPLAY; i++) {
        this->textFields[TASK_LIST_START + i].row = 5+i;
        this->textFields[TASK_LIST_START + i].col = 64;
    }

    this->buttons[ADD_TASK].row = this->buttons[NEW_PICKUP].row;
    this->buttons[ADD_TASK].col = this->buttons[NEW_DROPOFF].col + 10;
    strncpy(this->buttons[ADD_TASK].text, "ADD(A)", sizeof(this->buttons[ADD_TASK].text) - 1);
    this->buttons[ADD_TASK].key = 'a';
    this->buttons[VIEW_TASK].row = this->buttons[VC_TASK_ID].row;
    this->buttons[VIEW_TASK].col = this->buttons[VC_TASK_ID].col + 28;
    strncpy(this->buttons[VIEW_TASK].text, "VIEW(V)", sizeof(this->buttons[VIEW_TASK].text) - 1);
    this->buttons[VIEW_TASK].key = 'v';
    this->buttons[CANCEL_TASK].row = this->buttons[VC_TASK_ID].row;
    this->buttons[CANCEL_TASK].col = this->buttons[VIEW_TASK].col + 12;
    strncpy(this->buttons[CANCEL_TASK].text, "CANCEL(C)", sizeof(this->buttons[VIEW_TASK].text) - 1);
    this->buttons[CANCEL_TASK].key = 'c';
    this->buttons[QUIT_SCHEDULER].row = 24;
    this->buttons[QUIT_SCHEDULER].col = 94;
    strncpy(this->buttons[QUIT_SCHEDULER].text, "QUIT(Q)", sizeof(this->buttons[QUIT_SCHEDULER].text) - 1);
    this->buttons[QUIT_SCHEDULER].key = 'q';

    this->stderrRowBegin = 18;
    this->stderrRowEnd = 22;

    // Redirect stderr to a pipe
    pipe(this->stderrFDS);
    dup2(this->stderrFDS[1], fileno(stderr));

    // Create window
    this->win = initscr();

    // Set up default colors
    if (has_colors())
    {
        start_color();
        use_default_colors();
        init_pair(UI_TEXT_COLOR_RED, COLOR_RED, -1);
        init_pair(UI_TEXT_COLOR_GREEN, COLOR_GREEN, -1);
        init_pair(UI_TEXT_COLOR_MAGENTA, COLOR_MAGENTA, -1);
        init_pair(UI_CYAN_HIGHLIGHT, -1, COLOR_CYAN);
    }

    // Hide the cursor
    curs_set(0);

    // Draw the template
    mvwprintw(this->win, 0, 0, "%s", displayTemplate);

    // Draw buttons
    this->drawButtons();

    // Set up a stderr scrolling region
    //int maxx, maxy;
    //getmaxyx(this->win, maxy, maxx);
    //this->stderrRowEnd = maxy - 3;
    wsetscrreg(this->win, this->stderrRowBegin, this->stderrRowEnd);
    idlok(this->win, true);
    scrollok(this->win, true);

    // Enable support for function-key mapping
    noecho();
    nodelay(this->win, true);
    keypad(this->win, true);

    // Leave cursor at a sensible location
    move(CURSOR_ROW, CURSOR_COL);
    wrefresh(this->win);
}

// Finalize console display
void SchedulerUI::finishConsole()
{
    this->updateStderr();
    endwin();
    dup2(fileno(stdout), fileno(stderr));
}

void SchedulerUI::updateConsole()
{
	//MSGLOG(4, "");
    int ch = getch();
    if (ch != ERR)
        processKeyboard(ch);

    this->drawButtons();
    this->updateStderr();
    this->updateVehicleStatus();
    this->updateTaskList();

    // Leave cursor at a sensible location
    move(CURSOR_ROW, CURSOR_COL);
    wrefresh(this->win);
}

void SchedulerUI::drawButtons()
{
	//MSGLOG(4, "");
    int attr, len;
    for (unsigned i = 0; i < UI_BUTTON_TOKEN_LAST; i++) {
        attr = A_NORMAL;
        if (i == this->focusButtonInd)
            attr |= A_STANDOUT;
        len = strlen(buttons[i].text);
        if (len == 0)
            len = 1;
        mvwchgat(this->win, buttons[i].row, buttons[i].col, len, attr, 0, NULL);
    }
    touchwin(this->win);
    move(CURSOR_ROW, CURSOR_COL);
    wrefresh(this->win);
}

// Update stderr
void SchedulerUI::updateStderr()
{
	//MSGLOG(4, "");
    struct pollfd fds[1];
    char data[1024];

    if (this->stderrFDS[0] == 0)
        return;

    fds[0].fd = this->stderrFDS[0];
    fds[0].events = POLLIN;
    fds[0].revents = 0;

    unsigned size = 0;
    while (poll(fds, 1, 0) > 0 && size < sizeof(data) - 1)
        read(fds[0].fd, data + size++, 1);

    data[size] = 0;

    // Write to the screen
    if (size > 0) {
        wattrset(this->win, COLOR_PAIR(UI_TEXT_COLOR_MAGENTA));
        mvwprintw(this->win, this->stderrRowBegin, 0, "%s", data);
        wattrset(this->win, A_NORMAL);
    }
}

void SchedulerUI::updateTaskList()
{
	//MSGLOG(4, "");
	unsigned i = 0;
	const vector<Vehicle> & vehicles = scheduler.getVehicles();
	for( Scheduler::CVIT vit=vehicles.begin(); vit!=vehicles.end(); ++vit )
	{
		const list<Task> & tasks = vit->getTasks();
		list<Task>::const_iterator it = tasks.begin();

		for ( ++it; it != tasks.end(); ++it )
		{
			if (i >= NUM_TASK_DISPLAY)
				break;
			//if (i%2 == 1) wattrset(this->win, COLOR_PAIR(UI_CYAN_HIGHLIGHT));
			mvwprintw(this->win, this->textFields[TASK_LIST_START + i].row,
						this->textFields[TASK_LIST_START + i].col,
						/*|  ID | Customer |  Pick-Up   |  Drop-Off  | tWait \n*/
						"%3d | %8s | %10s | %10s | %5d",
						it->taskID, it->customerID.substr(0,8).c_str(),
						it->pickup.str().substr(0,10).c_str(),
						it->dropoff.str().substr(0,10).c_str(), it->twait);
			wattrset(this->win, A_NORMAL);
			i++;
		}
	}

    // Clear remaining rows
    for (; i < NUM_TASK_DISPLAY; i++)
    {
        mvwprintw(this->win, this->textFields[TASK_LIST_START + i].row,
                  this->textFields[TASK_LIST_START + i].col,
               /*|  ID | Customer |  Pick-Up   |  Drop-Off  | tWait \n*/
                  "                                                 ");
    }
}

void SchedulerUI::updateVehicleStatus()
{
	//MSGLOG(4, "");
	std::string vehicleID = DEFAULT_VEHICLE_ID;
	Scheduler::CVIT vit = scheduler.checkVehicleExist(vehicleID);

    mvwprintw(this->win, this->textFields[VEHICLE_ID].row, this->textFields[VEHICLE_ID].col,
              "%s", vehicleID.c_str());

    char vehStatusStr[64];
	SchedulerTypes::VehicleStatus vehStatus = vit->getStatus();
	string vss = SchedulerTypes::vehicleStatusStr(vehStatus) + "                  ";
	strncpy(vehStatusStr, vss.c_str(), sizeof(vehStatusStr) - 1);
    mvwprintw(this->win, this->textFields[VEHICLE_STATUS].row, this->textFields[VEHICLE_STATUS].col,
              "%s", vehStatusStr);

    if( vit->getTasks().empty() )
    {
        mvwprintw(this->win, this->textFields[CURRENT_TASK_ID].row,
                  this->textFields[CURRENT_TASK_ID].col,
                  "                ");
        mvwprintw(this->win, this->textFields[CURRENT_TASK_PICKUP].row,
                  this->textFields[CURRENT_TASK_PICKUP].col,
                  "                ");
        mvwprintw(this->win, this->textFields[CURRENT_TASK_DROPOFF].row,
                  this->textFields[CURRENT_TASK_DROPOFF].col,
                  "                ");
        mvwprintw(this->win, this->textFields[CURRENT_TASK_TIME_REMAINING].row,
                  this->textFields[CURRENT_TASK_TIME_REMAINING].col,
                  "                ");
    }
    else
    {
        Task t = vit->getCurrentTask();
        mvwprintw(this->win, this->textFields[CURRENT_TASK_ID].row,
                  this->textFields[CURRENT_TASK_ID].col,
                  "%3d      ", t.taskID);
        mvwprintw(this->win, this->textFields[CURRENT_TASK_PICKUP].row,
                  this->textFields[CURRENT_TASK_PICKUP].col,
                  "%10s     ", t.pickup.c_str());
        mvwprintw(this->win, this->textFields[CURRENT_TASK_DROPOFF].row,
                  this->textFields[CURRENT_TASK_DROPOFF].col,
                  "%10s     ", t.dropoff.c_str());
        mvwprintw(this->win, this->textFields[CURRENT_TASK_TIME_REMAINING].row,
                  this->textFields[CURRENT_TASK_TIME_REMAINING].col,
                  "%3d      ", t.tpickup + t.ttask);
    }
}

void SchedulerUI::processKeyboard(int ch)
{
    if (ch == KEY_RIGHT)
    {
        if (this->focusButtonInd < UI_BUTTON_TOKEN_LAST - 1)
            this->focusButtonInd++;
    }
    else if (ch == KEY_LEFT)
    {
        if (this->focusButtonInd > 0)
            this->focusButtonInd--;
    }
    else if (ch == KEY_DOWN)
    {
        if (this->focusButtonInd == NEW_PICKUP
            || this->focusButtonInd == NEW_DROPOFF
            || this->focusButtonInd == ADD_TASK)
        {
            this->focusButtonInd += 3;
        }
        else if (this->focusButtonInd == VC_TASK_ID
                 || this->focusButtonInd == VIEW_TASK
                 || this->focusButtonInd == CANCEL_TASK)
        {
            this->focusButtonInd = QUIT_SCHEDULER;
        }
    }
    else if (ch == KEY_UP)
    {
        if (this->focusButtonInd == QUIT_SCHEDULER)
        {
            this->focusButtonInd = CANCEL_TASK;
        }
        else if (this->focusButtonInd == VC_TASK_ID
                 || this->focusButtonInd == VIEW_TASK
                 || this->focusButtonInd == CANCEL_TASK)
        {
            this->focusButtonInd -= 3;
        }
    }

    if (this->focusButtonInd != NEW_PICKUP)
        this->focusNewPickup = false;
    if (this->focusButtonInd != NEW_DROPOFF)
        this->focusNewDropoff = false;
    if (this->focusButtonInd != VC_TASK_ID)
        this->focusTaskID = false;

    if (ch == '\n')
    {
        if (this->focusButtonInd == QUIT_SCHEDULER)
            this->schedulerStatus = SCHEDULER_QUIT;
        else if (this->focusButtonInd == VIEW_TASK)
            this->onUserViewTask();
        else if (this->focusButtonInd == CANCEL_TASK)
            this->onUserCancelTask();
        else if (this->focusButtonInd == ADD_TASK)
            this->onUserAddTask();
    }

    // Handle delete key
    if ((ch == 'd' || ch == KEY_BACKSPACE || ch == KEY_DC) &&
        ( this->focusButtonInd == NEW_PICKUP ||
          this->focusButtonInd == NEW_DROPOFF ||
          this->focusButtonInd == VC_TASK_ID )
        && strlen(this->buttons[this->focusButtonInd].text) > 0)
    {
        this->buttons[this->focusButtonInd].text[strlen(this->buttons[this->focusButtonInd].text) - 1] = 0;
        mvwprintw(this->win, this->buttons[this->focusButtonInd].row,
                  this->buttons[this->focusButtonInd].col, "%s ",
                  this->buttons[this->focusButtonInd].text);
    }

    // Handle button keys
    if (ch == this->buttons[QUIT_SCHEDULER].key)
        this->schedulerStatus = SCHEDULER_QUIT;
    else if (ch == this->buttons[VIEW_TASK].key)
        this->onUserViewTask();
    else if (ch == this->buttons[CANCEL_TASK].key)
        this->onUserCancelTask();
    else if (ch == this->buttons[ADD_TASK].key)
        this->onUserAddTask();

    // Get new pick up location
    if (this->focusButtonInd == NEW_PICKUP && ch >= '0' && ch <= '9')
    {
        if (!this->focusNewPickup ||
            ( strlen(this->buttons[NEW_PICKUP].text) == 1
              && this->buttons[NEW_PICKUP].text[0] == ' ') )
        {
            this->buttons[NEW_PICKUP].text[0] = ch;
            for (unsigned i = 1; i < sizeof(this->buttons[NEW_PICKUP].text); i++)
                this->buttons[NEW_PICKUP].text[i] = 0;
            mvwprintw(this->win, this->textFields[NEW_TASK_ID].row,
                      this->textFields[NEW_TASK_ID].col,
                      "                                            ");
        }
        else if (strlen(this->buttons[NEW_PICKUP].text) < sizeof(this->buttons[NEW_PICKUP].text))
        {
            this->buttons[NEW_PICKUP].text[strlen(this->buttons[NEW_PICKUP].text)] = ch;
        }

        this->focusNewPickup = true;
        mvwprintw(this->win, this->buttons[NEW_PICKUP].row, this->buttons[NEW_PICKUP].col,
                  this->buttons[NEW_PICKUP].text);
        if (strlen(this->buttons[NEW_PICKUP].text) < this->newPickupTextSize)
            mvwprintw(this->win, this->buttons[NEW_PICKUP].row,
                      this->buttons[NEW_PICKUP].col+strlen(this->buttons[NEW_PICKUP].text),
                      "%*s", this->newPickupTextSize - strlen(this->buttons[NEW_PICKUP].text) , "");
        this->newPickupTextSize = strlen(this->buttons[NEW_PICKUP].text);
    }

    // Get new dropoff location
    if (this->focusButtonInd == NEW_DROPOFF && ch >= '0' && ch <= '9')
    {
        if (!this->focusNewDropoff ||
            ( strlen(this->buttons[NEW_DROPOFF].text) == 1
            && this->buttons[NEW_DROPOFF].text[0] == ' ') )
        {
            this->buttons[NEW_DROPOFF].text[0] = ch;
            for (unsigned i = 1; i < sizeof(this->buttons[NEW_DROPOFF].text); i++)
                this->buttons[NEW_DROPOFF].text[i] = 0;
            mvwprintw(this->win, this->textFields[NEW_TASK_ID].row, this->textFields[NEW_TASK_ID].col,
                      "                                            ");
        }
        else if (strlen(this->buttons[NEW_DROPOFF].text) < sizeof(this->buttons[NEW_DROPOFF].text))
        {
            this->buttons[NEW_DROPOFF].text[strlen(this->buttons[NEW_DROPOFF].text)] = ch;
        }

        this->focusNewDropoff = true;
        mvwprintw(this->win, this->buttons[NEW_DROPOFF].row, this->buttons[NEW_DROPOFF].col,
                  this->buttons[NEW_DROPOFF].text);
        if (strlen(this->buttons[NEW_DROPOFF].text) < this->newDropoffTextSize)
            mvwprintw(this->win, this->buttons[NEW_DROPOFF].row,
                      this->buttons[NEW_DROPOFF].col+strlen(this->buttons[NEW_DROPOFF].text),
                      "%*s", this->newDropoffTextSize - strlen(this->buttons[NEW_DROPOFF].text) , "");
        this->newDropoffTextSize = strlen(this->buttons[NEW_DROPOFF].text);
    }

    // Get task ID to be viewed or cancelled
    if (this->focusButtonInd == VC_TASK_ID && ch >= '0' && ch <= '9')
    {
        if (!this->focusTaskID ||
            ( strlen(this->buttons[VC_TASK_ID].text) == 1
              && this->buttons[VC_TASK_ID].text[0] == ' ' ) )
        {
            this->buttons[VC_TASK_ID].text[0] = ch;
            for (unsigned i = 1; i < sizeof(this->buttons[VC_TASK_ID].text); i++)
                this->buttons[VC_TASK_ID].text[i] = 0;
            mvwprintw(this->win, this->textFields[VC_TASK_VEHICLE_ID].row,
                      this->textFields[VC_TASK_VEHICLE_ID].col,
                      "                                                          ");
            mvwprintw(this->win, this->textFields[VC_TASK_TIME_WAIT].row,
                      this->textFields[VC_TASK_TIME_WAIT].col,
                      "                                                          ");
        }
        else if (strlen(this->buttons[VC_TASK_ID].text) < sizeof(this->buttons[VC_TASK_ID].text))
        {
            this->buttons[VC_TASK_ID].text[strlen(this->buttons[VC_TASK_ID].text)] = ch;
        }

        this->focusTaskID = true;
        mvwprintw(this->win, this->buttons[VC_TASK_ID].row, this->buttons[VC_TASK_ID].col,
                  this->buttons[VC_TASK_ID].text);
        if (strlen(this->buttons[VC_TASK_ID].text) < this->taskIDTextSize)
            mvwprintw(this->win, this->buttons[VC_TASK_ID].row,
                      this->buttons[VC_TASK_ID].col+strlen(this->buttons[VC_TASK_ID].text),
                      "%*s", this->taskIDTextSize - strlen(this->buttons[VC_TASK_ID].text) , "");
        this->taskIDTextSize = strlen(this->buttons[VC_TASK_ID].text);
    }
}

void SchedulerUI::printText(UITextToken field, UITextColor color, const char *fmt, ...)
{
    va_list ap;
    va_start(ap,fmt);

    unsigned size = strlen(fmt);
    char *s = (char *) malloc(size+1);
    assert( s!=NULL );
    vsnprintf(s, size, fmt, ap);
    va_end(ap);

    for( unsigned i=strlen(s); i<size; i++ )
        s[i] = ' ';
    s[size] = '\0';

    wattrset(this->win, COLOR_PAIR(color) | A_BOLD);
    mvwprintw(this->win, this->textFields[field].row,
              this->textFields[field].col, s);
    wattrset(this->win, A_NORMAL);

    free(s);
}

void SchedulerUI::printButton(UIButtonToken field, UITextColor color, const char *fmt, ...)
{
    va_list ap;
    va_start(ap,fmt);

    unsigned size = strlen(fmt);
    char *s = (char *) malloc(size+1);
    assert( s!=NULL );
    vsnprintf(s, size, fmt, ap);
    va_end(ap);

    for( unsigned i=strlen(s); i<size; i++ )
        s[i] = ' ';
    s[size] = '\0';

    wattrset(this->win, COLOR_PAIR(color) | A_BOLD);
    mvwprintw(this->win, this->buttons[field].row,
              this->buttons[field].col, s);
    wattrset(this->win, A_NORMAL);

    free(s);
}

void SchedulerUI::onUserViewTask()
{
    // Check that all the characters are between 0 and 9
    if (strlen(this->buttons[VC_TASK_ID].text) == 0) {
        printText(VC_TASK_VEHICLE_ID, UI_TEXT_COLOR_RED, "Invalid Task ID!                                           ");
        return;
    }
    for (unsigned i = 0; i < strlen(this->buttons[VC_TASK_ID].text); i++)
    {
        if (this->buttons[VC_TASK_ID].text[i] < '0' || this->buttons[VC_TASK_ID].text[i] > '9')
        {
            printText(VC_TASK_VEHICLE_ID, UI_TEXT_COLOR_RED, "Invalid Task ID!                                           ");
            return;
        }
    }

    int taskID = atoi(this->buttons[VC_TASK_ID].text);
    SchedulerTypes::Duration waitTime = scheduler.getWaitTime(taskID);
	Task task = scheduler.getTask(taskID);
	mvwprintw(this->win, this->textFields[VC_TASK_VEHICLE_ID].row, this->textFields[VC_TASK_VEHICLE_ID].col,
			  "Vehicle ID:   %2d\t\tPickup station:  %10s       ", DEFAULT_VEHICLE_ID, task.pickup.c_str());
	mvwprintw(this->win, this->textFields[VC_TASK_TIME_WAIT].row, this->textFields[VC_TASK_TIME_WAIT].col,
			  "Waiting time: %2u\t\tDropoff station: %10s       ", waitTime, task.dropoff.c_str());

}

void SchedulerUI::onUserCancelTask()
{
    // Check that all the characters are between 0 and 9
    if (strlen(this->buttons[VC_TASK_ID].text) == 0) {
        printText(VC_TASK_VEHICLE_ID, UI_TEXT_COLOR_RED, "Invalid Task ID!                                           ");
        return;
    }
    for (unsigned i = 0; i < strlen(this->buttons[VC_TASK_ID].text); i++)
    {
        if (this->buttons[VC_TASK_ID].text[i] < '0' || this->buttons[VC_TASK_ID].text[i] > '9')
        {
            printText(VC_TASK_VEHICLE_ID, UI_TEXT_COLOR_RED, "Invalid Task ID!                                           ");
            return;
        }
    }

    int taskID = atoi(this->buttons[VC_TASK_ID].text);
    try {
        dbTalker.custCancel(taskID);
        printText(VC_TASK_VEHICLE_ID, UI_TEXT_COLOR_GREEN, "Task %d removed!                                           ", taskID);
    }
    catch( SchedulerException & e ) {
        printText(VC_TASK_VEHICLE_ID, UI_TEXT_COLOR_RED, "Invalid Task ID!                                           ");
        return;
    }
    mvwprintw(this->win, this->textFields[VC_TASK_TIME_WAIT].row,
              this->textFields[VC_TASK_TIME_WAIT].col,
              "                                                           ");
}

bool SchedulerUI::getStation(UIButtonToken button, Station *station)
{
	assert( button==NEW_PICKUP || button==NEW_DROPOFF );

	const char * buttonstr = button==NEW_PICKUP ? "pickup" : "dropoff";

	// Check that all the characters are between 0 and 9
	if (strlen(this->buttons[button].text) == 0) {
		printButton(button, UI_TEXT_COLOR_RED, "Invalid %s station!             ", buttonstr);
		return false;
	}
	for (unsigned i = 0; i < strlen(this->buttons[button].text); i++) {
		if (this->buttons[button].text[i] < '0' || this->buttons[button].text[i] > '9') {
			printText(NEW_TASK_ID, UI_TEXT_COLOR_RED, "Invalid %s station!           ", buttonstr);
			return false;
		}
	}

	const StationList & sl = scheduler.stationPaths.knownStations();
	try
	{
		*station = sl(atoi(this->buttons[button].text));
	}
	catch( StationDoesNotExistException & e )
	{
		printText(NEW_TASK_ID, UI_TEXT_COLOR_RED, "Invalid %s station!             ", buttonstr);
		return false;
	}
	return true;
}

void SchedulerUI::onUserAddTask()
{
	Station pickup, dropoff;
	if( !getStation(NEW_PICKUP,&pickup) || !getStation(NEW_DROPOFF,&dropoff) )
		return;

    try
    {
        unsigned tid = dbTalker.makeBooking("cust1", pickup, dropoff);
        printText(NEW_TASK_ID, UI_TEXT_COLOR_GREEN,
                  "Added task ID %u: <%s, %s>          ",
                  tid, pickup.c_str(), dropoff.c_str());
    }
    catch( SchedulerException & e )
    {
        if( e.type() == SchedulerException::NO_AVAILABLE_VEHICLE )
            printText(NEW_TASK_ID, UI_TEXT_COLOR_RED, "Vehicles not available!             ");
        return;
    }
}

SchedulerUI::SchedulerStatus SchedulerUI::getSchedulerStatus()
{
    return this->schedulerStatus;
}
