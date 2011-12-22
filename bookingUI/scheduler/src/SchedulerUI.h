#ifndef __SCHEDULER_UI__H__
#define __SCHEDULER_UI__H__

#include <ncurses.h>

#include "SchedulerTypes.h"
#include "Scheduler.h"
#include "DBTalker.h"

#define NUM_TASK_DISPLAY 12


class SchedulerUI
{
public:
    enum SchedulerStatus
    {
        SCHEDULER_RUNNING,
        SCHEDULER_QUIT
    };

private:
    enum UITextToken
    {
        VEHICLE_ID = 0,
        VEHICLE_STATUS,
        CURRENT_TASK_ID,
        CURRENT_TASK_PICKUP,
        CURRENT_TASK_DROPOFF,
        CURRENT_TASK_TIME_REMAINING,
        NEW_TASK_ID,
        VC_TASK_VEHICLE_ID,
        VC_TASK_TIME_WAIT,
        UI_USER_FIELD_LAST,
        TASK_LIST_START,
        UI_TEXT_TOKEN_LAST
    };

    enum UIButtonToken
    {
        NEW_PICKUP = 0,
        NEW_DROPOFF,
        ADD_TASK,
        VC_TASK_ID,
        VIEW_TASK,
        CANCEL_TASK,
        QUIT_SCHEDULER,
        UI_BUTTON_TOKEN_LAST
    };

    enum UITextColor {
        UI_TEXT_COLOR_RED=1,
        UI_TEXT_COLOR_GREEN,
        UI_TEXT_COLOR_MAGENTA,
        UI_CYAN_HIGHLIGHT
    };

    struct UIText {
        int row, col;    // Location
    };

    struct UIButton {
        int row, col;    // Location
        char text[64];   // Text for buttons
        char key;        // Bound key
    };


    // Window handle
    WINDOW *win;

    // Scheduler
    const Scheduler & scheduler;

    DBTalker & dbTalker;

    // Fields in the UI
    UIText textFields[UI_TEXT_TOKEN_LAST + NUM_TASK_DISPLAY - 1];
    UIButton buttons[UI_BUTTON_TOKEN_LAST];
    unsigned focusButtonInd;
    bool focusNewPickup, focusNewDropoff, focusTaskID;
    unsigned newPickupTextSize, newDropoffTextSize, taskIDTextSize;

    // Region for for stderr (scrolling)
    int stderrRowBegin;
    int stderrRowEnd;

    // Re-directed file descriptors for stderr
    int stderrFDS[2];

    // Status of the scheduler
    SchedulerStatus schedulerStatus;

private:
    /// Level of verbosity
    unsigned verbosity_level;

    FILE * logFile;

public:
    // Default constructor
    SchedulerUI(const Scheduler &, DBTalker &);
    ~SchedulerUI();

    void setLogFile(FILE *);
    void setVerbosityLevel(unsigned);

    // Initialize console display
    void initConsole();

    // Finalize console display
    void finishConsole();

    // Update console
    void updateConsole();

    // Get status of scheduler
    SchedulerStatus getSchedulerStatus();

private:
    // Draw button
    void drawButtons();

    // Update stderr
    void updateStderr();

    // Update task list
    void updateTaskList();

    // Update vehicle status, including current task
    void updateVehicleStatus();

    // Update keyboard
    void processKeyboard(int);

    void onUserViewTask();
    void onUserCancelTask();
    void onUserAddTask();

    bool getStation(UIButtonToken button, Station *station);

    void printText(UITextToken field, UITextColor c, const char *fmt, ...);
    void printButton(UIButtonToken field, UITextColor c, const char *fmt, ...);
};

#endif /*__SCHEDULER_UI__H__*/
