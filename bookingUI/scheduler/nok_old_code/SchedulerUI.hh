#ifndef SCHEDULERUI_HH_
#define SCHEDULERUI_HH_

#include <pthread.h>
#include <ncurses.h>
#include "Scheduler.hh"

namespace std
{

const int NUM_TASK_DISPLAY = 12;
const int CURSOR_ROW = 21;
const int CURSOR_COL = 0;

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

enum SchedulerStatus
  {
    SCHEDULER_RUNNING,
    SCHEDULER_QUIT
  };

struct UIText
{
  int row, col;    // Location
};

struct UIButton
{
  int row, col;    // Location
  char text[64];   // Text for buttons
  char key;        // Bound key
};

class SchedulerUI
{
  // Window handle
  WINDOW *win;

  // Scheduler
  Scheduler* scheduler;

  // Fields in the UI
  UIText textFields[UI_TEXT_TOKEN_LAST + NUM_TASK_DISPLAY - 1];
  UIButton buttons[UI_BUTTON_TOKEN_LAST];
  int focusButtonInd;
  bool focusNewPickup, focusNewDropoff, focusTaskID;
  int newPickupTextSize, newDropoffTextSize, taskIDTextSize;

  // Region for for stderr (scrolling)
  int stderrRowBegin;
  int stderrRowEnd;

  // Re-directed file descriptors for stderr
  int stderrFDS[2];

  // Status of the scheduler
  SchedulerStatus schedulerStatus;

public:
  // Default constructor
  SchedulerUI(Scheduler* scheduler);

  // Default destructor
  ~SchedulerUI();

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
};

} // namespace std

#endif /*SCHEDULERUI_HH_*/
