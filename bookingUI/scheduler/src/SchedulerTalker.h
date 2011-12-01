#ifndef SCHEDULERTALKER_HH_
#define SCHEDULERTALKER_HH_

#include <stdio.h>


class SchedulerTalker
{
public:
  // Default Constructor
  SchedulerTalker(std::string host, int port, int verbosity_level);

  // Default destructor
  virtual ~SchedulerTalker();

  // Send task status to the server
  bool sendTaskStatus(int usrID, int taskID, int twait, int vehicleID);

  // Check whether there is a new task
  bool checkTask();

  // Receive the latest task. Return true if this message is new. Otherwise, return false
  bool recvTask(int &usrID, int &taskID, int &pickup, int &dropoff);

  // An infinite loop that keep listening for new task
  void runMobileReceiver();

  // Stop running talker
  bool quit();

private:
  ClientSocket m_socket;
  bool m_quit, m_isconnected;
  bool m_newTaskRecv;
  int m_usrID, m_taskID, m_pickup, m_dropoff;
  int m_verbosity;
  FILE * logFile;
};

#endif // SCHEDULERTALKER_HH_
