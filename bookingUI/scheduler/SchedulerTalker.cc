#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <sstream>
#include <time.h>
#include <sys/stat.h>
#include "SchedulerTalker.hh"
#include "SocketException.hh"

using namespace std;

// Error handling
#define MSG(fmt, ...) \
  (fprintf(stderr, fmt "\n", ##__VA_ARGS__) ? 0 : 0)
#define ERROR(fmt, ...) \
  (fprintf(stderr, "ERROR %s:%d: " fmt "\n", __FILE__, __LINE__, ##__VA_ARGS__) ? -1 : 0)

const int UNINIT_INT = -2011;

SchedulerTalker::SchedulerTalker(string host, int port, int verbosityLevel)
{
  m_quit = false;
  m_newTaskRecv = false;
  m_usrID = UNINIT_INT;
  m_pickup = UNINIT_INT;
  m_dropoff = UNINIT_INT;
  m_verbosity = verbosityLevel;

  // log file
  ostringstream oss;
  char timestr[64];
  struct stat st;
  time_t t = time(NULL);
  strftime(timestr, sizeof(timestr), "%F-%a-%H-%M", localtime(&t));
  oss  << "log_stalker" << "." << timestr << ".log";
  string logFileName = oss.str();
  string suffix = "";
  
  // if it exists already, append .1, .2, .3 ... 
  for (int i = 1; stat((logFileName + suffix).c_str(), &st) == 0; i++) {
    ostringstream tmp;
    tmp << '.' << i;
    suffix = tmp.str();
  }
  logFileName += suffix;
  logFile = fopen(logFileName.c_str(), "w");

  //logFile = fopen ("log_stalker.txt","w");
  try {
    m_socket.init(host, port);
    m_socket.setSocketBlocking(true);
    m_isconnected = true;
    m_socket << ";0:0:0:0:0\n";
    //m_socket << ";server:0:0:0:0\n";
  }
  catch (SocketException e) {
    ERROR("%s", e.getErrMsg().c_str());
    fprintf (logFile, "\n%d: ERROR: %s", (int) time(NULL), e.getErrMsg().c_str());
    m_isconnected = false;
  }
}

SchedulerTalker::~SchedulerTalker() 
{
  fclose (logFile);
}

bool SchedulerTalker::sendTaskStatus(int usrID, int taskID, int twait, int vehicleID)
{
  if (!m_isconnected)
    return false;

  bool msgSent = false;
  while (!msgSent) {
    try {
      std::ostringstream ss;
      ss << ";" << "0:" << usrID << ":" << taskID << ":" << twait << ":" << vehicleID << "\n";
      //ss << ";" << "server:" << usrID << ":" << taskID << ":" << twait << ":" << vehicleID << "\n";
      m_socket << ss.str();
      fprintf (logFile, "\n%d: sending status: %s", (int) time(NULL), ss.str().c_str());
      msgSent = true;
    }
    catch (SocketException e) {
      ERROR("%s", e.getErrMsg().c_str());
      fprintf (logFile, "\n%d: ERROR: %s", (int) time(NULL), e.getErrMsg().c_str());
      msgSent = false;
      sleep(1);
    }
  }
  return msgSent;
}

bool SchedulerTalker::checkTask()
{
  return m_newTaskRecv;
}

bool SchedulerTalker::recvTask(int &usrID, int &taskID, int &pickup, int &dropoff)
{
  bool ret = m_newTaskRecv;
  usrID = m_usrID;
  taskID = m_taskID;
  pickup = m_pickup;
  dropoff = m_dropoff;
  fprintf (logFile, "\n%d: returning task: %d:%d:%d:%d:%d", (int) time(NULL), usrID, taskID, pickup, dropoff, ret);
  m_newTaskRecv = false;
  return ret;
}

void SchedulerTalker::runMobileReceiver()
{
  std::string msg;
  bool msgIncomplete = false;
  while (!m_quit) {
    fprintf(logFile, "\n%d: waiting for the vehicle to pick up new msg", (int) time(NULL));
    if (!m_newTaskRecv) {
      fprintf(logFile, "\n%d: processing msg: %s", (int) time(NULL), msg.c_str());
      if (msg.length() > 0 && !msgIncomplete) {
	// Msg format: customerID:picup:dropoff
	string taskStr;
	int beginTask = msg.find(";", 0);
	if (beginTask >= 0)
	  msg = msg.substr(beginTask, msg.length()-beginTask);
	else {
	  msg.clear();
	  msgIncomplete = false;
	  continue;
	}
	int endTask = msg.find(";", 1);
	if (endTask >= 0) {
	  taskStr = msg.substr(0, endTask);
	  msg = msg.substr(endTask, msg.length());
	}
	else {
	  taskStr = msg.substr(0, msg.length());
	  msg.clear();
	}

	int spacePos = taskStr.find ("\n", 0);

	while (spacePos >= 0 && spacePos < taskStr.length()-1) {
	  if (spacePos > 0) {
	    string str1 = taskStr.substr(0, spacePos);
	    string str2 = taskStr.substr(spacePos+1, taskStr.length());
	    taskStr = str1.append(str2);
	  }
	  else if (spacePos == 0)
	    taskStr = taskStr.substr(spacePos+1, taskStr.length());
	  spacePos = taskStr.find ("\n", 0);
	}
	
	if (spacePos > 0 && spacePos == taskStr.length()-1) {
	  taskStr = taskStr.substr(0, spacePos);
	}
	else {
	  msgIncomplete = true;
	  msg = taskStr;
	  fprintf(logFile, "\n%d: incomplete msg: %s", (int) time(NULL), msg.c_str());
	  continue;
	}

	/*
	while (spacePos >= 0) {
	  if (spacePos > 0 && spacePos < taskStr.length()-1) {
	    string str1 = taskStr.substr(0, spacePos);
	    string str2 = taskStr.substr(spacePos+1, taskStr.length());
	    taskStr = str1.append(str2);
	  }
	  else if (spacePos > 0)
	    taskStr = taskStr.substr(0, spacePos);
	  else
	    taskStr = taskStr.substr(spacePos+1, taskStr.length());
	  spacePos = taskStr.find ("\n", 0);
	}
	*/

	if (m_verbosity > 0) {
	  MSG("Received task %s", taskStr.c_str());
	  MSG("msg: %s", msg.c_str());
	}

	int firstColon = taskStr.find(":", 0);
	int secondColon = taskStr.find(":", firstColon+1);
	int thirdColon = taskStr.find(":", secondColon+1);
	if (firstColon < 0 || secondColon <= firstColon || thirdColon <= secondColon || thirdColon == msg.length()-1) {
	  msgIncomplete = true;
	  msg = taskStr;
	}
	else {
	  bool taskValid = true;
	  string usrIDStr = taskStr.substr(1, firstColon-1);
	  string taskIDStr = taskStr.substr(firstColon+1, secondColon-firstColon-1);
	  string pickupStr = taskStr.substr(secondColon+1, thirdColon-secondColon-1);
	  string dropoffStr = taskStr.substr(thirdColon+1, taskStr.length());
	  for (int i = 0; i < usrIDStr.length(); i++) {
	    if (usrIDStr.at(i) != '+' && usrIDStr.at(i) != '-' && usrIDStr.at(i) != ' ' &&
		(usrIDStr.at(i) < '0' || usrIDStr.at(i) > '9')) {
	      taskValid = false;
	      ERROR("Invalid customer ID: %s", usrIDStr.c_str());
	      fprintf (logFile, "\n%d: ERROR: Invalid customer ID: %s", (int) time(NULL), usrIDStr.c_str());
	      break;
	    }
	  }
	  if (taskValid) {
	    for (int i = 0; i < taskIDStr.length(); i++) {
	      if (taskIDStr.at(i) != ' ' && 
		  (taskIDStr.at(i) < '0' || taskIDStr.at(i) > '9')) {
		taskValid = false;
		ERROR("Invalid taskID: %s", taskIDStr.c_str());
		fprintf (logFile, "\n%d: ERROR: Invalid task ID: %s", (int) time(NULL), taskIDStr.c_str());
		break;
	      }
	    }
	  }
	  if (taskValid) {
	    for (int i = 0; i < pickupStr.length(); i++) {
	      if (pickupStr.at(i) != ' ' && pickupStr.at(i) != '-' &&
		  (pickupStr.at(i) < '0' || pickupStr.at(i) > '9')) {
		taskValid = false;
		ERROR("Invalid pickup: %s", pickupStr.c_str());
		fprintf (logFile, "\n%d: ERROR: Invalid pickup: %s", (int) time(NULL), pickupStr.c_str());
		break;
	      }
	    }
	  }
	  if (taskValid) {
	    for (int i = 0; i < dropoffStr.length(); i++) {
	      if (dropoffStr.at(i) != ' ' && dropoffStr.at(i) != '-' &&
		  (dropoffStr.at(i) < '0' || dropoffStr.at(i) > '9')) {
		taskValid = false;
		ERROR("Invalid dropoff: %s (%c)", dropoffStr.c_str(), dropoffStr.at(i));
		fprintf (logFile, "\n%d: ERROR: Invalid dropoff: %s", (int) time(NULL), dropoffStr.c_str());
		break;
	      }
	    }
	  }
	  if (taskValid) {
	    m_usrID = atoi(usrIDStr.c_str());
	    m_taskID = atoi(taskIDStr.c_str());
	    m_pickup = atoi(pickupStr.c_str());
	    m_dropoff = atoi(dropoffStr.c_str());
	    m_newTaskRecv = true;
	    fprintf (logFile, "\n%d: new task: %d:%d:%d:%d", (int) time(NULL), m_usrID, m_taskID, m_pickup, m_dropoff);
	  }
	  else
	    msgIncomplete = false;
	}
      }
      else {
	try {
	  std::string newmsg;
	  m_socket >> newmsg;
	  fprintf (logFile, "\n\n%d: new msg: %s", (int) time(NULL), newmsg.c_str());
	  msg.append(newmsg);
	  msgIncomplete = false;
	}
	catch (SocketException e) {
	  ERROR("%s", e.getErrMsg().c_str());
	}
      }
    }
    usleep(100000);
  }
}

bool SchedulerTalker::quit()
{
  m_quit = true;
}
