#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <sstream>
#include "RoutePlanner.hh"
#include "socket_handler/SocketException.hh"

using namespace std;

// Error handling
#define MSG(fmt, ...) \
  (fprintf(stderr, fmt "\n", ##__VA_ARGS__) ? 0 : 0)
#define ERROR(fmt, ...) \
  (fprintf(stderr, "ERROR %s:%d: " fmt "\n", __FILE__, __LINE__, ##__VA_ARGS__) ? -1 : 0)

RoutePlanner::RoutePlanner(string schHost, int schPort)
{
  m_schConnected = false;
  m_locConnected = false;
  this->initSchComm(schHost, schPort);
}

RoutePlanner::RoutePlanner(string host, int schPort, int locPort)
{
  m_schConnected = false;
  m_locConnected = false;
  this->initSchComm(host, schPort);
  this->initLocComm(host, locPort);
}

RoutePlanner::RoutePlanner(string schHost, int schPort, string locHost, int locPort)
{
  m_schConnected = false;
  m_locConnected = false;
  this->initSchComm(schHost, schPort);
  this->initLocComm(locHost, locPort);
}

void RoutePlanner::initSchComm(string host, int port)
{
  try {
    m_schSocket.init(host, port);
    m_schSocket.setSocketBlocking(true);
    m_schConnected = true;
  }
  catch (SocketException e) {
    ERROR("%s", e.getErrMsg().c_str());
    m_schConnected = false;
  }
}

void RoutePlanner::initLocComm(string host, int port)
{
  try {
    m_locSocket.init(host, port);
    m_locSocket.setSocketBlocking(true);
    m_locSocket << ";car:0:0:0\n";
    m_locConnected = true;
  }
  catch (SocketException e) {
    ERROR("%s", e.getErrMsg().c_str());
    m_locConnected = false;
  }
}

bool RoutePlanner::sendStatus(int vehicleID, VehicleStatus vehStatus, int tremain)
{
  if (!m_schConnected)
    return false;

  bool msgSent = false;
  while (!msgSent) {
    try {
      std::ostringstream ss;
      ss << ";" << vehicleID << ":" << vehStatus << ":" << tremain << "\n";
      m_schSocket << ss.str();
      msgSent = true;
    }
    catch (SocketException e) {
      ERROR("%s", e.getErrMsg().c_str());
      msgSent = false;
      sleep(1);
    }
  }
  return msgSent;
}

bool RoutePlanner::sendLocation(double lat, double lon)
{
  if (!m_locConnected)
    return false;

  bool msgSent = false;
  try {
    std::ostringstream ss;
    //ss << lat << " " << lon << "\n";
    ss << ";car:1:" << lat << ":" << lon << "\n";
    m_locSocket << ss.str();
    msgSent = true;
  }
  catch (SocketException e) {
    ERROR("%s", e.getErrMsg().c_str());
    msgSent = false;
  }
  return msgSent;
}

bool RoutePlanner::getNewTask(int &usrID, int &pickup, int &dropoff)
{
  bool msgIncomplete = false;
  if (msg.length() > 0) {
    string taskStr;
    int beginTask = msg.find(";", 0);
    if (beginTask >= 0)
      msg = msg.substr(beginTask, msg.length()-beginTask);
    else {
      msg.clear();
      msgIncomplete = true;
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
    }

    int firstColon = taskStr.find(":", 0);
    int secondColon = taskStr.find(":", firstColon+1);
    if (firstColon < 0 || secondColon <= firstColon) {
      msgIncomplete = true;
      msg = taskStr;
    }
    else {
      bool taskValid = true;
      string usrIDStr = taskStr.substr(1, firstColon-1);
      string pickupStr = taskStr.substr(firstColon+1, secondColon-firstColon-1);
      string dropoffStr = taskStr.substr(secondColon+1, taskStr.length());
      for (int i = 0; i < usrIDStr.length(); i++) {
	if (usrIDStr.at(i) != '+' && usrIDStr.at(i) != '-' && usrIDStr.at(i) != ' ' &&
	    (usrIDStr.at(i) < '0' || usrIDStr.at(i) > '9')) {
	  taskValid = false;
	  ERROR("Invalid customer ID: %s", usrIDStr.c_str());
	  break;
	}
      }
      if (taskValid) {
	for (int i = 0; i < pickupStr.length(); i++) {
	  if (pickupStr.at(i) != ' ' && 
	      (pickupStr.at(i) < '0' || pickupStr.at(i) > '9')) {
	    taskValid = false;
	    ERROR("Invalid pickup: %s", pickupStr.c_str());
	    break;
	  }
	}
      }
      if (taskValid) {
	for (int i = 0; i < dropoffStr.length(); i++) {
	  if (dropoffStr.at(i) != ' ' && 
	      (dropoffStr.at(i) < '0' || dropoffStr.at(i) > '9')) {
	    taskValid = false;
	    ERROR("Invalid dropoff: %s (%c)", dropoffStr.c_str(), dropoffStr.at(i));
	    break;
	  }
	}
      }
      if (taskValid) {
	usrID = atoi(usrIDStr.c_str());
	pickup = atoi(pickupStr.c_str());
	dropoff = atoi(dropoffStr.c_str());
	return true;
      }
      else
	msgIncomplete = true;
    }
  }

  if (msg.length() == 0 || msgIncomplete) {
    try {
      std::string newmsg;
      m_schSocket >> newmsg;
      msg.append(newmsg);
    }
    catch (SocketException e) {
      ERROR("%s", e.getErrMsg().c_str());
      return false;
    }
    int newUsrID, newPickup, newDropoff;
    bool ret = getNewTask(newUsrID, newPickup, newDropoff);
    usrID = newUsrID;
    pickup = newPickup;
    dropoff = newDropoff;
    return ret;
  }
}
