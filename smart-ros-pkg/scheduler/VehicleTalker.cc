#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <sstream>
#include <time.h>
#include "VehicleTalker.hh"
#include "SocketException.hh"

using namespace std;

// Error handling
#define MSG(fmt, ...) \
  (fprintf(stderr, fmt "\n", ##__VA_ARGS__) ? 0 : 0)
#define ERROR(fmt, ...) \
  (fprintf(stderr, "ERROR %s:%d: " fmt "\n", __FILE__, __LINE__, ##__VA_ARGS__) ? -1 : 0)

const int UNINIT_INT = -2011;

VehicleTalker::VehicleTalker(int port, int verbosityLevel)
{
  m_quit = false;
  m_newStatusRecv = false;
  m_vehInfo.status = VEHICLE_NOT_AVAILABLE;
  m_vehInfo.tremain = 1000000;
  m_verbosity = verbosityLevel;
  try {
    m_server.init(port);
    m_server.setSocketBlocking(true);
    m_server.accept ( m_socket );
    m_isconnected = true;
  }
  catch (SocketException e) {
    ERROR("%s", e.getErrMsg().c_str());
    m_isconnected = false;
  }
}

bool VehicleTalker::sendNewTask(int customerID, int pickup, int dropoff)
{
  if (!m_isconnected)
    return false;

  bool msgSent = false;
  while (!msgSent) {
    try {
      std::ostringstream ss;
      ss << ";" << "0:" << customerID << ":" << pickup << ":" << dropoff << "\n";
      m_socket << ss.str();
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

bool VehicleTalker::checkNewInfo()
{
  return m_newStatusRecv;
}

VehicleInfo VehicleTalker::getVehicleInfo()
{
  VehicleInfo ret = m_vehInfo;
  m_newStatusRecv = false;
  return ret;
}

void VehicleTalker::runVehicleReceiver()
{
  std::string msg;
  bool msgIncomplete = false;
  while (!m_quit) {
    if (!m_newStatusRecv) {
      if (msg.length() > 0 && !msgIncomplete) {
	// Msg format: customerID:picup:dropoff
	string statusStr;
	int beginStatus = msg.find(";", 0);
	if (beginStatus >= 0)
	  msg = msg.substr(beginStatus, msg.length()-beginStatus);
	else {
	  msg.clear();
	  msgIncomplete = false;
	  continue;
	}
	int endStatus = msg.find(";", beginStatus+1);
	if (endStatus >= 0) {
	  statusStr = msg.substr(1, endStatus-1);
	  msg = msg.substr(endStatus, msg.length());
	}
	else {
	  statusStr = msg.substr(1, msg.length());
	  msg.clear();
	}
	
	int spacePos = statusStr.find ("\n", 0);
	while (spacePos >= 0) {
	  if (spacePos > 0 && spacePos < statusStr.length()-1) {
	    string str1 = statusStr.substr(0, spacePos);
	    string str2 = statusStr.substr(spacePos+1, statusStr.length());
	    statusStr = str1.append(str2);
	  }
	  else if (spacePos > 0)
	    statusStr = statusStr.substr(0, spacePos);
	  else
	    statusStr = statusStr.substr(spacePos+1, statusStr.length());
	  spacePos = statusStr.find ("\n", 0);
	}
	
	if (m_verbosity > 0) {
	  MSG("Received status %s", statusStr.c_str());
	  MSG("msg: %s", msg.c_str());
	}
	
	int firstColon = statusStr.find(":", 0);
	int secondColon = statusStr.find(":", firstColon+1);
	if (firstColon < 0 || secondColon <= firstColon) {
	  msgIncomplete = true;
	  msg = statusStr;
	}
	else {
	  bool statusValid = true;
	  string vehIDStr = statusStr.substr(0, firstColon);
	  string vehStatusStr = statusStr.substr(firstColon+1, secondColon-firstColon-1);
	  string tremainStr = statusStr.substr(secondColon+1, statusStr.length());
	  for (int i = 0; i < vehIDStr.length(); i++) {
	    if (vehIDStr.at(i) != '+' && vehIDStr.at(i) != '-' && vehIDStr.at(i) != ' ' &&
		(vehIDStr.at(i) < '0' || vehIDStr.at(i) > '9')) {
	      statusValid = false;
	      ERROR("Invalid vehicle ID: %s", vehIDStr.c_str());
	      break;
	    }
	  }
	  if (statusValid) {
	    for (int i = 0; i < vehStatusStr.length(); i++) {
	      if (vehStatusStr.at(i) != ' ' && 
		  (vehStatusStr.at(i) < '0' || vehStatusStr.at(i) > '9')) {
		statusValid = false;
		ERROR("Invalid status: %s", vehStatusStr.c_str());
		break;
	      }
	    }
	  }
	  if (statusValid) {
	    for (int i = 0; i < tremainStr.length(); i++) {
	      if (tremainStr.at(i) != ' ' &&
		  (tremainStr.at(i) < '0' || tremainStr.at(i) > '9')) {
		statusValid = false;
		ERROR("Invalid tremain: %s (%c)", tremainStr.c_str(), tremainStr.at(i));
		break;
	      }
	    }
	  }
	  if (statusValid) {
	    m_vehInfo.vehicleID = atoi(vehIDStr.c_str());
	    int vehStatus = atoi(vehStatusStr.c_str());
	    if (vehStatus == VEHICLE_NOT_AVAILABLE)
	      m_vehInfo.status = VEHICLE_NOT_AVAILABLE;
	    else if (vehStatus == VEHICLE_ON_CALL)
	      m_vehInfo.status = VEHICLE_ON_CALL;
	    else if (vehStatus == VEHICLE_POB)
	      m_vehInfo.status = VEHICLE_POB;
	    else if (vehStatus == VEHICLE_BUSY)
	      m_vehInfo.status = VEHICLE_BUSY;
	    else if (vehStatus == VEHICLE_AVAILABLE)
	      m_vehInfo.status = VEHICLE_AVAILABLE;
	    else {
	      ERROR("Invalid vehicle status: %s", vehStatusStr.c_str());
	      m_vehInfo.status = VEHICLE_NOT_AVAILABLE;
	    }
	    m_vehInfo.tremain = atoi(tremainStr.c_str());
	    m_newStatusRecv = true;
	  }
	  else
	    msgIncomplete = false;
	}
      }
      else {
	try {
	  std::string newmsg;
	  m_socket >> newmsg;
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

bool VehicleTalker::quit()
{
  m_quit = true;
}
