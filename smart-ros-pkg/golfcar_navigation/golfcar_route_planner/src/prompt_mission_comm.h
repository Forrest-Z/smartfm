#ifndef __PROMPT_MISSION_COMM__H__
#define __PROMPT_MISSION_COMM__H__

#include "db_mission_comm.h"

class PromptMissionComm : public MissionComm
{
public:
    Mission waitForNewMission()
    {
        Mission m;

    }

};


    printStationList();
    currentStationID_ = promptForStation("Current station? ");
    ROS_INFO("Current station set to %s. READY!", currentStationID_.c_str());


void printStationList() const
{
    cout << "Station list:" <<endl;
    StationList::StationIterator s;
    for( s=sp_.knownStations().begin(); s<sp_.knownStations().end(); ++s )
        cout <<"    " <<s->number() <<") - " <<s->str() <<endl;
}


Station promptForStation(const string & prompt) const
{
    while( ros::ok() )
    {
        cout <<prompt;
        string temp = "";
        getline(cin, temp);
        int n = atoi(temp.c_str());
        if( sp_.knownStations().exists(n) )
            return sp_.knownStations()(n);
        else
            cout <<"You have entered an invalid station." <<endl;
    }
    return Station();
}


    printStationList();
    pickup_ = promptForStation("Pickup station? ");
    dropoff_ = promptForStation("Drop off station? ");


#endif
