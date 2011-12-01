#ifndef __STATION_NETWORK_H__
#define __STATION_NETWORK_H__

class StationNetwork
{
public:
    enum StationID
    {
        WORKSHOP,
        MCDONALD,
        EA,
        E3A
    };

    static const int STATION_NETWORK[][4];

private:
    unsigned numPairs;

public:
    /// Default Constructor
    StationNetwork();

    /// Return the travel time between 2 given stations.
    /// Return -1 if either station doesn't exist or there is no path between them.
    int travelTime(StationID stationID1, StationID stationID2);
};

#endif //__STATION_NETWORK_H__
