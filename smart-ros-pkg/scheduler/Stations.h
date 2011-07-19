#ifndef STATIONS_H_
#define STATIONS_H_

namespace stations 
{
  const int STATION_LIST[] = {1, 2, 3};

  enum {
    WORKSHOP = 1,
    MCDONALD = 2,
    EA = 3
  };
  
  // {origin, destimation, distance, estimated travel time}
  int STATION_NETWORK[][4] = {
    {WORKSHOP, MCDONALD, 300, 5},
    {WORKSHOP, EA, 600, 10},
    {MCDONALD, WORKSHOP, 300, 5},
    {MCDONALD, EA, 900, 15},
    {EA, WORKSHOP, 600, 10},
    {EA, MCDONALD, 900, 15}
  };
}


#endif /*STATIONS_H_*/
