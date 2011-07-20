#ifndef STATIONS_H_
#define STATIONS_H_

namespace stations 
{
  const int STATION_LIST[] = {1, 2, 3};

  enum {
    WORKSHOP = 1,
    MCDONALD = 2,
    EA = 3,
    E3A = 4
  };
  
  // {origin, destimation, distance, estimated travel time}
  int STATION_NETWORK[][4] = {
    {WORKSHOP, MCDONALD, 300, 5},
    {WORKSHOP, EA, 600, 10},
    {WORKSHOP, E3A, 600, 10},
    {MCDONALD, WORKSHOP, 300, 5},
    {MCDONALD, EA, 900, 15},
    {MCDONALD, E3A, 900, 15},
    {EA, WORKSHOP, 600, 10},
    {EA, MCDONALD, 900, 15},
    {EA, E3A, 100, 2},
    {E3A, WORKSHOP, 600, 10},
    {E3A, MCDONALD, 900, 15},
    {E3A, EA, 100, 2}
  };
}


#endif /*STATIONS_H_*/
