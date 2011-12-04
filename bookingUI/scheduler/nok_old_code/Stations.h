#ifndef STATIONS_H_
#define STATIONS_H_

namespace stations 
{
  const int STATION_LIST[] = {1, 2, 3, 4};

  enum {
    WORKSHOP = 1,
    MCDONALD = 2,
    EA = 3,
    E3A = 4
  };
  
  // {origin, destimation, distance, estimated travel time}
  int STATION_NETWORK[][4] = {
    {WORKSHOP, MCDONALD, 126, 126},
    {WORKSHOP, EA, 385, 385},
    {WORKSHOP, E3A, 460, 460},
    {MCDONALD, WORKSHOP, 127, 127},
    {MCDONALD, EA, 497, 497},
    {MCDONALD, E3A, 572, 572},
    {EA, WORKSHOP, 345, 345},
    {EA, MCDONALD, 467, 467},
    {EA, E3A, 121, 121},
    {E3A, WORKSHOP, 424, 424},
    {E3A, MCDONALD, 546, 546},
    {E3A, EA, 81, 81}
  };
}


#endif /*STATIONS_H_*/
