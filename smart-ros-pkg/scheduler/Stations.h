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
    {WORKSHOP, MCDONALD, 153, 153},
    {WORKSHOP, EA, 379, 379},
    {WORKSHOP, E3A, 450, 450},
    {MCDONALD, WORKSHOP, 107, 107},
    {MCDONALD, EA, 485, 485},
    {MCDONALD, E3A, 556, 556},
    {EA, WORKSHOP, 340, 340},
    {EA, MCDONALD, 493, 493},
    {EA, E3A, 151, 151},
    {E3A, WORKSHOP, 424, 424},
    {E3A, MCDONALD, 577, 577},
    {E3A, EA, 81, 81}
  };
}


#endif /*STATIONS_H_*/
