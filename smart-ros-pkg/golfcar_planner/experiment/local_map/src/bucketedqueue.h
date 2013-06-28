#ifndef _PRIORITYQUEUE2_H_
#define _PRIORITYQUEUE2_H_

#define MAXDIST 1000
#define RESERVE 64

#include <vector>
#include <set>
#include <queue>
#include <assert.h>
//#include "point.h"

//! Priority queue for integer coordinates with squared distances as priority.
/** A priority queue that uses buckets to group elements with the same priority.
 *  The individual buckets are unsorted, which increases efficiency if these groups are large.
 *  The elements are assumed to be integer coordinates, and the priorities are assumed
 *  to be squared euclidean distances (integers).
 */

#define INTPOINT IntPoint

/*! A light-weight integer point with fields x,y */
class IntPoint {
public:
  IntPoint() : x(0), y(0) {}
  IntPoint(int _x, int _y) : x(_x), y(_y) {}
  int x,y;
};

class BucketPrioQueue {
	friend class obsDistMetric;

public:
  /** Standard constructor. When called for the first time it creates a look up table 
   *  that maps square distanes to bucket numbers, which might take some time... 
   */
  BucketPrioQueue(); 
  //! Checks whether the Queue is empty
  bool empty();
  //! push an element
  void push(int prio, INTPOINT t);
  //! return and pop the element with the lowest squared distance */
  INTPOINT pop();

private:
  
  static void initSqrIndices();
  static std::vector<int> sqrIndices;
  static int numBuckets;
  int count;
  int nextBucket;

  std::vector<std::queue<INTPOINT> > buckets;
};

#endif
