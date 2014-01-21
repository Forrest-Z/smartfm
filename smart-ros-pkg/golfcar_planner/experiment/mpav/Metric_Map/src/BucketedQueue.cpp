/*
 * BucketedQueue.cpp
 *
 *  Created on: Dec 19, 2013
 *      Author: liuwlz
 */

#include <Metric_Map/BucketedQueue.hpp>

namespace MPAV{

	BucketPrioQueue::BucketPrioQueue() {
		// make sure the index array is created
		if (sqrIndices.size()==0)
			initSqrIndices();
		nextBucket = INT_MAX;
		buckets = std::vector<std::queue<INTPOINT> >(numBuckets);
		count = 0;
	}

	bool BucketPrioQueue::empty() {
		return (count==0);
	}

	void BucketPrioQueue::push(int prio, INTPOINT t) {
		if (prio>=(int)sqrIndices.size()) {
			fprintf(stderr, "error: priority %d is not a valid squared distance x*x+y*y, or x>MAXDIST or y>MAXDIST.\n", prio);
			exit(0);
		}
		int id = sqrIndices[prio];
		if (id<0) {
			fprintf(stderr, "error: priority %d is not a valid squared distance x*x+y*y, or x>MAXDIST or y>MAXDIST.\n", prio);
			exit(0);
		}
		buckets[id].push(t);
		if (id<nextBucket)
			nextBucket = id;
		count++;
	}

	INTPOINT BucketPrioQueue::pop() {
		int i;
		assert(count>0);

		for (i = nextBucket; i<(int)buckets.size(); i++) {
			if (!buckets[i].empty())
				break;
		}
		assert(i<(int)buckets.size());
		nextBucket = i;

		count--;
		INTPOINT p = buckets[i].front();
		buckets[i].pop();
		return p;
	}

	void BucketPrioQueue::initSqrIndices() {

		sqrIndices = std::vector<int>(2*MAXDIST*MAXDIST+1, -1);

		int count=0;
		for (int x=0; x<=MAXDIST; x++) {
			for (int y=0; y<=x; y++) {
				int sqr = x*x+y*y;
				sqrIndices[sqr] = count++;
			}
		}
		numBuckets = count;
	}
}

