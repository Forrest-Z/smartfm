#ifndef RANDOM_STREAMS_H
#define RANDOM_STREAMS_H

#include <iostream>
#include <vector>
#include <stdlib.h>
#include "util/random.h"

using namespace std;

/* This class encapsulates the streams of random numbers required for state
 * transitions during simulations.
 */
class RandomStreams {
private:
  vector<vector<double>> streams_; // streams_[i] is associated with i-th particle
	mutable int position_;
public:
	RandomStreams(vector<unsigned> seeds, int length);

	int NumStreams() const;
	int Length() const;

	void Advance() const;
	void Back() const;

	void position(int value) const;
	int position() const;

	bool Exhausted() const;

	double Entry(int stream) const;
	double Entry(int stream, int position) const;

	friend ostream& operator<<(ostream& os, const RandomStreams& stream);
};

#endif
