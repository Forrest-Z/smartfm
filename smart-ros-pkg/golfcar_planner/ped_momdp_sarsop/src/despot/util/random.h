#ifndef RANDOM_H
#define RANDOM_H

#include <vector>

using namespace std;

class Random {
private:
	unsigned seed_;

public:
	static Random RANDOM;

	Random(double seed);
	Random(unsigned seed);

	unsigned NextUnsigned();
	int NextInt(int n);
	int NextInt(int min, int max);

	double NextDouble();
	double NextDouble(double min, double max);

	double NextGaussian();

	int NextCategory(vector<double> category_probs);

	static int GetCategory(vector<double> category_probs, double rand_num);
};

#endif
