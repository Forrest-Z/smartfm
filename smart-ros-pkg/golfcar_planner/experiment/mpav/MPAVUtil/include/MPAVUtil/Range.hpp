/*
 * Range.hpp
 *
 *  Created on: Nov 23, 2013
 *      Author: liuwlz
 */

#ifndef RANGE_HPP_
#define RANGE_HPP_

#include <stdlib.h>
#include <math.h>
#include <assert.h>

using namespace std;

namespace MPAV{

	template <class T>
	bool WithIn(T variableIn, T center, T range){
		assert(range > 0);
		return (variableIn < (center+range) && variableIn > (center-range));
	}

	template <class T>
	T Saturate(T variableIn, T bound){
		assert(bound >0);
		T variableOut, tempBound;
		tempBound = variableIn > 0 ? bound: -bound;
		variableOut = fabs((double)variableIn) < bound ? variableIn : tempBound;
		return variableOut;
	}

	template <class T>
	double DistCal(T x_1, T y_1, T x_2, T y_2){
		return sqrt((x_2-x_1)*(x_2-x_1) + (y_2 - y_1)*(y_2 - y_1));
	}
}


#endif /* RANGE_HPP_ */
