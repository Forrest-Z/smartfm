#pragma once
#include<vector>
#include"coord.h"
#include"param.h"

struct Path : std::vector<COORD> {
    int nearest(COORD pos);
    int forward(int i, double len);
	Path interpolate();
};

