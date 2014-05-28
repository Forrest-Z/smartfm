#pragma once
#include<vector>
#include"coord.h"
#inlcude"param.h"

struct Path : std::vector<COORD> {
    int nearest(COORD pos);
    int forward(int i, double len);
};

