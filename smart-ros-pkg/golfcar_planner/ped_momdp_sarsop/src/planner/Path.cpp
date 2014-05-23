#include<Path.h>



int Path::nearest(COORD pos) {
    auto& path = *this;
    double dmin = EuclideanDistance(pos, path[0]);
    int imin = 0;
    for(int i=0; i<path.size(); i++) {
        double d = EuclideanDistance(pos, path[i]);
        if(dmin < d) {
            dmin = d;
            imin = i;
        }
    }
    return imin;
}

int Path::forward(int i, double len) {
    auto& path = *this;
    while(len > 0 and i<path.size()-1) {
        double d = EuclideanDistance(path[i], path[i+1]);
        len -= d;
    }
    return i;
}

