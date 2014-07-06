#include<Path.h>
#include<iostream>
using namespace std;



int Path::nearest(COORD pos) {
    auto& path = *this;
    double dmin = COORD::EuclideanDistance(pos, path[0]);
    int imin = 0;
    for(int i=0; i<path.size(); i++) {
        double d = COORD::EuclideanDistance(pos, path[i]);
        if(dmin > d) {
            dmin = d;
            imin = i;
        }
    }
    return imin;
}

int Path::forward(int i, double len) const {
    auto& path = *this;
    //while(len > 0 and i<path.size()-1) {
        //double d = COORD::EuclideanDistance(path[i], path[i+1]);
        //len -= d;
		//i++;
    //}
    i += int(len / ModelParams::PATH_STEP);
    if(i > path.size()-1) {
        i = path.size()-1;
    }
    return i;
}

double Path::getYaw(int i) const {
	//TODO review this code
	if(i == path.size()-1) return 0;

	const COORD& pos = path[i];
	const COORD& forward_pos = path[i+1];
	MyVector vec(pos.x - forward_pos.x, pos.y - forward_pos.y);
    double a = vec.GetAngle(); // is this the yaw angle?
	return a;
}

Path Path::interpolate() {
    auto& path = *this;
	Path p;
	for(int i=0; i<path.size()-1; i++) {
        double d = COORD::EuclideanDistance(path[i], path[i+1]);
		int n = int(d/ModelParams::PATH_STEP);
		double dx,dy;
		dx=(path[i+1].x-path[i].x)/n;
		dy=(path[i+1].y-path[i].y)/n;
		double nx,ny;
		nx=path[i].x;
		ny=path[i].y;
		for(int j=0;j<n;j++) {
			p.push_back(COORD(nx,ny));	
			nx+=dx;
			ny+=dy;
		}
	}
	p.push_back(path[path.size()-1]);
	return p;
}

void Path::cutjoin(const Path& p) {
	int i = nearest(p[0]);
	erase(at(i), end());
	insert(end(), p.begin(), p.end());
}
