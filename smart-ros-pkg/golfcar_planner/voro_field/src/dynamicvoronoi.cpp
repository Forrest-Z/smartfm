#include "dynamicvoronoi.h"

#include <math.h>
#include <iostream>

bool debug = false;

using namespace std;

DynamicVoronoi::DynamicVoronoi() {
  sqrt2 = sqrt(2.0);
  data = NULL;
  voro = NULL;
  path = NULL;
  gridMap = NULL;
  doubleThreshold = 1; //75
}

DynamicVoronoi::~DynamicVoronoi() {
}

void DynamicVoronoi::initializeEmpty(int _sizeX, int _sizeY, bool initGridMap) {

	if(debug)
		cout <<"initializeEmpty()"<<endl;
  sizeX = _sizeX;
  sizeY = _sizeY;
  if (data) {
    for (int x=0; x<sizeX; x++) delete[] data[x];
    delete[] data;
  }
  if (voro) {
	for (int x=0; x<sizeX; x++) delete[] voro[x];
	delete[] voro;
  }
  if (path) {
  	for (int x=0; x<sizeX; x++) delete[] path[x];
  	delete[] path;
  }
  data = new dataCell*[sizeX];
  voro = new voroCell*[sizeX];
  path = new pathCell*[sizeX];
  for (int x=0; x<sizeX; x++) {
	  data[x] = new dataCell[sizeY];
	  voro[x] = new voroCell[sizeY];
	  path[x] = new pathCell[sizeY];
  }

  if (initGridMap) {
    if (gridMap) {
      for (int x=0; x<sizeX; x++) delete[] gridMap[x];
      delete[] gridMap;
    }
    gridMap = new bool*[sizeX];
    for (int x=0; x<sizeX; x++) gridMap[x] = new bool[sizeY];
  }
  
  dataCell c;
  c.dist = INFINITY;
  c.sqdist = INT_MAX;
  c.obstX = invalidObstData;
  c.obstY = invalidObstData;
  c.voronoi = free;
  c.queueing = fwNotQueued;
  c.needsRaise = false;

  voroCell v;
  v.voroDist = INFINITY;
  v.voroSqdist = INT_MAX;
  v.voroX = invalidObstData;
  v.voroY = invalidObstData;
  v.voroQueueing = fwNotQueued;
  v.voroNeedsRaise = false;

  pathCell p;
  p.cost = INFINITY;

  for (int x=0; x<sizeX; x++)
    for (int y=0; y<sizeY; y++) {
    	data[x][y] = c;
    	voro[x][y] = v;
    	path[x][y] = p;
    }

  if (initGridMap) {
    for (int x=0; x<sizeX; x++) 
      for (int y=0; y<sizeY; y++) gridMap[x][y] = 0;
  }
}

void DynamicVoronoi::initializeMap(int _sizeX, int _sizeY, bool** _gridMap) {
  gridMap = _gridMap;
  initializeEmpty(_sizeX, _sizeY, false);

  if (debug)
  {
	  cout << sizeX <<"initializeMap"<< sizeY <<endl;
  }

  for (int x=0; x<sizeX; x++) {
    for (int y=0; y<sizeY; y++) {
      if (gridMap[x][y]) {
        dataCell c = data[x][y];
        if (!isOccupied(x,y,c)) {
          bool isSurrounded = true;
          for (int dx=-1; dx<=1; dx++) {
            int nx = x+dx;
            if (nx<=0 || nx>=sizeX-1) continue;
            for (int dy=-1; dy<=1; dy++) {
              if (dx==0 && dy==0) continue;
              int ny = y+dy;
              if (ny<=0 || ny>=sizeY-1) continue;

              if (!gridMap[nx][ny]) {
                isSurrounded = false;
                break;
              }
            }
          }
          if (isSurrounded) {
            c.obstX = x;
            c.obstY = y;
            c.sqdist = 0;
            c.dist=0;
            c.voronoi=occupied;
            c.queueing = fwProcessed;
            data[x][y] = c;
          } else setObstacle(x,y);
        }
      }
    }
  }
}

void DynamicVoronoi::occupyCell(int x, int y) {
  gridMap[x][y] = 1;
  setObstacle(x,y);
}
void DynamicVoronoi::clearCell(int x, int y) {
  gridMap[x][y] = 0;
  removeObstacle(x,y);
}

void DynamicVoronoi::setObstacle(int x, int y) {
  dataCell c = data[x][y];

  if(isOccupied(x,y,c)) return;
  
  addList.push_back(INTPOINT(x,y));
  c.obstX = x;
  c.obstY = y;
  data[x][y] = c;
}

void DynamicVoronoi::removeObstacle(int x, int y) {
  dataCell c = data[x][y];
  if(isOccupied(x,y,c) == false) return;
  removeList.push_back(INTPOINT(x,y));
  c.obstX = invalidObstData;
  c.obstY  = invalidObstData;    
  c.queueing = bwQueued;
  data[x][y] = c;
}

void DynamicVoronoi::exchangeObstacles(std::vector<INTPOINT> points) {
	if (debug)
		cout << "exchangeObstacle"<<lastObstacles.size()<<", "<<points.size()<<endl;
  for (unsigned int i=0; i<lastObstacles.size(); i++) {
    int x = lastObstacles[i].x;
    int y = lastObstacles[i].y;

    bool v = gridMap[x][y];
    if (v) continue;
    removeObstacle(x,y);
  }  

  lastObstacles.clear();
  cout << lastObstacles.size()<<endl;

  for (unsigned int i=0; i<points.size(); i++) {
    int x = points[i].x;
    int y = points[i].y;
    bool v = gridMap[x][y];
    if (v) continue;
    setObstacle(x,y);
    lastObstacles.push_back(points[i]);
  }  
  cout << lastObstacles.size()<<endl;

  for (int i =0; i< sizeX; i++ ){
	  for (int j = 0; j< sizeY; j++){
		  if (isVoronoi(i,j))
			  removeVoro(i,j);
	  }
  }
}


void DynamicVoronoi::updateObst(bool updateRealDist) {

	if (debug)
		cout << "updateObst"<<endl;

  commitAndColorize(updateRealDist);

  while (!open.empty()) {
    INTPOINT p = open.pop();
    int x = p.x;
    int y = p.y;
    dataCell c = data[x][y];

    if(c.queueing==fwProcessed) continue; 

    if (c.needsRaise) {
      // RAISE
      for (int dx=-1; dx<=1; dx++) {
        int nx = x+dx;
        if (nx<=0 || nx>=sizeX-1) continue;
        for (int dy=-1; dy<=1; dy++) {
          if (dx==0 && dy==0) continue;
          int ny = y+dy;
          if (ny<=0 || ny>=sizeY-1) continue;
          dataCell nc = data[nx][ny];
          if (nc.obstX!=invalidObstData && !nc.needsRaise) {
            if(!isOccupied(nc.obstX,nc.obstY,data[nc.obstX][nc.obstY])) {
              open.push(nc.sqdist, INTPOINT(nx,ny));
              nc.queueing = fwQueued;
              nc.needsRaise = true;
              nc.obstX = invalidObstData;
              nc.obstY = invalidObstData;
              if (updateRealDist) nc.dist = INFINITY;
              nc.sqdist = INT_MAX;
              data[nx][ny] = nc;
            } else {
              if(nc.queueing != fwQueued){
                open.push(nc.sqdist, INTPOINT(nx,ny));
                //voroQ.push(nc.sqdist, INTPOINT(nx, ny));
                //setVoro(nx,ny);
                nc.queueing = fwQueued;
                data[nx][ny] = nc;
              }
            }      
          }
        }
      }
      c.needsRaise = false;
      c.queueing = bwProcessed;
      data[x][y] = c;
    }
    else if (c.obstX != invalidObstData && isOccupied(c.obstX,c.obstY,data[c.obstX][c.obstY])) {

      // LOWER
      c.queueing = fwProcessed;
      c.voronoi = occupied;

      for (int dx=-1; dx<=1; dx++) {
        int nx = x+dx;
        if (nx<=0 || nx>=sizeX-1) continue;
        for (int dy=-1; dy<=1; dy++) {
          if (dx==0 && dy==0) continue;
          int ny = y+dy;
          if (ny<=0 || ny>=sizeY-1) continue;
          dataCell nc = data[nx][ny];
          if(!nc.needsRaise) {
            int distx = nx-c.obstX;
            int disty = ny-c.obstY;
            int newSqDistance = distx*distx + disty*disty;		
            bool overwrite =  (newSqDistance < nc.sqdist);
            if(!overwrite && newSqDistance==nc.sqdist) { 
              if (nc.obstX == invalidObstData || isOccupied(nc.obstX,nc.obstY,data[nc.obstX][nc.obstY])==false) overwrite = true;
            }
            if (overwrite) {
              open.push(newSqDistance, INTPOINT(nx,ny));
              nc.queueing = fwQueued;
              if (updateRealDist) {
                nc.dist = sqrt((double) newSqDistance);
              }
              nc.sqdist = newSqDistance;
              nc.obstX = c.obstX;
              nc.obstY = c.obstY;
            } else { 
            	//doubleThreshold = ((data[x][y].sqdist -2)<=(data[nx][ny].sqdist-2)?(data[x][y].sqdist -2):(data[nx][ny].sqdist-2)) ;
            	//cout<<doubleThreshold<<endl;
              checkVoro(x,y,nx,ny,c,nc);
            }
            data[nx][ny] = nc;
          }
        }
      }
    }
    data[x][y] = c;
  }
}

void DynamicVoronoi::setVoro(int x, int y) {
  voroCell c = voro[x][y];
  dataCell v = data[x][y];
  if(isVoroOccup(x,y,c)) return;
  voroAddList.push_back(INTPOINT(x,y));
  c.voroX = x;
  c.voroY = y;
  v.voronoi = free;
  voro[x][y] = c;
  data[x][y] = v;
}

void DynamicVoronoi::removeVoro(int x, int y) {
  voroCell c = voro[x][y];
  dataCell v = data[x][y];
  if(isVoroOccup(x,y,c) == false) return;
  voroRemoveList.push_back(INTPOINT(x,y));
  c.voroX = invalidObstData;
  c.voroY  = invalidObstData;
  c.voroQueueing = bwQueued;
  voro[x][y] = c;
  v.voronoi = occupied;
  data[x][y] = v;
}


void DynamicVoronoi::updateVoro(bool updateRealDist) {

	if (debug)
			cout << "updateVoro"<<endl;
  voroCommitAndColorize(updateRealDist);
  while (!voroOpen.empty()) {
    INTPOINT p = voroOpen.pop();
    int x = p.x;
    int y = p.y;
    voroCell c = voro[x][y];

    if(c.voroQueueing==fwProcessed) continue;

    if (c.voroNeedsRaise) {
      // RAISE
      for (int dx=-1; dx<=1; dx++) {
        int nx = x+dx;
        if (nx<=0 || nx>=sizeX-1) continue;
        for (int dy=-1; dy<=1; dy++) {
          if (dx==0 && dy==0) continue;
          int ny = y+dy;
          if (ny<=0 || ny>=sizeY-1) continue;
          voroCell nc = voro[nx][ny];
          if (nc.voroX!=invalidObstData && !nc.voroNeedsRaise) {
            if(!isVoroOccup(nc.voroX,nc.voroY,voro[nc.voroX][nc.voroY])) {
              voroOpen.push(nc.voroSqdist, INTPOINT(nx,ny));
              nc.voroQueueing = fwQueued;
              nc.voroNeedsRaise = true;
              nc.voroX = invalidObstData;
              nc.voroY = invalidObstData;
              if (updateRealDist) nc.voroDist = INFINITY;
              nc.voroSqdist = INT_MAX;
              voro[nx][ny] = nc;
            } else {
              if(nc.voroQueueing != fwQueued){
                voroOpen.push(nc.voroSqdist, INTPOINT(nx,ny));
                nc.voroQueueing = fwQueued;
                voro[nx][ny] = nc;
              }
            }
          }
        }
      }
      c.voroNeedsRaise = false;
      c.voroQueueing = bwProcessed;
      voro[x][y] = c;
    }
    else if (c.voroX != invalidObstData && isVoroOccup(c.voroX,c.voroY,voro[c.voroX][c.voroY])) {

      // LOWER
      c.voroQueueing = fwProcessed;

      for (int dx=-1; dx<=1; dx++) {
        int nx = x+dx;
        if (nx<=0 || nx>=sizeX-1) continue;
        for (int dy=-1; dy<=1; dy++) {
          if (dx==0 && dy==0) continue;
          int ny = y+dy;
          if (ny<=0 || ny>=sizeY-1) continue;
          voroCell nc = voro[nx][ny];
          if(!nc.voroNeedsRaise) {
            int distx = nx-c.voroX;
            int disty = ny-c.voroY;
            int newVoroSqDistance = distx*distx + disty*disty;
            bool overwrite =  (newVoroSqDistance < nc.voroSqdist);
            if(!overwrite && newVoroSqDistance==nc.voroSqdist) {
              if (nc.voroX == invalidObstData || isVoroOccup(nc.voroX,nc.voroY,voro[nc.voroX][nc.voroY])==false) overwrite = true;
            }
            if (overwrite) {
              voroOpen.push(newVoroSqDistance, INTPOINT(nx,ny));
              nc.voroQueueing = fwQueued;
              if (updateRealDist) {
                nc.voroDist = sqrt((double) newVoroSqDistance);
              }
              nc.voroSqdist = newVoroSqDistance;
              nc.voroX = c.voroX;
              nc.voroY = c.voroY;
            }
            voro[nx][ny] = nc;
          }
        }
      }
    }
    voro[x][y] = c;
  }
}

//To get the obstacle distance
float DynamicVoronoi::getDistance( int x, int y ) {
  if( (x>0) && (x<sizeX) && (y>0) && (y<sizeY)) return data[x][y].dist; 
  else return -INFINITY;
}

//To get the voro distance
float DynamicVoronoi::voroGetDistance( int x, int y ) {
  if( (x>0) && (x<sizeX) && (y>0) && (y<sizeY)) return voro[x][y].voroDist;
  else return -INFINITY;
}

bool DynamicVoronoi::isVoronoi( int x, int y ) {
  dataCell c = data[x][y];
  return (c.voronoi==free || c.voronoi==voronoiKeep);
}


void DynamicVoronoi::commitAndColorize(bool updateRealDist) {
  // ADD NEW OBSTACLES
  for (unsigned int i=0; i<addList.size(); i++) {
    INTPOINT p = addList[i];
    int x = p.x;
    int y = p.y;
    dataCell c = data[x][y];

    if(c.queueing != fwQueued){
      if (updateRealDist) c.dist = 0;
      c.sqdist = 0;
      c.obstX = x;
      c.obstY = y;
      c.queueing = fwQueued;
      c.voronoi = occupied;
      data[x][y] = c;
      open.push(0, INTPOINT(x,y));
    }
  }

  // REMOVE OLD OBSTACLES
  for (unsigned int i=0; i<removeList.size(); i++) {
    INTPOINT p = removeList[i];
    int x = p.x;
    int y = p.y;
    dataCell c = data[x][y];

    if (isOccupied(x,y,c)==true) continue; // obstacle was removed and reinserted
    open.push(0, INTPOINT(x,y));
    if (updateRealDist) c.dist  = INFINITY;
    c.sqdist = INT_MAX;
    c.needsRaise = true;
    data[x][y] = c;
  }
  removeList.clear();
  addList.clear();
}


void DynamicVoronoi::voroCommitAndColorize(bool updateRealDist) {
  // ADD NEW VORO
  for (unsigned int i=0; i<voroAddList.size(); i++) {
    INTPOINT p = voroAddList[i];
    int x = p.x;
    int y = p.y;
    voroCell c = voro[x][y];

    if(c.voroQueueing != fwQueued){
      if (updateRealDist) c.voroDist = 0;
      c.voroSqdist = 0;
      c.voroX = x;
      c.voroY = y;
      c.voroQueueing = fwQueued;
      voro[x][y] = c;
      voroOpen.push(0, INTPOINT(x,y));
    }
  }

  // REMOVE OLD VORO
  for (unsigned int i=0; i<voroRemoveList.size(); i++) {
    INTPOINT p = voroRemoveList[i];
    int x = p.x;
    int y = p.y;
    voroCell c = voro[x][y];

    if (isVoroOccup(x,y,c)==true) continue; // Voro was removed and reinserted
    voroOpen.push(0, INTPOINT(x,y));
    if (updateRealDist) c.voroDist  = INFINITY;
    c.voroSqdist = INT_MAX;
    c.voroNeedsRaise = true;
    voro[x][y] = c;
  }
  voroRemoveList.clear();
  voroAddList.clear();
}



void DynamicVoronoi::checkVoro(int x, int y, int nx, int ny, dataCell& c, dataCell& nc) {

  if ((c.sqdist>doubleThreshold || nc.sqdist>doubleThreshold) && nc.obstX!=invalidObstData) {
    if (abs(c.obstX-nc.obstX) > 12 || abs(c.obstY-nc.obstY) > 12) {
      //compute dist from x,y to obstacle of nx,ny	 
      int dxy_x = x-nc.obstX;
      int dxy_y = y-nc.obstY;
      int sqdxy = dxy_x*dxy_x + dxy_y*dxy_y;
      int stability_xy = sqdxy - c.sqdist;
      if (sqdxy - c.sqdist<0) return;

      //compute dist from nx,ny to obstacle of x,y
      int dnxy_x = nx - c.obstX;
      int dnxy_y = ny - c.obstY;
      int sqdnxy = dnxy_x*dnxy_x + dnxy_y*dnxy_y;
      int stability_nxy = sqdnxy - nc.sqdist;
      if (sqdnxy - nc.sqdist <0) return;

      //which cell is added to the Voronoi diagram?
      if(stability_xy <= stability_nxy && c.sqdist>2) {
        if (c.voronoi != free) {
          c.voronoi = free;
          setVoro(x,y);
          reviveVoroNeighbors(x,y);
          pruneQueue.push(INTPOINT(x,y));
        }
      }
      if(stability_nxy <= stability_xy && nc.sqdist>2) {
        if (nc.voronoi != free) {
          nc.voronoi = free;
          setVoro(nx,ny);
          reviveVoroNeighbors(nx,ny);
          pruneQueue.push(INTPOINT(nx,ny));
        }
      }
    }
  }
}


void DynamicVoronoi::reviveVoroNeighbors(int &x, int &y) {
  for (int dx=-1; dx<=1; dx++) {
    int nx = x+dx;
    if (nx<=0 || nx>=sizeX-1) continue;
    for (int dy=-1; dy<=1; dy++) {
      if (dx==0 && dy==0) continue;
      int ny = y+dy;
      if (ny<=0 || ny>=sizeY-1) continue;
      dataCell nc = data[nx][ny];
      if (nc.sqdist != INT_MAX && !nc.needsRaise && (nc.voronoi == voronoiKeep || nc.voronoi == voronoiPrune)) {
        nc.voronoi = free;
        setVoro(nx,ny);
        data[nx][ny] = nc;
        pruneQueue.push(INTPOINT(nx,ny));
      }
    }
  }
}


bool DynamicVoronoi::isOccupied(int x, int y) {
  dataCell c = data[x][y];
  return (c.obstX==x && c.obstY==y);
}

bool DynamicVoronoi::isOccupied(int &x, int &y, dataCell &c) { 
  return (c.obstX==x && c.obstY==y);
}


bool DynamicVoronoi::isVoroOccup(int x, int y) {
  voroCell c = voro[x][y];
  return (c.voroX==x && c.voroY==y);
}

bool DynamicVoronoi::isVoroOccup(int &x, int &y, voroCell &c) {
  return (c.voroX==x && c.voroY==y);
}


void DynamicVoronoi::prune() {
  // filler
	if(debug)
		std::cout << "prune"<<std::endl;
  while(!pruneQueue.empty()) {
    INTPOINT p = pruneQueue.front();
    pruneQueue.pop();
    int x = p.x;
    int y = p.y;

    if (data[x][y].voronoi==occupied) continue;
    if (data[x][y].voronoi==freeQueued) continue;

    data[x][y].voronoi = freeQueued;
    open.push(data[x][y].sqdist, p);

    /* tl t tr
       l c r
       bl b br */

    dataCell tr,tl,br,bl;
    tr = data[x+1][y+1];
    tl = data[x-1][y+1];
    br = data[x+1][y-1];
    bl = data[x-1][y-1];

    dataCell r,b,t,l;
    r = data[x+1][y];
    l = data[x-1][y];
    t = data[x][y+1];
    b = data[x][y-1];

    if (x+2<sizeX && r.voronoi==occupied) { 
      // fill to the right
      if (tr.voronoi!=occupied && br.voronoi!=occupied && data[x+2][y].voronoi!=occupied) {
        r.voronoi = freeQueued;
        open.push(r.sqdist, INTPOINT(x+1,y));
        data[x+1][y] = r;
      }
    } 
    if (x-2>=0 && l.voronoi==occupied) { 
      // fill to the left
      if (tl.voronoi!=occupied && bl.voronoi!=occupied && data[x-2][y].voronoi!=occupied) {
        l.voronoi = freeQueued;
        open.push(l.sqdist, INTPOINT(x-1,y));
        data[x-1][y] = l;
      }
    } 
    if (y+2<sizeY && t.voronoi==occupied) { 
      // fill to the top
      if (tr.voronoi!=occupied && tl.voronoi!=occupied && data[x][y+2].voronoi!=occupied) {
        t.voronoi = freeQueued;
        open.push(t.sqdist, INTPOINT(x,y+1));
        data[x][y+1] = t;
      }
    } 
    if (y-2>=0 && b.voronoi==occupied) { 
      // fill to the bottom
      if (br.voronoi!=occupied && bl.voronoi!=occupied && data[x][y-2].voronoi!=occupied) {
        b.voronoi = freeQueued;
        open.push(b.sqdist, INTPOINT(x,y-1));
        data[x][y-1] = b;
      }
    } 
  }


  while(!open.empty()) {
    INTPOINT p = open.pop();
    dataCell c = data[p.x][p.y];
    int v = c.voronoi;
    if (v!=freeQueued && v!=voronoiRetry) { // || v>free || v==voronoiPrune || v==voronoiKeep) {
      //      assert(v!=retry);
      continue;
    }

    markerMatchResult r = markerMatch(p.x,p.y);
    if (r==pruned) c.voronoi = voronoiPrune;
    else if (r==keep) c.voronoi = voronoiKeep;
    else { // r==retry
      c.voronoi = voronoiRetry;
      //      printf("RETRY %d %d\n", x, sizeY-1-y);
      pruneQueue.push(p);
    }
    data[p.x][p.y] = c;

    if (open.empty()) {
      while (!pruneQueue.empty()) {
        INTPOINT p = pruneQueue.front();
        pruneQueue.pop();
        open.push(data[p.x][p.y].sqdist, p);
      }
    }
  }
  //  printf("match: %d\nnomat: %d\n", matchCount, noMatchCount);
}


DynamicVoronoi::markerMatchResult DynamicVoronoi::markerMatch(int x, int y) {
  // implementation of connectivity patterns
  bool f[8];

  int nx, ny;
  int dx, dy;

  int i=0;
  int count=0;
  //  int obstacleCount=0;
  int voroCount=0;
  int voroCountFour=0;

  for (dy=1; dy>=-1; dy--) {
    ny = y+dy;
    for (dx=-1; dx<=1; dx++) {
      if (dx || dy) {
        nx = x+dx;
        dataCell nc = data[nx][ny];
        int v = nc.voronoi;
        bool b = (v<=free && v!=voronoiPrune); 
        //	if (v==occupied) obstacleCount++;
        f[i] = b;
        if (b) {
          voroCount++;
          if (!(dx && dy)) voroCountFour++;
        }
        if (b && !(dx && dy) ) count++;
        //	if (v<=free && !(dx && dy)) voroCount++;
        i++;
      }
    }
  }
  if (voroCount<3 && voroCountFour==1 && (f[1] || f[3] || f[4] || f[6])) {
    //    assert(voroCount<2);
    //    if (voroCount>=2) printf("voro>2 %d %d\n", x, y);
    return keep;
  }

  // 4-connected
  if ((!f[0] && f[1] && f[3]) || (!f[2] && f[1] && f[4]) || (!f[5] && f[3] && f[6]) || (!f[7] && f[6] && f[4])) return keep;
  if ((f[3] && f[4] && !f[1] && !f[6]) || (f[1] && f[6] && !f[3] && !f[4])) return keep;
  
  // keep voro cells inside of blocks and retry later
  if (voroCount>=5 && voroCountFour>=3 && data[x][y].voronoi!=voronoiRetry) {
    return retry;
  }

  return pruned;
}

void DynamicVoronoi::update()
{
	updateObst(true);
	//recheckVoro();
	updateVoro(true);
}

void DynamicVoronoi::updatePathCost(std::vector<geometry_msgs::Point32> &_cost, float resolution)
{
	for (int x = 0; x < sizeX; x++)
	{
		for(int y = 0; y < sizeY; y++)
		{
			/*

			if (data[x][y].dist > dmax || voro[x][y].voroDist > sqrt(sizeX*sizeX + sizeY*sizeY))
				path[x][y].cost = 0;
			else
			{
				float fallOff = 1;// ALPHA/(ALPHA + data[x][y].dist);
				float scaFac = voro[x][y].voroDist/(data[x][y].dist+voro[x][y].voroDist);
				float maxEdge = (dmax - data[x][y].dist)* (dmax - data[x][y].dist)/(dmax * dmax);
				path[x][y].cost = fallOff*scaFac*maxEdge;
			}
			*/

			//std::cout << path[x][y].cost <<",";
				path[x][y].cost = fabs(data[x][y].dist - voro[x][y].voroDist);

				if(data[x][y].sqdist == 0 )
					path[x][y].cost = FLT_MAX;
				float f =80+(path[x][y].cost*10);
				geometry_msgs::Point32 cost;
				cost.x = x*resolution -10;
				cost.y = -(y)*resolution +10;
				cost.z = f;
				_cost.push_back(cost);
		}
	}
}


void DynamicVoronoi::visualize(const char *filename, const char *voroFilename, const char* voroFieldFilename) {
  // write pgm files

  FILE* F = fopen(filename, "w");
  FILE* VF = fopen(voroFieldFilename, "w");
  FILE* V = fopen(voroFilename, "w");
  if (!F || !VF ||!V) {
    std::cerr << "could not open 'result.pgm' for writing!\n";
    return;
  }
  fprintf(F, "P6\n");
  fprintf(F, "%d %d 255\n", sizeX, sizeY);

  fprintf(V, "P6\n");
  fprintf(V, "%d %d 255\n", sizeX, sizeY);

  fprintf(VF, "P6\n");
  fprintf(VF, "%d %d 255\n", sizeX, sizeY);

  for(int y = sizeY-1; y >=0; y--){
    for(int x = 0; x<sizeX; x++){
      unsigned char c = 0;

      if (isVoronoi(x,y))
      {
    	  fputc(255, F);
    	  fputc(0, F);
    	  fputc(0, F);
      }
      else if (data[x][y].sqdist==0) {
        fputc( 0, F );
        fputc( 0, F );
        fputc( 0, F );
      } else {
        float f = 80+(data[x][y].dist*5);
        if (f>255) f=255;
        if (f<0) f=0;
        c = (unsigned char)f;
        fputc( c, F );
        fputc( c, F );
        fputc( c, F );
      }
      /*
      if (gridMap[x][y]) {
              fputc( 0, F );
              fputc( 0, F );
              fputc( 0, F );
            } else {
              fputc( 255, F );
              fputc( 255, F );
              fputc( 255, F );
            }
            */

      if (voro[x][y].voroSqdist==0)
      {
    	  fputc( 0, V );
    	  fputc( 0, V );
    	  fputc( 0, V );
      }
      else
      {
    	  float f = 80+(voro[x][y].voroDist*5);
    	  if (f>255) f=255;
    	  if (f<0) f=0;
    	  c = (unsigned char)f;
    	  fputc( c, V );
    	  fputc( c, V );
    	  fputc( c, V );
      }

      //float f = 255 - 255*path[x][y].cost;

      if (path[x][y].cost < 1.5)
      {
    	  fputc( 255, VF );
    	  fputc( 0, VF );
    	  fputc( 0, VF );
      }
      else
      {
      float f =80+(path[x][y].cost*10);

      if (f>255)
      {
    	  f = 255;
      }
      if (f<0)
      {
    	  f = 0;
      }

      c = (unsigned char)(255-f);
      fputc( c, VF );
      fputc( c, VF );
      fputc( c, VF );
      }
    }
  }
  fclose(F);
}
