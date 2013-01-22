#ifndef _DYNAMICVORONOI_H_
#define _DYNAMICVORONOI_H_


#include <stdlib.h>
#include <stdio.h>
#include <limits.h>
#include <float.h>
#include <queue>
#include <geometry_msgs/Point32.h>
#include "bucketedqueue.h"

const float resolution = 0.05;
const float ALPHA = 10000000;
const float DMAX = 1000;

//! A DynamicVoronoi object computes and updates a distance map and Voronoi diagram.
class DynamicVoronoi {

	friend class VoroField;
  
public:
  
  DynamicVoronoi();
  ~DynamicVoronoi();

  //! Initialization with an empty map
  void initializeEmpty(int _sizeX, int _sizeY, bool initGridMap=true);
  //! Initialization with a given binary map (false==free, true==occupied)
  void initializeMap(int _sizeX, int _sizeY, bool** _gridMap);

  void initilaizeVoroMap();
  //! add an obstacle at the specified cell coordinate
  void occupyCell(int x, int y);
  //! remove an obstacle at the specified cell coordinate
  void clearCell(int x, int y);
  //! remove old dynamic obstacles and add the new ones
  void exchangeObstacles(std::vector<INTPOINT> newObstacles);

  //! update distance map and Voronoi diagram to reflect the changes
  void updateObst(bool updateRealDist=true);
  //Update voro distance map
  void updateVoro(bool updateRealDist=true);
  // update all
  void update();
  //! prune the Voronoi diagram
  void prune();
  
  //! returns the obstacle distance at the specified location
  float getDistance( int x, int y );
  //! returns the voro distance at the specified location
  float voroGetDistance( int x, int y );
  //! returns whether the specified cell is part of the (pruned) Voronoi graph
  bool isVoronoi( int x, int y );
  //! checks whether the specficied location is occupied
  bool isOccupied(int x, int y);
  //! check whether the specified location is voro diagram
  bool isVoroOccup(int x, int y);
  //! write the current distance map and voronoi diagram as ppm file
  void visualize(const char* filename="obst.ppm", const char* voroFilename="voro.ppm", const char* voroFieldFilename="voro_field.ppm");
  //Update path cost
  void updatePathCost(std::vector<geometry_msgs::Point32> &_cost, float resolution);

  //! returns the horizontal size of the workspace/map
  unsigned int getSizeX() {return sizeX;}
  //! returns the vertical size of the workspace/map
  unsigned int getSizeY() {return sizeY;}

private:  
  struct dataCell {
	  float dist;
	  char voronoi;
	  char queueing;
	  int obstX;
	  int obstY;
	  bool needsRaise;
	  int sqdist;
  };

  struct voroCell {
	  float voroDist;
      char voroQueueing;
      int voroX;
      int voroY;
      bool voroNeedsRaise;
      int voroSqdist;
    };

  struct pathCell{
	  float cost;
  };

  typedef enum {voronoiKeep=-4, freeQueued = -3, voronoiRetry=-2, voronoiPrune=-1, free=0, occupied=1} State;
  typedef enum {fwNotQueued=1, fwQueued=2, fwProcessed=3, bwQueued=4, bwProcessed=1} QueueingState;
  typedef enum {invalidObstData = SHRT_MAX/2} ObstDataState;
  typedef enum {pruned, keep, retry} markerMatchResult;

  // methods for distance map and GVD
  void setObstacle(int x, int y);
  void removeObstacle(int x, int y);
  inline void checkVoro(int x, int y, int nx, int ny, dataCell& c, dataCell& nc);
  void recheckVoro();
  void commitAndColorize(bool updateRealDist=true);
  void voroCommitAndColorize(bool updateRealDist=true);
  inline void reviveVoroNeighbors(int &x, int &y);

  inline bool isOccupied(int &x, int &y, dataCell &c);
  inline bool isVoroOccup(int &x, int &y, voroCell &c);
  bool patternMatch(int x, int y);
  bool chkVoroConditions(dataCell& c, dataCell& nc);

  inline markerMatchResult markerMatch(int x, int y);


  //methods for vororoi map
  void setVoro(int x, int y);
  void removeVoro(int x, int y);

  // queues for obstacle distance map

  BucketPrioQueue open;
  std::queue<INTPOINT> pruneQueue;
  std::vector<INTPOINT> removeList;
  std::vector<INTPOINT> addList;
  std::vector<INTPOINT> lastObstacles;

  //queues for voro distance map
  BucketPrioQueue voroOpen;
  std::vector<INTPOINT> voroRemoveList;
  std::vector<INTPOINT> voroAddList;

  //queus for voroQ distance map
  BucketPrioQueue voroQ;
  std::vector<INTPOINT> voroQRemoveList;
  std::vector<INTPOINT> voroQAddList;

  // maps
  int sizeY;
  int sizeX;
  dataCell** data;
  voroCell** voro;
  pathCell** path;
  bool** gridMap;
  int obstID ;

  // parameters
  int padding;
  double doubleThreshold;

  double sqrt2;

  //  dataCell** getData(){ return data; }
};


#endif

