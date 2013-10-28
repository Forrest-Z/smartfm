#ifndef __RRTS_H_
#define __RRTS_H_


#include "kdtree.h"

#include <list>
#include <set>
#include <vector>
#include <set>
#include "dubins_car.h"
using namespace std;

namespace RRTstar {

  template < class T > class Planner;

  template < class T >
    class Vertex {

      public:

        typedef typename T::StateType state_t;
        typedef typename T::TrajectoryType trajectory_t;
        typedef typename T::SystemType system_t;

        typedef Vertex<T> vertex_t;

        vertex_t *parent;
        state_t *state;
        set<vertex_t*> children;
        double costFromParent;
        double costFromRoot;
        trajectory_t *trajFromParent;



        Vertex ();
        ~Vertex ();    
        Vertex (const Vertex &vertexIn);

        // int setState (State& stateIn);
        state_t& getState () {return *state;}
        state_t& getState () const {return *state;}
        vertex_t& getParent () {return *parent;}
        double getCost () {return costFromRoot;}

        friend class Planner<T>;
    };




  template < class T >
    class Planner {

      public:

        typedef typename T::StateType state_t;
        typedef typename T::TrajectoryType trajectory_t;
        typedef typename T::SystemType system_t;

        typedef Vertex<T> vertex_t;
        typedef struct kdtree kdtree_t;
        typedef struct kdres kdres_t;

        int numDimensions;


        double gamma;
        double goalSampleFreq;

        double lowerBoundCost;
        vertex_t *lowerBoundVertex;
        kdtree_t *kdtree;

        vertex_t *root;

        int insertIntoKdtree (vertex_t &vertexIn);

        int getNearestVertex (state_t& stateIn, vertex_t*& vertexPointerOut);    
        int getNearVertices (state_t& stateIn, vector<vertex_t*>& vectorNearVerticesOut);

        int checkUpdateBestVertex (vertex_t& vertexIn);

        vertex_t* insertTrajectory (vertex_t& vertexStartIn, trajectory_t& trajectoryIn);    
        int insertTrajectory (vertex_t& vertexStartIn, trajectory_t& trajectoryIn, vertex_t& vertexEndIn);

        int findBestParent (state_t& stateIn, vector<vertex_t*>& vectorNearVerticesIn,
            vertex_t*& vertexBestOut, trajectory_t& trajectoryOut, bool& exactConnection, list<float> &control);

        int updateBranchCost (vertex_t& vertexIn, int depth);    
        int rewireVertices (vertex_t& vertexNew, vector<vertex_t*>& vectorNearVertices); 

        int findDescendantVertices (vertex_t* vertexIn);
        int recomputeCost (vertex_t* vertexIn);

        int markCost (vertex_t& vertexIn);
        int checkTrajectory (vertex_t& vertexIn);


        list<vertex_t*> listVertices;
        int numVertices;

        system_t *system;

        Planner ();
        ~Planner ();

        int setGamma (double gammaIn);
        int setGoalSampleFrequency (double sampleFreqIn);

        int setSystem (system_t& system);
        vertex_t& getRootVertex ();
        int initialize ();


        int iteration ();

        int isSafeTrajectory(list<double*> &trajectory);
        int checkTree();
        int lazyCheckTree();

        int updateReachability (); 
        int switchRoot (double distance, list<double*> &traj, list<float>& control, list<int>& direction);

        double getBestVertexCost () {return lowerBoundCost;}
        vertex_t& getBestVertex () {return *lowerBoundVertex;}
        int getBestTrajectory (list<double*>& trajectory, list<float> &control);

        double getTrajectoryLength(list<double*> &trajectory);
    };

};

#endif
