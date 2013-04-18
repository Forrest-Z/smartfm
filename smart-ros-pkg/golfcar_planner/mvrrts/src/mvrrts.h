#ifndef __RRTS_H_
#define __RRTS_H_


#include "kdtree.h"

#include <list>
#include <set>
#include <vector>
#include <set>

#include "mvdubins.h"
#include "automata.h"
using namespace std;

namespace MVRRTstar{

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
        
        Level_of_US lus_from_root;
        Level_of_US lus_from_parent;
        trajectory_t *trajFromParent;

        Vertex ();
        ~Vertex ();    
        Vertex (const Vertex &vertexIn);

        // int setState (State& stateIn);
        state_t& getState () {return *state;}
        state_t& getState () const {return *state;}
        vertex_t& getParent () {return *parent;}
        Level_of_US getLUS () {return lus_from_root;}

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

        Level_of_US lowerBoundLUS;
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
        
        Level_of_US get_lus_between(state_t& start, state_t& end, trajectory_t& trajectory);
        int recomputeCost (vertex_t* vertexIn);

        int markCost (vertex_t& vertexIn);
        int checkTrajectory (vertex_t& vertexIn);

        list<vertex_t*> listVertices;
        int numVertices;
  
        Auto_AB *abar;
        system_t *system;

        Planner ();
        ~Planner ();

        int setGamma (double gammaIn);
        int setGoalSampleFrequency (double sampleFreqIn);

        int setSystem (system_t& system, Auto_AB& abar_in);
        vertex_t& getRootVertex ();
        int initialize ();


        int iteration ();

        int isSafeTrajectory(list<double*> &trajectory);
        int checkTree();

        int updateReachability (); 
        int switchRoot (double distance, list<double*> &traj, list<float>& control);

        Level_of_US getBestVertexLUS () {return lowerBoundLUS;}
        vertex_t& getBestVertex () {return *lowerBoundVertex;}
        int getBestTrajectory (list<double*>& trajectory, list<float> &control);

        double getTrajectoryLength(list<double*> &trajectory);
    };

};

#endif
