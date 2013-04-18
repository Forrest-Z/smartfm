#ifndef __MVRRTS_H_
#define __MVRRTS_H_

#include <rrts.h>

#include "mvdubins.h"
#include "automata.h"
using namespace RRTstar;

namespace MVRRTstar
{
  template < class T >
    class MVVertex : Vertex<T>
  {
    public:
      // overload typedef for convenience
      typedef MVVertex < T > vertex_t;

      Level_of_US* lus_from_parent;
      Level_of_US* lus_from_root;

      MVVertex() : Vertex<T>()
      {
        lus_from_root = new Level_of_US();
        lus_from_parent = new Level_of_US();
      }
      MVVertex(const vertex_t&);
      ~MVVertex()
      {
        if(lus_from_parent)
          delete lus_from_parent;
        if(lus_from_root)
          delete lus_from_root;
      }
  };

  template < class T >
  class MVPlanner : Planner<T>
  {
    public:
      typedef typename Planner<T>::system_t system_t;
      typedef typename Planner<T>::state_t state_t;
      typedef typename Planner<T>::trajectory_t trajectory_t;
      typedef MVVertex<T> vertex_t;

      Auto_AB* abar;
      Level_of_US lowerBoundLUS;
      
      MVPlanner() : Planner<T>()
      {
        abar = NULL;
        lowerBoundLUS.metric_cost = DBL_MAX;
      }
      int setSystem( system_t& system, Auto_AB& auto_a_bar);
      /*
      int findBestParent( state_t& stateNew, 
        vector<vertex_t*>& vectorNearVertices, 
        vertex_t* &vertexBest, 
        trajectory_t& trajectoryOut, 
        bool& exactConnection, 
        list<float>& controlOut);
      */

      /*
      vertex_t* generateVertex( vertex_t& vertexFrom, 
          trajectory_t& trajectoryIn);
      int updateBranchLUS(vertex_t& vertexIn, int depth);    
      */
      Level_of_US getBestVertexLUS() 
      { 
        return lowerBoundLUS; 
      }
  };
};

#endif
