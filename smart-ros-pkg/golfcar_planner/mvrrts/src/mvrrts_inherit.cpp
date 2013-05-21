#include "mvrrts.h"
using namespace std;
using namespace MVRRTstar;

template< class T>
MVVertex<T>::MVVertex(const vertex_t& vertexIn)
{
  // call parent class's constructor
  Vertex<T>::Vertex(vertexIn);
  
  if(vertexIn.lus_from_parent)
    lus_from_parent = new Level_of_US( *(vertexIn.lus_from_parent));
  else
    lus_from_parent = NULL;
  if(vertexIn.lus_from_root)
    lus_from_root = new Level_of_US( *(vertexIn.lus_from_root));
  else
    lus_from_root = NULL;
}

template< class T > 
bool compare_VW_Pairs( 
        pair< MVVertex<T>* , Level_of_US* > vwP_i, 
        pair< MVVertex<T>* , Level_of_US* > vwP_j) 
{
    // Return true if vertex-weight pair 'i' is 
    // less than vertex-weight pair 'j'
    return ( *(vwP_i.second) < *(vwP_j.second) );
}

template< class T >
int MVPlanner< T >::setSystem (system_t& systemIn, Auto_AB& auto_a_bar)
{
  Planner<T>::setSystem(systemIn);
  abar = &auto_a_bar;
  return 0;
}

/*
template< class T > 
int MVPlanner<T> :: findBestParent(state_t& stateNew, 
        vector<vertex_t*>& vectorNearVertices, 
        vertex_t* &vertexBest, 
        trajectory_t& trajectoryOut, 
        bool& exactConnection, 
        list<float>& controlOut) 
{

    Subset_of_Sigma lab_stateNew  = system->label_state(stateNew.x);

    unsigned int numNearVertices = vectorNearVertices.size();
    vector <pair<vertex_t*,Level_of_US*> > vertexWeightPairs(numNearVertices);
    
    unsigned int vec_elem_ctr = 0;

    for ( typename vector<vertex_t*>::iterator iter = 
            vectorNearVertices.begin(); 
            iter != vectorNearVertices.end(); iter++) 
    {
        state_t* stateNear = (*iter)->state;
        Subset_of_Sigma lab_stateNear 
            = system -> label_state( stateNear->x );

        Level_of_US* candidate_levelNew = 
            new Level_of_US(*((*iter)->lus_from_root));

        exactConnection = false;
        double metric_cost = system->evaluateExtensionCost( 
                *stateNear, stateNew, exactConnection);

        if ( (metric_cost > 0.0 ) && (metric_cost < DBL_MAX/2.0)) 
        {
            // add level of unsafety of transition from the near 
            // state to the new one to the candidate level of 
            // the new state from the root state
            (*candidate_levelNew) += abar->weight
              (transition_label( lab_stateNear, lab_stateNew, metric_cost) );
        }

        vertexWeightPairs[vec_elem_ctr].first = *iter;
        vertexWeightPairs[vec_elem_ctr].second = candidate_levelNew;
        
        vec_elem_ctr++;
    }

    sort(vertexWeightPairs.begin(), 
            vertexWeightPairs.end(), compare_VW_Pairs<T>);

    bool connectionEstablished = false;

    for ( typename vector <pair<vertex_t*,Level_of_US*> > :: 
            iterator iter = vertexWeightPairs.begin(); 
            iter != vertexWeightPairs.end(); iter++) 
    {
        vertex_t * vertexNear = (iter->first);
        exactConnection = false;
        // attempt to connect with obstacle checking & trace
        // inclusivity this time
        if ( system->extendTo(*(vertexNear->state),stateNew, 
                    trajectoryOut, exactConnection, controlOut) > 0 ) 
        {
            vertexBest = vertexNear;
            connectionEstablished = true; 
            break;
        }
    }
    
    // de-allocate
    for ( typename vector < pair < vertex_t* , Level_of_US* > > :: 
            iterator iter = vertexWeightPairs.begin(); 
            iter != vertexWeightPairs.end(); iter++) 
    {
        delete ( iter->second );
    }

    if (connectionEstablished) 
      return 1;
    else 
      return 0;
}
*/

template class MVRRTstar::MVVertex<MVDubinsCar>;
template class MVRRTstar::MVPlanner<MVDubinsCar>;
