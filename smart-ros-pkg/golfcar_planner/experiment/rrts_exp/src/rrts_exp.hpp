#ifndef __RRTS_EXP_H_
#define __RRTS_EXP_H_

#include "kdtree.h"
#include <list>
#include <set>
#include <vector>
#include <iostream>
#include <cfloat>
#include <cmath>
#include <algorithm>
#include "dubins_car_exp.h"

using namespace std;

namespace RRTstarExp {

  template < class typeparams > class RRTSPlanner;

  template < class typeparams >
    class VertexExp {

      typedef typename typeparams::StateTypeExp state_t;
      typedef typename typeparams::TrajectoryTypeExp trajectory_t;
      typedef typename typeparams::SystemTypeExp system_t;

      typedef VertexExp<typeparams> vertex_t;

      vertex_t *parent;
      state_t *state;
      set<vertex_t*> children;
      double costFromParent;
      double costFromRoot;
      trajectory_t *trajFromParent;

      public:

      VertexExp ();
      ~VertexExp ();
      VertexExp (const VertexExp &vertexIn);

      // int setState (State& stateIn);
      state_t& getState () {return *state;}
      state_t& getState () const {return *state;}
      vertex_t& getParent () {return *parent;}
      double getCost () {return costFromRoot;}

      friend class RRTSPlanner<typeparams>;
    };

  template < class typeparams >
  class RRTSPlanner {

	  typedef typename typeparams::StateTypeExp state_t;
	  typedef typename typeparams::TrajectoryTypeExp trajectory_t;
      typedef typename typeparams::SystemTypeExp system_t;

      typedef VertexExp<typeparams> vertex_t;
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

      public:

      list<vertex_t*> listVertices;
      int numVertices;

      system_t *system;

      RRTSPlanner ();
      ~RRTSPlanner ();

      int setGamma (double gammaIn);
      int setGoalSampleFrequency (double sampleFreqIn);

      int setSystem (system_t& system);
      vertex_t& getRootVertex ();
      int initialize ();

      int iteration (vector<double>& samples);

      int isSafeTrajectory(list<double*> &trajectory);
      int checkTree();

      int updateReachability (); 
      int switchRoot (double distance, list<double*> &traj, list<float>& control);

      double getBestVertexCost () {return lowerBoundCost;}
      vertex_t& getBestVertex () {return *lowerBoundVertex;}
      int getBestTrajectory (list<double*>& trajectory, list<float> &control);

      double getTrajectoryLength(list<double*> &trajectory);
    };


  //Implementation of RRTSPlanner
  template< class typeparams >
  RRTstarExp::VertexExp< typeparams >
  ::VertexExp () {
	  state = NULL;
	  parent = NULL;
	  trajFromParent = NULL;

	  costFromParent = 0.0;
	  costFromRoot = 0.0;
  }

  template< class typeparams >
  RRTstarExp::VertexExp< typeparams >
  ::~VertexExp () {

    if (state)
      delete state;

    parent = NULL;

    if (trajFromParent)
      delete trajFromParent;

    children.clear();
  }


  template< class typeparams >
  RRTstarExp::VertexExp< typeparams >
  ::VertexExp(const vertex_t& vertexIn) {

    if (vertexIn.state)
      state = new state_t (vertexIn.getState());
    else
      state = NULL;
    parent = vertexIn.parent;
    for (typename set<vertex_t*>::const_iterator iter = vertexIn.children.begin(); iter != vertexIn.children.end(); iter++)
      children.insert (*iter);
    costFromParent = vertexIn.costFromParent;
    costFromRoot = vertexIn.costFromRoot;
    if (vertexIn.trajFromParent)
      trajFromParent = new trajectory_t (*(vertexIn.trajFromParent));
    else
      trajFromParent = NULL;
  }

  // int VertexExp::setState (const State &stateIn) {
  //   *state = stateIn;
  //   return 1;
  // }


  template< class typeparams >
  RRTstarExp::RRTSPlanner< typeparams >
  ::RRTSPlanner () {

    gamma = 1.0;
    goalSampleFreq = 0.0;

    lowerBoundCost = DBL_MAX;
    lowerBoundVertex = NULL;

    kdtree = NULL;

    root = NULL;

    numVertices = 0;

    system = NULL;
  }


  template< class typeparams >
  RRTstarExp::RRTSPlanner< typeparams >
  ::~RRTSPlanner () {

    // Delete the kdtree structure
    if (kdtree) {
      kd_free (kdtree);
    }
  }


  template< class typeparams >
  int
  RRTstarExp::RRTSPlanner< typeparams >
  ::insertIntoKdtree (vertex_t& vertexIn) {

    //state_t &s = vertexIn.getState();
    //cout<<"insert_kdtree: "<<s[0]<<" "<<s[1]<<" "<<s[2]<<endl;
    double *stateKey = new double[numDimensions];
    system->getStateKey ( *(vertexIn.state), stateKey);
    kd_insert (kdtree, stateKey, &vertexIn);
    delete [] stateKey;

    return 1;
  }


  template< class typeparams >
  int
  RRTstarExp::RRTSPlanner< typeparams >
  ::getNearestVertex (state_t& stateIn, vertex_t*& vertexPointerOut) {

    // Get the state key for the query state
    double *stateKey = new double[numDimensions];
    system->getStateKey (stateIn, stateKey);

    // Search the kdtree for the nearest vertex
    kdres_t *kdres = kd_nearest (kdtree, stateKey);
    if (kd_res_end (kdres))
      vertexPointerOut = NULL;
    vertexPointerOut = (vertex_t*) kd_res_item_data (kdres);

    // Clear up the memory
    delete [] stateKey;
    kd_res_free (kdres);

    // Return a non-positive number if any errors
    if (vertexPointerOut == NULL)
      return 0;

    return 1;
  }


  template< class typeparams >
  int
  RRTstarExp::RRTSPlanner< typeparams >
  ::getNearVertices (state_t& stateIn, vector<vertex_t*>& vectorNearVerticesOut) {

    // Get the state key for the query state
    double *stateKey = new double[numDimensions];
    system->getStateKey (stateIn, stateKey);

    // Compute the ball radius
    double ballRadius = gamma * pow( log((double)(numVertices + 1.0))/((double)(numVertices + 1.0)), 1.0/((double)numDimensions) );

    // Search kdtree for the set of near vertices
    kdres_t *kdres = kd_nearest_range (kdtree, stateKey, ballRadius);
    delete [] stateKey;

    // Create the vector data structure for storing the results
    int numNearVertices = kd_res_size (kdres);
    if (numNearVertices == 0) {
      vectorNearVerticesOut.clear();
      return 1;
    }
    vectorNearVerticesOut.resize(numNearVertices);

    // Place pointers to the near vertices into the vector
    int i = 0;
    kd_res_rewind (kdres);
    while (!kd_res_end(kdres)) {
      vertex_t *vertexCurr = (vertex_t *) kd_res_item_data (kdres);
      vectorNearVerticesOut[i] = vertexCurr;
      kd_res_next (kdres);
      i++;
    }

    // Free temporary memory
    kd_res_free (kdres);

    return 1;
  }

  template< class typeparams >
  int
    RRTstarExp::RRTSPlanner< typeparams >
  ::updateReachability ()
  {
    updateBranchCost(*root, 1);
    lowerBoundCost = getBestVertexCost();

    vertex_t &bestVertex = getBestVertex();
    lowerBoundVertex = &bestVertex;

    return 1;
  }


  template< class typeparams >
  int
  RRTstarExp::RRTSPlanner< typeparams >
  ::checkUpdateBestVertex (vertex_t& vertexIn) {

    if (system->isReachingTarget(vertexIn.getState()))
    {
      state_t &s = vertexIn.getState();
      double costCurr = vertexIn.getCost() + system->getGoalCost(s.x);
      if ( (lowerBoundVertex == NULL) || ( (lowerBoundVertex != NULL) && (costCurr < lowerBoundCost)) ) {

        lowerBoundVertex = &vertexIn;
        lowerBoundCost = costCurr;
      }
    }

    return 1;
  }

  template< class typeparams >
  typename RRTstarExp::RRTSPlanner<typeparams>::vertex_t*
  RRTstarExp::RRTSPlanner< typeparams >
  ::insertTrajectory (vertex_t& vertexStartIn, trajectory_t& trajectoryIn) {

    // Check for admissible cost-to-go
    if (lowerBoundVertex != NULL) {
      double costToGo = system->evaluateCostToGo (trajectoryIn.getEndState());
      if (costToGo >= 0.0)
        if (lowerBoundCost < vertexStartIn.getCost() + costToGo)
          return NULL;
    }

    // Create a new end vertex
    vertex_t* vertexNew = new vertex_t;
    vertexNew->state = new state_t;
    vertexNew->parent = NULL;
    trajectoryIn.getEndState(vertexNew->getState());
    insertIntoKdtree (*vertexNew);
    this->listVertices.push_front (vertexNew);
    this->numVertices++;

    // Insert the trajectory between the start and end vertices
    insertTrajectory (vertexStartIn, trajectoryIn, *vertexNew);

    return vertexNew;
  }


  template< class typeparams >
  int
  RRTstarExp::RRTSPlanner< typeparams >
  ::insertTrajectory (vertex_t& vertexStartIn, trajectory_t& trajectoryIn, vertex_t& vertexEndIn) {

    // Update the costs
    vertexEndIn.costFromParent = trajectoryIn.evaluateCost();
    vertexEndIn.costFromRoot = vertexStartIn.costFromRoot + vertexEndIn.costFromParent;
    checkUpdateBestVertex (vertexEndIn);

    // Update the trajectory between the two vertices
    if (vertexEndIn.trajFromParent)
      delete vertexEndIn.trajFromParent;
    vertexEndIn.trajFromParent = new trajectory_t (trajectoryIn);

    // Update the parent to the end vertex
    if (vertexEndIn.parent)
      vertexEndIn.parent->children.erase (&vertexEndIn);
    vertexEndIn.parent = &vertexStartIn;

    // Add the end vertex to the set of chilren
    vertexStartIn.children.insert (&vertexEndIn);

    return 1;
  }


  template< class typeparams >
  int
  RRTstarExp::RRTSPlanner< typeparams >
  ::setSystem (system_t& systemIn) {

    system = &systemIn;

    numDimensions = system->getNumDimensions();

    if(listVertices.size() > 0)
    {
      // Delete all the vertices
      for (typename list<vertex_t*>::iterator iter = listVertices.begin(); iter != listVertices.end(); iter++)
        delete *iter;
    }
    listVertices.clear();

    numVertices = 0;
    lowerBoundCost = DBL_MAX;
    lowerBoundVertex = NULL;

    // Clear the kdtree
    if (kdtree) {
      kd_free (kdtree);
    }
    kdtree = kd_create (numDimensions);

    // Initialize the root vertex
    root = new vertex_t;
    root->state = new state_t (system->getRootState());
    root->costFromParent = 0.0;
    root->costFromRoot = 0.0;
    root->trajFromParent = NULL;

    return 1;
  }



  template< class typeparams >
  typename RRTstarExp::RRTSPlanner< typeparams >::vertex_t&
  RRTstarExp::RRTSPlanner< typeparams >
  ::getRootVertex () {

    return *root;
  }



  template< class typeparams >
  int
  RRTstarExp::RRTSPlanner< typeparams >
  ::initialize () {

    if (!system)
      return 0;

    vertex_t *rootBackup = NULL;
    if (root)
      rootBackup = new vertex_t (*root);

    for (typename list<vertex_t*>::iterator iter = listVertices.begin(); iter != listVertices.end(); iter++)
    {
      delete *iter;
    }

    listVertices.clear();
    numVertices = 0;
    lowerBoundCost = DBL_MAX;
    lowerBoundVertex = NULL;

    if (kdtree) {
      kd_free (kdtree);
    }
    kdtree = kd_create (system->getNumDimensions());

    // Initialize the variables
    numDimensions = system->getNumDimensions();
    root = rootBackup;
    if (root)
    {
      root->children.clear();
      root->costFromParent = 0.0;
      root->costFromRoot = 0.0;
      root->trajFromParent = NULL;

      listVertices.push_back(root);
      insertIntoKdtree (*root);
      numVertices++;
    }
    lowerBoundCost = DBL_MAX;
    lowerBoundVertex = NULL;

    //cout<<"380"<<endl;
    return 1;
  }

  template< class typeparams >
  int
  RRTstarExp::RRTSPlanner< typeparams >
  ::setGamma (double gammaIn) {

    if (gammaIn < 0.0)
      return 0;

    gamma = gammaIn;

    return 1;
  }


  template< class typeparams >
  int
  RRTstarExp::RRTSPlanner< typeparams >
  ::setGoalSampleFrequency (double sampleFreqIn) {

    if ( (sampleFreqIn < 0.0) || (sampleFreqIn > 1.0) )
      return 0;

    goalSampleFreq = sampleFreqIn;

    return 1;
  }


  template< class typeparams >
  int compareVertexCostPairs (pair<typename RRTstarExp::VertexExp<typeparams>*,double> i,
      pair<typename RRTstarExp::VertexExp<typeparams>*,double> j) {

    return (i.second < j.second);
  }


  template< class typeparams >
  int
  RRTstarExp::RRTSPlanner< typeparams >
  ::findBestParent (state_t& stateIn, vector<vertex_t*>& vectorNearVerticesIn,
      vertex_t*& vertexBest, trajectory_t& trajectoryOut, bool& exactConnection, list<float>& controlOut) {

    // Compute the cost of extension for each near vertex
    int numNearVertices = vectorNearVerticesIn.size();

    vector< pair<vertex_t*,double> > vectorVertexCostPairs(numNearVertices);

    int i = 0;
    for (typename vector<vertex_t*>::iterator iter = vectorNearVerticesIn.begin(); iter != vectorNearVerticesIn.end(); iter++)
    {

      vectorVertexCostPairs[i].first = *iter;
      exactConnection = false;
      double trajCost = system->evaluateExtensionCost ( *((*iter)->state), stateIn, exactConnection);
      vectorVertexCostPairs[i].second = (*iter)->costFromRoot + trajCost;
      i++;
    }

    // Sort vertices according to cost
    sort (vectorVertexCostPairs.begin(), vectorVertexCostPairs.end(), compareVertexCostPairs<typeparams>);

    // Try out each extension according to increasing cost
    i = 0;
    bool connectionEstablished = false;
    for (typename vector<pair<vertex_t*,double> >::iterator iter = vectorVertexCostPairs.begin();
        iter != vectorVertexCostPairs.end(); iter++) {

      vertex_t* vertexCurr = iter->first;

      // Extend the current vertex towards stateIn (and this time check for collision with obstacles)
      exactConnection = false;
      if (system->extendTo(*(vertexCurr->state), stateIn, trajectoryOut, exactConnection, controlOut, true) > 0) {
        vertexBest = vertexCurr;
        connectionEstablished = true;
        break;
      }
    }

    // Return success if a connection was established
    if (connectionEstablished)
      return 1;

    // If the connection could not be established then return zero
    return 0;
  }


  template< class typeparams >
  int
  RRTstarExp::RRTSPlanner< typeparams >
  ::updateBranchCost (vertex_t& vertexIn, int depth) {


    // Update the cost for each children
    for (typename set<vertex_t*>::iterator iter = vertexIn.children.begin(); iter != vertexIn.children.end(); iter++) {

      vertex_t& vertex = **iter;

      vertex.costFromRoot = vertexIn.costFromRoot + vertex.costFromParent;

      checkUpdateBestVertex (vertex);

      updateBranchCost (vertex, depth + 1);
    }

    return 1;
  }


  template< class typeparams >
  int
    RRTstarExp::RRTSPlanner< typeparams >
  ::markCost (vertex_t& vertexIn)
  {
    vertexIn.costFromRoot = -1.0;

    vertex_t &parent = vertexIn.getParent();
    parent.children.erase(&vertexIn);

    for (typename set<vertex_t*>::iterator iter = vertexIn.children.begin(); iter != vertexIn.children.end(); iter++)
    {
      markCost(**iter);
    }
    return 1;
  }

  template< class typeparams >
  int
    RRTstarExp::RRTSPlanner< typeparams >
  ::checkTrajectory (vertex_t& vertexIn)
  {
    vertex_t &parent = vertexIn.getParent();
    state_t  &state_par = parent.getState();
    state_t &state_in = vertexIn.getState();

    trajectory_t traj;
    list<float> tmp_control;
    bool exactConnection;
    if(system->extendTo(state_par, state_in, traj, exactConnection, tmp_control, true))
    {
      for (typename set<vertex_t*>::iterator iter = vertexIn.children.begin(); iter != vertexIn.children.end(); iter++)
      {
        checkTrajectory(**iter);
      }
    }
    else
    {
      //cout<<"marking: "<<state_in[0]<<" "<<state_in[1]<<" "<<state_in[2]<<endl;
      markCost(vertexIn);
    }
    return 1;
  }



  /*TODO:Called by RRTSPlanner::get_plan(), not sure the function
   *
   *
   */
  template< class typeparams >
  int
    RRTstarExp::RRTSPlanner< typeparams >
  ::checkTree()
  {
    if( root->children.size() > 0)
    {
      for (typename set<vertex_t*>::iterator iter = root->children.begin(); iter != root->children.end(); iter++)
      {
        vertex_t &vertex = **iter;
        checkTrajectory(vertex);
      }
      /*
         cout<<"listVertices 553: "<<listVertices.size()<<endl;
         for(typename list<vertex_t*>::iterator lvi=listVertices.begin(); lvi!=listVertices.end(); lvi++)
         {
         vertex_t *v = (*lvi);
         state_t &s = (*lvi)->getState();
         cout<<s[0]<<" "<<s[1]<<" "<<s[2]<<" : "<<v->costFromRoot<<endl;
         }
         */
      list<vertex_t*> listSurvivingVertices;
      for (typename list<vertex_t*>::iterator iter = listVertices.begin(); iter != listVertices.end(); iter++)
      {
        vertex_t *vertex  = *iter;
        if( vertex->costFromRoot > -0.5)
          listSurvivingVertices.push_front(vertex);
        else
        {
          //cout<<"deleting vertex"<<endl;
          delete vertex;
        }
      }

      if (kdtree) {
        kd_free (kdtree);
      }
      kdtree = kd_create (system->getNumDimensions());

      listVertices.clear();
      numVertices = 0;
      for (typename list<vertex_t*>::iterator iter = listSurvivingVertices.begin(); iter != listSurvivingVertices.end(); iter++)
      {
        listVertices.push_front(*iter);
        numVertices++;

        insertIntoKdtree (**iter);
      }
      /*
         cout<<"listVertices 587: "<<listVertices.size()<<endl;
         for(typename list<vertex_t*>::iterator lvi=listVertices.begin(); lvi!=listVertices.end(); lvi++)
         {
         vertex_t *v = (*lvi);
         state_t &s = (*lvi)->getState();
         cout<<s[0]<<" "<<s[1]<<" "<<s[2]<<" : "<<v->costFromRoot<<endl;
         }
         */
      lowerBoundVertex = NULL;
      lowerBoundCost = DBL_MAX;

      listSurvivingVertices.clear();
      recomputeCost (root);
    }
    return 1;
  }


  template< class typeparams >
  int
  RRTstarExp::RRTSPlanner< typeparams >
  ::rewireVertices (vertex_t& vertexNew, vector<vertex_t*>& vectorNearVertices) {


    // Repeat for all vertices in the set of near vertices
    for (typename vector<vertex_t*>::iterator iter = vectorNearVertices.begin(); iter != vectorNearVertices.end(); iter++) {

      vertex_t& vertexCurr = **iter;

      // Check whether the extension results in an exact connection
      bool exactConnection = false;
      double costCurr = system->evaluateExtensionCost (*(vertexNew.state), *(vertexCurr.state), exactConnection);
      if ( (exactConnection == false) || (costCurr < 0) )
        continue;

      // Check whether the cost of the extension is smaller than current cost
      double totalCost = vertexNew.costFromRoot + costCurr;
      if (totalCost < vertexCurr.costFromRoot - 0.001) {

        // Compute the extension (checking for collision)
        trajectory_t trajectory;
        list<float> tmp_control;
        if (system->extendTo (*(vertexNew.state), *(vertexCurr.state), trajectory, exactConnection, tmp_control, true) <= 0 )
          continue;

        // Insert the new trajectory to the tree by rewiring
        insertTrajectory (vertexNew, trajectory, vertexCurr);

        // Update the cost of all vertices in the rewired branch
        updateBranchCost (vertexCurr, 0);
      }
    }

    return 1;
  }


  template< class typeparams >
  int
  RRTstarExp::RRTSPlanner< typeparams >
  ::iteration (vector<double>& samples) {

    // 1. Sample a new state
    state_t stateRandom;
    double randGoalSampling = ((double)rand())/(RAND_MAX + 1.0);

    if (randGoalSampling < goalSampleFreq ) {
      if (system->sampleGoalState (stateRandom) <= 0)
        return 0;
    }
    else {
      if (system->sampleState (stateRandom) <= 0)
        return 0;
    }

    samples.push_back(stateRandom.x[0]);
    samples.push_back(stateRandom.x[1]);
    samples.push_back(stateRandom.x[2]);

    //cout<<"got a free vertex"<<endl;
  	//cout << "sample state"<< endl;
    // 2. Compute the set of all near vertices
    vector<vertex_t*> vectorNearVertices;
    getNearVertices (stateRandom, vectorNearVertices);

    // 3. Find the best parent and extend from that parent
    vertex_t* vertexParent = NULL;
    trajectory_t trajectory;
    list<float> control;
    bool exactConnection = false;

    if (vectorNearVertices.size() == 0) {

      // 3.a Extend the nearest
      if (getNearestVertex (stateRandom, vertexParent) <= 0)
        return 0;
      if (system->extendTo(vertexParent->getState(), stateRandom, trajectory, exactConnection, control, true) <= 0)
        return 0;
    }
    else {

      // 3.b Extend the best parent within the near vertices
      if (findBestParent (stateRandom, vectorNearVertices, vertexParent, trajectory, exactConnection, control) <= 0)
        return 0;
    }

    // 3.c add the trajectory from the best parent to the tree
    vertex_t* vertexNew = insertTrajectory (*vertexParent, trajectory);
    if (vertexNew == NULL)
      return 0;

    // 4. Rewire the tree
    if (vectorNearVertices.size() > 0)
      rewireVertices (*vertexNew, vectorNearVertices);

    return 1;
  }


  template< class typeparams >
  int
    RRTstarExp::RRTSPlanner< typeparams >
  ::findDescendantVertices (vertex_t* vertexIn)
  {
    vertexIn->costFromRoot = (-1.0) * vertexIn->costFromRoot - 1.0;  // Mark the node so that we can understand
    // later that it is in the list of descendants

    for (typename set<vertex_t*>::iterator iter = vertexIn->children.begin(); iter != vertexIn->children.end(); iter++) {

      vertex_t* vertexCurr = *iter;

      findDescendantVertices (vertexCurr);
    }

    return 1;
  }


  template< class typeparams >
  int
  RRTstarExp::RRTSPlanner< typeparams >
  ::recomputeCost (vertex_t* vertexIn) {

    for (typename set<vertex_t*>::iterator iter = vertexIn->children.begin(); iter != vertexIn->children.end(); iter++)
    {
      vertex_t* vertexCurr = *iter;

      double costCurr = vertexIn->costFromRoot + vertexCurr->costFromParent;

      vertexCurr->costFromRoot = costCurr;

      if (system->isReachingTarget(vertexIn->getState())) {

        if ( (lowerBoundVertex == NULL) || ( (lowerBoundVertex != NULL) && (costCurr < lowerBoundCost)) ) {

          lowerBoundVertex = vertexCurr;
          lowerBoundCost = costCurr;
        }
      }

      recomputeCost (vertexCurr);
    }

    return 1;
  }

  // note trajret, controlret is always only appended
  template< class typeparams >
  int
    RRTstarExp::RRTSPlanner< typeparams >
  ::switchRoot (double distanceIn, list<double*> &trajret, list<float> &controlret)
  {
    //cout<<"calling switch_root"<<endl;

    // If there is no path reaching the goal, then return failure
    if (lowerBoundVertex == NULL)
      return 0;


    // 1. Find the new root position
    list<vertex_t*> listBestVertex;
    vertex_t* vertexCurrBest = lowerBoundVertex;
    while (vertexCurrBest) {
      listBestVertex.push_front (vertexCurrBest);
      vertexCurrBest = &(vertexCurrBest->getParent());
    }

    double *stateRootNew = new double[numDimensions];
    vertex_t* vertexChildNew = NULL;

    bool stateFound = false;

    state_t& rootState = root->getState();

    double *stateArrPrev = new double [numDimensions];
    for (int i = 0; i < numDimensions; i++)
      stateArrPrev[i] = rootState[i];
    //cout<<"stateArrPrev: "<< stateArrPrev[0]<<" "<<stateArrPrev[1]<<" "<<stateArrPrev[2]<<endl;

    double distTotal = 0.0;
    for (typename list<vertex_t*>::iterator iter = listBestVertex.begin(); iter != listBestVertex.end(); iter++)
    {

      vertex_t* vertexCurr = *iter;

      vertex_t& vertexParent = vertexCurr->getParent();
      if (&vertexParent == NULL)
        continue;

      state_t& stateCurr = vertexCurr->getState();

      state_t& stateParent = vertexParent.getState();
      list<double*> trajectory;
      list<float> control;
      system->getTrajectory (stateParent, stateCurr, trajectory, control, true);

      /*
         cout<<"switchroot gettraj size: "<< trajectory.size()<<endl;
         for (list<double*>::iterator iter = trajectory.begin(); iter != trajectory.end(); iter++)
         {
         double *s = *iter;
         cout<<s[0]<<" "<<s[1]<<" "<<s[2]<<endl;
         }
         */

      list<float>::iterator iterControl = control.begin();
      for (list<double*>::iterator iter = trajectory.begin(); iter != trajectory.end(); iter++)
      {
        double *stateArrCurr = *iter;

        double distCurr = sqrt( (stateArrCurr[0]-stateArrPrev[0])*(stateArrCurr[0]-stateArrPrev[0])
            + (stateArrCurr[1]-stateArrPrev[1])*(stateArrCurr[1]-stateArrPrev[1]) );

        distTotal += distCurr;
        //cout<<"distTotal: "<<distTotal<<endl;

        // write code for copying trajectory, control into traj here
        double *stateTmp = new double[3];
        stateTmp[0] = stateArrCurr[0];  stateTmp[1] = stateArrCurr[1]; stateTmp[2] = stateArrCurr[2];
        float controlTmp = *iterControl;
        trajret.push_back(stateTmp);
        controlret.push_back(controlTmp);

        // copy it every time
        for (int i = 0; i < numDimensions; i++)
          stateRootNew[i] = stateArrCurr[i];
        vertexChildNew = vertexCurr;

        if (distTotal >= distanceIn)
        {
          stateFound = true;
          break;
        }

        for (int i = 0; i < numDimensions; i++)
          stateArrPrev[i] = stateArrCurr[i];

        iterControl++;
      }

      // Free the temporary memory occupied by the states
      for (list<double*>::iterator iter = trajectory.begin(); iter != trajectory.end(); iter++) {
        double *stateArrCurr = *iter;
        delete [] stateArrCurr;
      }
      if (stateFound)
      {
        break;
      }

      // what is the this line for??
      vertexCurr = &vertexParent;
    }
    //cout<<"trajret size: "<< trajret.size()<<endl;
    delete [] stateArrPrev;

    state_t &vertexChildNewState = vertexChildNew->getState();
    if( (stateRootNew[0] == vertexChildNewState[0]) && (stateRootNew[1] == vertexChildNewState[1]) &&\
        (stateRootNew[2] == vertexChildNewState[2]) )
    {
      //cout<<"switch_root length > length of best_trajectory"<<endl;
      state_t& stateRoot = root->getState();
      for (int i = 0; i < numDimensions; i++)
        stateRoot[i] = stateRootNew[i];
      /*
         cout<<"listVertices before initialize: "<<listVertices.size()<<endl;
         for(typename list<vertex_t*>::iterator lvi=listVertices.begin(); lvi!=listVertices.end(); lvi++)
         {
         state_t &s = (*lvi)->getState();
         cout<<s[0]<<" "<<s[1]<<" "<<s[2]<<endl;
         }
         */
      initialize();
      return 1;
    }
    else
    {
      // goal is farther than committed trajectory

      // 2. Find and store all the decendandts of the new root
      findDescendantVertices (vertexChildNew);

      // 3. Create the new root vertex
      vertex_t* vertexRoot = new vertex_t;
      vertexRoot->state = new state_t;
      state_t& stateRoot = vertexRoot->getState();
      for (int i = 0; i < numDimensions; i++)
        stateRoot[i] = stateRootNew[i];
      vertexRoot->children.insert (vertexChildNew);

      root = vertexRoot;

      // 4. Connect the new root vertex to the new child
      trajectory_t connectingTrajectory;
      list <float> connectingControl;
      bool exactConnection;

      // if connection possible, switch to new root, else call vertex_new as the new root
      if (system->extendTo(vertexRoot->getState(), vertexChildNew->getState(), connectingTrajectory, exactConnection, connectingControl, true) <= 0)
      {
        cout << "switchRoot ERR: No extend, reinitializing" << endl;

        state_t& stateRoot = root->getState();
        for (int i = 0; i < numDimensions; i++)
          stateRoot[i] = stateRootNew[i];
        initialize();
        return 1;
      }

      if (vertexChildNew->trajFromParent)
        delete vertexChildNew->trajFromParent;

      vertexChildNew->trajFromParent = new trajectory_t(connectingTrajectory);
      vertexChildNew->costFromParent = connectingTrajectory.evaluateCost();
      vertexChildNew->parent = vertexRoot;
      vertexChildNew->costFromRoot = (-1.0)*vertexChildNew->costFromParent - 1.0;
      //cout<<"vertexchild_new: "<< vertexChildNew->costFromRoot<< " "<< vertexChildNew->costFromParent<<endl;

      // 5. Clear the kdtree
      if (kdtree) {
        kd_free (kdtree);
      }
      kdtree = kd_create (system->getNumDimensions());

      // 6. Delete all the unmarked vertices and spare the marked vertices
      list<vertex_t*> listSurvivingVertices;
      for (typename list<vertex_t*>::iterator iter = listVertices.begin(); iter != listVertices.end(); iter++) {

        vertex_t* vertexCurrDel = *iter;

        // If the vertex was marked
        if (vertexCurrDel->costFromRoot < -0.5) {

          // Revert the mark
          vertexCurrDel->costFromRoot = (-1.0)*(vertexCurrDel->costFromRoot + 1.0);

          // Add the vertex to the list of surviving vetices
          listSurvivingVertices.push_front (vertexCurrDel);
        }
        else {

          delete vertexCurrDel;
        }
      }

      listVertices.clear();
      numVertices = 0;

      lowerBoundVertex = NULL;
      lowerBoundCost = DBL_MAX;


      // 7. Repopulate the list of all vertices and recreate the kdtree
      listVertices.push_front (vertexRoot);
      insertIntoKdtree (*vertexRoot);
      numVertices++;

      for (typename list<vertex_t*>::iterator iter = listSurvivingVertices.begin(); iter != listSurvivingVertices.end(); iter++) {

        vertex_t* vertexCurrSur = *iter;

        insertIntoKdtree (*vertexCurrSur);

        listVertices.push_front (vertexCurrSur);
        numVertices++;
      }

      listSurvivingVertices.clear();

      recomputeCost (vertexRoot);
      cout<<"after switchRoot vertices left: "<< numVertices << endl;

      // 8. Clear temporary memory
      delete [] stateRootNew;

      return 1;
    }
  }




  template< class typeparams >
  int
  RRTstarExp::RRTSPlanner< typeparams >
  ::getBestTrajectory (list<double*>& trajectoryOut, list<float> &controlOut ) {

    if (lowerBoundVertex == NULL)
    {
      return 0;
    }
    vertex_t* vertexCurr = lowerBoundVertex;

    while (vertexCurr) {

      state_t& stateCurr = vertexCurr->getState();

      double *stateArrCurr = new double[3];
      stateArrCurr[0] = stateCurr[0];
      stateArrCurr[1] = stateCurr[1];
      stateArrCurr[2] = stateCurr[2];

      trajectoryOut.push_front (stateArrCurr);

      vertex_t& vertexParent = vertexCurr->getParent();

      if (&vertexParent != NULL) {

        state_t& stateParent = vertexParent.getState();

        list<double*> trajectory;
        list<float> control;
        system->getTrajectory (stateParent, stateCurr, trajectory, control, false);

        trajectory.reverse ();
        control.reverse();
        for (list<double*>::iterator iter = trajectory.begin(); iter != trajectory.end(); iter++)
        {
          double *stateArrFromParentCurr = *iter;

          stateArrCurr = new double[3];
          stateArrCurr[0] = stateArrFromParentCurr[0];
          stateArrCurr[1] = stateArrFromParentCurr[1];
          stateArrCurr[2] = stateArrFromParentCurr[2];

          trajectoryOut.push_front (stateArrCurr);

          delete [] stateArrFromParentCurr;
        }
        for (list<float>::iterator iter = control.begin(); iter != control.end(); iter++)
        {
          controlOut.push_back(*iter);
        }
      }

      vertexCurr = &vertexParent;

      delete [] stateArrCurr;
    }

    return 1;
  }

  template< class typeparams >
  int
    RRTstarExp::RRTSPlanner< typeparams >
  ::isSafeTrajectory(list<double*>& trajectory)
  {

    //TODO: Modify it into efficient collision checker.
    int max_obs_check = 6;
    int obs_counter = 0;
    for (list<double*>::iterator iter = trajectory.begin(); iter != trajectory.end(); iter++)
    {
      obs_counter++;
      if(obs_counter == max_obs_check)
      {
        obs_counter = 0;
        double *stateRef = *iter;
        if( system->IsInCollision(stateRef) )
          return false;
      }
    }
    return true;
  }


  template< class typeparams >
  double
    RRTstarExp::RRTSPlanner< typeparams >
  ::getTrajectoryLength(list<double*>& trajectory)
  {
    double len = 0;
    for (list<double*>::iterator iter = trajectory.begin(); iter != trajectory.end(); iter++)
    {
      list<double*>::iterator next = iter;
      if(next != trajectory.end())
      {
        next++;
        double *currState = *iter;
        double *nextState = *next;
        //cout<<"list states: "<< currState[0]<<" "<<currState[1]<<" "<< nextState[0]<<" "<< nextState[1]<<endl;
        len += sqrt( (currState[0] - nextState[0])*(currState[0] - nextState[0]) + (currState[1] - nextState[1])*(currState[1] - nextState[1]) );
      }
      else
        break;
    }
    return len;
  }

}

#endif
