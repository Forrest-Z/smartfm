#ifndef __RRTS_HPP_
#define __RRTS_HPP_


#include "rrts.h"

#include <iostream>

#include <cfloat>
#include <cmath>

#include <algorithm>

using namespace std;

template< class T >
RRTstar::Vertex< T >
::Vertex () {

  state = NULL;
  parent = NULL;
  trajFromParent = NULL;

  costFromParent = 0.0;
  costFromRoot = 0.0;
}


template< class T >
RRTstar::Vertex< T >
::~Vertex () {

  if (state)
    delete state;

  parent = NULL;

  if (trajFromParent)
    delete trajFromParent;

  children.clear();
}


template< class T >
RRTstar::Vertex< T >
::Vertex(const vertex_t& vertexIn) {

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


// int Vertex::setState (const State &stateIn) {
//   *state = stateIn;
//   return 1;
// }


template< class T >
RRTstar::Planner< T >
::Planner () {

  gamma = 1.0;
  goalSampleFreq = 0.0;

  lowerBoundCost = DBL_MAX;
  lowerBoundVertex = NULL;

  kdtree = NULL; 

  root = NULL;

  numVertices = 0;

  system = NULL;
}


template< class T >
RRTstar::Planner< T >
::~Planner () {

  // Delete the kdtree structure
  if (kdtree) {
    kd_free (kdtree);
  }
}


template< class T >
int 
RRTstar::Planner< T >
::insertIntoKdtree (vertex_t& vertexIn) {

  //state_t &s = vertexIn.getState();
  //cout<<"insert_kdtree: "<<s[0]<<" "<<s[1]<<" "<<s[2]<<endl;
  double *stateKey = new double[numDimensions];
  system->getStateKey ( *(vertexIn.state), stateKey);
  kd_insert (kdtree, stateKey, &vertexIn);
  delete [] stateKey;

  return 1;
}


template< class T >
int 
RRTstar::Planner< T >
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


template< class T >
int 
RRTstar::Planner< T >
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

template< class T >
int
  RRTstar::Planner< T >
::updateReachability () 
{
  updateBranchCost(*root, 1);
  lowerBoundCost = getBestVertexCost();

  vertex_t &bestVertex = getBestVertex();
  lowerBoundVertex = &bestVertex;

  return 1;
}


template< class T >
int
RRTstar::Planner< T >
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

template< class T >
typename RRTstar::Planner<T>::vertex_t*
RRTstar::Planner< T >
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


template< class T >
int 
RRTstar::Planner< T >
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


template< class T >
int 
RRTstar::Planner< T >
::setSystem (system_t& systemIn) {

  system = &systemIn;

  numDimensions = system->getNumDimensions ();

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



template< class T >
typename RRTstar::Planner< T >::vertex_t& 
RRTstar::Planner< T >
::getRootVertex () {

  return *root;
}



template< class T >
int 
RRTstar::Planner< T >
::initialize () {

  // If there is no system, then return failure
  if (!system)
    return 0;

  // Backup the root
  vertex_t *rootBackup = NULL;
  if (root)
    rootBackup = new vertex_t (*root);

  //cout<<"341: "<< listVertices.size()<<endl;
  //cout<<"deleting list of vertices"<<endl;
  // Delete all the vertices
  for (typename list<vertex_t*>::iterator iter = listVertices.begin(); iter != listVertices.end(); iter++)
  {
    //vertex_t *v = *iter;
    //state_t &s = v->getState();
    //cout<<"v.state: "<< s[0]<<" "<<s[1]<<" "<<s[2]<<endl;
    delete *iter;
  }
  //cout<<"345"<<endl;

  listVertices.clear();
  numVertices = 0;
  lowerBoundCost = DBL_MAX;
  lowerBoundVertex = NULL;

  // Clear the kdtree
  if (kdtree) {
    kd_free (kdtree);
  }
  kdtree = kd_create (system->getNumDimensions());

  //cout<<"356"<<endl;

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


template< class T >
int 
RRTstar::Planner< T >
::setGamma (double gammaIn) {

  if (gammaIn < 0.0)
    return 0;

  gamma = gammaIn;

  return 1;
}


template< class T >
int 
RRTstar::Planner< T >
::setGoalSampleFrequency (double sampleFreqIn) {

  if ( (sampleFreqIn < 0.0) || (sampleFreqIn > 1.0) )
    return 0;

  goalSampleFreq = sampleFreqIn;

  return 1;  
}


template< class T >
int compareVertexCostPairs (pair<typename RRTstar::Vertex<T>*,double> i, 
    pair<typename RRTstar::Vertex<T>*,double> j) {

  return (i.second < j.second);
}


template< class T >
int 
RRTstar::Planner< T >
::findBestParent (state_t& stateIn, vector<vertex_t*>& vectorNearVerticesIn,
    vertex_t*& vertexBest, trajectory_t& trajectoryOut, bool& exactConnection, list<float>& controlOut) {


  // Compute the cost of extension for each near vertex
  int numNearVertices = vectorNearVerticesIn.size();

  vector< pair<vertex_t*,double> > vectorVertexCostPairs;

  int i = 0;
  for (typename vector<vertex_t*>::iterator iter = vectorNearVerticesIn.begin(); iter != vectorNearVerticesIn.end(); iter++) 
  {

    exactConnection = false;
    double trajCost = system->evaluateExtensionCost ( *((*iter)->state), stateIn, exactConnection);
    vectorVertexCostPairs.push_back(make_pair(*iter, (*iter)->costFromRoot + trajCost));
    i++;
  }

  // Sort vertices according to cost
  sort (vectorVertexCostPairs.begin(), vectorVertexCostPairs.end(), compareVertexCostPairs<T>);

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


template< class T >
int 
RRTstar::Planner< T >
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


template< class T >
int 
  RRTstar::Planner< T >
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

template< class T >
int 
  RRTstar::Planner< T >
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

template< class T >
int 
  RRTstar::Planner< T >
::lazyCheckTree()
{
  list<double*> trajectory;
  list<float> control_trajectory;
  getBestTrajectory(trajectory, control_trajectory);
  
  if(!isSafeTrajectory(trajectory))
    checkTree();
  
  for(list<double*>::iterator i=trajectory.begin(); i!= trajectory.end(); i++)
    delete *i;
  
  return 0;
}

template< class T >
int 
  RRTstar::Planner< T >
::checkTree()
{
  if( root->children.size() > 0)
  {
    //cout<<"547"<<endl;
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


template< class T >
int 
RRTstar::Planner< T >
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


template< class T >
int 
RRTstar::Planner< T >
::iteration () {

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
  //cout<<"got a free vertex"<<endl;

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


template< class T >
int 
  RRTstar::Planner< T >
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


template< class T >
int 
RRTstar::Planner< T >
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
template< class T >
int 
  RRTstar::Planner< T >
::switchRoot (double distanceIn, list<double*> &trajret, list<float> &controlret, list<int>& directionret) 
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
    list<int> direction;
    system->getTrajectory (stateParent, stateCurr, trajectory, control, &direction, true);

    /*
       cout<<"switchroot gettraj size: "<< trajectory.size()<<endl;
       for (list<double*>::iterator iter = trajectory.begin(); iter != trajectory.end(); iter++) 
       {
       double *s = *iter;
       cout<<s[0]<<" "<<s[1]<<" "<<s[2]<<endl;
       }
       */

    list<float>::iterator iterControl = control.begin();
    list<int>::iterator iterDirection = direction.begin();
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
      int directionTmp = *iterDirection;
      trajret.push_back(stateTmp);
      controlret.push_back(controlTmp);
      directionret.push_back(directionTmp);

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
  //cout<<"norm_state: "<< system->norm_state(stateRootNew, vertexChildNewState.x) << endl;
  if(system->norm_state(stateRootNew, vertexChildNewState.x) < 1e-20)
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
    //vertexChildNew->costFromRoot = (-1.0)*vertexChildNew->costFromParent - 1.0;
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
        //cout<<"vertexCurrDel: "<< vertexCurrDel->costFromRoot<<endl;

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




template< class T >
int 
RRTstar::Planner< T >
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
      system->getTrajectory (stateParent, stateCurr, trajectory, control, NULL, false);

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

    //delete [] stateArrCurr;
  }

  return 1;
}


template< class T >
int 
  RRTstar::Planner< T >
::isSafeTrajectory(list<double*>& trajectory)
{
  for (list<double*>::iterator iter = trajectory.begin(); iter != trajectory.end(); iter++)
  {
    if( system->IsInCollision(*iter) )
      return false;
  }
  return true;
}


template< class T >
double
  RRTstar::Planner< T >
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

// need prototype templates so that gcc understands the
// template with which these classes will be called while
// instantiation. this is required to remove undefined
// references while linking
template class RRTstar::Vertex<DubinsCar>;
template class RRTstar::Planner<DubinsCar>;
#endif
