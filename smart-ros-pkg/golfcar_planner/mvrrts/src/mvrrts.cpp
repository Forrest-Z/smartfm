#ifndef __RRTS_HPP_
#define __RRTS_HPP_


#include "mvrrts.h"

#include <iostream>

#include <cfloat>
#include <cmath>

#include <algorithm>

using namespace std;

template< class T >
MVRRTstar::Vertex< T >
::Vertex () {

  state = NULL;
  parent = NULL;
  trajFromParent = NULL;
}


template< class T >
MVRRTstar::Vertex< T >
::~Vertex () {

  if (state)
    delete state;

  parent = NULL;

  if (trajFromParent)
    delete trajFromParent;

  children.clear();
}


template< class T >
MVRRTstar::Vertex< T >
::Vertex(const vertex_t& vertexIn) {

  if (vertexIn.state)
    state = new state_t (vertexIn.getState());
  else 
    state = NULL;
  parent = vertexIn.parent;
  for (typename set<vertex_t*>::const_iterator iter = vertexIn.children.begin(); iter != vertexIn.children.end(); iter++)
    children.insert (*iter);
  if (vertexIn.trajFromParent)
    trajFromParent = new trajectory_t (*(vertexIn.trajFromParent));
  else 
    trajFromParent = NULL;

  lus_from_root = vertexIn.lus_from_root;
  lus_from_parent = vertexIn.lus_from_parent;
}


// int Vertex::setState (const State &stateIn) {
//   *state = stateIn;
//   return 1;
// }


template< class T >
MVRRTstar::Planner< T >
::Planner () {

  gamma = 1.0;
  goalSampleFreq = 0.0;

  lowerBoundLUS.metric_cost = DBL_MAX;
  lowerBoundVertex = NULL;

  kdtree = NULL; 

  root = NULL;

  numVertices = 0;

  system = NULL;

  abar = NULL;
}


template< class T >
MVRRTstar::Planner< T >
::~Planner () {

  if (kdtree)
    kd_free (kdtree);

  if(abar)
    delete abar;
}


template< class T >
int 
MVRRTstar::Planner< T >
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
MVRRTstar::Planner< T >
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
MVRRTstar::Planner< T >
::getNearVertices (state_t& stateIn, vector<vertex_t*>& vectorNearVerticesOut) {

  // Get the state key for the query state
  double *stateKey = new double[numDimensions];
  system->getStateKey (stateIn, stateKey);
  //cout<<"regionOperating: "<< system->regionOperating.size[0]<<" "<<
  //  system->regionOperating.size[1]<<" "<<
  //  system->regionOperating.size[2]<<endl;

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
  MVRRTstar::Planner< T >
::updateReachability () 
{
  updateBranchCost(*root, 1);
  lowerBoundLUS = getBestVertexLUS();

  vertex_t &bestVertex = getBestVertex();
  lowerBoundVertex = &bestVertex;

  return 1;
}


template< class T >
int
MVRRTstar::Planner< T >
::checkUpdateBestVertex (vertex_t& vertexIn) {

  if (system->isReachingTarget(vertexIn.getState()))
  {
    Level_of_US& lus_vertex = vertexIn.lus_from_root;
    if( ((lowerBoundVertex != NULL) && (lus_vertex < lowerBoundLUS)) || (lowerBoundVertex == NULL))
    {
      lowerBoundVertex = &vertexIn;
      lowerBoundLUS = lus_vertex;
    }
  }

  return 1;
}

template< class T >
typename MVRRTstar::Planner<T>::vertex_t*
  MVRRTstar::Planner< T >
::insertTrajectory (vertex_t& vertexStartIn, trajectory_t& trajectoryIn) 
{
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
MVRRTstar::Planner< T >
::insertTrajectory (vertex_t& vertexStartIn, trajectory_t& trajectoryIn, vertex_t& vertexEndIn) {

  // Update the costs
  vertexEndIn.lus_from_root = vertexStartIn.lus_from_root;

  vertexEndIn.lus_from_parent = get_lus_between(*vertexStartIn.state, *vertexEndIn.state, trajectoryIn);
  vertexEndIn.lus_from_root += vertexEndIn.lus_from_parent;

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
MVRRTstar::Planner< T >
::setSystem (system_t& systemIn, Auto_AB& abar_in) {

  system = &systemIn;
  abar = &abar_in;

  numDimensions = system->getNumDimensions ();

  if(listVertices.size() > 0)
  {
    // Delete all the vertices
    for (typename list<vertex_t*>::iterator iter = listVertices.begin(); iter != listVertices.end(); iter++)
      delete *iter;
  }
  listVertices.clear();

  numVertices = 0;
  lowerBoundLUS.metric_cost = DBL_MAX;
  lowerBoundVertex = NULL;

  // Clear the kdtree
  if (kdtree) {
    kd_free (kdtree);
  }
  kdtree = kd_create (numDimensions);

  // Initialize the root vertex
  root = new vertex_t;
  root->state = new state_t (system->getRootState());
  root->trajFromParent = NULL;

  return 1;
}



template< class T >
typename MVRRTstar::Planner< T >::vertex_t& 
MVRRTstar::Planner< T >
::getRootVertex () {

  return *root;
}



template< class T >
int 
MVRRTstar::Planner< T >
::initialize () {

  // If there is no system, then return failure
  if (!system || !abar)
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

  lowerBoundLUS = Level_of_US();
  lowerBoundLUS.metric_cost = DBL_MAX;
  lowerBoundVertex = NULL;

  // Clear the kdtree
  if (kdtree)
    kd_free (kdtree);
  kdtree = kd_create (system->getNumDimensions());

  //cout<<"356"<<endl;

  // Initialize the variables
  numDimensions = system->getNumDimensions();
  root = rootBackup;
  if (root)
  {
    root->children.clear();
    root->lus_from_parent = Level_of_US();
    root->lus_from_root = Level_of_US();
    root->trajFromParent = NULL;

    listVertices.push_back(root);
    insertIntoKdtree (*root);
    numVertices++;
  }

  //cout<<"380"<<endl;
  return 1;
}


template< class T >
int 
MVRRTstar::Planner< T >
::setGamma (double gammaIn) {

  if (gammaIn < 0.0)
    return 0;

  gamma = gammaIn;

  return 1;
}


template< class T >
int 
MVRRTstar::Planner< T >
::setGoalSampleFrequency (double sampleFreqIn) {

  if ( (sampleFreqIn < 0.0) || (sampleFreqIn > 1.0) )
    return 0;

  goalSampleFreq = sampleFreqIn;

  return 1;  
}

template< class T > bool compareVertexCostPairs( 
    pair< typename MVRRTstar::Vertex<T>* , Level_of_US* > vwP_i, 
    pair< typename MVRRTstar::Vertex<T>* , Level_of_US* > vwP_j) {
  // Return true if vertex-weight pair 'i' is 
  // less than vertex-weight pair 'j'
  return ( *(vwP_i.second) < *(vwP_j.second) );
}

template< class T >
int 
MVRRTstar::Planner< T >
::findBestParent (state_t& stateIn, vector<vertex_t*>& vectorNearVerticesIn,
    vertex_t*& vertexBest, trajectory_t& trajectoryOut, bool& exactConnection, list<float>& controlOut) {

  Subset_of_Sigma stateIn_label = system->label_state(stateIn.x);

  // Compute the cost of extension for each near vertex
  int numNearVertices = vectorNearVerticesIn.size();

  vector< pair<vertex_t*,Level_of_US*> > vertexCostPairs;

  int i = 0;
  for (typename vector<vertex_t*>::iterator iter = vectorNearVerticesIn.begin(); iter != vectorNearVerticesIn.end(); iter++) 
  {
    vertex_t* v = *iter;
    Subset_of_Sigma vertex_label = system->label_state(v->state->x);
    Level_of_US* candidate_lus = new Level_of_US(v->lus_from_root);

    exactConnection = false;
    double traj_cost = system->evaluateExtensionCost ( *((*iter)->state), stateIn, exactConnection);
    if((traj_cost > 0) && (traj_cost < DBL_MAX/2.0))
    {
      *candidate_lus += abar->weight(transition_label(vertex_label, stateIn_label, traj_cost));
      vertexCostPairs.push_back(make_pair(v, candidate_lus));
      i++;
    }
  }

  // Sort vertices according to cost
  sort (vertexCostPairs.begin(), vertexCostPairs.end(), compareVertexCostPairs<T>);

  // Try out each extension according to increasing cost
  i = 0;
  bool connectionEstablished = false;
  for(typename vector<pair<vertex_t*, Level_of_US*> >::iterator iter = vertexCostPairs.begin();
      iter != vertexCostPairs.end(); iter++)
  {
    vertex_t* vertexCurr = iter->first;

    // Extend the current vertex towards stateIn (and this time check for collision with obstacles)
    exactConnection = false;
    if (system->extendTo(*(vertexCurr->state), stateIn, trajectoryOut, exactConnection, controlOut, true) > 0) 
    {
      vertexBest = vertexCurr;
      connectionEstablished = true;
      break;
    }
  }
  // deallocate lus pointers
  for(typename vector<pair<vertex_t*, Level_of_US*> >::iterator iter = vertexCostPairs.begin();
      iter != vertexCostPairs.end(); iter++)
    delete iter->second;

  // Return success if a connection was established
  if (connectionEstablished)
    return 1;

  // If the connection could not be established then return zero
  return 0;
}


template< class T >
int 
MVRRTstar::Planner< T >
::updateBranchCost (vertex_t& vertexIn, int depth) {


  // Update the cost for each children
  for (typename set<vertex_t*>::iterator iter = vertexIn.children.begin(); iter != vertexIn.children.end(); iter++) 
  {

    vertex_t& vertex = **iter;

    vertex.lus_from_root += vertex.lus_from_parent;

    checkUpdateBestVertex (vertex);

    updateBranchCost (vertex, depth + 1);
  }

  return 1;
}


template< class T >
int 
  MVRRTstar::Planner< T >
::markCost (vertex_t& vertexIn)
{
  vertexIn.lus_from_root.metric_cost = -1.0;

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
  MVRRTstar::Planner< T >
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
  MVRRTstar::Planner< T >
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
      if( vertex->lus_from_root.metric_cost > -0.5)
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
    lowerBoundLUS = Level_of_US();
    lowerBoundLUS.metric_cost = DBL_MAX;

    listSurvivingVertices.clear();
    recomputeCost (root);
  }
  return 1;
}


template< class T >
int 
  MVRRTstar::Planner< T >
::rewireVertices (vertex_t& vertexNew, vector<vertex_t*>& vectorNearVertices) 
{
  state_t& state_new = *(vertexNew.state);

  // Repeat for all vertices in the set of near vertices
  for (typename vector<vertex_t*>::iterator iter = vectorNearVertices.begin(); iter != vectorNearVertices.end(); iter++) 
  {
    vertex_t& vertex_curr = **iter; 
    state_t& state_curr = *(vertex_curr.state);

    // Check whether the extension results in an exact connection
    bool exactConnection = false;
    double cost_curr = system->evaluateExtensionCost (state_new, state_curr, exactConnection);
    if ( (exactConnection == false) || (cost_curr < 0) )
      continue;

    // calculate lus of the trajectory
    Level_of_US lus_new = Level_of_US(vertexNew.lus_from_root);
    Subset_of_Sigma label_new = system->label_state(state_new.x);
    Subset_of_Sigma label_curr = system->label_state(state_curr.x);
    lus_new += abar->weight(transition_label(label_new, label_curr, cost_curr));

    // Check whether the cost of the extension is smaller than current cost
    if(lus_new < vertex_curr.lus_from_root)
    {
      // Compute the extension (checking for collision)
      trajectory_t trajectory;
      list<float> tmp_control;
      if (system->extendTo (state_new, state_curr, trajectory, exactConnection, tmp_control, true) <= 0 ) 
        continue;

      // Insert the new trajectory to the tree by rewiring
      insertTrajectory (vertexNew, trajectory, vertex_curr);

      // Update the cost of all vertices in the rewired branch
      updateBranchCost (vertex_curr, 0);
    }
  }

  return 1;
}


template< class T >
int 
MVRRTstar::Planner< T >
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
  else 
  {
    // 3.b Extend the best parent within the near vertices
    if (findBestParent (stateRandom, vectorNearVertices, vertexParent, trajectory, exactConnection, control) <= 0) 
    {
      return 0;
    }
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
  MVRRTstar::Planner< T >
::findDescendantVertices (vertex_t* vertexIn) 
{
  // Mark the node so that we can understand 
  // later that it is in the list of descendants
  vertexIn->lus_from_root.metric_cost = (-1.0) * vertexIn->lus_from_root.metric_cost - 1.0;  

  for (typename set<vertex_t*>::iterator iter = vertexIn->children.begin(); iter != vertexIn->children.end(); iter++) {
    vertex_t* vertexCurr = *iter;
    findDescendantVertices (vertexCurr);
  }
  return 1;
}


template< class T >
Level_of_US
  MVRRTstar::Planner< T >
::get_lus_between(state_t& start, state_t& end, trajectory_t& trajectory)
{
  Subset_of_Sigma start_label = system->label_state(start.x);
  Subset_of_Sigma end_label = system->label_state(end.x);
  return Level_of_US(abar->weight(transition_label(start_label, end_label, trajectory.totalVariation)));
}

template< class T >
int 
MVRRTstar::Planner< T >
::recomputeCost (vertex_t* vertexIn) {

  for (typename set<vertex_t*>::iterator iter = vertexIn->children.begin(); iter != vertexIn->children.end(); iter++) 
  {
    vertex_t* vertexCurr = *iter;

    vertexCurr->lus_from_root = vertexIn->lus_from_root + vertexCurr->lus_from_parent;
    Level_of_US& lus_vertex = vertexCurr->lus_from_root;

    if (system->isReachingTarget(vertexIn->getState())) 
    {
      if( ((lowerBoundVertex != NULL) && (lus_vertex < lowerBoundLUS)) || (lowerBoundVertex == NULL))
      {
        lowerBoundVertex = vertexCurr;
        lowerBoundLUS = lus_vertex;
      }
    }
    recomputeCost (vertexCurr);
  }

  return 1;
}

// note trajret, controlret is always only appended
template< class T >
int 
  MVRRTstar::Planner< T >
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
  cout<<"norm_state: "<< system->norm_state(stateRootNew, vertexChildNewState.x) <<endl;
  if(system->norm_state(stateRootNew, vertexChildNewState.x) < 1e-4)
  {
    cout<<"switch_root length > length of best_trajectory"<<endl;
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
    cout<<"goal further than max_committed_trajectory_length"<<endl;

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
      /*
      state_t& stateRoot = root->getState();
      for (int i = 0; i < numDimensions; i++)
        stateRoot[i] = stateRootNew[i];
      */
      initialize();
      return 1;
    }

    if (vertexChildNew->trajFromParent)
      delete vertexChildNew->trajFromParent;

    vertexChildNew->trajFromParent = new trajectory_t(connectingTrajectory);
    vertexChildNew->parent = vertexRoot;
    vertexChildNew->lus_from_parent = get_lus_between(*(vertexRoot->state), *(vertexChildNew->state), connectingTrajectory);
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
      if (vertexCurrDel->lus_from_root.metric_cost < -0.5) {

        // Revert the mark 
        vertexCurrDel->lus_from_root.metric_cost = (-1.0)*(vertexCurrDel->lus_from_root.metric_cost + 1.0);

        // Add the vertex to the list of surviving vetices
        listSurvivingVertices.push_front (vertexCurrDel);
      }
      else 
      {
        delete vertexCurrDel;
      }
    }

    listVertices.clear();
    numVertices = 0;

    lowerBoundVertex = NULL;
    lowerBoundLUS = Level_of_US();
    lowerBoundLUS.metric_cost = DBL_MAX;

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
MVRRTstar::Planner< T >
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


template< class T >
int 
  MVRRTstar::Planner< T >
::isSafeTrajectory(list<double*>& trajectory)
{
  int max_obs_check = 4;
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


template< class T >
double
  MVRRTstar::Planner< T >
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
template class MVRRTstar::Vertex<MVDubinsCar>;
template class MVRRTstar::Planner<MVDubinsCar>;
#endif
