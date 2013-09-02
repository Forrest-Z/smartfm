/*
 * CCRRTS.h
 *
 *  Created on: Jul 12, 2013
 *      Author: liuwlz
 */
/*
 * Implement Chance Constrained RRT*: 1)Algorithm, 2)Implementation Issues
 */

#ifndef CCRRTS_H_
#define CCRRTS_H_

#include <kdtree.h>
#include <kdb.h>
#include <cmath>
#include <algorithm>
#include <list>
#include <set>

#include "cc_system.h"

namespace CCRRTSNS{

	template <class T> class CCRRTS;

	template <class T>
	class CCVertex{
		typedef typename T::CCStateType state_t;
		typedef typename T::CCTrajectoryType trajectory_t;
		typedef typename T::CCSystemType system_t;

		typedef CCVertex<T> vertex_t;

	public:
		vertex_t *parent;
		state_t *state;
		set<vertex_t*> children;
		Level_OF_Risk lorFromParent;
		Level_OF_Risk lorFromRoot;
		trajectory_t *trajFromParent;

		CCVertex ();
		~CCVertex ();
		CCVertex (const CCVertex &vertexIn);

		state_t& getState () {return *state;}
		state_t& getState () const {return *state;}
		vertex_t& getParent () {return *parent;}
		Level_OF_Risk getLor() {return lorFromRoot;}
		double getTurningRadius(){return trajFromParent->getDubinsRadius();}
		friend class CCRRTS<T>;
	};

	template <class T>
	class CCRRTS {
		typedef typename T::CCStateType state_t;
		typedef typename T::CCTrajectoryType trajectory_t;
		typedef typename T::CCSystemType system_t;

		typedef CCVertex<T> vertex_t;
		typedef struct kdtree kdtree_t;
		typedef struct kdres kdres_t;

	public:

		int numDimensions;

		double gamma;
		double goalSampleFreq;
		Level_OF_Risk lowerBoundLor;
		vertex_t *lowerBoundVertex;
		kdtree_t *kdtree;

		vertex_t *root;

		int insertIntoKdtree (vertex_t &vertexIn);
		int getNearestVertex (state_t& stateIn, vertex_t*& vertexPointerOut);
		int getNearVertices (state_t& stateIn, vector<vertex_t*>& vectorNearVerticesOut);

		int checkUpdateBestVertex (vertex_t& vertexIn);
		int updateReachability ();

		vertex_t* insertTrajectory (vertex_t& vertexStartIn, trajectory_t& trajectoryIn);
		int insertTrajectory (vertex_t& vertexStartIn, trajectory_t& trajectoryIn, vertex_t& vertexEndIn);

		int findBestParent (state_t& stateIn, vector<vertex_t*>& vectorNearVerticesIn,
				vertex_t*& vertexBestOut, trajectory_t& trajectoryOut, bool& exactConnection, list<float> &control);

		int updateBranchCost (vertex_t& vertexIn, int depth);
		int rewireVertices (vertex_t& vertexNew, vector<vertex_t*>& vectorNearVertices);

		int findDescendantVertices (vertex_t* vertexIn);

		Level_OF_Risk get_risk_between(vertex_t& start, vertex_t& end, trajectory_t& trajectory);
		int recomputeLor (vertex_t* vertexIn);
		int recomputeCov (vertex_t* vertexIn);

		int markCost (vertex_t& vertexIn);
		int checkTrajectory (vertex_t& vertexIn);

		list<vertex_t*> listVertices;
		int numVertices;

		system_t* system;
		RiskEvaluate* risk_assess;

		CCRRTS();
		~CCRRTS();

		int setGamma (double gammaIn);
		int setGoalSampleFrequency (double sampleFreqIn);
		int setSystem (system_t& system);
		int setRiskLimit(double risk);

		int initialize ();
		int iteration (vector<double> &samples);

		int isSafeTrajectory(list<double*> &trajectory);
		int checkTree();
		int lazyCheckTree();

		vertex_t& getRootVertex ();
		Level_OF_Risk getBestVertexLor () {return lowerBoundLor;}
		vertex_t& getBestVertex () {return *lowerBoundVertex;}
		int getBestTrajectory (list<double*>& trajectory, list<float> &control);
		double getTrajectoryLength(list<double*> &trajectory);
		int getCommitTrajectory(list<double*> &traj, list<float>& control);
		int switchRoot (double distance, list<double*> &traj, list<float>& control);
	};
};

using namespace CCRRTSNS;

template <class T>
CCVertex <T>
::CCVertex (){
	state = NULL;
	parent = NULL;
	trajFromParent = NULL;
}

template <class T>
CCVertex <T>
:: ~CCVertex(){
	if (state)
		delete state;
	parent = NULL;

	if (trajFromParent)
		delete trajFromParent;
	children.clear();
}

template <class T>
CCVertex <T>
::CCVertex(const CCVertex &vertexIn){

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

	lorFromParent = vertexIn.lorFromParent;
	lorFromRoot = vertexIn.lorFromRoot;
}

template <class T>
CCRRTS <T>
::CCRRTS(){
	gamma = 15;
	goalSampleFreq = 0.2;

	lowerBoundLor.metric_cost = DBL_MAX;
	lowerBoundLor.risk = DBL_MAX;
	lowerBoundVertex = NULL;

	kdtree = NULL;
	root = NULL;
	numVertices = 0;
	system = NULL;
}

template <class T>
CCRRTS <T>
::~CCRRTS(){
	if (kdtree)
		kd_free(kdtree);
}

template <class T>
int
CCRRTS <T>
::insertIntoKdtree(vertex_t& vertexIN){
	double *stateKey = new double[numDimensions];
	system->getStateKey ( *(vertexIN.state), stateKey);
	kd_insert (kdtree, stateKey, &vertexIN);
	delete [] stateKey;
	return 1;
}

template <class T>
int CCRRTS <T>
::getNearestVertex(state_t& stateIn, vertex_t*& vertexPointerOut){
	double *stateKey = new double[numDimensions];
	system->getStateKey (stateIn, stateKey);

	// Search the kdtree for the nearest vertex
	kdres_t *kdres = kd_nearest (kdtree, stateKey);
	if (kd_res_end (kdres))
		vertexPointerOut = NULL;
	vertexPointerOut = (vertex_t*) kd_res_item_data (kdres);

	delete [] stateKey;
	kd_res_free (kdres);

	// Return a non-positive number if any errors
	if (vertexPointerOut == NULL)
		return 0;
	return 1;
}

template< class T >
int
CCRRTS< T >
::getNearVertices (state_t& stateIn, vector<vertex_t*>& vectorNearVerticesOut) {

	// Get the state key for the query state
	double *stateKey = new double[numDimensions];
	system->getStateKey (stateIn, stateKey);

	// Compute the ball radius
	double ballRadius = 2.5 * pow( log((double)(numVertices + 1.0))/((double)(numVertices + 1.0)), 1.0/((double)numDimensions) );

	double steer_dist = system->distance_limit/30;

	double radius;
	if (ballRadius > steer_dist)
		radius = steer_dist;
	else
		radius = ballRadius;

	ROS_INFO("Ball Radius: %f, %f, %f",steer_dist, ballRadius, radius);

	// Search kdtree for the set of near vertices
	kdres_t *kdres = kd_nearest_range (kdtree, stateKey, radius);
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
CCRRTS< T >
::updateReachability (){

  recomputeLor(root);
  lowerBoundLor = getBestVertexLor();

  vertex_t &bestVertex = getBestVertex();
  lowerBoundVertex = &bestVertex;

  return 1;
}

template< class T >
int
CCRRTS<T>
::checkUpdateBestVertex (vertex_t& vertexIn) {
	if (system->isReachingTarget(vertexIn.getState())){
		Level_OF_Risk& risk_vertex = vertexIn.lorFromRoot;
		if( ((lowerBoundVertex != NULL) && (risk_vertex < lowerBoundLor)) || (lowerBoundVertex == NULL)){
			lowerBoundVertex = &vertexIn;
			lowerBoundLor = risk_vertex;
		}
	}
	return 1;
}

template< class T >
typename CCRRTS<T>::vertex_t*
CCRRTS < T >
::insertTrajectory (vertex_t& vertexStartIn, trajectory_t& trajectoryIn) {
/*
	Level_OF_Risk new_lor = vertexStartIn.lorFromRoot;
	Level_OF_Risk lor_from_parent, lor_to_go;
	lor_from_parent.metric_cost = trajectoryIn.evaluateCost();
	lor_from_parent.risk = trajectoryIn.evaluateRisk();
	new_lor += lor_from_parent;

	double distToGo = system->evaluateCostToGo (trajectoryIn.getEndState());
	if (distToGo >= 0.0){
		lor_to_go.metric_cost = new_lor.metric_cost;
		lor_to_go.risk = new_lor.risk;
		lor_to_go.metric_cost += distToGo;
	}
	// Create a new end vertex
	if( (!lowerBoundVertex) || (lor_to_go < lowerBoundLor)){

		//ROS_INFO("insertTrajectory");
*/
		vertex_t* vertexNew = new vertex_t;
		 vertexNew->state = new state_t;
		 vertexNew->parent = NULL;

		 trajectoryIn.getEndState(vertexNew->getState());
		 insertIntoKdtree (*vertexNew);
		 this->listVertices.push_front (vertexNew);
		 this->numVertices++;
		 insertTrajectory (vertexStartIn, trajectoryIn, *vertexNew);
		 ROS_INFO("insertTrajectory: lor_from_parent: %f", trajectoryIn.evaluateCost());
		 return vertexNew;

	/*
	}
	return NULL;
	*/
}

template< class T >
int
CCRRTS < T >
::insertTrajectory (vertex_t& vertexStartIn, trajectory_t& trajectoryIn, vertex_t& vertexEndIn) {

	Level_OF_Risk new_lor = vertexStartIn.lorFromRoot;
	Level_OF_Risk lor_from_parent, lor_to_go;
	lor_from_parent.metric_cost = trajectoryIn.evaluateCost();
	lor_from_parent.risk = trajectoryIn.evaluateRisk();
	new_lor += lor_from_parent;

	vertexEndIn.lorFromRoot = new_lor;
	vertexEndIn.lorFromParent = lor_from_parent;

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
	//ROS_INFO("insertTrajectory_direct: lor_of_parent: %f, lor_from_parent: %f, lor_from_root %f", vertexStartIn.lorFromRoot.metric_cost, vertexEndIn.lorFromParent.metric_cost, vertexEndIn.lorFromRoot.metric_cost);
	vertexStartIn.children.insert (&vertexEndIn);

	return 1;
}

template< class T >
int
CCRRTS < T >
::setSystem (system_t& systemIn) {

	system = &systemIn;

	numDimensions = system->getNumDimensions ();

	if(listVertices.size() > 0){
    // Delete all the vertices
		for (typename list<vertex_t*>::iterator iter = listVertices.begin(); iter != listVertices.end(); iter++)
			delete *iter;
	}
	listVertices.clear();

	numVertices = 0;
	lowerBoundLor.metric_cost = DBL_MAX;
	lowerBoundLor.risk = DBL_MAX;
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
	root->lorFromParent.risk = 0.0;
	root->lorFromParent.metric_cost = 0.0;
	root->lorFromRoot.metric_cost = 0.0;
	root->lorFromRoot.risk = 0.0;
	return 1;
}

template< class T >
typename CCRRTS< T >::vertex_t&
CCRRTS< T >
::getRootVertex () {

	return *root;
}

template< class T >
int
CCRRTS< T >
::initialize () {

  // If there is no system, then return failure
	if (!system)
		return 0;

  // Backup the root
	vertex_t *rootBackup = NULL;
	if (root)
		rootBackup = new vertex_t (*root);

	for (typename list<vertex_t*>::iterator iter = listVertices.begin(); iter != listVertices.end(); iter++){
		delete *iter;
	}

	listVertices.clear();
	numVertices = 0;

	lowerBoundLor = Level_OF_Risk();
	lowerBoundLor.metric_cost = DBL_MAX;
	lowerBoundLor.risk = DBL_MAX;
	lowerBoundVertex = NULL;

	// Clear the kdtree
	if (kdtree)
		kd_free (kdtree);
	kdtree = kd_create (system->getNumDimensions());

  // Initialize the variables
	numDimensions = system->getNumDimensions();
	root = rootBackup;
	if (root){
		root->children.clear();
		root->lorFromParent.risk = 0.0;
		root->lorFromParent.metric_cost = 0.0;
		root->lorFromRoot.risk = 0.0;
		root->lorFromRoot.metric_cost = 0.0;
		root->trajFromParent = NULL;

		listVertices.push_back(root);
		insertIntoKdtree (*root);
		numVertices++;
	}
	return 1;
}

template< class T >
int
CCRRTS< T >
::setGamma (double gammaIn) {

  if (gammaIn < 0.0)
    return 0;

  gamma = gammaIn;

  return 1;
}

template< class T >
int
CCRRTS< T >
::setGoalSampleFrequency (double sampleFreqIn) {

  if ( (sampleFreqIn < 0.0) || (sampleFreqIn > 1.0) )
    return 0;

  goalSampleFreq = sampleFreqIn;

  return 1;
}

template< class T >
int
CCRRTS< T >
::setRiskLimit (double risk) {

  if ( (risk < 0.0) || (risk > 1.0) )
    return 0;

  system->risk_limit = risk;

  return 1;
}

template< class T >
bool compareVertexRiskPairs(
		pair< typename CCRRTSNS::CCVertex<T>* , Level_OF_Risk* > lor_i,
		pair< typename CCRRTSNS::CCVertex<T>* , Level_OF_Risk* > lor_j) {
	return ( *(lor_i.second) < *(lor_j.second) );
}

template< class T >
int
CCRRTS< T >
::findBestParent (state_t& stateIn, vector<vertex_t*>& vectorNearVerticesIn,
	vertex_t*& vertexBest, trajectory_t& trajectoryOut, bool& exactConnection, list<float>& controlOut) {

	//ROS_INFO("findBestParent");

	vector< pair<vertex_t*,Level_OF_Risk*> > vertexRiskPairs;

	for (typename vector<vertex_t*>::iterator iter = vectorNearVerticesIn.begin(); iter != vectorNearVerticesIn.end(); iter++){
		vertex_t* v = *iter;
		Level_OF_Risk* candidate_lor = new Level_OF_Risk(v->lorFromRoot);
		trajectory_t temp_traj;
		exactConnection = false;
		Level_OF_Risk increment_lor;

		if(system->extendTo(*(v->state), stateIn, true, true, temp_traj, exactConnection, controlOut) > 0){
			increment_lor.risk = temp_traj.evaluateRisk();
			increment_lor.metric_cost = temp_traj.evaluateCost();
			*candidate_lor += increment_lor;
			vertexRiskPairs.push_back(make_pair(v, candidate_lor));
			//ROS_INFO("FineBestParent: Check cost: %f, traj_cost %f", (*candidate_lor).metric_cost, increment_lor.metric_cost);
		}
	}
	// Sort vertices according to lor
	sort (vertexRiskPairs.begin(), vertexRiskPairs.end(), compareVertexRiskPairs<T>);

	// Try out each extension according to increasing cost
	bool connectionEstablished = false;
	for(typename vector<pair<vertex_t*, Level_OF_Risk*> >::iterator iter = vertexRiskPairs.begin(); iter != vertexRiskPairs.end(); iter++){

		vertex_t* vertexCurr = iter->first;
		// Extend the current vertex towards stateIn (and this time check for collision with obstacles)
		exactConnection = false;
		if (system->extendTo(*(vertexCurr->state), stateIn, true, true, trajectoryOut, exactConnection, controlOut) > 0){
			vertexBest = vertexCurr;
			//ROS_INFO("FindBestParent, extendto Metric Check: %f,  Risk Check: %f",trajectoryOut.evaluateCost(), trajectoryOut.evaluateRisk());
			connectionEstablished = true;
			break;
		}
	}

	for(typename vector<pair<vertex_t*, Level_OF_Risk*> >::iterator iter = vertexRiskPairs.begin(); iter != vertexRiskPairs.end(); iter++)
		delete iter->second;

	if (connectionEstablished)
		return 1;

	ROS_INFO("Connection Failed");
	return -1;
}

//mark the vertexes that have invalid trajectory
template< class T >
int
CCRRTS< T >
::markCost (vertex_t& vertexIn){
	ROS_INFO("MakrCost: Please be careful");
	vertexIn.lorFromRoot.metric_cost = -1.0;
	vertex_t &parent = vertexIn.getParent();
	parent.children.erase(&vertexIn);
	for (typename set<vertex_t*>::iterator iter = vertexIn.children.begin(); iter != vertexIn.children.end(); iter++){
		markCost(**iter);
	}
	return 1;
}

template< class T >
int
CCRRTS< T >
::checkTrajectory (vertex_t& vertexIn){

	vertex_t &parent = vertexIn.getParent();
	state_t  &state_par = parent.getState();
	state_t &state_in = vertexIn.getState();

	trajectory_t traj;
	list<float> tmp_control;
	bool exactConnection;
	if(system->extendTo(state_par, state_in, true, true, traj, exactConnection, tmp_control)){
		for (typename set<vertex_t*>::iterator iter = vertexIn.children.begin(); iter != vertexIn.children.end(); iter++){
			checkTrajectory(**iter);
		}
	}
	else{
		markCost(vertexIn);
	}
	return 1;
}

template< class T >
int
CCRRTS< T >
::lazyCheckTree(){
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
CCRRTS< T >
::checkTree(){
	if( root->children.size() > 0){
		for (typename set<vertex_t*>::iterator iter = root->children.begin(); iter != root->children.end(); iter++){
			vertex_t &vertex = **iter;
			checkTrajectory(vertex);
		}

		list<vertex_t*> listSurvivingVertices;
		for (typename list<vertex_t*>::iterator iter = listVertices.begin(); iter != listVertices.end(); iter++){
			vertex_t *vertex  = *iter;
			if( vertex->lorFromRoot.metric_cost > -0.5)
				listSurvivingVertices.push_front(vertex);
			else{
				delete vertex;
			}
		}

		if (kdtree) {
			kd_free (kdtree);
		}
		kdtree = kd_create (system->getNumDimensions());

		listVertices.clear();
		numVertices = 0;
		for (typename list<vertex_t*>::iterator iter = listSurvivingVertices.begin(); iter != listSurvivingVertices.end(); iter++){
			listVertices.push_front(*iter);
			numVertices++;
			insertIntoKdtree (**iter);
		}

		ROS_INFO("Vertex number, %d", numVertices);

		lowerBoundVertex = NULL;
		lowerBoundLor.metric_cost = DBL_MAX;
		lowerBoundLor.risk = DBL_MAX;

		listSurvivingVertices.clear();
		recomputeLor (root);
	}
	return 1;
}

template< class T >
int
CCRRTS< T >
::rewireVertices (vertex_t& vertexNew, vector<vertex_t*>& vectorNearVertices){
	state_t& state_new = *(vertexNew.state);

	//ROS_INFO("Rewire RRTS Tree");
	for (typename vector<vertex_t*>::iterator iter = vectorNearVertices.begin(); iter != vectorNearVertices.end(); iter++){
		vertex_t& vertex_curr = **iter;
		state_t& state_curr = *(vertex_curr.state);

		// Check whether the extension results in an exact connection
;
		bool exactConnection = false;
		trajectory_t trajectory;
		list<float> tmp_control;

		if (system->extendTo(state_new, state_curr, true, true, trajectory, exactConnection, tmp_control) < 0 )
			continue;

		// calculate lor of the trajectory
		Level_OF_Risk new_lor = Level_OF_Risk(vertexNew.lorFromRoot);
		Level_OF_Risk incremental_lor;
		incremental_lor.risk = trajectory.evaluateRisk();
		incremental_lor.metric_cost = trajectory.evaluateCost();
		new_lor += incremental_lor;

		//ROS_INFO("Rewire vertexNew: metric_cost %f, Current metric: %f", new_lor.metric_cost, vertex_curr.lorFromRoot.metric_cost );
		//ROS_INFO("RewireVertex, evaluateExtention Metric Check: %f, Risk Check: %f", incremental_lor.metric_cost, incremental_lor.risk);

		if(new_lor < vertex_curr.lorFromRoot){
			insertTrajectory (vertexNew, trajectory, vertex_curr);
			recomputeLor(&vertex_curr);
			recomputeCov(&vertex_curr);
		}
	}
	return 1;
}

template< class T >
int
CCRRTS< T >
::recomputeCov (vertex_t* vertexIn) {

	for (typename set<vertex_t*>::iterator iter = vertexIn->children.begin(); iter != vertexIn->children.end(); iter++){
		vertex_t* vertexCurr = *iter;

		vertex_t &parent = vertexCurr->getParent();
		state_t  &state_par = parent.getState();
		state_t &state_in = vertexCurr->getState();

		list<double*> tmp_traj;
		list<float> tmp_control;
		bool tmp_exact_connection = false;
		vector<double> temp_risk;
		double *end_state = new double[7];

		double time = system->extend_dubins_all (
				state_par.x, state_in.x,
				true, true, true, temp_risk,
				tmp_exact_connection, end_state, &tmp_traj, tmp_control, vertexCurr->getTurningRadius());

		ROS_INFO("RecomputerCov: State_orig: x: %f, y: %f, x_cov: %f, y_cov_: %f, xy_cov: %f. State_new: x: %f, y: %f, x_cov: %f, y_cov_: %f, xy_cov: %f",
				state_in.x[0], state_in.x[1],state_in.x[3],state_in.x[4],state_in.x[6],
				end_state[0],end_state[1],end_state[3],end_state[4],end_state[6]);
		recomputeCov(vertexCurr);
	}
	return 1;
}

template <class T>
int
CCRRTS <T>
::iteration(vector<double> &samples){

	// 1. Sample a new state
	//ROS_INFO("Iteration");
	state_t stateRandom;
	double randGoalSampling = ((double)rand())/(RAND_MAX + 1.0);
	if (randGoalSampling < 0.05 ) {
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


	// 2. Compute the set of all near vertices
	vector<vertex_t*> vectorNearVertices;
	getNearVertices (stateRandom, vectorNearVertices);

	// 3. Find the best parent and extend from that parent
	vertex_t* vertexParent = NULL;
	trajectory_t trajectory;
	list<float> control;
	bool exactConnection = false;

	if (vectorNearVertices.size() == 0) {
	    // 3.a Extend the neares
		if (getNearestVertex (stateRandom, vertexParent) <= 0)
			return 0;
		if (system->extendTo(vertexParent->getState(), stateRandom, true, true, trajectory, exactConnection, control) <= 0)
			return 0;
	}
	else{
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

	/*

	vector<vertex_t*> vectorNearVertices;
	vertex_t* vertexParent = NULL;
	vertex_t* vertexNearest = NULL;
	state_t stateNew;

	trajectory_t trajectory, temp_traj;
	list<float> control;
	bool exactConnection = false;

	if (getNearestVertex (stateRandom, vertexNearest) <= 0)
		return 0;
	if (system->extendTo(vertexNearest->getState(), stateRandom, true, true, temp_traj, exactConnection, control) < 0)
		return 0;

	temp_traj.getEndState(stateNew);

	getNearVertices (stateNew, vectorNearVertices);

	if (vectorNearVertices.size() > 0){
		if (findBestParent (stateNew, vectorNearVertices, vertexParent, trajectory, exactConnection, control) <= 0)
			return 0;
		//ROS_INFO("Get Best Parent");
	}
	else{
		vertexParent = vertexNearest;
		trajectory = temp_traj;
	}

	vertex_t* vertexNew = insertTrajectory (*vertexParent, trajectory);

	if (vectorNearVertices.size() > 0)
		rewireVertices (*vertexNew, vectorNearVertices);

	return 1;
	*/
}

template< class T >
int
CCRRTS< T >
::updateBranchCost (vertex_t& vertexIn, int depth) {
	// Update the cost for each children
	for (typename set<vertex_t*>::iterator iter = vertexIn.children.begin(); iter != vertexIn.children.end(); iter++){
		vertex_t& vertex = **iter;
		vertex.lorFromRoot += vertex.lorFromParent;
		checkUpdateBestVertex (vertex);
		updateBranchCost (vertex, depth + 1);
	}
	return 1;
}

template< class T >
int
CCRRTS< T >
::findDescendantVertices (vertex_t* vertexIn){
  // Mark the node so that we can understand later that it is in the list of descendants
	vertexIn->lorFromRoot.metric_cost = (-1.0) * vertexIn->lorFromRoot.metric_cost - 1.0;
	for (typename set<vertex_t*>::iterator iter = vertexIn->children.begin(); iter != vertexIn->children.end(); iter++) {
		vertex_t* vertexCurr = *iter;
		findDescendantVertices (vertexCurr);
	}
  return 1;
}

template< class T >
int
CCRRTS< T >
::recomputeLor (vertex_t* vertexIn) {

	for (typename set<vertex_t*>::iterator iter = vertexIn->children.begin(); iter != vertexIn->children.end(); iter++){
		vertex_t* vertexCurr = *iter;

		vertexCurr->lorFromRoot = vertexIn->lorFromRoot + vertexCurr->lorFromParent;
		Level_OF_Risk& lor_vertex = vertexCurr->lorFromRoot;

		if (system->isReachingTarget(vertexIn->getState())){
			if( ((lowerBoundVertex != NULL) && (lor_vertex < lowerBoundLor)) || (lowerBoundVertex == NULL)){
				lowerBoundVertex = vertexCurr;
				lowerBoundLor = lor_vertex;
			}
		}
		recomputeLor (vertexCurr);
	}
	return 1;
}

template< class T >
int
CCRRTS< T >
::getBestTrajectory (list<double*>& trajectoryOut, list<float> &controlOut ) {

	if (lowerBoundVertex == NULL){
		return 0;
	}
	vertex_t* vertexCurr = lowerBoundVertex;

	while (vertexCurr) {
		state_t& stateCurr = vertexCurr->getState();
		double *stateArrCurr = new double[7];
		for (int i = 0; i < 7; i++)
			stateArrCurr[i] = stateCurr[i];

		trajectoryOut.push_front (stateArrCurr);

		vertex_t& vertexParent = vertexCurr->getParent();

		if (&vertexParent != NULL) {

			state_t& stateParent = vertexParent.getState();
			list<double*> trajectory;
			list<float> control;
			system->getTrajectory (stateParent, stateCurr, trajectory, control, vertexCurr->getTurningRadius());

			trajectory.reverse ();
			control.reverse();
			for (list<double*>::iterator iter = trajectory.begin(); iter != trajectory.end(); iter++){
				double *stateArrFromParentCurr = *iter;

				stateArrCurr = new double[7];
				for (int i = 0; i < 7; i++)
					stateArrCurr[i] = stateArrFromParentCurr[i];
				trajectoryOut.push_front (stateArrCurr);
				delete [] stateArrFromParentCurr;
			}
			for (list<float>::iterator iter = control.begin(); iter != control.end(); iter++){
				controlOut.push_back(*iter);
			}
		}
		vertexCurr = &vertexParent;
	}
	return 1;
}

template< class T >
int
CCRRTS< T >
::isSafeTrajectory(list<double*>& trajectory){
	for (list<double*>::iterator iter = trajectory.begin(); iter != trajectory.end(); iter++){
		if( system->IsInCollisionLazy(*iter) )
			return false;
	}
	return true;
}

template< class T >
double
CCRRTS< T >
::getTrajectoryLength(list<double*>& trajectory){
	double len = 0;
	for (list<double*>::iterator iter = trajectory.begin(); iter != trajectory.end(); iter++){
		list<double*>::iterator next = iter;
		if(next != trajectory.end()){
			next++;
			double *currState = *iter;
			double *nextState = *next;
			len += sqrt( (currState[0] - nextState[0])*(currState[0] - nextState[0]) + (currState[1] - nextState[1])*(currState[1] - nextState[1]) );
		}
		else
			break;
	}
	return len;
}

template <class T>
int
CCRRTS <T>
::getCommitTrajectory(list<double*> &trajret, list<float> &controlret){
	if (lowerBoundVertex == NULL)
		return 0;

	// 1. Find the new root position
	list<vertex_t*> listBestVertex;
	vertex_t* vertexCurrBest = lowerBoundVertex;
	while (vertexCurrBest) {
		listBestVertex.push_front (vertexCurrBest);
		vertexCurrBest = &(vertexCurrBest->getParent());
	}

	for (typename list<vertex_t*>::iterator iter = listBestVertex.begin(); iter != listBestVertex.end(); iter++){

		vertex_t* vertexCurr = *iter;

		vertex_t& vertexParent = vertexCurr->getParent();
		if (&vertexParent == NULL)
			continue;

		state_t& stateCurr = vertexCurr->getState();
	    state_t& stateParent = vertexParent.getState();

	    list<double*> trajectory;
	    list<float> control;
	    system->getTrajectory (stateParent, stateCurr, trajectory, control, true, true);

	    list<float>::iterator iterControl = control.begin();
	    for (list<double*>::iterator iter = trajectory.begin(); iter != trajectory.end(); iter++){
	    	double *stateArrCurr = *iter;
	    	// write code for copying trajectory, control into traj here
	    	double *stateTmp = new double[3];
	    	stateTmp[0] = stateArrCurr[0];  stateTmp[1] = stateArrCurr[1]; stateTmp[2] = stateArrCurr[2];
	    	float controlTmp = *iterControl;
	    	trajret.push_back(stateTmp);
	    	controlret.push_back(controlTmp);

	    	iterControl++;
	    }

	    for (list<double*>::iterator iter = trajectory.begin(); iter != trajectory.end(); iter++) {
	    	double *stateArrCurr = *iter;
	    	delete [] stateArrCurr;
	    }
	}
	return 1;
}

template <class T>
int
CCRRTS <T>
::switchRoot (double distanceIn, list<double*> &trajret, list<float> &controlret){
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

	double *stateRootNew = new double[7];
	vertex_t* vertexChildNew = NULL;

	bool stateFound = false;

	state_t& rootState = root->getState();

	double *stateArrPrev = new double [7];
	for (int i = 0; i < 7; i++)
		stateArrPrev[i] = rootState[i];
	//cout<<"stateArrPrev: "<< stateArrPrev[0]<<" "<<stateArrPrev[1]<<" "<<stateArrPrev[2]<<endl;

	double distTotal = 0.0;
	for (typename list<vertex_t*>::iterator iter = listBestVertex.begin(); iter != listBestVertex.end(); iter++){

		vertex_t* vertexCurr = *iter;

		vertex_t& vertexParent = vertexCurr->getParent();
		if (&vertexParent == NULL)
			continue;

		state_t& stateCurr = vertexCurr->getState();
	    state_t& stateParent = vertexParent.getState();

	    list<double*> trajectory;
	    list<float> control;
	    system->getTrajectory (stateParent, stateCurr, trajectory, control, true, true);

	    list<float>::iterator iterControl = control.begin();
	    for (list<double*>::iterator iter = trajectory.begin(); iter != trajectory.end(); iter++){
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
	    	for (int i = 0; i < 7; i++)
	    		stateRootNew[i] = stateArrCurr[i];
	    	vertexChildNew = vertexCurr;

	    	if (distTotal >= distanceIn){
	    		stateFound = true;
	    		break;
	    	}

	    	for (int i = 0; i < 7; i++)
	    		stateArrPrev[i] = stateArrCurr[i];

	    	iterControl++;
	    }

	    // Free the temporary memory occupied by the states
	    for (list<double*>::iterator iter = trajectory.begin(); iter != trajectory.end(); iter++) {
	    	double *stateArrCurr = *iter;
	    	delete [] stateArrCurr;
	    }
	    if (stateFound){
	    	break;
	    }

	    // what is the this line for??
	    vertexCurr = &vertexParent;
	}
	//cout<<"trajret size: "<< trajret.size()<<endl;
	delete [] stateArrPrev;

	state_t &vertexChildNewState = vertexChildNew->getState();
	if(system->norm_state(stateRootNew, vertexChildNewState.x) < 1e-4){
	    cout<<"switch_root length > length of best_trajectory"<<endl;
	    state_t& stateRoot = root->getState();
	    for (int i = 0; i < 7; i++)
	    	stateRoot[i] = stateRootNew[i];
	    initialize();
	    return 1;
	}

	else{
		cout<<"goal further than max_committed_trajectory_length"<<endl;
	    // 2. Find and store all the decendandts of the new root
	    findDescendantVertices (vertexChildNew);

	    // 3. Create the new root vertex
	    vertex_t* vertexRoot = new vertex_t;
	    vertexRoot->state = new state_t;
	    state_t& stateRoot = vertexRoot->getState();
	    for (int i = 0; i < 7; i++)
	    	stateRoot[i] = stateRootNew[i];
	    vertexRoot->children.insert (vertexChildNew);

	    root = vertexRoot;

	    // 4. Connect the new root vertex to the new child
	    trajectory_t connectingTrajectory;
	    list <float> connectingControl;
	    bool exactConnection;

	    // if connection possible, switch to new root, else call vertex_new as the new root
	    if (system->extendTo(vertexRoot->getState(), vertexChildNew->getState(),true, true, connectingTrajectory, exactConnection, connectingControl) <= 0){
	    	cout << "switchRoot ERR: No extend, reinitializing" << endl;
	    	initialize();
	    	return 1;
	    }

	    if (vertexChildNew->trajFromParent)
	    	delete vertexChildNew->trajFromParent;

	    vertexChildNew->trajFromParent = new trajectory_t(connectingTrajectory);
	    vertexChildNew->parent = vertexRoot;
	    vertexChildNew->lorFromParent.risk = connectingTrajectory.evaluateRisk();
	    vertexChildNew->lorFromParent.metric_cost = connectingTrajectory.evaluateCost();
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
	    	if (vertexCurrDel->lorFromRoot.metric_cost < -0.5) {
	    		// Revert the mark
	    		vertexCurrDel->lorFromRoot.metric_cost = (-1.0)*(vertexCurrDel->lorFromRoot.metric_cost + 1.0);
	    		// Add the vertex to the list of surviving vetices
	    		listSurvivingVertices.push_front (vertexCurrDel);
	    	}
	    	else
	    		delete vertexCurrDel;
	    }

	    listVertices.clear();
	    numVertices = 0;

	    lowerBoundVertex = NULL;
	    lowerBoundLor = Level_OF_Risk();
	    lowerBoundLor.metric_cost = DBL_MAX;
	    lowerBoundLor.risk = DBL_MAX;

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

	    recomputeLor (vertexRoot);

	    cout<<"after switchRoot vertices left: "<< numVertices << endl;

	    // 8. Clear temporary memory
	    delete [] stateRootNew;

	    return 1;
	}
}
#endif /* CCRRTS_H_ */
