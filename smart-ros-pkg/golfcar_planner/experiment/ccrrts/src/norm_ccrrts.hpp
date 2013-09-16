/*
 * NormRRTS.h
 *
 *  Created on: Ang 12, 2013
 *      Author: liuwlz
 */
/*
 * Implement Chance Constrained RRT*: 1)Algorithm, 2)Implementation Issues
 */

#ifndef NormRRTS_H_
#define NormRRTS_H_

#include <kdtree.h>
#include <kdb.h>
#include <cmath>
#include <algorithm>
#include <list>
#include <set>

#include "norm_system.h"

namespace NormRRTSNS{

	template <class T> class NormRRTS;

	template <class T>
	class NormVertex{

		typedef typename T::NormStateType state_t;
		typedef typename T::NormTrajectoryType trajectory_t;
		typedef typename T::NormSystemType system_t;

		typedef NormVertex<T> vertex_t;

	public:
		vertex_t *parent;
		state_t *state;
		set<vertex_t*> children;
		Level_OF_Risk lorFromParent;
		Level_OF_Risk lorFromRoot;
		trajectory_t *trajFromParent;

		NormVertex ();
		~NormVertex ();
		NormVertex (const NormVertex &vertexIn);

		state_t& getState () {return *state;}
		state_t& getState () const {return *state;}
		vertex_t& getParent () {return *parent;}
		Level_OF_Risk getLor() {return lorFromRoot;};
		friend class NormRRTS<T>;
	};

	template <class T>
	class NormRRTS {
		typedef typename T::NormStateType state_t;
		typedef typename T::NormTrajectoryType trajectory_t;
		typedef typename T::NormSystemType system_t;

		typedef NormVertex<T> vertex_t;
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

		NormRRTS();
		~NormRRTS();

		int setGamma (double gammaIn);
		int setGoalSampleFrequency (double sampleFreqIn);
		int setSystem (system_t& system);
		int setRiskLimit(double risk);

		int initialize ();
		int iteration (vector<double> &samples);

		int isSafeTrajectory(list<double*> &trajectory);
		int checkTree();
		int lazyCheckTree();
		int clearInvalidBranch(vertex_t& vertexIn);

		vertex_t& getRootVertex ();
		Level_OF_Risk getBestVertexLor () {return lowerBoundLor;}
		vertex_t& getBestVertex () {return *lowerBoundVertex;}
		int getBestTrajectory (list<double*>& trajectory, list<float> &control);
		double getTrajectoryLength(list<double*> &trajectory);
		int getCommitTrajectory(list<double*> &traj, list<float>& control);
		int switchRoot (double distance, list<double*> &traj, list<float>& control);
	};
};

using namespace NormRRTSNS;

template <class T>
NormVertex <T>
::NormVertex (){
	state = NULL;
	parent = NULL;
	trajFromParent = NULL;
}

template <class T>
NormVertex <T>
:: ~NormVertex(){
	if (state)
		delete state;
	parent = NULL;

	if (trajFromParent)
		delete trajFromParent;
	children.clear();
}

template <class T>
NormVertex <T>
::NormVertex(const NormVertex &vertexIn){

	if (vertexIn.state)
		state = new state_t (vertexIn.getState());
	else
	    state = NULL;

	if (vertexIn.trajFromParent)
		trajFromParent = new trajectory_t (*(vertexIn.trajFromParent));
	else
		trajFromParent = NULL;

	parent = vertexIn.parent;
	for (typename set<vertex_t*>::const_iterator iter = vertexIn.children.begin(); iter != vertexIn.children.end(); iter++)
		children.insert (*iter);

	lorFromParent = vertexIn.lorFromParent;
	lorFromRoot = vertexIn.lorFromRoot;
}

template <class T>
NormRRTS <T>
::NormRRTS(){
	gamma = 1.8;
	goalSampleFreq = 0.00;

	lowerBoundLor.metric_cost = DBL_MAX;
	lowerBoundLor.risk = DBL_MAX;
	lowerBoundVertex = NULL;

	kdtree = NULL;
	root = NULL;
	numVertices = 0;
	system = NULL;
}

template <class T>
NormRRTS <T>
::~NormRRTS(){
	if (kdtree)
		kd_free(kdtree);
}

template <class T>
int
NormRRTS <T>
::insertIntoKdtree(vertex_t& vertexIN){
	double *stateKey = new double[numDimensions];
	system->getStateKey ( *(vertexIN.state), stateKey);
	kd_insert (kdtree, stateKey, &vertexIN);
	delete [] stateKey;
	//ROS_INFO("InsertToKdtree, stateKey: x:%f, y:%f", stateKey[0], stateKey[1]);
	return 1;
}

template <class T>
int NormRRTS <T>
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

	//ROS_INFO("getNearestVertex");

	// Return a non-positive number if any errors
	if (vertexPointerOut == NULL)
		return 0;

	return 1;
}

//TODO
template< class T >
int
NormRRTS< T >
::getNearVertices (state_t& stateIn, vector<vertex_t*>& vectorNearVerticesOut) {

	//ROS_INFO("GetNearVertex");
	// Get the state key for the query state
	double *stateKey = new double[numDimensions];
	system->getStateKey (stateIn, stateKey);

	// Compute the ball radius
	double ballRadius = 150 * pow( log((double)(numVertices + 1.0))/((double)(numVertices + 1.0)), 1.0/((double)numDimensions) );

	double radius;

	//Minis 0.01 to ensure that the nearest vertex is not included in near vertex for processing
	if (ballRadius > system->distance_limit)
		radius = system->distance_limit - 0.01 ;
	else
		radius = ballRadius;

	//ROS_INFO("Ball Radius: %f", radius);

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
NormRRTS< T >
::updateReachability (){
	//ROS_INFO("UpdateReachability");
	recomputeLor(root);
	lowerBoundLor = getBestVertexLor();

	vertex_t &bestVertex = getBestVertex();
	lowerBoundVertex = &bestVertex;

	return 1;
}

template< class T >
int
NormRRTS<T>
::checkUpdateBestVertex (vertex_t& vertexIn) {
	//ROS_INFO("checkUpdateBestVertex");
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
typename NormRRTS<T>::vertex_t*
NormRRTS < T >
::insertTrajectory (vertex_t& vertexStartIn, trajectory_t& trajectoryIn) {
	//ROS_INFO("insertTrajectory");
	if (trajectoryIn.evaluateCost() < 0 || trajectoryIn.evaluateCost() > DBL_MAX/2){
		//ROS_INFO("Get Trajectory Length Problem");
		exit(0);
	}
	/*
	Level_OF_Risk new_lor = vertexStartIn.lorFromRoot;
	Level_OF_Risk lor_from_parent;
	lor_from_parent.metric_cost = trajectoryIn.evaluateCost();
	lor_from_parent.risk = trajectoryIn.evaluateRisk();
	new_lor += lor_from_parent;
	*/

	//ROS_INFO("insertTrajectory: lor_from_root: %f, lor_from_parent:%f, now_lor: %f", vertexStartIn.lorFromRoot.metric_cost, lor_from_parent.metric_cost, new_lor.metric_cost);

	vertex_t* vertexNew = new vertex_t;
	vertexNew->state = new state_t;
	vertexNew->parent = NULL;

	/*
	vertexNew->lorFromRoot= new_lor;
	vertexNew->lorFromParent = lor_from_parent;
	*/
	trajectoryIn.getEndState(vertexNew->getState());
	insertIntoKdtree (*vertexNew);

	this->listVertices.push_front (vertexNew);
	this->numVertices++;
	insertTrajectory (vertexStartIn, trajectoryIn, *vertexNew);
	//ROS_INFO("insertTrajectory: lor_from_parent: %f", trajectoryIn.evaluateCost());
	return vertexNew;
}

template< class T >
int
NormRRTS < T >
::insertTrajectory (vertex_t& vertexStartIn, trajectory_t& trajectoryIn, vertex_t& vertexEndIn) {
	// Update the trajectory between the two vertices

	if (trajectoryIn.evaluateCost() < 0 || trajectoryIn.evaluateCost() > DBL_MAX/2){
		ROS_INFO("Problem_1");
		exit(0);
	}

	Level_OF_Risk new_lor = vertexStartIn.lorFromRoot;
	Level_OF_Risk lor_from_parent;
	lor_from_parent.metric_cost = trajectoryIn.evaluateCost();
	lor_from_parent.risk = trajectoryIn.evaluateRisk();
	new_lor += lor_from_parent;

	vertexEndIn.lorFromRoot = new_lor;
	vertexEndIn.lorFromParent = lor_from_parent;

	if (vertexEndIn.trajFromParent)
		delete vertexEndIn.trajFromParent;

	vertexEndIn.trajFromParent = new trajectory_t (trajectoryIn);

	//Update the covariance
	state_t end_state = trajectoryIn.getEndState();
	for (int i = 2; i < 5; i++)
		vertexEndIn.state->x[i] = end_state[i];

	// Update the parent to the end vertex
	if (vertexEndIn.parent)
		vertexEndIn.parent->children.erase (&vertexEndIn);
	vertexEndIn.parent = &vertexStartIn;

	state_t *stateIn = vertexStartIn.state;
	// Add the end vertex to the set of chilren
	vertexStartIn.children.insert (&vertexEndIn);
	checkUpdateBestVertex (vertexEndIn);

	return 1;
}

template< class T >
int
NormRRTS < T >
::setSystem (system_t& systemIn) {

	//ROS_INFO("Set System");
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
typename NormRRTS< T >::vertex_t&
NormRRTS< T >
::getRootVertex () {

	return *root;
}

template< class T >
int
NormRRTS< T >
::initialize () {

	//ROS_INFO("Initializw");
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
NormRRTS< T >
::setGamma (double gammaIn) {

  if (gammaIn < 0.0)
    return 0;

  gamma = gammaIn;

  return 1;
}

template< class T >
int
NormRRTS< T >
::setGoalSampleFrequency (double sampleFreqIn) {

  if ( (sampleFreqIn < 0.0) || (sampleFreqIn > 1.0) )
    return 0;

  goalSampleFreq = sampleFreqIn;

  return 1;
}

template< class T >
int
NormRRTS< T >
::setRiskLimit (double risk) {

  if ( (risk < 0.0) || (risk > 1.0) )
    return 0;

  system->risk_limit = risk;

  return 1;
}

template< class T >
bool compareVertexRiskPairs(
		pair< typename NormRRTSNS::NormVertex<T>* , Level_OF_Risk* > lor_i,
		pair< typename NormRRTSNS::NormVertex<T>* , Level_OF_Risk* > lor_j) {
	return ( *(lor_i.second) < *(lor_j.second) );
}

template< class T >
int
NormRRTS< T >
::findBestParent (state_t& stateIn, vector<vertex_t*>& vectorNearVerticesIn,
	vertex_t*& vertexBest, trajectory_t& trajectoryOut, bool& exactConnection, list<float>& controlOut) {

	vector< pair<vertex_t*,Level_OF_Risk*> > vertexRiskPairs;

	int i = 0;
	for (typename vector<vertex_t*>::iterator iter = vectorNearVerticesIn.begin(); iter != vectorNearVerticesIn.end(); iter++){
		vertex_t* v = *iter;
		Level_OF_Risk* candidate_lor = new Level_OF_Risk(v->lorFromRoot);
		trajectory_t temp_traj;
		exactConnection = false;
		Level_OF_Risk increment_lor;

		if (system->extendTo(*(v->state), stateIn, true, true, temp_traj, exactConnection, controlOut) > 0){
			increment_lor.risk = temp_traj.evaluateRisk();
			increment_lor.metric_cost = temp_traj.evaluateCost();
			*candidate_lor += increment_lor;
			//ROS_INFO("FineBestParent: Check cost: %f, traj_cost %f", (*candidate_lor).metric_cost, increment_lor.metric_cost);
			vertexRiskPairs.push_back(make_pair(v, candidate_lor));
		}
	}

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
	return 0;
}

template< class T >
int
NormRRTS< T >
::markCost (vertex_t& vertexIn){
	ROS_INFO("MarkCost");
	vertexIn.lorFromRoot.metric_cost = -2.0;
	for (typename set<vertex_t*>::iterator iter = vertexIn.children.begin(); iter != vertexIn.children.end(); iter++){
		markCost(**iter);
	}
	return 1;
}

template< class T >
int
NormRRTS< T >
::checkTrajectory (vertex_t& vertexIn){
	//ROS_INFO("CheckTrajectory");
	state_t &state_in = vertexIn.getState();
	vertex_t &parent = vertexIn.getParent();
	state_t  &state_par = parent.getState();

	trajectory_t traj;
	list<float> tmp_control;
	bool exactConnection;
	if(system->extendTo(state_par, state_in, true, true, traj, exactConnection, tmp_control) > 0){
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
NormRRTS< T >
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
NormRRTS< T >
::checkTree(){
	if( root->children.size() > 0){

		list<vertex_t*> listSurvivingVertices;
		for (typename list<vertex_t*>::iterator iter = listVertices.begin(); iter != listVertices.end(); iter++){
			vertex_t *vertex  = *iter;
			if( vertex->lorFromRoot.metric_cost > -0.5)
				listSurvivingVertices.push_front(vertex);
			else{
				vertex_t &parent = vertex->getParent();
				parent.children.erase(vertex);
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

		//ROS_INFO("Vertex number, %d", numVertices);

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
NormRRTS< T >
::rewireVertices (vertex_t& vertexNew, vector<vertex_t*>& vectorNearVertices){
	state_t& state_new = *(vertexNew.state);

	for (typename vector<vertex_t*>::iterator iter = vectorNearVertices.begin(); iter != vectorNearVertices.end(); iter++){

		vertex_t& vertex_curr = **iter;

		state_t& state_curr = *(vertex_curr.state);

		bool exactConnection = false;
		trajectory_t trajectory;
		list<float> tmp_control;
		if (system->extendTo(state_new, state_curr, true, true, trajectory, exactConnection, tmp_control) < 0 )
			continue;

		Level_OF_Risk *new_lor = new Level_OF_Risk(vertexNew.lorFromRoot);

		//ROS_INFO("Rewire RRTS Tree: vertexNew: metric_cost %f, risk: %f", new_lor->metric_cost, new_lor->risk );
		//ROS_INFO("Rewire RRTS Tree: vertexCurrent: metric_cost %f, risk: %f", vertex_curr.lorFromRoot.metric_cost, vertex_curr.lorFromRoot.risk );

		Level_OF_Risk incremental_lor;
		incremental_lor.risk = trajectory.evaluateRisk();
		incremental_lor.metric_cost = trajectory.evaluateCost();

		*new_lor += incremental_lor;

		//ROS_INFO("Rewire vertexNew: metric_cost %f, Current metric: %f", new_lor->metric_cost, vertex_curr.lorFromRoot.metric_cost );

		// Check whether the cost of the extension is smaller than current cost
		if(*new_lor < vertex_curr.lorFromRoot){
			//ROS_INFO("Got Rewired");
			insertTrajectory (vertexNew, trajectory, vertex_curr);
			recomputeLor(&vertex_curr);
			recomputeCov(&vertex_curr);
		}
	}
	//checkTree();
	return 1;
}

template< class T >
int
NormRRTS< T >
::recomputeCov (vertex_t* vertexIn) {
	//ROS_INFO("Recompure Cov");
	for (typename set<vertex_t*>::iterator iter = vertexIn->children.begin(); iter != vertexIn->children.end(); iter++){
		vertex_t* vertexCurr = *iter;
		state_t &state_in = vertexCurr->getState();
		vertex_t &parent = vertexCurr->getParent();
		state_t  &state_par = parent.getState();

		vector<double> tmp_risk;
		double *end_state = new double[5];

		double extend = system->extend_line(state_par.x, state_in.x, tmp_risk, end_state);

		if (extend > 0){
			for (int i = 0 ; i < 5;i++)
				state_in.x[i] = end_state[i];
		}
		else{
			ROS_INFO("Cannot find traj: %f", extend);
			//markCost(*vertexCurr);
		}
		delete[] end_state;
		recomputeCov(vertexCurr);
	}
	return 1;
}

template <class T>
int
NormRRTS <T>
::iteration(vector<double> &samples){

	//ROS_INFO("Iteration");
	// 1. Sample a new state
	state_t stateRandom;

	double randGoalSampling = ((double)rand())/(RAND_MAX + 1.0);
	if (randGoalSampling < 0.01 ) {
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

	/*
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
		if (system->extendTo(vertexParent->getState(), stateRandom, true, true, trajectory, exactConnection, control) <= 0)
			return 0;
		return 0;
	}
	else{
	    // 3.b Extend the best parent within the near vertices
		if (findBestParent (stateRandom, vectorNearVertices, vertexParent, trajectory, exactConnection, control) <= 0)
			return 0;
	}

	//TODO: Wield here, the distance to near vertex is larger than the ball radius
	//double near_dist =sqrt((stateRandom[0]-vertexParent->state->x[0])*(stateRandom[0]-vertexParent->state->x[0])
	//	+(stateRandom[1] - vertexParent->state->x[1])*(stateRandom[1] - vertexParent->state->x[1]));

	//cout << "Near Vertex Distance" << near_dist <<endl;

	// 3.c add the trajectory from the best parent to the tree
	vertex_t* vertexNew = insertTrajectory (*vertexParent, trajectory);
	if (vertexNew == NULL)
		return 0;

	// 4. Rewire the tree
	if (vectorNearVertices.size() > 0)
		rewireVertices (*vertexNew, vectorNearVertices);

	 */

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
}

template< class T >
int
NormRRTS< T >
::updateBranchCost (vertex_t& vertexIn, int depth) {
	//ROS_INFO("UpdateBranchCost");
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
NormRRTS< T >
::recomputeLor (vertex_t* vertexIn) {

	//ROS_INFO("RecomputeLor");
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
		recomputeLor(vertexCurr);
	}
	return 1;
}

template< class T >
int
NormRRTS< T >
::getBestTrajectory (list<double*>& trajectoryOut, list<float> &controlOut ) {
	//ROS_INFO("getBestTrajectory");
	if (lowerBoundVertex == NULL){
		return 0;
	}
	vertex_t* vertexCurr = lowerBoundVertex;

	while (vertexCurr) {
		state_t& stateCurr = vertexCurr->getState();
		double *stateArrCurr = new double[5];
		for (int i = 0; i < 5; i++)
			stateArrCurr[i] = stateCurr[i];

		trajectoryOut.push_front (stateArrCurr);

		vertex_t& vertexParent = vertexCurr->getParent();

		if (&vertexParent != NULL) {

			state_t& stateParent = vertexParent.getState();
			list<double*> trajectory;
			list<float> control;
			system->getTrajectory (stateParent, stateCurr, trajectory, control, true, true);

			trajectory.reverse ();
			control.reverse();
			for (list<double*>::iterator iter = trajectory.begin(); iter != trajectory.end(); iter++){
				double *stateArrFromParentCurr = *iter;
				stateArrCurr = new double[5];
				for (int i = 0; i < 5; i++)
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
NormRRTS< T >
::isSafeTrajectory(list<double*>& trajectory){
	for (list<double*>::iterator iter = trajectory.begin(); iter != trajectory.end(); iter++){
		if( system->IsInCollisionLazy(*iter) )
			return false;
	}
	return true;
}

template< class T >
double
NormRRTS< T >
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
#endif /* NormRRTS_H_ */
