#include <iostream>

#include <ctime>

#include <common/globals.h>
#include "../lcmtypes/lcmtypes.h"


using namespace std;

#include "systems/single_integrator.hpp"
#include "rrts.hpp"

typedef SingleIntegrator::StateType state_t;
typedef SingleIntegrator::TrajectoryType trajectory_t;
typedef SingleIntegrator::SystemType system_t;

typedef RRTstar::Vertex <SingleIntegrator> vertex_t;
typedef RRTstar::Planner <SingleIntegrator> planner_t; 


int publishTree (lcm_t *lcm, planner_t& planner, system_t& system);
int publishTraj (lcm_t *lcm, planner_t& planner, system_t& system);


int main () {
  
  
  planner_t rrts;
  
  cout << "RRTstar is alive" << endl;
  

  lcm_t *lcm = globals_get_lcm ();


  system_t system;
  
  system.setNumDimensions (3);
  
  system.regionOperating.setNumDimensions(3);
  system.regionOperating.center[0] = 0.0;
  system.regionOperating.center[1] = 0.0;
  system.regionOperating.center[2] = 0.0;
  system.regionOperating.size[0] = 20.0;
  system.regionOperating.size[1] = 20.0;
  system.regionOperating.size[2] = 10.0;
  
  system.regionGoal.setNumDimensions(3);
  system.regionGoal.center[0] = 2.0;
  system.regionGoal.center[1] = 2.0;
  system.regionGoal.center[2] = 2.0;
  system.regionGoal.size[0] = 2.0;
  system.regionGoal.size[1] = 2.0;
  system.regionGoal.size[2] = 2.0;
  
  
  region *obstacle;
  
  obstacle = new region;
  obstacle->setNumDimensions(3);
  obstacle->center[0] = 0.5;
  obstacle->center[1] = 0.5;
  obstacle->center[2] = 0.5;
  obstacle->size[0] = 0.5;
  obstacle->size[1] = 0.5;
  obstacle->size[2] = 0.5;
  
  // system.obstacles.push_front (obstacle);
  
  
  rrts.setSystem (system);
  
  vertex_t &root = rrts.getRootVertex();  
  state_t &rootState = root.getState();
  rootState[0] = 0.0;
  rootState[1] = 0.0;
  rootState[2] = 0.0;
  
  rrts.initialize ();
  
  rrts.setGamma (1.5);
  rrts.setGoalSampleFrequency (0.1);
  
  clock_t start = clock();
  
  for (int i = 0; i < 10000; i++) 
    rrts.iteration ();
  
  clock_t finish = clock();
  cout << "Time : " << ((double)(finish-start))/CLOCKS_PER_SEC << endl;
  
  //return 1;
  
  publishTree (lcm, rrts, system);
  publishTraj (lcm, rrts, system);
  
  return 1;
}




int publishTraj (lcm_t *lcm, planner_t& planner, system_t& system) {
  
  
  cout << "Publishing trajectory -- start" << endl;
  
  vertex_t& vertexBest = planner.getBestVertex ();

  if (&vertexBest == NULL) {
    cout << "no best vertex" << endl;
    return 0;
  }

  list<double*> stateList;

  planner.getBestTrajectory (stateList);
  
  lcmtypes_opttree_traj_t *opttraj = (lcmtypes_opttree_traj_t *) malloc (sizeof (lcmtypes_opttree_traj_t));
  
  opttraj->num_states = stateList.size();
  opttraj->states = (lcmtypes_optsystem_state_t *) malloc (opttraj->num_states * sizeof (lcmtypes_optsystem_state_t));
  
  int stateIndex = 0;
  for (list<double*>::iterator iter = stateList.begin(); iter != stateList.end(); iter++) {
    
    double* stateRef = *iter;
    opttraj->states[stateIndex].x = stateRef[0];
    opttraj->states[stateIndex].y = stateRef[1];
    if (system.getNumDimensions() > 2)
      opttraj->states[stateIndex].z = stateRef[2];
    else
      opttraj->states[stateIndex].z = 0.0;
    
    delete [] stateRef;

    stateIndex++;
  }
  

  lcmtypes_opttree_traj_t_publish (lcm, "OPTTREE_OPTTRAJ", opttraj);

  lcmtypes_opttree_traj_t_destroy (opttraj);

  cout << "Publishing trajectory -- end" << endl;
  
  return 1;
  
}



int publishTree (lcm_t *lcm, planner_t& planner, system_t& system) {
  
  
  cout << "Publishing the tree -- start" << endl;
  
  
  lcmtypes_opttree_tree_t *tree = (lcmtypes_opttree_tree_t *) malloc (sizeof (lcmtypes_opttree_tree_t));
  tree->num_nodes = planner.numVertices; 
  
  
  if (tree->num_nodes > 0) {    
    
    
    tree->nodes = (lcmtypes_opttree_node_t *) malloc (tree->num_nodes * sizeof(lcmtypes_opttree_node_t));
    tree->traj_from_parent = (lcmtypes_opttree_traj_t *) malloc (tree->num_nodes * sizeof(lcmtypes_opttree_traj_t));

    int nodeIndex = 0;
    for (list<vertex_t*>::iterator iter = planner.listVertices.begin(); iter != planner.listVertices.end(); iter++) {

      
      vertex_t &vertexCurr = **iter;
      state_t &stateCurr = vertexCurr.getState ();
      
      tree->nodes[nodeIndex].nodeid = nodeIndex;
      tree->nodes[nodeIndex].dead_node = false;
      tree->nodes[nodeIndex].state.x = stateCurr[0];
      tree->nodes[nodeIndex].state.y = stateCurr[1];
      if (system.getNumDimensions() > 2) 
	tree->nodes[nodeIndex].state.z = stateCurr[2];
      else 
	tree->nodes[nodeIndex].state.z = 0.0;
      tree->nodes[nodeIndex].distance_from_root = vertexCurr.getCost ();

      tree->traj_from_parent[nodeIndex].num_states = 0;
      tree->traj_from_parent[nodeIndex].states = NULL;
      
      nodeIndex++;
      
    }
    
  }
  else {
    tree->nodes = NULL;
  }
  
  if (tree->num_nodes > 1) {
    tree->num_edges = tree->num_nodes - 1;
    tree->edges = (int32_t **) malloc (tree->num_edges * sizeof(int32_t *));
    tree->traj_edges = (lcmtypes_opttree_traj_t *) malloc (tree->num_edges * sizeof (lcmtypes_opttree_tree_t));
    
    for (int i = 0; i < tree->num_edges; i++) {
      tree->traj_edges[i].num_states = 0;
      tree->traj_edges[i].states = NULL;
    }
    
    int edgeIndex = 0;
    for (list<vertex_t*>::iterator iter = planner.listVertices.begin(); iter != planner.listVertices.end(); iter++) {
      
      vertex_t &vertexCurr = **iter;
      
      if (&(vertexCurr.getParent()) == NULL) 
  	continue;
      
      int parentIndex = 0;
      bool parentFound = false;
      for (list<vertex_t*>::iterator iterParent = planner.listVertices.begin(); 
	   iterParent != planner.listVertices.end(); iterParent++) {
	
  	vertex_t *vertexParentCurr = *iterParent;

  	if ( &(vertexCurr.getParent())  == vertexParentCurr) {
	  
  	  parentFound = true; 
  	  break;
  	}
  	parentIndex++;
      }
      
      if (parentFound == false) {
  	cout << "ERROR: No parent found" << endl; 
      }
      tree->edges[edgeIndex] = (int32_t *) malloc (2 * sizeof(int32_t));
      tree->edges[edgeIndex][0] = edgeIndex;
      tree->edges[edgeIndex][1] = parentIndex;
      
      edgeIndex++;
    }

  }
  else {
    tree->num_edges = 0;
    tree->edges = NULL;
  }

  lcmtypes_opttree_tree_t_publish (lcm, "OPTTREE_TREE", tree);

  lcmtypes_opttree_tree_t_destroy (tree);

  cout << "Publishing the tree -- end" << endl;

  return 1;
}
