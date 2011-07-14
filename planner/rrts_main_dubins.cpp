#include <iostream>

#include <ctime>


using namespace std;

#include "systems/dubins_car.hpp"
#include "rrts.hpp"

typedef DubinsCar::StateType state_t;
typedef DubinsCar::TrajectoryType trajectory_t;
typedef DubinsCar::SystemType system_t;

typedef RRTstar::Vertex <DubinsCar> vertex_t;
typedef RRTstar::Planner <DubinsCar> planner_t; 



int main () {
  
  
  planner_t rrts;
  
  cout << "RRTstar is alive" << endl;
  
  

  // Initialize the system
  //   : Here we initialize the operating region and the obstacles
  //     as a list. However, the System class code can be modified 
  //     easily to change the obstacle representation to a cost map
  
  system_t system;
    
  system.regionOperating.center[0] = 30.0;
  system.regionOperating.center[1] = 0.0;
  system.regionOperating.center[2] = 0.0;
  system.regionOperating.size[0] = 200.0;
  system.regionOperating.size[1] = 20.0;
  system.regionOperating.size[2] = 2.0 * M_PI;
  
  system.regionGoal.center[0] = 100.0;
  system.regionGoal.center[1] = 0.0;
  system.regionGoal.center[2] = 0.0;
  system.regionGoal.size[0] = 2.0;
  system.regionGoal.size[1] = 2.0;
  system.regionGoal.size[2] = 0.1 * M_PI;

  region *obstacle;
  obstacle = new region;
  obstacle->center[0] = 12.0;
  obstacle->center[1] = 0.0;
  obstacle->center[2] = 0.0;
  obstacle->size[0] = 8.0;
  obstacle->size[1] = 5.0;
  obstacle->size[2] = 0.0;
  system.obstacles.push_front (obstacle);
  
  // Tell the planner the system class
  rrts.setSystem (system);

  // Create the root vertex
  vertex_t &root = rrts.getRootVertex();  
  state_t &rootState = root.getState();
  rootState[0] = 0.0;
  rootState[1] = 0.0;
  rootState[2] = 0.0;

  // Initialize the planner
  rrts.initialize ();
  
  // Set planner parameters
  rrts.setGamma (2.0);
  rrts.setGoalSampleFrequency (0.1);



  // Run the planner until the goal region is reached 
  clock_t start = clock();
  clock_t timePrev = clock ();

  int iterationCounter = 0;
  while (1) {
    
    clock_t timeCurr = clock();
    if (((double)(timeCurr-timePrev))/CLOCKS_PER_SEC < 1.5)  {
      if (iterationCounter%100 == 0) {
		  cout << "Cost : " << rrts.getBestVertexCost() << endl;
		  //publishTraj (lcm, rrts, system);
      }
      rrts.iteration ();
      iterationCounter++;
      
    }
    else {
    
      if (rrts.switchRoot (5.0) <= 0)
	break;
      timePrev = clock ();
    }

  }

  clock_t finish = clock();
  cout << "Time : " << ((double)(finish-start))/CLOCKS_PER_SEC << endl;


  return 1;
}


#if 0

int publishTraj (lcm_t *lcm, planner_t& planner, system_t& system) {

  vertex_t& vertexBest = planner.getBestVertex ();

  if (&vertexBest == NULL)
    return 0;

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
    opttraj->states[stateIndex].z = 0.0;

    delete [] stateRef;

    stateIndex++;
  }
  

  lcmtypes_opttree_traj_t_publish (lcm, "OPTTREE_OPTTRAJ", opttraj);

  lcmtypes_opttree_traj_t_destroy (opttraj);
  
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
      tree->nodes[nodeIndex].state.z = 0.0;
      tree->nodes[nodeIndex].distance_from_root = vertexCurr.getCost ();

            
      vertex_t& vertexParent = vertexCurr.getParent();
      if (&vertexParent != NULL) {
	state_t& stateParent = vertexParent.getState();
	list<double*> trajectory;
	if (system.getTrajectory (stateParent, stateCurr, trajectory) == 0) {
	  cout << "ERROR: Trajectory can not be regenerated" << endl;
	  return 0;
	}
	
	tree->traj_from_parent[nodeIndex].num_states = trajectory.size();
	if (tree->traj_from_parent[nodeIndex].num_states) {
	  tree->traj_from_parent[nodeIndex].states = (lcmtypes_optsystem_state_t *) 
	    malloc (tree->traj_from_parent[nodeIndex].num_states * sizeof (lcmtypes_optsystem_state_t));
	  int stateIndex = 0;
	  for (list<double*>::iterator it_state = trajectory.begin(); it_state != trajectory.end(); it_state++) {
	    double *stateCurr = *it_state;
	    tree->traj_from_parent[nodeIndex].states[stateIndex].x = stateCurr[0];
	    tree->traj_from_parent[nodeIndex].states[stateIndex].y = stateCurr[1];
	    tree->traj_from_parent[nodeIndex].states[stateIndex].z = 0.0;
	    stateIndex++;
	    delete [] stateCurr;
	  }
	}
	else 
	  tree->traj_from_parent[nodeIndex].states = NULL;
      }
      else {
	tree->traj_from_parent[nodeIndex].num_states = 0;
	tree->traj_from_parent[nodeIndex].states = NULL;
      }
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


#endif 
