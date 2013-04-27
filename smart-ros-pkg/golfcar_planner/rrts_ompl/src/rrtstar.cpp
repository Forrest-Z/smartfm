/*
 * rrtstar.cpp
 *
 *  Created on: Jan 24, 2013
 *      Author: liuwlz
 */

#include <rrts_ompl/rrtstar.h>

bool debug_flag = false;


sampler_ allocValidSampler(const base::SpaceInformation *si){
	return sampler_(new configuredStateSampler(si));
}


rrts::rrts(){
	space = new space_(new base::DubinsStateSpace(1.0, false));
    simplesetup = new simplesetup_((*space));
    spaceinfo = new spaceinfo_(simplesetup->getSpaceInformation());
	root = new state_((*space));
	goal = new state_((*space));

	first_root = true;
	first_goal = true;

	sampler_type = false;
}

rrts::~rrts(){
}

//TODO:Check validality of state based on: (1)satisfy the bounds; (2) satify the obstacle avoidance;
//(3): satisfy the kinematic constrains etc later.
bool checkStateValid(const base::SpaceInformation *si, const base::State *state){
	const base::DubinsStateSpace::StateType *s = state->as<base::DubinsStateSpace::StateType>();
	double x=s->getX(), y=s->getY();
	return si->satisfiesBounds(s);
}

int rrts::initplanner(int width, int height){
	try{
		setBound(width ,height);
	    setValidChecker();
	}
	catch(Exception &e){
		return 0;
	}
	return 1;
}

int rrts::setBound(int width, int height){
	bound_ bounds(2);
    bounds.setLow(0,0);
    bounds.setHigh(0,width);
    bounds.setLow(1,0);
    bounds.setHigh(1,height);
    (*space)->as<base::SE2StateSpace>()->setBounds(bounds);
	return 1;
}

int rrts::setValidChecker(){
	simplesetup->setStateValidityChecker(boost::bind(&checkStateValid, spaceinfo->get(), _1));
    simplesetup->getSpaceInformation()->setStateValidityCheckingResolution(1.0);
    return 1;
}

int rrts::setRoot(double _root[]){
	/*A little tricky here. The root and goal are defined as pointer of type ScpoedState*,
	 * root->get() will return the pointer of type State*;
	 * Or can use (*root) instead of root->get() to return the type ScpoedState
	 */
	(*root)[0] = _root[0];
	(*root)[1] = _root[1];
	(*root)[2] = _root[2];
	if(first_root){
		simplesetup->addStartState(*root);
		first_root = false;
	}
	else
		simplesetup->setStartState(*root);
	return 1;
}

int rrts::setGoal(double _goal[]){
	(*goal)[0] = _goal[0];
	(*goal)[1] = _goal[1];
	(*goal)[2] = _goal[2];
	simplesetup->setGoalState(*goal);
	return 1;
}

int rrts::planning(vector<double> &path_reals){

	planner_ rrts_planner(new geometric::RRTstar(simplesetup->getSpaceInformation()));
    simplesetup->setPlanner(rrts_planner);
    if (sampler_type)
    	simplesetup->getSpaceInformation()->setValidStateSamplerAllocator(allocValidSampler);
    simplesetup->setup();
    if (debug_flag)
    	simplesetup->print();

    // attempt to solve the problem within 5 seconds of planning time
    if (simplesetup->solve(50.0) == base::PlannerStatus::EXACT_SOLUTION){
        std::cout << "Solution Found" << std::endl;
        path_ path =  simplesetup->getSolutionPath();
        geometric::PathSimplifierPtr pathsimpler = simplesetup->getPathSimplifier();
        pathsimpler->reduceVertices(path);
        pathsimpler->collapseCloseVertices(path);
        path.interpolate(1000);

        std::ofstream file;
        file.open("path.txt");
        for (unsigned int i=0; i < path.getStateCount(); ++i){
            path_reals = state_((*space), path.getState(i)).reals();
            file << "path	" << path_reals[0] <<'	'<< path_reals[1] << '	' << path_reals[2] << endl;
        }
        file.close();
    }
    else
        std::cout << "No solution found" << std::endl;
	return 1;
}

int rrts::visualize(vector<rrts_ompl::rrts_edge> &rrts_tree){
    //Output planner data

	plan_data = new planner_date_(simplesetup->getSpaceInformation());
	simplesetup->getPlannerData(*plan_data);

	if (debug_flag)
		cout << "Number of vertex"<<plan_data->numVertices() << endl;

	std::ofstream file;
    file.open("vertex.txt");

    for (unsigned int i =0 ; i < plan_data->numVertices(); i++){

    	rrts_ompl::rrts_edge rrts_vertex;

		base::PlannerDataVertex plan_vertex(plan_data->getVertex(i));
		const base::DubinsStateSpace::StateType *s = plan_vertex.getState()->as<base::DubinsStateSpace::StateType>();
		int vertex_edge_num = plan_data->getEdges(plan_data->vertexIndex(plan_vertex), rrts_vertex.edge_list);
		rrts_vertex.vertex_value.push_back(s->getX());
		rrts_vertex.vertex_value.push_back(s->getY());
		rrts_vertex.vertex_value.push_back(s->getYaw());
		rrts_vertex.vertex_value.push_back(plan_vertex.getTag());
		rrts_vertex.vertex_value.push_back(vertex_edge_num);

		rrts_tree.push_back(rrts_vertex);

		file<<"vertex	"<< rrts_vertex.vertex_value[0] << "	"<< rrts_vertex.vertex_value[1] << endl;
	}
    file.close();
	return 1;
}
