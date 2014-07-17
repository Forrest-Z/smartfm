#ifndef CONFIG_H
#define CONFIG_H

#include <string>

using namespace std;

struct Config {
	int search_depth;
	double discount;
	unsigned int root_seed;
	double time_per_move;  // CPU time available to construct the search tree
	int n_particles;
	double pruning_constant;
	double xi; // xi * gap(root) is the target uncertainty at the root.
	int sim_len; // Number of steps to run the simulation for.
	string default_action;
	bool create_mdp_bound; // TODO: remove
	const char* bound_file; // TODO: remove?
	int max_policy_sim_len; // Maximum number of steps for simulating the default policy
	double noise;

	Config() :
		search_depth(20),
		discount(0.95),
		root_seed(42),
		time_per_move(1),
		n_particles(500),
		pruning_constant(0),
		xi(0.95),
		sim_len(40),
		default_action(""),
		create_mdp_bound(false),
		bound_file(NULL),
		max_policy_sim_len(20),
		noise(0)
	{
	}
};

#endif
