/*
 * GraphPF.cpp
 *
 *  Created on: Dec 7, 2012
 *      Author: demian
 */

#include <isam/isam.h>
#include <boost/random/mersenne_twister.hpp>
#include <boost/random/normal_distribution.hpp>
#include <boost/random/variate_generator.hpp>
#include <boost/random/uniform_real.hpp>
#include <boost/random/uniform_int.hpp>
#include <numeric>
struct particle
{
	particle() : node_idx(0){}
	//although node_idx should be an integer, this allows motion output in a continuous 1D system, only then it get discretized into int
	double node_idx;
	double weight;
	//the heading would be just +1 or -1 depending on the polarity
};

class GraphParticleFilter
{
public:
	GraphParticleFilter(vector< vector<double> > scores_array, isam::Slam *slam, int particle_no, int skip_reading):
		slam_(slam), scores_(scores_array), skip_reading_(skip_reading)

	{
		//initialize particles with node_idx as -1
		particles_.resize(particle_no);
	}

	int getCloseloop(int matching_node)
	{
		//because number of nodes will only increment with the time, we will just track the number of node
		nodes_heading_.clear();

		list<isam::Node*> nodes = slam_->get_nodes();

		for(std::list<isam::Node*>::const_iterator it = nodes.begin(); it!=nodes.end(); it++) {
			isam::Node& node = **it;
			nodes_heading_.push_back(node.vector(isam::ESTIMATE)[2]);
		}
		updateMotion();
		int cl_idx = updateWeightAndSampling(matching_node);


		//finally check the modal of the current filter state
		//if number of particles converged into one node
		//return the close loop to back end
		return cl_idx;
	}

private:
	isam::Slam *slam_;
	vector<particle> particles_;
	//using this for fast development
	vector< vector<double> > scores_;
	vector<double> nodes_heading_;
	int skip_reading_;

	//perform motion based on normal distribution

	typedef boost::mt19937                     ENG;    // Mersenne Twister
	typedef boost::normal_distribution<double> NORM_DIST;   // Normal Distribution
	typedef boost::variate_generator<ENG&,NORM_DIST> NORM_GEN;    // Variate generator
	//typedef boost::uni
	ENG  eng;

	void updateMotion()
	{
		//Propagate the particles in a meaningful way
		//this is typically call at each new node is added at
		//fixed travel distance



		for(size_t i=0; i<particles_.size(); i++)
		{
			double ang_dist = fmutil::angDist(nodes_heading_[nodes_heading_.size()-1], nodes_heading_[(int)particles_[i].node_idx]);
			ang_dist = fabs(ang_dist/M_PI); //scaling to 0-1, where a near 0 dist means 2 nodes having the same heading and otherwise
			ang_dist = (1- ang_dist)*2 -1;// scaling to -1 to 1, where -1 means the nodes having opposite direction, and 1 means same heading hence just inc/dec as the symbols

			NORM_DIST dist(ang_dist,0.5); NORM_GEN gen(eng,dist);
			//with mean defined by the relative difference in orientation
			particles_[i].node_idx += gen();
			//let the particles bounded by the available node
			particles_[i].node_idx = fmutil::bound(0., particles_[i].node_idx, (double)nodes_heading_.size()-1);
		}

		cout<<"Updating motion"<<endl;
		std::map<int, int> particle_sum;
		for(size_t i=0; i<particles_.size(); i++)
		{
			++particle_sum[round(particles_[i].node_idx)];
		}

		printParticles(particle_sum);

	}

	int roll_die(int range) {
	    boost::uniform_int<> dist(0, range);
	    boost::variate_generator<boost::mt19937&, boost::uniform_int<> > die(eng, dist);
	    return die();
	}

	int roll_weighted_die(vector<double> &probabilities) {
		std::vector<double> cumulative;

		std::partial_sum(probabilities.begin(), probabilities.end(),
				std::back_inserter(cumulative));
		boost::uniform_real<> dist(0, cumulative.back());
		boost::variate_generator<boost::mt19937&, boost::uniform_real<> > die(eng, dist);

		return (std::lower_bound(cumulative.begin(), cumulative.end(), die()) - cumulative.begin());
	}

	void printParticles(std::map<int, int> particle_sum)
	{
		for(std::map<int, int>::iterator i=particle_sum.begin(); i!=particle_sum.end(); i++)
					 std::cout << std::fixed << std::setprecision(1) << std::setw(2)
					                  << i->first << ' ' << std::string(i->second, '*') << '\n';
	}
	int updateWeightAndSampling(int matching_node)
	{
		//get likelihood from the front end
		//check first if the node is already obtained before to reduce
		//computation

		//put out the particles with quantities of the weight for sampling
		cout<<"Attempting matching with node "<<matching_node/skip_reading_<<endl;
		vector<double> weight_prob;
		for(size_t i=0; i<particles_.size(); i++)
		{
			double score = scores_[matching_node/skip_reading_][round(particles_[i].node_idx)];

			//penalize nearby particles
			if(abs(round(particles_[i].node_idx)*skip_reading_ - matching_node) < 20)
				score /= 2;
			particles_[i].weight = score;
			weight_prob.push_back(score);
		}

		vector<particle> new_particles;

		//retain 80 percent of the particles
		//cout<<"Weight Sampling, selected particles"<<endl;
		for(size_t i=0; i<particles_.size()*0.8; i++)
		{
			int new_particle_idx = roll_weighted_die(weight_prob);
			//cout<< new_particle_idx <<" ("<<round(particles_[new_particle_idx].node_idx)<<":"<<particles_[new_particle_idx].weight<<") ";
			new_particles.push_back(particles_[new_particle_idx]);
		}
		size_t inject_particle_no = particles_.size() - new_particles.size();
		for(size_t i=0; i<inject_particle_no; i++)
		{
			int random_node_idx = roll_die(slam_->get_nodes().size()-1);
			particle part;
			part.node_idx = random_node_idx;
			new_particles.push_back(part);
			//cout<<" ("<<random_node_idx<<"*) ";
		}
		//cout<<endl;

		particles_ = new_particles;
		cout<<"After sampling"<<endl;
		std::map<int, int> particle_sum;
		for(size_t i=0; i<particles_.size(); i++)
		{
			++particle_sum[round(particles_[i].node_idx)];
		}

		printParticles(particle_sum);

		//pass through a sliding window to collect possible close loop

		std::map<int, int> particle_accumulated;
		uint window_size = 2;
		assert(window_size%2==0);

		cout<<"particle_sum size "<<particle_sum.size()<<endl;
		for(int i=0; i<(int)particle_sum.size(); i++)
		{

			map<int, int>::iterator it1 = particle_sum.begin();
			std::advance(it1, i);
			int node_idx = it1->first;
			int start = node_idx-window_size/2;
			int end = node_idx+window_size/2;
			for(int j=start; j<=end; j++)
			{
				if(particle_sum.find(j)!=particle_sum.end())
					particle_accumulated[node_idx] += particle_sum[j];
			}
		}

		int max_accu = 0;
		int max_node_id=0;
		for(std::map<int, int>::iterator i=particle_accumulated.begin(); i!=particle_accumulated.end(); i++)
		{
			if(max_accu < i->second)
			{
				max_accu = i->second;
				max_node_id = i ->first;
			}
			cout << i->first << ':' << i->second<< "("<<scores_[matching_node/skip_reading_][i->first]<<") ";
		}
		cout<<endl;

		int max_neighbor_node_id = -1;
		double max_neighbor_score= 0;

		//search maximum score within the sliding window
		cout<<"max_node_id:"<<max_node_id<<endl;
		if(abs(max_node_id*skip_reading_ - matching_node)>10)
		{
			//fixme: more elegant direct sorting would be great
			for(int i=max_node_id-(int)window_size/2; i<=max_node_id+(int)window_size/2; i++)
			{
				//make sure the node id is present in the particles
				if(particle_sum.find(i) == particle_sum.end()) continue;
				double matching_score = scores_[matching_node/skip_reading_][i];
				if(max_neighbor_score < matching_score)
				{
					max_neighbor_node_id = i;
					max_neighbor_score = matching_score;
				}
			}
		}
		if( max_neighbor_score >50. && max_neighbor_node_id != -1 && max_accu > 45)
		{
				cout<<"close loop found at node "<<max_neighbor_node_id <<" with score "<<max_neighbor_score;
				return max_neighbor_node_id*skip_reading_;
		}
		else return -1;

		//random generation of uniform number by using the size of the particles as the bound
		//boost:
	}

};
