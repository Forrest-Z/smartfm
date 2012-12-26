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
	//the heading would be just +1 or -1 depending on the polarity
};

class GraphParticleFilter
{
public:
	GraphParticleFilter(vector< vector<double> > &scores_array, vector<vector<double> > &rotations, isam::Slam *slam, vector<sensor_msgs::PointCloud> *pc_vec, int particle_no, int skip_reading):
		slam_(slam), pc_vec_(pc_vec), scores_(scores_array), rotations_(rotations), skip_reading_(skip_reading)

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
			nodes_heading_.push_back(node.vector(isam::ESTIMATE)[3]);
		}
		fmutil::Stopwatch sw_motion("updateMotion", true);
		updateMotion();
		sw_motion.end();

		fmutil::Stopwatch sw_sample("updateWeightAndSampling", true); //~100 ms cause by RasterMap
		int cl_idx = updateWeightAndSampling(matching_node);
		sw_sample.end();


		//finally check the modal of the current filter state
		//if number of particles converged into one node
		//return the close loop to back end
		return cl_idx;
	}

private:
	isam::Slam *slam_;

	vector<sensor_msgs::PointCloud> *pc_vec_;
	vector<particle> particles_;
	//using this for fast development
	vector< vector<double> > scores_;
	vector<vector<double> > rotations_;
	vector<double> nodes_heading_;
	int skip_reading_;

	//perform motion based on normal distribution

	typedef boost::mt19937                     ENG;    // Mersenne Twister
	typedef boost::normal_distribution<double> NORM_DIST;   // Normal Distribution
	typedef boost::variate_generator<ENG&,NORM_DIST> NORM_GEN;    // Variate generator
	//typedef boost::uni
	ENG  eng;
	map<int, double> nodes_match_heading_;
	void updateMotion()
	{
		//Propagate the particles in a meaningful way
		//this is typically call at each new node is added at
		//fixed travel distance



		for(size_t i=0; i<particles_.size(); i++)
		{
			double ang_dist = M_PI;
			//use matched result instead for more robust heading value. If no match found, default to same heading
			if(nodes_match_heading_.find((int)particles_[i].node_idx)!=nodes_match_heading_.end())
			  ang_dist = nodes_match_heading_[(int)particles_[i].node_idx];//fmutil::angDist(nodes_heading_[nodes_heading_.size()-1], nodes_heading_[(int)particles_[i].node_idx]);
			ang_dist = fabs(ang_dist/M_PI); //scaling to 0-1, where a near 0 dist means 2 nodes having the same heading and otherwise
			ang_dist = (1- ang_dist)*2 -1;// scaling to -1 to 1, where -1 means the nodes having opposite direction, and 1 means same heading hence just inc/dec as the symbols

			NORM_DIST dist(ang_dist,0.5); NORM_GEN gen(eng,dist);
			//with mean defined by the relative difference in orientation
			particles_[i].node_idx += gen();
			//let the particles bounded by the available node
			particles_[i].node_idx = fmutil::bound(0., particles_[i].node_idx, (double)nodes_heading_.size()-1);
		}

		//cout<<"Updating motion"<<endl;
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
	{/*
		for(std::map<int, int>::iterator i=particle_sum.begin(); i!=particle_sum.end(); i++)
					 std::cout << std::fixed << std::setprecision(1) << std::setw(2)
					                  << i->first << ' ' << std::string(i->second, '*') << '\n';*/
	}
	int updateWeightAndSampling(int matching_node)
	{
		cout<<"updateWeightAndSampling: "<<matching_node<<endl;
		//get likelihood from the front end
		//check first if the node is already obtained before to reduce
		//computation

		//put out the particles with quantities of the weight for sampling
		//cout<<"Attempting matching with node "<<matching_node/skip_reading_<<endl;
		fmutil::Stopwatch sw1("sw1");
		vector<double> weight_prob;
		weight_prob.resize(particles_.size());
		//RasterMapPCL rmpcl;
		//rmpcl.setInputPts((*pc_vec_)[matching_node].points);
		map<int, int> unique_nodes;
		sw1.end();
		//find unique nodes
		for(size_t i=0; i<particles_.size(); i++)
			++unique_nodes[round(particles_[i].node_idx)];

		//fill in the unique nodes with locally cached score
		map<int,double> local_cached_score, local_cached_weight;
		//local_cached_score.size(score_cache.size());
		cout<<"Unique nodes size="<<unique_nodes.size()<<endl;
		vector<int> unique_nodes_array;
		for(std::map<int, int>::iterator i=unique_nodes.begin(); i!=unique_nodes.end(); i++)
		{
			std::cout << i->first << ':' << i->second<<" ";
			unique_nodes_array.push_back(i->first);
		}
		cout<<endl;

		fmutil::Stopwatch sw2("sw2");

		nodes_match_heading_.clear();
//#pragma omp parallel for
		for(size_t i=0; i<unique_nodes_array.size(); i++)
		{
			double score;
			//was using unique_nodes by directly advancing the iterator using STL, does not work

			//reminder: pc_vec_ has the complete data

/*			transform_info best_tf = rmpcl.getBestTf((*pc_vec_)[unique_nodes_array[i]*skip_reading_]);
#pragma omp critical
			nodes_match_heading_[unique_nodes_array[i]] = best_tf.rotation;
			RasterMapPCL rmpcl_ver;
			rmpcl_ver.setInputPts(best_tf.real_pts, true);
			double temp_score = rmpcl_ver.getScore((*pc_vec_)[matching_node].points);
			score = sqrt( temp_score * best_tf.score);
			*/score = scores_[matching_node/skip_reading_][unique_nodes_array[i]];
			nodes_match_heading_[unique_nodes_array[i]] = rotations_[matching_node/skip_reading_][unique_nodes_array[i]];
			if(score == 0.01) cout<<"Error, unexpected matching of nodes being evaluated: "
						<<matching_node/skip_reading_<<":"<<unique_nodes_array[i]<<endl;
			//cout<<matching_node/skip_reading_<<"-"<<unique_nodes_array[i]<<":"<<score<<"->"<<score2<<" ";
			//penalize nearby particles
			//todo:
			//reduce penalize and look for peak to account for multiple possible close loop
			//find out why there is a significant drift in the raw sensor data
			double dist = fabs(unique_nodes_array[i] - matching_node/skip_reading_);// < 10)
			double weight_score;
			if((int)dist == 1) weight_score = score/5;
			else weight_score = score - 40*exp(-dist*0.07);
			if(weight_score < 0) weight_score = 0;
			local_cached_score[unique_nodes_array[i]] = score;
			local_cached_weight[unique_nodes_array[i]] =weight_score;
		}
		sw2.end();
		//cout<<endl;

		fmutil::Stopwatch sw3("sw3");
		//layout the array weight probabilities with locally cached scores
		for(size_t i=0; i<particles_.size(); i++)
		{
			//cout<<round(particles_[i].node_idx)<<endl;
			weight_prob[i] = local_cached_weight[round(particles_[i].node_idx)];
		}

		vector<particle> new_particles;

		//retain 80 percent of the particles
		//cout<<"Weight Sampling, selected particles"<<endl;
		for(size_t i=0; i<particles_.size()*0.8; i++)
		{
			int new_particle_idx = roll_weighted_die(weight_prob);
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
		//cout<<"After sampling"<<endl;
		std::map<int, int> particle_sum;
		for(size_t i=0; i<particles_.size(); i++)
		{
			++particle_sum[round(particles_[i].node_idx)];
		}

		printParticles(particle_sum);
		sw3.end();
		//pass through a sliding window to collect possible close loop
		fmutil::Stopwatch sw4("sw4");
		std::map<int, int> particle_accumulated;
		uint window_size = 2;
		assert(window_size%2==0);

		//cout<<"particle_sum size "<<particle_sum.size()<<endl;
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
			cout << i->first << ':' << i->second<< "("<<local_cached_score[i->first]<<", "<<local_cached_weight[i->first]<<") ";
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
				double matching_score = local_cached_score[i];
				if(max_neighbor_score < matching_score)
				{
					max_neighbor_node_id = i;
					max_neighbor_score = matching_score;
				}
			}
		}
		sw4.end();
		//just a hack for not to activate a close loop when the graph is too small
		if( matching_node < 100)
				return -1;

		if( max_neighbor_score >40. && max_neighbor_node_id != -1 && max_accu > 20)
		{
				cout<<"close loop found at node "<<max_neighbor_node_id <<" with score "<<max_neighbor_score<<endl;
				return max_neighbor_node_id*skip_reading_;
		}
		else return -1;

		//random generation of uniform number by using the size of the particles as the bound
		//boost:
	}

};
