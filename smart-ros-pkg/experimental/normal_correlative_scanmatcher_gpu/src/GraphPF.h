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
#include "boost/tuple/tuple.hpp"
#include <csm_cuda/csm.h>

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
	map<int, int> falseCL_node_;
	MySQLHelper *mysql_;
        map<int, int> unique_nodes_;
        map<int, geometry_msgs::Point> nodes_pose_;
	ros::Publisher source_pub_, match_pub_, matched_pub_;
	CsmGPU<pcl::PointNormal> *cm_;
	GraphParticleFilter(int particle_no, int skip_reading, string frontend_file):
		skip_reading_(skip_reading), frontend_file_(frontend_file)

	{
	  //unsigned int seed = static_cast<unsigned int>(std::time(0));
	  //eng.seed(seed);
	  //cout<<seed<<" seed is used"<<endl;
		//initialize particles with node_idx as -1
		ros::NodeHandle nh;
		source_pub_ = nh.advertise<pcl::PointCloud<pcl::PointNormal> >("gf_source_cloud", 1, true);
		match_pub_ = nh.advertise<pcl::PointCloud<pcl::PointNormal> >("gf_match_cloud", 1, true);
		matched_pub_ = nh.advertise<pcl::PointCloud<pcl::PointNormal> >("gf_matched_cloud", 1, true);
		particles_.resize(particle_no);
	}
	
	void probNormalize(vector<double> &prob){
	  double total = 0.0;
	  for(size_t i=0; i<prob.size(); i++){
	    total+=prob[i];
	    cout<<total<<": "<<prob[i]<<" ";
	  }
	  if(total > 1e-5){
	    for(size_t i=0; i<prob.size(); i++)
	      prob[i]/=total;
	  }
	}
	int getCloseloop(isam::Slam *slam, int matching_node)
	{
		//because number of nodes will only increment with the time, we will just track the number of node
		nodes_heading_.clear();

		list<isam::Node*> nodes = slam->get_nodes();
		for(std::list<isam::Node*>::const_iterator it = nodes.begin(); it!=nodes.end(); it++) {
			isam::Node& node = **it;
			nodes_heading_.push_back((double)node.vector(isam::ESTIMATE)[3]);
		}
		fmutil::Stopwatch sw_motion("updateMotion", true);
		updateMotion();
		sw_motion.end();

		fmutil::Stopwatch sw_sample("updateWeightAndSampling", true); //~100 ms cause by RasterMap
		int cl_idx = updateWeightAndSampling(slam, matching_node);
		sw_sample.end();


		//finally check the modal of the current filter state
		//if number of particles converged into one node
		//return the close loop to back end
		return cl_idx;
	}
	ScoreData sd_;
private:
	  template<typename T>
	inline boost::tuples::tuple<T,T,T> matrixToYawPitchRoll(const Eigen::Matrix<T,3,3>& r)
	{
	  Eigen::Matrix<T,3,1> euler = r.eulerAngles(2, 1, 0);
	  return boost::tuples::make_tuple(euler(0,0), euler(1,0), euler(2,0));
	}

	vector<vector<sensor_msgs::PointCloud> > pc_vecs_;
	vector<particle> particles_;
	//using this for fast development
	//vector< vector<double> > scores_;
	//vector<vector<double> > rotations_;
	vector<double> nodes_heading_;
	int skip_reading_;
	string frontend_file_;
	//perform motion based on normal distribution

	typedef boost::mt19937                     ENG;    // Mersenne Twister
	typedef boost::normal_distribution<double> NORM_DIST;   // Normal Distribution
	typedef boost::variate_generator<ENG&,NORM_DIST> NORM_GEN;    // Variate generator
	//typedef boost::uni
	ENG  eng;
	map<int, double> nodes_match_heading_;
	map<int, pcl::PointCloud<pcl::PointNormal> > matching_clouds_;
	
	void updateMotion()
	{
		//Propagate the particles in a meaningful way
		//this is typically call at each new node is added at
		//fixed travel distance



		for(size_t i=0; i<particles_.size(); i++)
		{
			double ang_dist = M_PI;
			//use matched result instead for more robust heading value. If no match found, default to reversed heading
			if(nodes_match_heading_.find((int)particles_[i].node_idx)!=nodes_match_heading_.end())
			  ang_dist = nodes_match_heading_[(int)particles_[i].node_idx];//fmutil::angDist(nodes_heading_[nodes_heading_.size()-1], nodes_heading_[(int)particles_[i].node_idx]);
			ang_dist = fabs(ang_dist/M_PI); //scaling to 0-1, where a near 0 dist means 2 nodes having the same heading and otherwise
			ang_dist = (1- ang_dist)*2 -1;// scaling to -1 to 1, where -1 means the nodes having opposite direction, and 1 means same heading hence just inc/dec as the symbols
			int direction = 1, move_dist = 1;
			if(ang_dist<0) direction = -1;
			int p_motion = roll_die(10);
			switch (p_motion) {
			  case 8:
			    move_dist = 0;
			    break;
			  case 9:
			    move_dist = 2;
			    break;
			  case 10:
			    move_dist = 3;
			    break;
			}
			particles_[i].node_idx += move_dist * direction;
			NORM_DIST dist(ang_dist,0.5); NORM_GEN gen(eng,dist);
			//with mean defined by the relative difference in orientation
			//particles_[i].node_idx += gen();
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

	int roll_weighted_die(vector<double> &probabilities, bool debug=false) {
		std::vector<double> cumulative;
		if(debug){
		  cout<<"*****Start rolling the die*****"<<endl;
		  for(size_t i=0; i<probabilities.size(); i++){
		    cout<<probabilities[i]<<" ";
		  }
		  cout<<endl;
		  cout<<"**********End display***********"<<endl;
		}
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
	int updateWeightAndSampling(isam::Slam *slam, int matching_node)
	{
		cout<<"updateWeightAndSampling: "<<matching_node<<endl;
		//get likelihood from the front end
		//check first if the node is already obtained before to reduce
		//computation
    bool print_sw = true;
		//put out the particles with quantities of the weight for sampling
		//cout<<"Attempting matching with node "<<matching_node/skip_reading_<<endl;
		fmutil::Stopwatch sw1("sw1");
		vector<double> weight_prob;
		weight_prob.resize(particles_.size());
		/*RasterMapPCL rmpcl;
    vector<sensor_msgs::PointCloud> matching_pcs;
    for(int i=0; i<3; i++) matching_pcs.push_back(pc_vecs_[i][matching_node]);
		rmpcl.setInputPts(matching_pcs);
		*/map<int, int> unique_nodes;
		sw1.end(print_sw);
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
                unique_nodes_ = unique_nodes;
		fmutil::Stopwatch sw2("sw2");

		nodes_match_heading_.clear();
		map<int, ScoreData> scores;
		
		double res_ = 0.1;
		string frontend_file = frontend_file_;
		int startfile_idx= frontend_file.find_last_of("/")+1;
		string folder = frontend_file.substr(0, startfile_idx);
		stringstream input_file;
		input_file <<folder<<setfill('0')<<setw(5)<<matching_node*skip_reading_<<".pcd";    
		pcl::PointCloud<pcl::PointNormal> input_cloud;
		pcl::io::loadPCDFile(input_file.str(), input_cloud);
		cout<<"Load pcd file: "<<input_file.str()<<endl;
		fmutil::Stopwatch ncm1("NCM initialize");
		matching_clouds_[matching_node*skip_reading_] = input_cloud;
		
		vector<pcl::PointCloud<pcl::PointNormal> > input_clouds = append_input_cloud(input_cloud, frontend_file, input_file.str());
		//input_cloud = pcl_downsample(input_cloud, res_*2, res_*2, res_*2);
		cout<<"Src cloud "<<matching_node*skip_reading_<<" size="<< input_cloud.points.size()<<endl;
		size_t free_byte ;
		size_t total_byte ;
		cudaMemGetInfo( &free_byte, &total_byte ) ;
		cout<<"Cuda usage before: remaining "<<free_byte/1024000.0<<"MiB out of "<<total_byte/1024000.0<<"MiB"<<endl;
		cm_ = new CsmGPU<pcl::PointNormal>(0.1, cv::Point2d(50.0, 75.0), input_cloud, false);
		ncm1.end();
		fmutil::Stopwatch ncm2("NCM bruteForceSearch");
		fmutil::Stopwatch ncm3("NCM veriScore");
		input_cloud.header.stamp = ros::Time::now();
		input_cloud.header.frame_id = "map";
		//source_pub_.publish(input_cloud);
		
		//#pragma omp parallel for
		for(size_t i=0; i<unique_nodes_array.size(); i++)
		{
			//was using unique_nodes by directly advancing the iterator using STL, does not work

			//reminder: pc_vec_ has the complete data
			ScoreData sd;
			sd.node_dst = matching_node*skip_reading_;
			sd.node_src = unique_nodes_array[i]*skip_reading_;
			int j = sd.node_src;
			//added the second OR to ensure node too near not going to be evaluated
			if(matching_clouds_.find(j) == matching_clouds_.end() || abs(unique_nodes_array[i] - matching_node) < 20) {
			  sd.score = 0.;
			  
			}
			else {
			  pcl::PointCloud<pcl::PointNormal> matching_cloud;
			  //#pragma omp critical
			  matching_cloud = matching_clouds_[j];
			  matching_cloud.header.stamp = ros::Time::now();
			  matching_cloud.header.frame_id = "map";
			  ncm2.start();
			  poseResult best_match;
			  best_match.x = best_match.y = best_match.r = 0.0;
			  //match_pub_.publish(matching_cloud);
			  ros::spinOnce();
// 			  cout<<"Matching cloud "<<j<<" size="<< matching_cloud.points.size()<<endl;
			  
		
		//why not the same result? The input and matching cloud appears to be the same...
			  //Remember to add PCL_NORMAL compiler flags!!!
			  best_match = cm_->getBestMatch(2.0, 2.0, 5, 20, 20, 180, matching_cloud, best_match);
// 	  cout<<"Best score 1 found: "<< best_match.x<<", "<<best_match.y<<", "<<best_match.r<<": "<<best_match.score<<endl;
			  best_match = cm_->getBestMatch(0.5, 0.5, 2, 2, 2, 10, matching_cloud, best_match);
// 	  cout<<"Best score 2 found: "<< best_match.x<<", "<<best_match.y<<", "<<best_match.r<<": "<<best_match.score<<endl;
			  best_match = cm_->getBestMatch(0.1, 0.1, 0.5, 0.5, 0.5, 4.0, matching_cloud, best_match);
// 	  cout<<"Best score 3 found: "<< best_match.x<<", "<<best_match.y<<", "<<best_match.r<<": "<<best_match.score<<endl;
			  ncm2.end(false);
			  sd.score = best_match.score;
// 			  cout<<"i x j "<<sd.node_dst<<" x "<<j<<" score= "<<sd.score<<endl;
			  sd.x = best_match.x;
			  sd.y = best_match.y;
			  sd.t = best_match.r;
			  sd.score_ver = sd.score;
			  //sd.score = 100;
			  //std::cout << "Press ENTER to continue...";
			  //std::cin.ignore( std::numeric_limits <std::streamsize> ::max(), '\n' );

			}
			//#pragma omp critical
			  scores[unique_nodes_array[i]] = sd;
		}
		delete cm_;
		cudaMemGetInfo( &free_byte, &total_byte ) ;
		cout<<"Cuda usage after: remaining "<<free_byte/1024000.0<<"MiB out of "<<total_byte/1024000.0<<"MiB"<<endl;
	      
		/*
		for(size_t i=0; i<unique_nodes_array.size(); i++)
		{
		  ScoreData sd;
		  sd.node_dst = matching_node*skip_reading_;
		  sd.node_src = unique_nodes_array[i]*skip_reading_;
		  double score;
		  
		  bool data_retrieved = mysql_->getData(sd);
		  
		  if(data_retrieved) {
		    //changed to arg_min from product
		      score =  sd.score;
		      if(score > sd.score_ver) score = sd.score_ver;
		      score *= 100;
		  }
		  else score = 0.;
		
		  sd.score = score;
		  scores[unique_nodes_array[i]] = sd;
			
		}*/
		
		sw2.end(print_sw);
		fmutil::Stopwatch sw2a("sw2a");
		for(size_t i=0; i<unique_nodes_array.size(); i++)
		{
			
      
			//penalize the falseCL node
			if(falseCL_node_.find(unique_nodes_array[i]) != falseCL_node_.end())
			{
				//score/= falseCL_node_[unique_nodes_array[i]];
			}
			
			nodes_match_heading_[unique_nodes_array[i]] = scores[unique_nodes_array[i]].t / 180. * M_PI;
			if(scores[unique_nodes_array[i]].score == 0.01) cout<<"Error, unexpected matching of nodes being evaluated: "
						<<matching_node<<":"<<unique_nodes_array[i]<<endl;
			//cout<<matching_node/skip_reading_<<"-"<<unique_nodes_array[i]<<":"<<score<<"->"<<score2<<" ";
			//penalize nearby particles

			//reduce penalize and look for peak to account for multiple possible close loop - done!

			//find out why there is a significant drift in the raw sensor data - done, pitch value incorrect, set to zero for now
			double dist = fabs(unique_nodes_array[i] - matching_node);// < 10)
			double weight_score;//10	0.3
			double a_t = 10;//5
			double b_t = 0.3;//0.7
			//weight_score = score / (a_t*exp(-dist*b_t)+1);
			//if(weight_score < 0) weight_score = 0;
			
			local_cached_score[unique_nodes_array[i]] = scores[unique_nodes_array[i]].score;
			
			local_cached_weight[unique_nodes_array[i]] =scores[unique_nodes_array[i]].score;//weight_score;
		}
		
		sw2a.end(print_sw);
		//cout<<endl;

		fmutil::Stopwatch sw3("sw3");
		//layout the array weight probabilities with locally cached scores
		for(size_t i=0; i<particles_.size(); i++)
		{
// 			cout<<round(particles_[i].node_idx)<<endl;
			weight_prob[i] = local_cached_weight[round(particles_[i].node_idx)];
		}

		vector<particle> new_particles;

		//retain 80 percent of the particles
		//cout<<"Weight Sampling, selected particles"<<endl;
		
		//normalize
		probNormalize(weight_prob);
		for(size_t i=0; i<weight_prob.size(); i++)
		    cout<<i<<":"<<weight_prob[i]<<" ";
                cout<<endl;
		for(size_t i=0; i<particles_.size()*0.6; i++)
		{
			int new_particle_idx = roll_weighted_die(weight_prob);
			new_particles.push_back(particles_[new_particle_idx]);
		}
		size_t inject_particle_no = particles_.size() - new_particles.size();
                
                //get the distribution of node according to distance and node idx
                geometry_msgs::Point latest_pose = nodes_pose_[matching_node-1];
                vector<double> weighted_node_distance;
//                 cout<<"Weighted dist with xy: "<<latest_pose.x<<"|"<<latest_pose.y<<endl;
		int node_offset = 20; //was 10 with skip = 2
                for(int i=0; i<matching_node-node_offset; i++) {
                  if(nodes_pose_.find(i) == nodes_pose_.end()) {
                    weighted_node_distance.push_back(1.0/sqrt(latest_pose.x*latest_pose.x + latest_pose.y*latest_pose.y));
                    continue;
                  }
                  
                  double dist_x = nodes_pose_[i].x - latest_pose.x;
                  double dist_y = nodes_pose_[i].y - latest_pose.y;
                  double dist = sqrt(dist_x*dist_x + dist_y*dist_y);
		  double dist_score = exp(-0.1*dist);
                  weighted_node_distance.push_back(dist_score);
                }
                probNormalize(weighted_node_distance);
		for(size_t i=0; i<weighted_node_distance.size(); i++)
		    cout<<i<<":"<<weighted_node_distance[i]<<" ";
                cout<<endl;
                cout<<"Random selection: "<<endl;
		for(size_t i=0; i<inject_particle_no; i++)
		{
		  bool debug = false;
		  if(i==0) debug = false;
			//int random_node_idx = 
			int random_node_idx; 
			int node_number = slam->get_nodes().size()-1;
			//added to fix boost::uniform_real<RealType>::uniform_real(RealType, RealType) [with RealType = double]: Assertion `min_arg <= max_arg' failed.
			if(node_number<0) node_number = 0;
			if(weighted_node_distance.size() ==0) 
                         random_node_idx = roll_die(node_number);
                        else
			  random_node_idx = roll_weighted_die(weighted_node_distance, debug);
                  //      if(weighted_node_distance.size() > 0)
                    //      weighted_node_distance.erase(weighted_node_distance.begin()+random_node_idx);
			particle part;
			part.node_idx = random_node_idx;
			new_particles.push_back(part);
			cout<<" ("<<random_node_idx<<"*) ";
		}
		cout<<endl;

		particles_ = new_particles;
		//cout<<"After sampling"<<endl;
		std::map<int, int> particle_sum;
		for(size_t i=0; i<particles_.size(); i++)
		{
			++particle_sum[round(particles_[i].node_idx)];
		}

		printParticles(particle_sum);
		sw3.end(print_sw);
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
		if(abs(max_node_id - matching_node)>10)
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
		sw4.end(print_sw);
		
		sd_ = scores[max_neighbor_node_id];
		cout<<"Max match details: "<<sd_.x<<" "<<sd_.y<<" "<<sd_.t<<" "<<sd_.score<<" "<<sd_.score_ver<<" "<<sd_.node_src<<" "<<sd_.node_dst<<endl;
		//just a hack for not to activate a close loop when the graph is too small
		if( matching_node < 20)
				return -1;
		
		if( max_neighbor_score >65. && max_neighbor_node_id != -1 && max_accu > 3)
		{
				cout<<"close loop found at node "<<max_neighbor_node_id <<" with score "<<max_neighbor_score<<endl;
				return max_neighbor_node_id;
		}
		else return -1;

		//random generation of uniform number by using the size of the particles as the bound
		//boost:
	}

};
