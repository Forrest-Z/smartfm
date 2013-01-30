#define _USE_MATH_DEFINES
#include <bitset>
#include <iostream>
#include <string>
#include <stdint.h>
#include <boost/random/mersenne_twister.hpp>
#include <boost/random/normal_distribution.hpp>
#include <boost/random/variate_generator.hpp>
#include <boost/random/uniform_real.hpp>
#include <boost/random/uniform_int.hpp>
#include <numeric>
#include <vector>
#include <sstream>
#include <stdlib.h>
#include <cmath>
#include <limits>
#include <ctime>
#include <algorithm>
#include <iterator>
#include <complex>
#include <map>
#include "NormalMatchingGene.h"

using namespace std;

bool sortGeneScore(Positional2dGene gen1, Positional2dGene gen2)
{
	return gen1.score > gen2.score;
}

typedef boost::mt19937 ENG;

class GeneticOptimization2D
{
	ENG  eng;
	NormalsCorrelativeMatchingProblem<3> problem;

	map<int, int> record_crossover_type;
public:
	vector<Positional2dGene> genes;
	GeneticOptimization2D(int pop_size, NormalsCorrelativeMatchingProblem<3> vproblem, vector<Positional2dGene> ini_genes):
		eng(time(0)), problem(vproblem)
	{
		for(size_t i=0; i<pop_size; i++)
		{
			genes.push_back(chooseInitialGene(ini_genes));
		}
	}

	GeneticOptimization2D(int pop_size, NormalsCorrelativeMatchingProblem<3> vproblem): eng(time(0)), problem(vproblem)
	{
		ValueAndRange vnr_x(-15, 15, 0.1);
		ValueAndRange vnr_y(-20, 20, 0.1);
		ValueAndRange vnr_r(160, 240, 1.);
		//-2.000 0.300 185
		//vnr_x.setRealValue(-2.);
		//vnr_y.setRealValue(0.3);
		//vnr_r.setRealValue(-175);
		for(int i=0; i<pop_size; i++)
		{
			genes.push_back(Positional2dGene(randomizeVNR(vnr_x), randomizeVNR(vnr_y), randomizeVNR(vnr_r)));
			//genes.push_back(Positional2dGene(vnr_x, vnr_y, vnr_r));
		}


	}

	void generationOpt()
	{
		Positional2dGene best_genes;
		best_genes.score = 0.;
		for(int iteration=0; iteration<10; iteration++)
		//while(best_genes.score<0.4)
		{
			for(size_t i=0; i<genes.size(); i++)
				genes[i].updateScore(problem);
			
			vector<Positional2dGene> mating_genes, mated_genes, mutated_genes;

			//printGenesDetails(genes);
			//cout<<"reproduction: "<<endl;
			reproductionOperator(genes, mating_genes);
			sort(mating_genes.begin(), mating_genes.end(), sortGeneScore);
			//cout<<mating_genes[0].score<<" "<<mating_genes[mating_genes.size()-1].score<<endl;
			if(best_genes.score < mating_genes[0].score)
			{
				best_genes = mating_genes[0];
			}
			/*else
			{
				//injecting best genes back to the pool
				mating_genes.erase(mating_genes.end()-1);
				mating_genes.push_back(best_genes);
			}*/
			sort(mating_genes.begin(), mating_genes.end(), sortGeneScore);
			cout<<mating_genes[0].score<<" "<<mating_genes[mating_genes.size()-1].score<<"    \xd"<<flush;
			if(mating_genes[0].score == 1.) break;
			//printGenesDetails(mating_genes);

			//cout<<"crossver: "<<endl;
			this->crossOverOperator(mating_genes, mated_genes);
			//printGenesDetails(mated_genes);

			//cout<<"mutation: "<<endl;
			this->mutationOpertator(mated_genes, mutated_genes);
			//printGenesDetails(mutated_genes);
			genes = mutated_genes;
		}
		cout<<endl;
		for(size_t i=0; i<genes.size(); i++)
				genes[i].updateScore(problem);
			sort(genes.begin(), genes.end(), sortGeneScore);
			cout<<genes[0].score<<" "<<genes[genes.size()-1].score<<endl;
			
			printGenesDetails(genes,1);
			for(map<int,int>::iterator it=record_crossover_type.begin(); it!=record_crossover_type.end(); it++)
			{
				cout<<it->first<<":"<<it->second<<" ";
			}
			cout<<endl;
	}

	ValueAndRange randomizeVNR(ValueAndRange data)
	{
		int ub_int = ((data.getMax() - data.getMin())/data.getRes());

		boost::uniform_int<> dist(0, ub_int);
		boost::variate_generator<boost::mt19937&, boost::uniform_int<> > die(eng, dist);
		data.setRealValue((die()*data.getRes())+data.getMin());
		return data;
	}
	//use to randomly choose with its rough score as weight
	Positional2dGene chooseInitialGene(vector<Positional2dGene> &input_genes)
	{
		vector<double> scores;
		for(size_t i=0; i<input_genes.size(); i++)
		{
			scores.push_back(input_genes[i].score);
		}
		int selectedGeneIdx = rollWeightedDie(scores);
		Positional2dGene selectedGene = input_genes[selectedGeneIdx];
		selectedGene.data = selectedGene.VNRtoGray(selectedGene.pose);
		input_genes.erase(input_genes.begin()+selectedGeneIdx);
		return selectedGene;
	}
	int rollWeightedDie(vector<double> &probabilities)
	{
		std::vector<double> cumulative;

		std::partial_sum(probabilities.begin(), probabilities.end(),
				std::back_inserter(cumulative));
		boost::uniform_real<> dist(0, cumulative.back());
		boost::variate_generator<boost::mt19937&, boost::uniform_real<> > die(eng, dist);

		return (std::lower_bound(cumulative.begin(), cumulative.end(), die()) - cumulative.begin());
	}

	void reproductionOperator(vector<Positional2dGene> &input_genes, vector<Positional2dGene> &output_genes)
	{
		//now that each gene has its score, generate a mating pool
		output_genes.clear();
		vector<double> probabilities;
		//cout<<"Cadidate genes: ";
		for(size_t i=0; i<input_genes.size(); i++)
		{
			probabilities.push_back(input_genes[i].score);
			//cout<<BinaryStrToInt(input_genes[i].data)<<":"<<input_genes[i].score<<" ";
		}
		//cout<<endl;
		//cout<<"New genes pool: ";
		for(size_t i=0; i<input_genes.size(); i++)
		{
			int new_selected_gene = rollWeightedDie(probabilities);
			output_genes.push_back(input_genes[new_selected_gene]);
			//cout<<BinaryStrToInt(output_genes[i].data)<<":"<<output_genes[i].score<<" ";
		}
		//cout<<endl;

	}

	void updateWithBound(Positional2dGene &gene)
	{
		vector<double> gene_v = gene.GraytoPose();
		for(int j=0; j<3; j++)
			gene_v[j] = fmutil::bound(gene.pose[j].getMin(), gene_v[j], gene.pose[j].getMax());
		for(int j=0; j<3; j++)
			gene.pose[j].setRealValue(gene_v[j]);
		gene.data = gene.VNRtoGray(gene.pose);
	}

	void elementCrossOver(vector<Positional2dGene> &mated_gene)
	{
	    //crossover by exchanging y and x,r
	    boost::uniform_int<> dist(0, 3);
	    boost::variate_generator<boost::mt19937&, boost::uniform_int<> > die(eng, dist);
	    int switch_mode = die();
	    vector<string> gene1 = mated_gene[0].getIndiviualStr();
	    vector<string> gene2 = mated_gene[1].getIndiviualStr();
	    stringstream gene1ss, gene2ss;
	    record_crossover_type[switch_mode]++;

	    switch (switch_mode)
	    {
	    case 0://sandwich switch
	    gene1ss<<gene1[0]<<gene2[1]<<gene1[2];
	    gene2ss<<gene2[0]<<gene1[1]<<gene2[2];
	    break;
	    case 1://switch x
	        gene1ss<<gene2[0]<<gene1[1]<<gene1[2];
	        gene2ss<<gene1[0]<<gene2[1]<<gene2[2];
	        break;
	    case 2://switch y
	        gene1ss<<gene1[0]<<gene2[1]<<gene1[2];
	        gene2ss<<gene2[0]<<gene1[1]<<gene2[2];
	        break;
	    case 3://switch last
	        gene1ss<<gene1[0]<<gene1[1]<<gene2[2];
	        gene2ss<<gene2[0]<<gene2[1]<<gene1[2];
	        break;
	    case 4://switch xy
	        gene1ss<<gene2[0]<<gene2[1]<<gene1[2];
	        gene2ss<<gene1[0]<<gene1[1]<<gene2[2];
	        break;
	    case 5://switch yz
	        gene1ss<<gene1[0]<<gene2[1]<<gene2[2];
	        gene2ss<<gene2[0]<<gene1[1]<<gene1[2];
	        break;
	    case 6://cross switch xy
	        gene1ss<<gene2[1]<<gene2[0]<<gene1[2];
	        gene2ss<<gene1[1]<<gene1[0]<<gene2[2];
	        break;
	    }

	    mated_gene[0].data = gene1ss.str();
	    mated_gene[1].data = gene2ss.str();
	}

	void crossOverOperator(vector<Positional2dGene> &mating_genes, vector<Positional2dGene> &mated_genes)
	{
		mated_genes.clear();
		while(mating_genes.size()>0)
		{
			vector<Positional2dGene> selected_pair_gene = pairingGenes(mating_genes);
			assert(selected_pair_gene.size() == 2);
			boost::uniform_int<> dist(0, 1);
			boost::variate_generator<boost::mt19937&, boost::uniform_int<> > die(eng, dist);
			
			Positional2dGene child_gene = selected_pair_gene[0];
			//performing a 50% mask crossover, may produce infeasible answer
			for(int i=0; i<child_gene.data.size(); i++)
			{
			    int mask = die();
			    if(mask)
			        child_gene.data[i] = selected_pair_gene[1].data[i];
			}
			Positional2dGene parent_copy;
			if(selected_pair_gene[0].score > selected_pair_gene[1].score)
			    parent_copy = selected_pair_gene[0];
			else
			    parent_copy = selected_pair_gene[1];

			updateWithBound(child_gene);
			//here 50% of parent genes are reserved, hence an incremental updating population.
			mated_genes.push_back(child_gene);
			mated_genes.push_back(parent_copy);
			//updateWithBound(mated_gene[1]);
			//mated_genes.insert(mated_genes.begin(), mated_gene.begin(), mated_gene.end());

		}
	}

	void mutationOpertator(vector<Positional2dGene> &mated_genes, vector<Positional2dGene> &mutated_genes)
	{
		mutated_genes = mated_genes;
		for(size_t i=0; i<mutated_genes.size(); i++)
		{
			boost::uniform_int<> dist(1, 1000);
			boost::variate_generator<boost::mt19937&, boost::uniform_int<> > die(eng, dist);
			for(size_t j=0; j<mutated_genes[i].data.size(); j++)
			{
				if(die() == 1)
				{
					//vector<double> pose = mutated_genes[i].GraytoPose();
					//cout<<"Mutation occur! Origin: "<<pose[0]<<" "<<pose[1]<<" "<<pose[2]<<endl;
					if(mutated_genes[i].data[j] == '1') mutated_genes[i].data[j] = '0';
					else mutated_genes[i].data[j] = '1';

					updateWithBound(mutated_genes[i]);

					//pose = mutated_genes[i].GraytoPose();
					//cout<<"To: "<<pose[0]<<" "<<pose[1]<<" "<<pose[2]<<endl;
				}
			}
		}
	}
	vector<Positional2dGene> pairingGenes(vector<Positional2dGene> &genes)
	{
		//get a pair of gene and remove it from the mating pool
		boost::uniform_int<> dist(1, genes.size()-1);
		boost::variate_generator<boost::mt19937&, boost::uniform_int<> > die(eng, dist);
		int selected_pair_idx = die();
		vector<Positional2dGene> paired_genes;
		paired_genes.push_back(genes[0]);
		paired_genes.push_back(genes[selected_pair_idx]);
		genes.erase(genes.begin()+selected_pair_idx);
		genes.erase(genes.begin());
		return paired_genes;
	}

	void printGenesDetails(vector<Positional2dGene>  &genes, int number=-1)
	{
		size_t size = 0;
		if(number==-1) size = genes.size();
		else size = number;
		for(size_t i=0; i<size; i++)
		{
			Positional2dGene gene = genes[i];
			/*for(int i=0; i<3; i++)
				cout<< gene.pose[i].getRealValue()<<"\t";*/
			cout<<gene.data<<"\t";
			vector<double> pose = gene.GraytoPose();
			for(int i=0; i<3; i++)
				cout<< pose[i]<<"\t";
			cout<<endl;
		}
	}
};


int main(int argc, char** argv)
{
	//move into ubuntu
	NormalsCorrelativeMatchingProblem<3> problem;
	pcl::PointCloud<pcl::PointNormal> input_cloud, matching_cloud;
//	pcl::io::loadPCDFile("amcl2_pcd/00750.pcd", input_cloud);
//	pcl::io::loadPCDFile("amcl2_pcd/01112.pcd", matching_cloud);
	pcl::io::loadPCDFile(argv[1], input_cloud);
	pcl::io::loadPCDFile(argv[2], matching_cloud);

	//for initialization purpose
	/*problem.trans_res_ = 1.0;
	problem.init(input_cloud, matching_cloud);
	vector<Positional2dGene> initial_genes;
	ValueAndRange vnr_x(-10, 10, 0.1);
	ValueAndRange vnr_y(-20, 20, 0.1);
	ValueAndRange vnr_r(-180, 179, 1.);
	Positional2dGene best_rough_gene(vnr_x, vnr_y, vnr_r);
	best_rough_gene.score = 0.;
	for(double i=-180; i<180; i+=2)
	{
	    cout<<i<<": "<<endl;
		for(double j=-10; j<10; j++)
		{
			for(double k=-20; k<20; k++)
			{
				double manual_pose[] = {j, k, i/180.*M_PI};
				vnr_x.setRealValue(j); vnr_y.setRealValue(k); vnr_r.setRealValue(i);
				initial_genes.push_back(Positional2dGene(vnr_x, vnr_y, vnr_r));
				double score = problem.evaluate(manual_pose);
				initial_genes[initial_genes.size()-1].score = score;
				if(best_rough_gene.score<score) best_rough_gene = initial_genes[initial_genes.size()-1];
				cout<<score<<" ";
			}
		//for(int k=0; k<3; k++)
		//cout<<best_rough_gene.pose[k].getRealValue()<<" ";
		//cout<<best_rough_gene.score<<"      \xd"<<flush;
			cout<<endl;
		}
	}
	cout<<endl;*/
	problem.trans_res_ = 0.1;
	problem.init(input_cloud, matching_cloud);
	//problems::Ackley<3> problem;
	//problem.init();
	GeneticOptimization2D go2d(2000, problem);//, initial_genes);

	go2d.generationOpt();

	string str;
	getline(cin,str);	

}
