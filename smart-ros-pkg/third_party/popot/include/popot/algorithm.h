#ifndef POPOT_ALGORITHM_H
#define POPOT_ALGORITHM_H

#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <iomanip>
#include <vector>
#include <map>
#include <algorithm>

#include "neighborhood.h"
#include "individuals.h"
#include <fstream>

namespace popot
{
  namespace PSO
  {
    namespace algorithm
    {
      typedef enum
      {
	SYNCHRONOUS_EVALUATION,
	ASYNCHRONOUS_EVALUATION
      } EvaluationMode;

      /**
       * Standard particle swarm optimization
       * @param PARAMS Defines the parameters of the swarm (e.g. the evaluation mode : synchronous or asynchronous)
       * @param PARTICLE The particle type you want to use, e.g. swarm::particle::TraditionalParticle
       * @param TOPOLOGY Defines the topology, e.g. swarm::topology::VonNeuman
       * @param STOP_CRITERIA Defines the condition to stop the evolution of the swarm
       */
      template< typename PARAMS, typename PARTICLE, typename TOPOLOGY, typename STOP_CRITERIA>
        class Base
      {
      public:
	typedef typename PARTICLE::BestType BestType;
	typedef PARTICLE  ParticleType;
	typedef typename PARTICLE::NeighborhoodType NeighborhoodType;

      private:
	PARTICLE * particles;
	BestType best_particle;
	std::vector<NeighborhoodType * > neighborhoods;
	std::map< int , std::vector<int> > neighborhood_membership;
	int swarm_size;
	int *particles_indexes;

      public:
	int epoch;

	int nb_new_neigh;

      public:
	Base(void)
	  {
	    nb_new_neigh = 0;

	    swarm_size = TOPOLOGY::size();

	    // Declare our particles
	    particles = new PARTICLE[swarm_size];
	    particles_indexes = new int[swarm_size];

	    for(int i = 0 ; i < swarm_size ; ++i)
	      {
		particles[i].init(); // This initializes position and velocity
		particles_indexes[i] = i;
	      }

	    if(VERBOSE_BENCH)
	      std::cout << "After initialization " << RNG_GENERATOR::nb_calls << std::endl;

	    // We now form groups of particles depending on the topology
	    TOPOLOGY::fillNeighborhoods(particles, neighborhoods, neighborhood_membership);
	    nb_new_neigh ++;

	    // We now browse all the neighborhoods and find the best particles
	    // within each of them
	    // and initialize the best best particle of the whole swarm
	    best_particle = *(neighborhoods[0]->findBest());
	    for(unsigned int i = 1 ; i < neighborhoods.size() ; ++i)
	      if(neighborhoods[i]->findBest()->compare(best_particle) < 0)
		best_particle = *(neighborhoods[i]->getBest());
	    
	    if(VERBOSE_BENCH)
	      std::cout << "After topology 2 :" << RNG_GENERATOR::nb_calls << std::endl;

	    epoch = 0;

	  }

	virtual ~Base(void)
	  {
	    delete[] particles;
	    delete[] particles_indexes;
	  }

	/**
	 * Returns the fitness of the best individual (Required for benchmarking)
	 */ 
	double getBestFitness() const
	{
	  return best_particle.getFitness();
	}
	
	/**
	 * Copies the position of the best invidual (Required for benchmarking)
	 * The given array must be initialized with the appropriate size before calling this method
	 */
	void getBestPosition(double * pos) const
	{
	  for(int i = 0 ; i < PARTICLE::Problem::nb_parameters ; ++i)
	    pos[i] = best_particle.getPosition(i);
	}


	/**
	 * Returns the size of the swarm
	 */
	int getSize(void)
	{
	  return swarm_size;
	}

	/**
	 * One step of the swarm
	 */
	double step(void)
	{
	  // For each particle,
	  // - Update the particle's velocity
	  // - Enforce velocity boundaries
	  // - Move the particle to its new position
	  // - Enforce position boundaries 
	  // - Update the particle's best position
	  // - Update the swarm's best position

	  if(VERBOSE_BENCH)
	    std::cout << "Random before looping : " << RNG_GENERATOR::nb_calls << " calls " << std::endl;

	  // If we use an asynchronous update, we first shuffle the particles
	  if(PARAMS::evaluation_mode() == ASYNCHRONOUS_EVALUATION)
	    {
	      // If comparing to SPSO, the following should be commented
	      if(PARAMS::random_shuffle())
		popot::math::random_shuffle_indexes(particles_indexes, swarm_size);

	      int particle_index;
	      for(unsigned int i = 0 ; i < swarm_size ; ++i)
		{
		  particle_index = particles_indexes[i];

		  // First find the best informant within the neighborhood
		  particles[particle_index].getNeighborhood()->findBest();

		  // Update the velocities and positions
		  particles[particle_index].updateVelocity();
		  particles[particle_index].updatePosition();

		  // Confine the positions and velocities if required
		  particles[particle_index].confine();

		  // Compute the fitness of the new position
		  particles[particle_index].evaluateFitness();

		  // And see if we update the personal best
		  particles[particle_index].updateBestPosition();

		  if(VERBOSE_BENCH)
		    std::cout << std::endl;
		}
	    }
	  else // Synchronous evaluation
	    {
	      // In synchronous mode
	      // we first update all the current positions and evaluate their fitness
	      int particle_index;
	      for(unsigned int i = 0 ; i < swarm_size ; ++i)
		{
		  particles[i].updateVelocity();		  
		  particles[i].updatePosition();

		  particles[i].confine();

		  particles[i].evaluateFitness();
		}
	      // before changing the personal bests


	      // The personal best position is updated after all the positions of the particles
	      // have been updated because the neighborhoods hold a pointer to a personal best
	      // As this best in the neighborhood is used by the particles when they
	      // update their velocity, we must ensure that, for a synchronous evaluation
	      // updating the best is done after all the positions updates
	      for(unsigned int i = 0 ; i < swarm_size ; ++i)
		particles[i]. updateBestPosition();

	      // We now update the best particle of all the neighborhoods
	      for(unsigned int i = 0 ; i < neighborhoods.size() ; ++i)
		neighborhoods[i]->findBest();
	    }

	  // Update the best particle the whole swarm ever had
	  double old_fitness = best_particle.getFitness();

	  best_particle = *(neighborhoods[0]->findBest());
	  for(unsigned int i = 1 ; i < neighborhoods.size() ; ++i)
	    if(neighborhoods[i]->findBest()->compare(best_particle) < 0)
	      best_particle = *(neighborhoods[i]->getBest());

	  if(best_particle.getFitness() >= old_fitness)
	    {
	      if(VERBOSE_BENCH)
		std::cout << "REGENERATING NEW NEIGHBORHOODS !!!!!!!!!!!!!!!!! " << best_particle.getFitness() << ">= " << old_fitness << std::endl;
	      // We consider that there is no improvement in the best particle
	      // and ask the topology if it wants to regenerate its topology
	      TOPOLOGY::regenerateNeighborhoods(particles, neighborhoods, neighborhood_membership);
	      nb_new_neigh ++;
	    }
	  else
	    if(VERBOSE_BENCH)
	      std::cout << "Keeping old NEIGHBORHOOD !!!!!! " << best_particle.getFitness() << " < " << old_fitness << std::endl;
	  
	  epoch++;
	  return best_particle.getFitness() - old_fitness;
	}

	/**
	 * Current epoch
	 */
	int getEpoch(void) const
	{
	  return epoch;
	}

	/**
	 * Run until stop criteria is met. 
	 * @param verbose set it to 1 to print the best fitness at each iteration
	 */
	void run(int verbose=0)
	{
	  while(!STOP_CRITERIA::stop(getBest().getFitness(), epoch))
	    {
	      step();
	      if(verbose) std::cout << '\r' << std::setw(6) << std::setfill('0') << epoch << " " << getBest().getFitness() << std::setw(5) << std::setfill(' ') << ' ' << std::flush;
	    }
	  if(verbose) std::cout << std::endl;
	}

	/**
	 * Returns the current best particle of the whole swarm (best of the personal best)
	 */
	const BestType& getBest(void) const
	  {
	    return best_particle;
	  }

	/**
	 * Returns a reference to the vector of particles
	 */
	PARTICLE* getParticles(void)
	  {
	    return particles;
	  }

	/**
	 * Print in the console the status of the swarm
	 * @param mode 0 displays all the particles, 1 display only the best
	 */
	void print(int mode=0)
	{
	  if(PARAMS::evaluation_mode() == SYNCHRONOUS_EVALUATION)
	    {
	      printf("Using a synchronous evaluation mode \n");
	    }
	  else if(PARAMS::evaluation_mode() == ASYNCHRONOUS_EVALUATION)
	    printf("Using an asynchronous evaluation mode \n");
	  else
	    printf("WARNING : Unrecognized evaluation mode \n");

	  switch(mode)
	    {
	    case 0:
	      {
		for(int i = 0 ; i < swarm_size ; ++i)
		  {
		    std::cout << "Particle " << std::setw(3) << std::setfill(' ') << i
			      << " : " << "\n" << particles[i] << std::endl;
		  }
		// Display the best particle
		std::cout << "Best particle : \n" << best_particle << std::endl;
	      }
	      break;
	    case 1:
	      {
		// Display the best particle
		std::cout << "Best particle : \n" << best_particle << std::endl;
	      }
	      break;
	    case 2:
	      {
		for(int i = 0 ; i < swarm_size ; ++i)
		  {
		    std::cout << "Particle " << std::setw(3) << std::setfill(' ') << i
			      << " : " << particles[i].getFitness() << " ; Best : " << particles[i].getBestPosition().getFitness() << std::endl;
		  }
		// Display the best particle
		std::cout << "Best particle : \n" << best_particle.getFitness() << std::endl;
	      }
	      break;
	    default:
	      break;
	    }
	}

	
	std::map< int , std::vector<int> >* getNeighborhoodMembership(void)
	  {
	    return &neighborhood_membership;
	  }

	/**
	 * Generate a graph of the connections between the particles using DOT
	 */
	void generateGraph(std::string filename)
	{
	  printf("----------- \n Generating a graph of the connectivity \n");
	  int nb_particles = swarm_size;

	  // From this connectivity matrix, we generate the file
	  // to be used with the DOT command
	  std::ofstream outfile(filename.c_str());
	  outfile << "digraph Connections {" << std::endl;
	  outfile << "node [shape=circle,fixedsize=true,width=0.6];  ";
	  for(int i = 0 ; i < nb_particles ; ++i)
	    {
	      outfile << i << ";";
	    }
	  outfile << std::endl;

	  // Generate a connectivity matrix
	  int *connectivity_matrix = new int[nb_particles*nb_particles];
	  for(int i = 0 ; i < nb_particles*nb_particles ; ++i)
	    connectivity_matrix[i] = 0;

	  for(int i = 0 ; i < nb_particles ; ++i)
	    for(unsigned int j = 0 ; j < neighborhood_membership[i].size(); ++j)
	      connectivity_matrix[i*nb_particles+neighborhood_membership[i][j]] += 1;

	  // Put in the connections
	  for(int i = 0 ; i < nb_particles ; ++i)
	    {
	      for(int j = 0 ; j < i ; ++j)
		{
		  if(connectivity_matrix[i*nb_particles+j] && !connectivity_matrix[j*nb_particles+i])
		    outfile << j << "->" << i << ";" << std::endl;
		  else if(connectivity_matrix[i*nb_particles+j] && connectivity_matrix[j*nb_particles+i])
		    outfile << j << "->" << i << " [dir=both];" << std::endl;
		}
	      if(connectivity_matrix[i*nb_particles+i])
		outfile << i << "->" << i << ";" << std::endl;
	    }

	  // The title and the end of the graphic
	  outfile << "overlap=false" << std::endl;
	  outfile << "label=\"Swarm's connections\"" << std::endl;
	  outfile << "fontsize=12;" << std::endl;
	  outfile << "}" << std::endl;
	  outfile.close();

	  printf("You can generate the graph by calling e.g.: \n");
	  printf(" neato -Tpng %s > graph.png\n", filename.c_str());
	  printf("Other export formats can be used with dot/neato : svg, eps, dia, xfig, ...\n");

	  // Compute the distribution of the informants
	  // We first compute the number of informants for each particle
	  // The informants of particle i are at line i
	  int * nb_informants = new int[nb_particles];
	  int max_nb_informants = 0;
	  for(int i = 0 ; i < nb_particles ; ++i)
	    {
	      nb_informants[i] = 0;
	      for(int j = 0 ; j < nb_particles ; ++j)
		nb_informants[i] += connectivity_matrix[i*nb_particles+j];
	      if(nb_informants[i] > max_nb_informants)
		max_nb_informants = nb_informants[i];
	    }
	  printf("\n");
	  printf("Max number of informants : %i \n", max_nb_informants);

	  outfile.open((filename + ".conn").c_str());
	  int nb_part = 0;
	  for(int i = 0 ; i <= max_nb_informants ; ++i)
	    {
	      // How many particles have nb_informants <= i ?
	      nb_part = 0;
	      for(int j = 0 ; j < nb_particles ; ++j)
		if(nb_informants[j] == i)
		  nb_part ++;
	      outfile << i << "\t" << nb_part/double(nb_particles) << std::endl;
	    }
	  outfile.close();
	  printf("-------------\n");

	  delete[] connectivity_matrix;
	}
      };





    } // namespace algorithm
  } // namespace PSO

  namespace ABC
  {
    namespace algorithm
    {

      template< typename ABC_PARAMS, typename PROBLEM, typename STOP_CRITERIA>
      class Base
      {
	typedef popot::ABC::individuals::FoodSource<PROBLEM> FoodSourceType;

      private:
	int epoch;
	int CS;
	int dimension;
	int limitForScout;
	int nb_employed;
	int nb_onlookers;
	double * probabilities;

	FoodSourceType * foodSources;
	FoodSourceType bestSource;

      private:
	void findBestSource(void)
	{
	  double bestf = bestSource.getFitness();
	  for(int i = 0 ; i < nb_employed ; ++i)
	    {
	      if(bestf < foodSources[i].getFitness())
		{
		  bestf = foodSources[i].getFitness();
		  bestSource = foodSources[i];
		}
	    }
	}

	void employedPhase(void)
	{
	  // In the employed bees phase, we generate new solutions
	  // around each nectar source
	  int change_dim;
	  int other_source;
	  double phi;
	  FoodSourceType new_source;
	  double new_param_value;
	  double sum_fitnesses = 0;
	  for(int i = 0 ; i < nb_employed ; ++i)
	    {
	      // Randomly select another source, different from the current source
	      other_source = (int) popot::math::uniform_random(0, nb_employed);
	      while(other_source == i)
		other_source = (int) popot::math::uniform_random(0, nb_employed);

	      foodSources[i].combine(foodSources[other_source]);

	      probabilities[i] = (foodSources[i]).getFitness();
	      sum_fitnesses += probabilities[i];
	    }

	  // At the end, we normalize the probabilities for each source
	  for(int i = 0 ; i < nb_employed ; ++i)
	    probabilities[i] /= sum_fitnesses;
	}

	void onlookerPhase(void)
	{
	  // Onlooker phase
	  // Each onlooker bee selects a food source
	  // based on its probability (reflecting its relative fitness)
	  int selected_source;
	  int other_source;
	  for(int i = 0 ; i < nb_onlookers ; ++i)
	    {
	      // Select a source based on its fitness
	      selected_source = popot::math::random_from_array(probabilities);

	      // Randomly select another source, different from the current source
	      other_source = (int) popot::math::uniform_random(0, nb_employed);
	      while(other_source == i)
		other_source = (int) popot::math::uniform_random(0, nb_employed);

	      foodSources[i].combine(foodSources[other_source]);
	      
	    }
	}

	void scoutPhase(void)
	{
	  // Scout phase
	  // We browse all the food sources
	  // If a source has a counter higher than the limit
	  // we reset it
	  for(int i = 0 ; i < nb_employed ; ++i)
	    if(foodSources[i].getCounter() >= limitForScout)
	      foodSources[i].init();
	}

      public:
	Base(void)
	  {
	    // Set up some parameters
	    CS = ABC_PARAMS::ColonySize();
	    dimension = PROBLEM::nb_parameters;
	    limitForScout = (CS * dimension) / 2;
	    nb_employed = CS/2;
	    nb_onlookers = CS/2;
	    epoch = 0;

	    // Initialize our populations
	    foodSources = new FoodSourceType[nb_employed];

	    // And the probabilities of their solutions
	    probabilities = new double[nb_employed];

	  }

	virtual ~Base(void)
	  {
	    delete[] foodSources;
	    delete[] probabilities;
	  }

	void init(void)
	{
	  // Initialize the positions and fitnesses
	  // of the employed bees
	  for(int i = 0 ; i < nb_employed ; ++i)
	    foodSources[i].init();

	  bestSource = foodSources[0];

	  epoch = 0;

	  // Keep track of the best solution
	  findBestSource();
	}

	int getEpoch(void)
	{
	  return epoch;
	}


	void step(void)
	{
	  // Employed bees phase
	  employedPhase();
	  
	  // Onlooker bees phase
	  onlookerPhase();

	  // Scout bees phase
	  scoutPhase();

	  // Memorize the best solution
	  findBestSource();

	  epoch ++;
	}

	FoodSourceType& getBest()
	  {
	    return bestSource;
	  }

	void run(void)
	{
	  while(!STOP_CRITERIA::stop(bestSource.getFValue(),epoch))
	    {
	      step();
	    }
	}

	void print(void)
	{
	  std::cout << "Artificial Bee Colony " << std::endl;
	  std::cout << "Food Sources : " << std::endl;
	  for(int i = 0 ; i < nb_employed ; ++i)
	      std::cout << foodSources[i] << std::endl;
	  std::cout << "Best source : " << bestSource << std::endl;

	}

      }; // class Base
    } // namespace algorithm
  } // namespace ABC
} // namespace popot

#endif // POPOT_ALGORITHM_H
