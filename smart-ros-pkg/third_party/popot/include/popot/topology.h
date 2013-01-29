#ifndef POPOT_TOPOLOGY_H
#define POPOT_TOPOLOGY_H

#include <vector>
#include <map>
#include "neighborhood.h"

// Define a connection matrix 
// and make a generic method which takes this connection matrix;
// and fills in the neighborhoods

namespace popot
{
  namespace PSO
  {
    namespace topology
    {

      /**
       *
       * Basic topology methods
       */
     template<int SIZE, typename PARTICLE>
        class Base
      {
      protected:
	static void connectParticles(PARTICLE * particles,
				     std::map< int, std::vector<int> > &neighbordhood_membership,
				     int * who_informs_whom)
	{
	  // Just clean up the neighborhood memberships
	  for(int i = 0 ; i < SIZE ; ++i)
	    neighbordhood_membership[i].clear();

	  if(VERBOSE_BENCH)
	    {
	      for(int i = 0 ; i < SIZE ; ++i)
		{
		  for(int j = 0 ;  j< SIZE ; ++j)
		    printf("%i ", who_informs_whom[i*SIZE+j]);
		  printf("\n");
		}
	    }
		

	  for(int j = 0 ; j < SIZE ; ++j)
	    {
	      particles[j].getNeighborhood()->clear();

	      // We browse column j of who_informs_whom
	      // if 1.0, then we add particle i to the neighborhood of particle j
	      for(int i = 0 ; i < SIZE ; ++i)
		  // If particle i informs particle j
		  if(who_informs_whom[i*SIZE + j])
		    {
		      // Particle i belongs to the neighborhood of particle j
		      neighbordhood_membership[i].push_back(j);
		      // And we add particle i to the neighborhood of particle j
		      particles[j].getNeighborhood()->add(&(particles[i]));
		    }

	      //for(int i = 0 ; i < SIZE ; ++i)
	      // std::cout << "Particle " << i << " :" << particles[i].getNeighborhood()->size() << std::endl;
	    }
	}

	static void getNeighborhoodList(PARTICLE * particles,
					std::vector< typename PARTICLE::NeighborhoodType *> &neighborhoods)
	{
	  neighborhoods.clear();
	  for(int i = 0 ; i < SIZE ; ++i)
	    neighborhoods.push_back(particles[i].getNeighborhood());
	}

      public:
	// Method called when there is no improvement in the global best
	// By default it does nothing.
	static void regenerateNeighborhoods(PARTICLE * particles,
					    std::vector< typename PARTICLE::NeighborhoodType *> &neighborhoods,
					    std::map< int, std::vector<int> > &neighbordhood_membership)
	{
	}

      public:
	static int size() { return SIZE;}
      };

      /**
       * Full toplogy
       * @short Each of the N particles receives information from the N others
       */
     template<int SIZE, typename PARTICLE, bool SELF=false>
        class Full : public Base<SIZE, PARTICLE>
      {
      public:
	static void fillNeighborhoods(PARTICLE * particles,
				      std::vector< typename PARTICLE::NeighborhoodType *> &neighborhoods,
				      std::map< int, std::vector<int> > &neighbordhood_membership)
	{
	  // Generate the connection matrix
	  int *who_informs_whom = new int[SIZE * SIZE];
	  // Column j indicates which particle informs particle j
	  // therefore, line i indicates which particles the particle i informs

	  // Set the connection matrix to 1 everywhere
	  for(int i = 0 ; i < SIZE*SIZE ; ++i)
	    who_informs_whom[i] = 1.0;
	  if(!SELF)
	    for(int i = 0 ; i < SIZE ; ++i)
	      who_informs_whom[i*SIZE + i] = 0.0;

	  // Given the connectivity matrix, we now connect the particles
	  Base<SIZE, PARTICLE>::getNeighborhoodList(particles, neighborhoods);
	  Base<SIZE, PARTICLE>::connectParticles(particles, neighbordhood_membership, who_informs_whom);
	  
	  delete[] who_informs_whom;
	}
      };

      /**
       * Ring toplogy
       * @short A ring topology connects the particles on a ring plus a connection to itself
       */
     template<int SIZE, typename PARTICLE, bool SELF=false>
        class Ring : public Base<SIZE, PARTICLE>
      {
      public:
	static void fillNeighborhoods(PARTICLE *particles,
				      std::vector< typename PARTICLE::NeighborhoodType *> &neighborhoods,
				      std::map< int, std::vector<int> > &neighbordhood_membership)
	{
	  // Generate the connection matrix
	  int *who_informs_whom = new int[SIZE * SIZE];
	  // Column j indicates which particle informs particle j
	  // therefore, line i indicates which particles the particle i informs

	  // Set the connection matrix to 0 everywhere
	  for(int i = 0 ; i < SIZE*SIZE ; ++i)
	    who_informs_whom[i] = 0.0;

	  // A particle informs itself?
	  if(SELF)
	    for(int i = 0 ; i < SIZE ; ++i)
	      who_informs_whom[i*SIZE + i] = 1.0;

	  int i_neigh;
	  for(int i = 0 ; i < SIZE ; ++i)
	    {
	      // Particle i is informed by the particle on the left and the particle on its right
	      i_neigh = i + 1;
	      if(i_neigh >= SIZE)
		i_neigh = 0;
	      who_informs_whom[i_neigh*SIZE + i] = 1.0;

	      i_neigh = i - 1;
	      if(i_neigh < 0)
		i_neigh = SIZE-1;
	      who_informs_whom[i_neigh*SIZE + i] = 1.0;

	      if(SELF)
		who_informs_whom[i*SIZE + i] = 1.0;
	    }

	  // Given the connectivity matrix, we now connect the particles
	  Base<SIZE, PARTICLE>::getNeighborhoodList(particles, neighborhoods);
	  Base<SIZE, PARTICLE>::connectParticles(particles, neighbordhood_membership, who_informs_whom);

	  delete[] who_informs_whom;
	}
      };

      /**
       * Von Neuman topology
       * @short The von Neuman topology connects the particles on a 2D toric grid
       *        with the nearest four neighbours plus itself
       */
     template<int SIZE, typename PARTICLE, bool SELF=false>
        class VonNeuman : public Base<SIZE, PARTICLE>
      {
      public:
	static const int WIDTH;
	static const int HEIGHT;
	static void fillNeighborhoods(PARTICLE *particles,
				      std::vector< typename PARTICLE::NeighborhoodType *> &neighborhoods,
				      std::map< int, std::vector<int> > &neighbordhood_membership)
	{


	  // Generate the connection matrix
	  int *who_informs_whom = new int[VonNeuman::size() * VonNeuman::size()];
	  // Column j indicates which particle informs particle j
	  // therefore, line i indicates which particles the particle i informs

	  // Set the connection matrix to 0 everywhere
	  for(int i = 0 ; i < VonNeuman::size()*VonNeuman::size() ; ++i)
	    who_informs_whom[i] = 0.0;

	  // A particle informs itself?
	  if(SELF)
	    for(int i = 0 ; i < VonNeuman::size() ; ++i)
	      who_informs_whom[i*VonNeuman::size() + i] = 1.0;

	  int i_neigh, j_neigh;
	  int index_part, index_neigh;
	  for(int i = 0 ; i < HEIGHT ; ++i)
	    {
	      for(int j = 0 ; j < WIDTH ; ++j)
		{

		  index_part = i*WIDTH + j;

		  // Add the 4 closest neighboors that are informing particle index_part
		  i_neigh = i - 1;
		  j_neigh = j;
		  if(i_neigh < 0)
		    i_neigh = HEIGHT - 1;
		  index_neigh = i_neigh * WIDTH + j_neigh;
		  who_informs_whom[index_neigh*VonNeuman::size() + index_part] = 1.0;

		  i_neigh = i + 1;
		  j_neigh = j;
		  if(i_neigh >= HEIGHT)
		    i_neigh = 0;
		  index_neigh = i_neigh * WIDTH + j_neigh;
		  who_informs_whom[index_neigh*VonNeuman::size() + index_part] = 1.0;


		  i_neigh = i;
		  j_neigh = j - 1;
		  if(j_neigh < 0)
		    j_neigh = WIDTH - 1;
		  index_neigh = i_neigh * WIDTH + j_neigh;
		  who_informs_whom[index_neigh*VonNeuman::size() + index_part] = 1.0;


		  i_neigh = i;
		  j_neigh = j + 1;
		  if(j_neigh >= WIDTH)
		    j_neigh = 0;
		  index_neigh = i_neigh * WIDTH + j_neigh;
		  who_informs_whom[index_neigh*VonNeuman::size() + index_part] = 1.0;

		  if(SELF)
		    who_informs_whom[i*SIZE + i] = 1.0;
		}
	    }
	
	  // Given the connectivity matrix, we now connect the particles
	  Base<SIZE, PARTICLE>::getNeighborhoodList(particles, neighborhoods);
	  Base<SIZE, PARTICLE>::connectParticles(particles, neighbordhood_membership, who_informs_whom);

	  delete[] who_informs_whom;
	}
      };
     template<int SIZE, typename PARTICLE, bool SELF> const int VonNeuman<SIZE, PARTICLE, SELF>::WIDTH = int(sqrt(SIZE));
     template<int SIZE, typename PARTICLE, bool SELF> const int VonNeuman<SIZE, PARTICLE, SELF>::HEIGHT = int(SIZE / VonNeuman<SIZE, PARTICLE>::WIDTH);


      /**
       * Random topology
       * @short Some probabilistic connectivity so that most of the particles will have K informants
       */
      template<int SIZE, int K, typename PARTICLE, bool SELF=true>
        class RandomInformants : public Base<SIZE, PARTICLE>
      {
      public:
	static void fillNeighborhoods(PARTICLE * particles,
				      std::vector< typename PARTICLE::NeighborhoodType *> &neighborhoods,
				      std::map< int, std::vector<int> > &neighbordhood_membership)
	{
	  // Generate the connection matrix
	  int *who_informs_whom = new int[SIZE * SIZE];
	  // Column j indicates which particle informs particle j
	  // therefore, line i indicates which particles the particle i informs


	  // Set the connection matrix to 0 everywhere
	  for(int i = 0 ; i < SIZE*SIZE ; ++i)
	    who_informs_whom[i] = 0.0;

	  // A particle informs itself
	  if(SELF)
	    for(int i = 0 ; i < SIZE ; ++i)
	      who_informs_whom[i*SIZE + i] = 1.0;

	  for(int i = 0 ; i < SIZE ; ++i)
	    {
	      // For line i, we draw uniformely with replacement K-1 particles this particle will inform
	      for(int k = 0 ; k < K-1 ; ++k)
		who_informs_whom[i*SIZE + popot::math::uniform_integer(0, SIZE-1)] = 1.0; 
	    }

	  // Given the connectivity matrix, we now connect the particles
	  Base<SIZE, PARTICLE>::getNeighborhoodList(particles, neighborhoods);
	  Base<SIZE, PARTICLE>::connectParticles(particles, neighbordhood_membership, who_informs_whom);

	  delete[] who_informs_whom;
	}

	static void regenerateNeighborhoods( PARTICLE *particles,
					    std::vector< typename PARTICLE::NeighborhoodType *> &neighborhoods,
					    std::map< int, std::vector<int> > &neighbordhood_membership)
	{
	  fillNeighborhoods(particles,neighborhoods,neighbordhood_membership);
	}
      };

      template<int SIZE, int K, typename PARTICLE, bool SELF=true>
        class FixedRandomInformants : public Base<SIZE, PARTICLE>
      {
      public:
	static void fillNeighborhoods(PARTICLE * particles,
				      std::vector< typename PARTICLE::NeighborhoodType *> &neighborhoods,
				      std::map< int, std::vector<int> > &neighbordhood_membership)
	{
	  // Generate the connection matrix
	  int *who_informs_whom = new int[SIZE * SIZE];
	  // Column j indicates which particle informs particle j
	  // therefore, line i indicates which particles the particle i informs

	  // Set the connection matrix to 0 everywhere
	  for(int i = 0 ; i < SIZE*SIZE ; ++i)
	    who_informs_whom[i] = 0.0;

	  // A particle informs itself
	  if(SELF)
	    for(int i = 0 ; i < SIZE ; ++i)
	      who_informs_whom[i*SIZE + i] = 1.0;

	  for(int i = 0 ; i < SIZE ; ++i)
	    {
	      // For line i, we draw uniformely with replacement K-1 particles this particle will inform
	      for(int k = 0 ; k < K-1 ; ++k)
		who_informs_whom[i*SIZE + popot::math::uniform_integer(0, SIZE-1)] = 1.0; 
	    }

	  // Given the connectivity matrix, we now connect the particles
	  Base<SIZE, PARTICLE>::getNeighborhoodList(particles, neighborhoods);
	  Base<SIZE, PARTICLE>::connectParticles(particles, neighbordhood_membership, who_informs_whom);

	  delete[] who_informs_whom;
	}

	static void regenerateNeighborhoods( PARTICLE *particles,
					    std::vector< typename PARTICLE::NeighborhoodType *> &neighborhoods,
					    std::map< int, std::vector<int> > &neighbordhood_membership)
	{
	}
      };


      /**
       * Adaptive random topology
       * @short Some probabilistic connectivity so that most of the particles will have K informants
       *        the topology is regenerated at each unsucessful step
       */
      template<int SIZE, int K, typename PARTICLE, bool SELF=true>
        class AdaptiveRandom : public Base<SIZE, PARTICLE>
      {
      public:
	static void fillNeighborhoods(PARTICLE * particles,
				      std::vector< typename PARTICLE::NeighborhoodType *> &neighborhoods,
				      std::map< int, std::vector<int> > &neighbordhood_membership)
	{
	  double p = 1.0 - pow(1.0 - 1.0/SIZE,K);

	  // Generate the connection matrix
	  int *who_informs_whom = new int[SIZE * SIZE];
	  // Column j indicates which particle informs particle j
	  // therefore, line i indicates which particles the particle i informs

	  // A particle informs itself
	  if(SELF)
	    for(int i = 0 ; i < SIZE ; ++i)
	      who_informs_whom[i*SIZE + i] = 1.0;

	  for(int i = 0 ; i < SIZE ; ++i)
	    {
	      for(int k = 0 ; k < SIZE ; ++k)
		if( k != i)
		  {
		    if(popot::math::uniform_random(0.0,1.0) < p)
		      who_informs_whom[k*SIZE + i] = 1.0; // k informs i
		    else
		      who_informs_whom[k*SIZE + i] = 0.0;
		  }
	    }

	  // Given the connectivity matrix, we now connect the particles
	  Base<SIZE, PARTICLE>::getNeighborhoodList(particles, neighborhoods);
	  Base<SIZE, PARTICLE>::connectParticles(particles, neighbordhood_membership, who_informs_whom);

	  delete[] who_informs_whom;
	}

	static void regenerateNeighborhoods(PARTICLE *particles,
					    std::vector< typename PARTICLE::NeighborhoodType *> &neighborhoods,
					    std::map< int, std::vector<int> > &neighbordhood_membership)
	{
	  fillNeighborhoods(particles,neighborhoods,neighbordhood_membership);
	}
      };


    } // topology
  } // PSO
} // popot

#endif // POPOT_TOPOLOGY_H
