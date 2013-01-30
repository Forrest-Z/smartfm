#ifndef POPOT_NEIGHBORHOOD_H
#define POPOT_NEIGHBORHOOD_H

#include <vector>
#include "exceptions.h"
#include "maths.h"

namespace popot
{
  namespace PSO
  {

    namespace particle
    {
      template<typename PROBLEM,typename POSITION_INITIALIZER> class BaseParticle;
    }

    namespace neighborhood
    {

      /**
       * Neighborhood
       * @param PARTICLE The particle type you want to use, e.g. swarm::particle::TraditionalParticle
       * \brief A neighborhood contains a vector of particles belonging to the neighborhood as well as a pointer (not a copy!)
       *        to the best particle. The best particle is the first with the lowest fitness 
       */
      template<typename PARTICLE>
        class Neighborhood
        {
	public:
	  typedef typename PARTICLE::BestType BestType;
	  typedef PARTICLE InNeighborhoodType;

        protected:
	  std::vector<InNeighborhoodType *> _particles;
	  const BestType *_best_particle;

        public:
	  Neighborhood(void){
	    _best_particle = 0;
	  };


	  Neighborhood(const Neighborhood & other)
	    {
	      _best_particle = other.getBest();
	      for(unsigned int i = 0 ; i < other.size() ; ++i)
		_particles.push_back(other.get(i));
	    }

	  virtual ~Neighborhood(void){
	    clear();
	  };

	  void add(InNeighborhoodType * p)
	  {
	    _particles.push_back(p);
	  }

	  int size() const
	  {
	    return _particles.size();
	  }

	  void clear()
	  {
	    _particles.clear();
	  }

	  InNeighborhoodType * get(int i) const
	  {
	    if(i >= 0 && i < size())
	      return _particles[i];
	    else
	      throw popot::Exception::IndexOutOfRange(i, size());
	  }

	  const BestType * findBest(void)
	  {
	    if(_particles.size() == 0)
	      throw popot::Exception::FindBestFromEmptyNeighborhood();

	    _best_particle = &(_particles[0]->getBestPosition());
	    for(unsigned int i = 1 ; i < _particles.size() ; ++i)
	      {
		if(_particles[i]->getBestPosition().compare(*_best_particle) < 0)
		  _best_particle = &(_particles[i]->getBestPosition());
	      }

	    return _best_particle;
	  }

	  void updateBest(InNeighborhoodType * p)
	  {
	    if(_best_particle == 0)
	      throw popot::Exception::BestParticleNotInitialized();

	    if(p->getBestPosition()->compare(*_best_particle) < 0)
	      _best_particle = &(p->getBestPosition());
	  }

	  const BestType* getBest(void) const
	  {
	    return _best_particle;
	  }

	  void print(void)
	  {
	    std::cout << "Neighborhood hosting " << size() << " particles : " << std::endl;
	    std::cout << "Best particle " << getBest() << " with fitness : " << getBest()->getFitness() << std::endl;
	    for(int i = 0 ; i < size() ; ++i)
	      std::cout << "Particle " << i << " : " << get(i) << std::endl;
	  }
        };


      /**
       * ProbabilisticNeighborhood
       * @param PARTICLE The particle type you want to use, e.g. swarm::particle::Particle
       * \brief A neighborhood contains a vector of particles belonging to the neighborhood as well as a pointer (not a copy!)
       *        to the best particle; The selection of the best is probabilistic
       */
      /*
      template<typename PARTICLE, typename PARAMS>
        class ProbabilisticNeighborhood
        {
	public:
	  typedef typename PARTICLE::BestType BestType;
	  typedef PARTICLE InNeighborhoodType;

        protected:
	  std::vector<InNeighborhoodType *> _particles;
	  const BestType *_best_particle;

        public:
	  ProbabilisticNeighborhood(void){
	    _best_particle = 0;
	  };

	  virtual ~ProbabilisticNeighborhood(void){
	    clear();
	  };

	  void add(InNeighborhoodType * p)
	  {
	    _particles.push_back(p);
	  }

	  int size() const
	  {
	    return _particles.size();
	  }

	  void clear()
	  {
	    _particles.clear();
	  }

	  InNeighborhoodType * get(int i)
	  {
	    if(i >= 0 && i < size())
	      return _particles[i];
	    else
	      throw popot::Exception::IndexOutOfRange(i, size());
	  }

	  BestType * findBest(void)
	  {
	    if(_particles.size() == 0)
	      throw popot::Exception::FindBestFromEmptyNeighborhood();

	    // Collect the fitnesses
	    double fitnesses[_particles.size()];
	    for(unsigned int i = 0 ; i < _particles.size() ; ++i)
	      fitnesses[i] = _particles[i]->getBestPosition()->getFitness();

	    // Perform a Gibbs sampling with this array of fitnesses
	    // to select the local best
	    _best_particle = &(_particles[popot::math::random_gibbs_from_array(fitnesses,_particles.size(), PARAMS::inv_temperature())]->getBestPosition());

	    return _best_particle;
	  }

	  void updateBest(InNeighborhoodType * p)
	  {
	    if(_best_particle == 0)
	      throw popot::Exception::BestParticleNotInitialized();

	    if(p->getBestPosition()->compare(_best_particle) < 0)
	      _best_particle = &(p->getBestPosition());
	  }

	  const BestType * getBest(void)
	  {
	    return _best_particle;
	  }
        };
      */



    } // namespace neighborhood
  } // namespace PSO
} // namespace popot

#endif // POPOT_NEIGHBORHOOD_H
