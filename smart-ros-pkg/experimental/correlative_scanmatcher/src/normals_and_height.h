#include <popot/rng_generators.h>

typedef popot::rng::CRNG RNG_GENERATOR;
#include <popot/popot.h>
#include <stdio.h>
#include <iostream>

#include "heightmatching_pso.h"
typedef NormalsAndHeightMatchingProblem<3> Problem;
typedef popot::PSO::initializer::PositionUniformRandom PositionInit;
typedef popot::PSO::initializer::VelocityZero VelocityInit;
typedef popot::PSO::particle::ModifiedBareboneParticle<Problem, PositionInit, VelocityInit> Particle;
typedef popot::PSO::topology::RandomInformants<60, 3, Particle> Topology;
class PSO_Params
{
public:
  static bool random_shuffle() { return false;}
  static int evaluation_mode() {return popot::PSO::algorithm::ASYNCHRONOUS_EVALUATION;}
};
class StopCriteria
{
public:
  static bool stop(double fitness, int epoch)
  {
    return Problem::stop(fitness, epoch);
  }
};
typedef popot::PSO::algorithm::Base<PSO_Params, Particle, Topology, StopCriteria> PSO;


