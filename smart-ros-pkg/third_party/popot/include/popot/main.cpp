#include "rng_generators.h"
typedef popot::rng::CRNG RNG_GENERATOR;

#include "popot.h"

// Dropwave 
void popot::problems::Dropwave::init(void)
{
  count = 0;
}

void popot::problems::Dropwave::free(void)
{
}

double popot::problems::Dropwave::get_lbound(int index)
{
  return -5.12;
}

double popot::problems::Dropwave::get_ubound(int index)
{
  return 5.12;
}

bool popot::problems::Dropwave::stop(double fitness, int epoch)
{
  return (fitness <= -1.0+1e-4) || (count >= 20000);
}

double popot::problems::Dropwave::evaluate(void * x)
{
  double * params = (double*) x;
  count++;
  return -(1.0 + cos(12.0*sqrt(pow(params[0],2.0) + pow(params[1],2.0))))
    / (0.5 * (pow(params[0],2.0) + pow(params[1],2.0))+2.0);
}
int     popot::problems::Dropwave::count;

int main(int argc, char * argv[])
{

}
