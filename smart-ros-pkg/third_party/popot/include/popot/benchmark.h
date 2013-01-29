#ifndef POPOT_BENCHMARK
#define POPOT_BENCHMARK

namespace popot
{
  namespace benchmark
  {
    /**
     * Benchmarking class; Given an algorithm and a number of trials for this algorithm
     * it computes the mean error, std, etc... and various statistics on the performances
     * We suppose ALGO provides :
     * - a constructor without arguments, 
     * - a step method, 
     * - a getBestFitness method,
     * - a getBestPosition method
     * - a getEpoch() 
     * The Problem is also the one used as template for ALGO; It provides the stop method and the function 
     * evaluations counter
     *
     * Tips : To automatically estimate the number of trials
     * for getting a good approximate of the mean error of the algorithm, 
     * we may use the (Maurer & Pontil,2009) inequality with the sample mean and sample variance
     */
    template<typename ALGO, typename PROBLEM, int NB_TRIALS>
    class Benchmark
    {
    private:
      double _mean_error;
      double _std_error;
      double _mean_nb_f_evaluations;
      double _log_progress;
      int    _nb_failures;
      double * _best_position;
      double _best_fitness;
      double _success_rate;
      int _trial_counter;

    public:
      Benchmark() : _mean_error(0.0),_std_error(0.0), _mean_nb_f_evaluations(0.0),
	_log_progress(0.0), _nb_failures(0), _best_position(0), _best_fitness(0), _success_rate(0.0), _trial_counter(0)
      {
	_best_position = new double[PROBLEM::nb_parameters];
      }

      ~Benchmark()
      {
	delete[] _best_position;
      }

      /**
       * Returns the mean error over NB_TRIALS
       */
      double getMeanError(void) const
      {
	return _mean_error;
      }

      /**
       * Returns the std error over NB_TRIALS
       */
      double getStdError(void) const
      {
	return _std_error;
      }

      /**
       * Returns the mean number of function evaluations to meet the stop criteria over NB_TRIALS
       */
      double getMeanFEvaluations(void) const
      {
	return _mean_nb_f_evaluations;
      }

      /**
       * Returns the log progress over NB_TRIALS
       * NOT FUNCTIONAL YET
       */
      double getLogProgress(void) const
      {
	return _log_progress;
      } 

      /**
       * Returns the success rate over NB_TRIALS
       */
      double getSuccessRate(void) const
      {
	return _success_rate;
      } 

      /**
       * Returns the best position over NB_TRIALS
       */
      void getBestPosition(double * pos) const
      {
	memcpy(pos, _best_position, PROBLEM::nb_parameters*sizeof(double));
      } 

      /**
       * Returns the best fitness over NB_TRIALS
       */
      double getBestFitness(void) const
      {
	return _best_fitness;
      }
 
      /**
       * Runs the benchmarks and computes the statistics
       */
      void run(int mode=0)
      {
	_mean_error = 0.0;
	_std_error = 0.0;
	_mean_nb_f_evaluations = 0.0;
	_log_progress = 0.0;
	_nb_failures = 0;
	_best_fitness = 0;
	_success_rate = 0.0;
	_trial_counter = 0;

	ALGO* algo;
	// Some temporary variables used to compute online the mean, std, .. 
	double sum_error = 0;
	double sum_square_errors=0;
	double sum_fe = 0;
	double sum_log = 0;
	double init_fitness = 0;
	
	for(int i = 1 ; i <= NB_TRIALS ; ++i)
	  {
	    _trial_counter++;
	    PROBLEM::init();
	    
	    algo = new ALGO();
	    init_fitness = algo->getBestFitness();
	    algo->run();

	    // Update the statistics
	    sum_error += algo->getBestFitness();
	    sum_square_errors += algo->getBestFitness()*algo->getBestFitness();
	    sum_fe += PROBLEM::count;
	    sum_log += (log(algo->getBestFitness()) - log(init_fitness))/double(PROBLEM::count);
	    if(PROBLEM::has_failed(algo->getBestFitness()))
	      _nb_failures++;
	    if(i == 1 || algo->getBestFitness() < _best_fitness)
	      {
		_best_fitness = algo->getBestFitness();
		algo->getBestPosition(_best_position);
	      }
	      
	    // Eventually print the statistics
	    if(mode)
	      {
		_mean_error = sum_error/double(i);
		_std_error = sqrt((sum_square_errors - 2.0 * _mean_error * sum_error + double(i) * _mean_error*_mean_error)/double(i));
		_mean_nb_f_evaluations = sum_fe/double(i);
		_log_progress = sum_log/double(i);
		_success_rate = 100.*double(i-_nb_failures)/double(i);

		print(std::cout);
	      }
	    
	    // Free the memory
	    PROBLEM::free();
	    delete algo;
	  }
		     
	// Before leaving, ensure that the statistics are up-to-date;
	_mean_error = sum_error/double(NB_TRIALS);
	_std_error = sqrt((sum_square_errors - 2.0 * _mean_error * sum_error + double(NB_TRIALS) * _mean_error*_mean_error)/double(NB_TRIALS));
	_mean_nb_f_evaluations = sum_fe/double(NB_TRIALS);
	_log_progress = sum_log/double(NB_TRIALS);
	_success_rate = 100.*double(NB_TRIALS-_nb_failures)/double(NB_TRIALS);
      }

      /**
       * Print the statistics
       */
      void print(std::ostream & os) const
      {
	os << "Trial=" << _trial_counter << ";";
	os << "Error(mean)= " << getMeanError() << ";";
	os << "Error(std)= " << getStdError() << ";";
	os << "FE(mean)= " << getMeanFEvaluations() << ";";
	os << "Log_progress(mean)= " << getLogProgress() << ";";
	os << "Best_min= " << getBestFitness() << ";";
	os << "Success_rate= " << getSuccessRate() << "%;";
	os << std::endl;
      }

      friend std::ostream & operator <<(std::ostream & os, const Benchmark &b)
	{
	  b.print(os);
	  return os;
	}
    };
  } // benchmark
} // popot

#endif
