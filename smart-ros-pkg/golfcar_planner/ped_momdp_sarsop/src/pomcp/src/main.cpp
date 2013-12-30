#include "mcts.h"
#include "rocksample.h"
#include "pedestrian.h"
#include "pedestrian_dynamic_utown.h"
#include "experiment.h"
#include <boost/program_options.hpp>
#include <iomanip>

using namespace std;
using namespace boost::program_options;


//MyMap  my_map;
//MyWindow my_window(&my_map,0);
//Car car(&my_map);
//SFM sfm_global(&my_map,&my_window);
//PEDESTRIAN_CHANGELANE sim(5,10);

void UnitTests()
{
    cout << "Testing UTILS" << endl;
    UTILS::UnitTest();
    cout << "Testing COORD" << endl;
    COORD::UnitTest();
    cout << "Testing MCTS" << endl;
    MCTS::UnitTest();
}

void disableBufferedIO(void)
{
    setbuf(stdout, NULL);
    setbuf(stdin, NULL);
    setbuf(stderr, NULL);
    setvbuf(stdout, NULL, _IONBF, 0);
    setvbuf(stdin, NULL, _IONBF, 0);
    setvbuf(stderr, NULL, _IONBF, 0);
}



int main(int argc, char* argv[])
{
    MCTS::PARAMS searchParams;
    EXPERIMENT::PARAMS expParams;
    SIMULATOR::KNOWLEDGE knowledge;
    string problem, outputfile, policy;
    int size, number, treeknowledge = 1, rolloutknowledge = 1, smarttreecount = 10;
    double smarttreevalue = 1.0;

    options_description desc("Allowed options");
    desc.add_options()
        ("help", "produce help message")
        ("test", "run unit tests")
        ("problem", value<string>(&problem), "problem to run")
        ("outputfile", value<string>(&outputfile)->default_value("output.txt"), "summary output file")
        ("policy", value<string>(&policy), "policy file (explicit POMDPs only)")
        ("size", value<int>(&size), "size of problem (problem specific)")
        ("number", value<int>(&number), "number of elements in problem (problem specific)")
        ("timeout", value<double>(&expParams.TimeOut), "timeout (seconds)")
		("nped",value<int>(&expParams.NPed),"control the num of ped in this experiment")
		("seed",value<unsigned>(&expParams.Seed),"seed for random number generation")
        ("mindoubles", value<int>(&expParams.MinDoubles), "minimum power of two simulations")
        ("maxdoubles", value<int>(&expParams.MaxDoubles), "maximum power of two simulations")
        ("runs", value<int>(&expParams.NumRuns), "number of runs")
        ("accuracy", value<double>(&expParams.Accuracy), "accuracy level used to determine horizon")
        ("horizon", value<int>(&expParams.UndiscountedHorizon), "horizon to use when not discounting")
        ("numsteps", value<int>(&expParams.NumSteps), "number of steps to run when using average reward")
        ("verbose", value<int>(&searchParams.Verbose), "verbosity level")
        ("autoexploration", value<bool>(&expParams.AutoExploration), "Automatically assign UCB exploration constant")
        ("exploration", value<double>(&searchParams.ExplorationConstant), "Manual value for UCB exploration constant")
        ("usetransforms", value<bool>(&searchParams.UseTransforms), "Use transforms")
        ("transformdoubles", value<int>(&expParams.TransformDoubles), "Relative power of two for transforms compared to simulations")
        ("transformattempts", value<int>(&expParams.TransformAttempts), "Number of attempts for each transform")
        ("userave", value<bool>(&searchParams.UseRave), "RAVE")
        ("ravediscount", value<double>(&searchParams.RaveDiscount), "RAVE discount factor")
        ("raveconstant", value<double>(&searchParams.RaveConstant), "RAVE bias constant")
        ("treeknowledge", value<int>(&knowledge.TreeLevel), "Knowledge level in tree (0=Pure, 1=Legal, 2=Smart)")
        ("rolloutknowledge", value<int>(&knowledge.RolloutLevel), "Knowledge level in rollouts (0=Pure, 1=Legal, 2=Smart)")
        ("smarttreecount", value<int>(&knowledge.SmartTreeCount), "Prior count for preferred actions during smart tree search")
        ("smarttreevalue", value<double>(&knowledge.SmartTreeValue), "Prior value for preferred actions during smart tree search")
        ("disabletree", value<bool>(&searchParams.DisableTree), "Use 1-ply rollout action selection")
		("useqmdp",value<bool>(&searchParams.UseQmdp),"use qmdp")
		("usetime",value<bool>(&searchParams.UseTime),"use time")
		("reuse",value<bool>(&searchParams.Reuse),"belief reuse or not")
		("leavevalue",value<int>(&searchParams.LeaveValue),"leave value, -1 for random rollout")
        ;

    variables_map vm;
    store(parse_command_line(argc, argv, desc), vm);
    notify(vm);
	srand(expParams.Seed);
	//srand(unsigned(time(0)));

	//knowledge.RolloutLevel=2;
    if (vm.count("help"))
    {
        cout << desc << "\n";
        return 1;
    }

    if (vm.count("problem") == 0)
    {
        cout << "No problem specified" << endl;
        return 1;
    }

    if (vm.count("test"))
    {
        cout << "Running unit tests" << endl;
        UnitTests();
        return 0;
    }

    SIMULATOR* real = 0;
    SIMULATOR* simulator = 0;

     if (problem == "rocksample")
    {
        real = new ROCKSAMPLE(size, number);
        simulator = new ROCKSAMPLE(size, number);
    }

	else if(problem=="tiger")
	{
		real=new TIGER(searchParams.UseQmdp);
		simulator=new TIGER(searchParams.UseQmdp);
	}
	else if(problem == "pedestrian")
	{
		real=new PEDESTRIAN(10);
		simulator=new PEDESTRIAN(10);
	}
	/*
	else if(problem == "pedestrian_dynamic")
	{
		
		if(searchParams.UseQmdp)
		{
		    cout<<"use qmdp"<<endl;
			real=new PEDESTRIAN_DYNAMIC_REAL(false,10);
			simulator=new PEDESTRIAN_DYNAMIC_REAL(true,10);
		}
		else
		{
		        cout<<"no qmdp"<<endl;
			real=new PEDESTRIAN_DYNAMIC_REAL(false,10);
			simulator=new PEDESTRIAN_DYNAMIC_REAL(false,10);
		}

					//real=new PEDESTRIAN_DYNAMIC_REAL(9);
			//simulator=new PEDESTRIAN_DYNAMIC_REAL(9);
		real->DisplayModel();
		simulator->DisplayModel();
	}*/
	else if(problem == "pedestrian_changelane")
	{
		simulator=real=new PEDESTRIAN_CHANGELANE(5,10);
		//simulator=new PEDESTRIAN_CHANGELANE(5,10);

		//sim.sfm=&sfm_global;
		//sim.rob_map=my_window.rob_map;
		//real=&sim;
		//simulator=&sim;
	}
    else 
    {
        cout << "Unknown problem" << endl;
        exit(1);
        //while(
    }


    simulator->SetKnowledge(knowledge);
    EXPERIMENT experiment(*real, *simulator, outputfile, expParams, searchParams);
    experiment.DiscountedReturn();

	if(!(problem == "pedestrian_changelane"))  
	{   
		delete real;
		delete simulator;
	}
	return 0;
}
