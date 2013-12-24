#include "experiment.h"
#include "boost/timer.hpp"
#include "battleship.h"
#include "rocksample.h"
#include "pedestrian.h"
#include "pedestrian_changelane.h"
#include "pocman.h"
#include "time.h"
#include <sys/time.h>

using namespace std;

EXPERIMENT::PARAMS::PARAMS()
:   NumRuns(1000),
    NumSteps(100000),
    SimSteps(1000),
    TimeOut(3600),
    MinDoubles(0),
    MaxDoubles(20),
    TransformDoubles(-4),
    TransformAttempts(1000),
    Accuracy(0.01),
    UndiscountedHorizon(1000),
    AutoExploration(true)
{
}

EXPERIMENT::EXPERIMENT(const SIMULATOR& real,
    const SIMULATOR& simulator, const string& outputFile,
    EXPERIMENT::PARAMS& expParams, MCTS::PARAMS& searchParams)
:   Real(real),
    Simulator(simulator),
    OutputFile(outputFile.c_str()),
    ExpParams(expParams),
    SearchParams(searchParams)
{
    if (ExpParams.AutoExploration)
    {
        if (SearchParams.UseRave)
            SearchParams.ExplorationConstant = 0;
        else
            SearchParams.ExplorationConstant = simulator.GetRewardRange();
    }
    MCTS::InitFastUCB(SearchParams.ExplorationConstant);
}
TESTPROBLEM*testproblem;
ROCKSAMPLE*rockproblem;
TIGER*tigerproblem;
PEDESTRIAN*pedproblem;
PEDESTRIAN_DYNAMIC*pedproblem_d;
PEDESTRIAN_CHANGELANE*pedproblem_c;

int obs_seq[12]={5,6,7,8,9,10,11,12,13,14,15,16};
void EXPERIMENT::Run()
{
    boost::timer timer;


    MCTS mcts(Simulator, SearchParams);
	mcts.root_list.push_back(mcts.Root);

	
//    rockproblem=new ROCKSAMPLE(Real.GetSize(),Real.GetNRocks());
//	mcts.rockproblem=rockproblem;
	tigerproblem=new TIGER();
	mcts.tigerproblem=tigerproblem;
	testproblem=new TESTPROBLEM(15);
	testproblem->loadQMDP();
	pedproblem=new PEDESTRIAN(15);
	pedproblem_d=new PEDESTRIAN_DYNAMIC(10);
	pedproblem_c=new PEDESTRIAN_CHANGELANE(10);
	
	/*
	testproblem->vnode_list.push_back(mcts.Root);
	HISTORY h_init;
	testproblem->history_list.push_back(h_init);*/
	//mcts.testproblem=testproblem;
	mcts.pedproblem_d=pedproblem_d;
	//mcts.pedproblem_c=pedproblem_c;

    double undiscountedReturn = 0.0;
    double discountedReturn = 0.0;
    double discount = 1.0;
    bool terminal = false;
    bool outOfParticles = false;
    int t;

	//cout<<"start creating start state"<<endl;
    STATE* state = Real.CreateStartState();
	//cout<<"finish creating start state"<<endl;
    //TESTPROBLEM_STATE* teststate=safe_cast<TESTPROBLEM_STATE*>(state);
    //ROS_INFO("start run loop");
    //teststate->Goal=1;
    if (SearchParams.Verbose >= 1)
        Real.DisplayState(*state, cout);

	int act_seq[3]={0,0,2};
	//int obs_seq[3]={0,0,0};
	//mcts.count_out.open("count.txt");
	//mcts.ucbvalues_out.open("ucb.txt");
	double last_time;
    for (t = 0; t < ExpParams.NumSteps; t++)
	//for(t=0;t<3;t++)
    {
	    mcts.MaxTreeDepth=0;
		if(SearchParams.Verbose>=1)
		{
			cout<<"current step : "<<t<<endl;
		}
        int observation;
        double reward;

		char buf1[50],buf2[50],buf3[50],buf4[50],buf5[50];
		sprintf(buf1,"./data2/Max Tree Depth %d.txt",t);
		sprintf(buf2,"./data2/Listen %d.txt",t);
		sprintf(buf3,"./data2/Open Left %d.txt",t);
		sprintf(buf4,"./data2/Open Right %d.txt",t);
		sprintf(buf5,"./data2/Merge %d.txt",t);

		/*
		mcts.depth_out.open(buf1);
		mcts.listen_out.open(buf2);
		mcts.openl_out.open(buf3);
		mcts.openr_out.open(buf4);*/;

		mcts.merge_out.open(buf5);

		long long L1,L2,L3;
		timeval tv;
		gettimeofday(&tv,NULL);
		L1=tv.tv_sec*1000*1000+tv.tv_usec;	
        
		int action = mcts.SelectAction();

		gettimeofday(&tv,NULL);
		L2=tv.tv_sec*1000*1000+tv.tv_usec;

		printf("total time for this run %lld\n",L2-L1);

			//t=clock()-t;
		//cout<<"totoal time for this run "<<(float)t/CLOCKS_PER_SEC<<endl;
		//action=act_seq[t];
        terminal = Real.Step(*state, action, observation, reward);
		//observation=obs_seq[t];
		/////////////////////////////////////////////////////////////////////////
		/*generate the tree*/
		if (SearchParams.Verbose>=1)
		{
				char buf[20];
				sprintf(buf,"final tree%d.txt",t);
				ofstream out(buf);
				//out<<"hello world";
				mcts.DisplayValue(20,out);
				out.close();
				cout<<"Tree generated!"<<endl;
		}
		/***********************/
		/////////////////////////////////////////////////////////////////////////
		//
        Results.Reward.Add(reward);
        undiscountedReturn += reward;
        discountedReturn += reward * discount;
		discount *= Real.GetDiscount();
		cout<<"time elapsed for current action: "<<timer.elapsed()-last_time<<endl;
		last_time=timer.elapsed();


		if (SearchParams.Verbose>=1)
		{
        	cout<<"current discount factor : "<<discount<<endl;
		}
		if (SearchParams.Verbose >= 1)
        {
            Real.DisplayAction(action, cout);
            Real.DisplayState(*state, cout);
            Real.DisplayObservation(*state, observation, cout);
            Real.DisplayReward(reward, cout);
        }

        if (terminal)
        {
            cout << "Terminated" << endl;
            break;
        }

		//cout<<"start updating believes"<<endl;
		//cout<<"start updating"<<endl;
       	outOfParticles = !mcts.Update(action, observation, reward);
		
		if (SearchParams.Verbose>=1)
		{
				char buf[50];
				sprintf(buf,"final tree after update%d.txt",t);
				ofstream out(buf);
				//out<<"hello world";
				mcts.DisplayValue(20,out);
				out.close();
				cout<<"Tree generated!"<<endl;
		}


		//cout<<"finish updating"<<endl;
		//cout<<"finish updating believes"<<endl;
		/*;

		mcts.depth_out.close();
		mcts.listen_out.close();
		mcts.openl_out.close();
		mcts.openr_out.close();*/
		mcts.merge_out.close();
        //Real.DisplayBeliefs(mcts.BeliefState(),cout);
        if (outOfParticles)
            break;

        if (timer.elapsed() > ExpParams.TimeOut)
        {
            cout << "Timed out after " << t << " steps in "
                << Results.Time.GetTotal() << "seconds" << endl;
            break;
        }
    }


    if (outOfParticles)
    {
        cout << "Out of particles, finishing episode with SelectRandom" << endl;
        HISTORY history = mcts.GetHistory();
        while (++t < ExpParams.NumSteps)
        {
            int observation;
            double reward;

            // This passes real state into simulator!
            // SelectRandom must only use fully observable state
            // to avoid "cheating"
            int action = Simulator.SelectRandom(*state, history, mcts.GetStatus());
            terminal = Real.Step(*state, action, observation, reward);

            Results.Reward.Add(reward);
            undiscountedReturn += reward;
            discountedReturn += reward * discount;
            discount *= Real.GetDiscount();


            if (SearchParams.Verbose >= 1)
            {
                Real.DisplayAction(action, cout);
                Real.DisplayState(*state, cout);
                Real.DisplayObservation(*state, observation, cout);
                Real.DisplayReward(reward, cout);
            }

            if (terminal)
            {
                cout << "Terminated" << endl;
                break;
            }

            history.Add(action, observation);
        }
    }
	for(int i=0;i<mcts.root_list.size();i++)
	{
	//	VNODE::Free(mcts.root_list[i], Simulator);
	}
	mcts.root_list.clear();
    Results.Time.Add(timer.elapsed());
    Results.UndiscountedReturn.Add(undiscountedReturn);
    Results.DiscountedReturn.Add(discountedReturn);
    cout << "Discounted return = " << discountedReturn
        << " average = " << Results.DiscountedReturn.GetMean() << endl;
    cout << "Undiscounted return = " << undiscountedReturn
        << " average = " << Results.UndiscountedReturn.GetMean() << endl;
}





void EXPERIMENT::Run_QMDP()
{
    boost::timer timer;
    double undiscountedReturn = 0.0;
    double discountedReturn = 0.0;
    double discount = 1.0;
    bool terminal = false;
	int t;



    STATE* state = Real.CreateStartState();
    TESTPROBLEM_STATE* teststate=safe_cast<TESTPROBLEM_STATE*>(state);
    //ROS_INFO("start run loop");
    //teststate->Goal=1;
    if (SearchParams.Verbose >= 1)
        Real.DisplayState(*state, cout);
	double last_time=0;
    for (t = 0; t < ExpParams.NumSteps; t++)
    {
		if(SearchParams.Verbose>=1)
		{
			cout<<"current step : "<<t<<endl;

		}
        int observation;
        double reward;
        //int action = mcts.SelectAction();
		//int action=rockproblem->QMDP_SelectAction(state);
		int action=testproblem->QMDP_SelectAction(state);
        terminal = Real.Step(*state, action, observation, reward);

        Results.Reward.Add(reward);
        undiscountedReturn += reward;
        discountedReturn += reward * discount;
		discount *= Real.GetDiscount();
		if (SearchParams.Verbose>=1)
		{
        	cout<<"current discount factor : "<<discount<<endl;
		}
		if (SearchParams.Verbose >= 1)
        {
            Real.DisplayAction(action, cout);
            Real.DisplayState(*state, cout);
            Real.DisplayObservation(*state, observation, cout);
            Real.DisplayReward(reward, cout);
        }

        if (terminal)
        {
            cout << "Terminated" << endl;
            break;
        }
        if (timer.elapsed() > ExpParams.TimeOut)
        {
            cout << "Timed out after " << t << " steps in "
                << Results.Time.GetTotal() << "seconds" << endl;
            break;
        }
    }


    Results.Time.Add(timer.elapsed());
    Results.UndiscountedReturn.Add(undiscountedReturn);
    Results.DiscountedReturn.Add(discountedReturn);
    cout << "Discounted return = " << discountedReturn
        << ", average = " << Results.DiscountedReturn.GetMean() << endl;
    cout << "Undiscounted return = " << undiscountedReturn
        << ", average = " << Results.UndiscountedReturn.GetMean() << endl;
}

void EXPERIMENT::Run_Simple()  //rocksample simple policy
{
    boost::timer timer;
    double undiscountedReturn = 0.0;
    double discountedReturn = 0.0;
    double discount = 1.0;
    bool terminal = false;
    bool outOfParticles = false;
    int t;

    STATE* state = Real.CreateStartState();
    //TESTPROBLEM_STATE* teststate=safe_cast<TESTPROBLEM_STATE*>(state);
    //ROS_INFO("start run loop");
    //teststate->Goal=1;
    if (SearchParams.Verbose >= 1)
        Real.DisplayState(*state, cout);

	STATE* newstate=Real.Copy(*state);
	ROCKSAMPLE_STATE*rockstate=safe_cast<ROCKSAMPLE_STATE*>(newstate);

	int probSize=Real.GetNRocks();//assume size = 8
	for(t=0;t<probSize;t++) 
	{
		if(SearchParams.Verbose>=1)
		{
			cout<<"current step : "<<t<<endl;
		}
        int observation;
        double reward;
        int action = 5+t;
        terminal = Real.Step(*state, action, observation, reward);
		if(observation==1)
		{
			rockstate->Rocks[t].Valuable=true;
		}
		else
		{
			rockstate->Rocks[t].Valuable=false;
		}

        Results.Reward.Add(reward);
        undiscountedReturn += reward;
        discountedReturn += reward * discount;
		discount *= Real.GetDiscount();
		if (SearchParams.Verbose>=1)
		{
        	cout<<"current discount factor : "<<discount<<endl;
		}
		if (SearchParams.Verbose >= 1)
        {
            Real.DisplayAction(action, cout);
            Real.DisplayState(*state, cout);
            Real.DisplayObservation(*state, observation, cout);
            Real.DisplayReward(reward, cout);
        }

	}
    for (t = 0; t < ExpParams.NumSteps-probSize; t++)
    {
		if(SearchParams.Verbose>=1)
		{
			cout<<"current step : "<<t<<endl;
		}
        int observation;
        double reward;
        int action = rockproblem->QMDP_SelectAction(rockstate);

		Real.Step(*rockstate, action, observation, reward);
        terminal = Real.Step(*state, action, observation, reward);

        Results.Reward.Add(reward);
        undiscountedReturn += reward;
        discountedReturn += reward * discount;
		discount *= Real.GetDiscount();
		if (SearchParams.Verbose>=1)
		{
        	cout<<"current discount factor : "<<discount<<endl;
		}
		if (SearchParams.Verbose >= 1)
        {
            Real.DisplayAction(action, cout);
            Real.DisplayState(*state, cout);
            Real.DisplayObservation(*rockstate, observation, cout);
            Real.DisplayReward(reward, cout);
        }
        if (terminal)
        {
            cout << "Terminated" << endl;
            break;
        }
    }
	
	
	Results.Time.Add(timer.elapsed());
    Results.UndiscountedReturn.Add(undiscountedReturn);
    Results.DiscountedReturn.Add(discountedReturn);
    cout << "Discounted return = " << discountedReturn
        << ", average = " << Results.DiscountedReturn.GetMean() << endl;
    cout << "Undiscounted return = " << undiscountedReturn
        << ", average = " << Results.UndiscountedReturn.GetMean() << endl;

}

void EXPERIMENT::Run_Tiger()
{
    boost::timer timer;
    double undiscountedReturn = 0.0;
    double discountedReturn = 0.0;
    double discount = 1.0;
    bool terminal = false;
	int t;

	
    STATE* state = Real.CreateStartState();
    //TESTPROBLEM_STATE* teststate=safe_cast<TESTPROBLEM_STATE*>(state);
    //ROS_INFO("start run loop");
    //teststate->Goal=1;
    if (SearchParams.Verbose >= 1)
        Real.DisplayState(*state, cout);
    for (t = 0; t < ExpParams.NumSteps; t++)
    {
		if(SearchParams.Verbose>=1)
		{
			cout<<"current step : "<<t<<endl;

		}
        int observation;
        double reward;
        //int action = mcts.SelectAction();
		int action=tigerproblem->QMDP_SelectAction();
        terminal = Real.Step(*state, action, observation, reward);
		tigerproblem->UpdateBelief(action,observation);
		

        Results.Reward.Add(reward);
        undiscountedReturn += reward;
        discountedReturn += reward * discount;
		discount *= Real.GetDiscount();
		if (SearchParams.Verbose>=1)
		{
        	cout<<"current discount factor : "<<discount<<endl;
		}
		if (SearchParams.Verbose >= 1)
        {
            Real.DisplayAction(action, cout);
            Real.DisplayState(*state, cout);
            Real.DisplayObservation(*state, observation, cout);
            Real.DisplayReward(reward, cout);
        }

        if (terminal)
        {
            cout << "Terminated" << endl;
            break;
        }

        if (timer.elapsed() > ExpParams.TimeOut)
        {
            cout << "Timed out after " << t << " steps in "
                << Results.Time.GetTotal() << "seconds" << endl;
            break;
        }
    }


    Results.Time.Add(timer.elapsed());
    Results.UndiscountedReturn.Add(undiscountedReturn);
    Results.DiscountedReturn.Add(discountedReturn);
    cout << "Discounted return = " << discountedReturn
        << ", average = " << Results.DiscountedReturn.GetMean() << endl;
    cout << "Undiscounted return = " << undiscountedReturn
        << ", average = " << Results.UndiscountedReturn.GetMean() << endl;
}

void EXPERIMENT::Run_Tiger_Handcraft_Policy()
{
	boost::timer timer;
    double undiscountedReturn = 0.0;
    double discountedReturn = 0.0;
    double discount = 1.0;
    bool terminal = false;
	int t;

	
    STATE* state = Real.CreateStartState();
    //TESTPROBLEM_STATE* teststate=safe_cast<TESTPROBLEM_STATE*>(state);
    //ROS_INFO("start run loop");
    //teststate->Goal=1;
    if (SearchParams.Verbose >= 1)
        Real.DisplayState(*state, cout);

	int pos=0;
    for (t = 0; t < ExpParams.NumSteps; t++)
    {
		if(SearchParams.Verbose>=1)
		{
			cout<<"current step : "<<t<<endl;
		}
        int observation;
        double reward;
        //int action = mcts.SelectAction();
		int action;
		if(pos>=2)
		{
			action=1;
		}
		else if(pos<=-2)
		{
			action=2;
		}
		else
		{
			action=0;
		}
		//int action=tigerproblem->QMDP_SelectAction();
        terminal = Real.Step(*state, action, observation, reward);
		//tigerproblem->UpdateBelief(action,observation);
		if(action==0&&observation==0)
		{
			pos--;	
		}
		else  if(action==0&&observation==1)
		{
			pos++;
		}
		else
		{
			pos=0;
		}

        Results.Reward.Add(reward);
        undiscountedReturn += reward;
        discountedReturn += reward * discount;
		discount *= Real.GetDiscount();
		if (SearchParams.Verbose>=1)
		{
        	cout<<"current discount factor : "<<discount<<endl;
		}
		if (SearchParams.Verbose >= 1)
        {
            Real.DisplayAction(action, cout);
            Real.DisplayState(*state, cout);
            Real.DisplayObservation(*state, observation, cout);
            Real.DisplayReward(reward, cout);
        }

        if (terminal)
        {
            cout << "Terminated" << endl;
            break;
        }

        if (timer.elapsed() > ExpParams.TimeOut)
        {
            cout << "Timed out after " << t << " steps in "
                << Results.Time.GetTotal() << "seconds" << endl;
            break;
        }
    }


    Results.Time.Add(timer.elapsed());
    Results.UndiscountedReturn.Add(undiscountedReturn);
    Results.DiscountedReturn.Add(discountedReturn);
    cout << "Discounted return = " << discountedReturn
        << ", average = " << Results.DiscountedReturn.GetMean() << endl;
    cout << "Undiscounted return = " << undiscountedReturn
        << ", average = " << Results.UndiscountedReturn.GetMean() << endl;
	
}

void EXPERIMENT::MultiRun()
{
    //srand(ExpParams.Seed);
	srand(unsigned(time(0)));
    //rockproblem->Init_QMDP2();
    //rockproblem->WriteQMDP();
    //rockproblem->LoadQMDP();
    //testproblem->display_QMDP();
	//rockproblem->DisplayQMDP();
	//
	
	//tigerproblem=new TIGER();
	/*tigerproblem->Init_QMDP();
	tigerproblem->DisplayQMDP();*/


    for (int n = 0; n < ExpParams.NumRuns; n++)
    {
        cout << "Starting run " << n + 1 << " with "
            << SearchParams.NumSimulations << " simulations... " << endl;
       	Run();
		//Run_QMDP();
		//Run_Simple();
		//Run_Tiger();
		//Run_Tiger_Handcraft_Policy();
		//tigerproblem->Reset();
        if (Results.Time.GetTotal() > ExpParams.TimeOut)
        {
            cout << "Timed out after " << n << " runs in "
                << Results.Time.GetTotal() << "seconds" << endl;
            break;
        }
    }
}

void EXPERIMENT::DiscountedReturn()
{
    cout << "Main runs" << endl;
    OutputFile << "Simulations\tRuns\tUndiscounted return\tUndiscounted error\tDiscounted return\tDiscounted error\tTime\n";

    SearchParams.MaxDepth = Simulator.GetHorizon(ExpParams.Accuracy, ExpParams.UndiscountedHorizon);
    ExpParams.SimSteps = Simulator.GetHorizon(ExpParams.Accuracy, ExpParams.UndiscountedHorizon);
    ExpParams.NumSteps = Real.GetHorizon(ExpParams.Accuracy, ExpParams.UndiscountedHorizon);

    for (int i =18; i <= 18; i++)
    {
        SearchParams.NumSimulations = 1 << i;
        //SearchParams.NumStartStates = 1 << i;
		SearchParams.NumStartStates=50000; 
        if (i + ExpParams.TransformDoubles >= 0)
            SearchParams.NumTransforms = 1 << (i + ExpParams.TransformDoubles);
        else
            SearchParams.NumTransforms = 1;
        SearchParams.MaxAttempts = SearchParams.NumTransforms * ExpParams.TransformAttempts;

        Results.Clear();
	
		MultiRun();

		cout << "Simulations = " << SearchParams.NumSimulations << endl
				<< "Runs = " << Results.Time.GetCount() << endl
				<< "Undiscounted return = " << Results.UndiscountedReturn.GetMean()
				<< " +- " << Results.UndiscountedReturn.GetStdErr() << endl
            << "Discounted return = " << Results.DiscountedReturn.GetMean()
            << " +- " << Results.DiscountedReturn.GetStdErr() << endl
            << "Time = " << Results.Time.GetMean() << endl;
        OutputFile << SearchParams.NumSimulations << "\t"
            << Results.Time.GetCount() << "\t"
            << Results.UndiscountedReturn.GetMean() << "\t"
            << Results.UndiscountedReturn.GetStdErr() << "\t"
            << Results.DiscountedReturn.GetMean() << "\t"
            << Results.DiscountedReturn.GetStdErr() << "\t"
            << Results.Time.GetMean() << endl;
    }
}

void EXPERIMENT::AverageReward()
{
    cout << "Main runs" << endl;
    OutputFile << "Simulations\tSteps\tAverage reward\tAverage time\n";

    SearchParams.MaxDepth = Simulator.GetHorizon(ExpParams.Accuracy, ExpParams.UndiscountedHorizon);
    ExpParams.SimSteps = Simulator.GetHorizon(ExpParams.Accuracy, ExpParams.UndiscountedHorizon);

    for (int i = ExpParams.MinDoubles; i <= ExpParams.MaxDoubles; i++)
    {
        SearchParams.NumSimulations = 1 << i;
        SearchParams.NumStartStates = 1 << i;
        if (i + ExpParams.TransformDoubles >= 0)
            SearchParams.NumTransforms = 1 << (i + ExpParams.TransformDoubles);
        else
            SearchParams.NumTransforms = 1;
        SearchParams.MaxAttempts = SearchParams.NumTransforms * ExpParams.TransformAttempts;

        Results.Clear();
        Run();

        cout << "Simulations = " << SearchParams.NumSimulations << endl
            << "Steps = " << Results.Reward.GetCount() << endl
            << "Average reward = " << Results.Reward.GetMean()
            << " +- " << Results.Reward.GetStdErr() << endl
            << "Average time = " << Results.Time.GetMean() / Results.Reward.GetCount() << endl;
        OutputFile << SearchParams.NumSimulations << "\t"
            << Results.Reward.GetCount() << "\t"
            << Results.Reward.GetMean() << "\t"
            << Results.Reward.GetStdErr() << "\t"
            << Results.Time.GetMean() / Results.Reward.GetCount() << endl;
    }
}

//----------------------------------------------------------------------------
