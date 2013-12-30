#include "experiment.h"
#include "boost/timer.hpp"
#include "battleship.h"
#include "rocksample.h"
#include "pedestrian.h"
#include "pedestrian_dynamic_utown.h"
#include "pocman.h"
#include "time.h"
#include <sys/time.h>
#include "pedestrian_changelane.h"
#include "world_simulator.h"

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
    SIMULATOR& simulator, const string& outputFile,
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
PEDESTRIAN_DYNAMIC_REAL*pedproblem_d;

int obs_seq[12]={5,6,7,8,9,10,11,12,13,14,15,16};
int crush_count;


//extern PEDESTRIAN_CHANGELANE sim;
void EXPERIMENT::Run()
{
    boost::timer timer;

    STATE* state = Real.CreateStartState();
	cout<<"Start State Created!"<<endl;

	Real.DisplayState(*state,cout);
    MCTS mcts(Simulator, SearchParams,state);
	//MCTS mcts(Simulator,SearchParams);
	cout<<"MCTS created"<<endl;
	mcts.root_list.push_back(mcts.Root);

	
//    rockproblem=new ROCKSAMPLE(Real.GetSize(),Real.GetNRocks());
//	mcts.rockproblem=rockproblem;
	tigerproblem=new TIGER();
	mcts.tigerproblem=tigerproblem;
	testproblem=new TESTPROBLEM(15);
	testproblem->loadQMDP();
	pedproblem=new PEDESTRIAN(15);
	pedproblem_d=new PEDESTRIAN_DYNAMIC_REAL(9);
	Simulator.DisplayModel();
	
	/*
	testproblem->vnode_list.push_back(mcts.Root);
	HISTORY h_init;
	testproblem->history_list.push_back(h_init);*/
	//mcts.testproblem=testproblem;
	mcts.pedproblem_d=pedproblem_d;
	mcts.tigerproblem=tigerproblem;
	mcts.pedproblem_c=new PEDESTRIAN_CHANGELANE(5,10);
	//mcts.pedproblem_c=pedproblem_c;

    double undiscountedReturn = 0.0;
    double discountedReturn = 0.0;
    double discount = 1.0;
    bool terminal = false;
    bool outOfParticles = false;
    int t;


    if (SearchParams.Verbose >= 1)
        Real.DisplayState(*state, cout);

	bool crashed=false;
    for (t = 0; t < ExpParams.NumSteps; t++)
	//for(t=0;t<1;t++)
    {
	    mcts.MaxTreeDepth=0;
		if(SearchParams.Verbose>=1)
		{
			cout<<"current step : "<<t<<endl;
		}
        OBS_TYPE observation;
		double reward;

		int action = mcts.SelectAction();
		//sim.debug=true;
        terminal = Real.Step(*state, action, observation, reward);
	//	sim.debug=false;
		cout<<"Observation "<<observation<<endl;
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
		if(reward==-50000) crush_count++;
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
			if(reward<=-1000) crashed=true; 
            break;
        }

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
            OBS_TYPE observation;
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
	}
	mcts.root_list.clear();
    Results.Time.Add(timer.elapsed());
    Results.UndiscountedReturn.Add(undiscountedReturn);
    Results.DiscountedReturn.Add(discountedReturn);

	if(crashed)
		cout << "\nTotal  fail reward after " << t+1 << " steps = "<<endl; 
	else
		cout << "\nTotal success reward after " << t+1 << " steps = "<<endl;
}

int N_Ped=1;
void EXPERIMENT::Run_MultiPed()
{
    boost::timer timer;
	STATE* state_list[N_Ped];
	STATE* robot_state=Real.CreateStartState();
	PEDESTRIAN_DYNAMIC_REAL_STATE * rob_state=safe_cast<PEDESTRIAN_DYNAMIC_REAL_STATE*>(robot_state);
	PEDESTRIAN_DYNAMIC_REAL_STATE * ped_state;
	for(int ii=0;ii<N_Ped;ii++)
	{
    	state_list[ii] = Real.CreateStartState();
		ped_state=safe_cast<PEDESTRIAN_DYNAMIC_REAL_STATE*>(state_list[ii]);
		ped_state->RobPos.Y=rob_state->RobPos.Y;
		ped_state->Vel=rob_state->Vel;
	}



	MCTS* mcts_list[N_Ped];
	for(int ii=0;ii<N_Ped;ii++)
	{
		mcts_list[ii]=new MCTS(Simulator,SearchParams,state_list[ii]);
		mcts_list[ii]->pedproblem_d=new PEDESTRIAN_DYNAMIC_REAL(10);	
	}
	



    double undiscountedReturn = 0.0;
    double discountedReturn = 0.0;
	double discount = 1.0;
    bool terminal = false;
    bool outOfParticles = false;

	double last_time;
	int safeAction;
	int t;
	bool crushed=false;
    for (t = 0; t < ExpParams.NumSteps; t++)
    {
		for(int ii=0;ii<N_Ped;ii++) 	mcts_list[ii]->MaxTreeDepth=0;
        OBS_TYPE observation;
        double reward;
		safeAction=1;
		int currAction;
		for(int ii=0;ii<N_Ped;ii++)
		{
			currAction = mcts_list[ii]->SelectAction();
			if( (currAction!=safeAction) && (safeAction!=2) )
			{
				if(currAction==2)
					safeAction = 2;
				else if (currAction==0)
					safeAction =0;
			}		
		}
		

		Real.RobStep(*rob_state,safeAction);
		cout<<"safeAction :"<<safeAction<<endl;
		rob_state=safe_cast<PEDESTRIAN_DYNAMIC_REAL_STATE*>(rob_state);
		Real.DisplayState(*rob_state,cout);
		for(int ii=0;ii<N_Ped;ii++)
		{
			terminal = Real.Step(*(state_list[ii]), safeAction, observation, reward);
			ped_state=safe_cast<PEDESTRIAN_DYNAMIC_REAL_STATE*>(state_list[ii]);
			ped_state->RobPos.Y=rob_state->RobPos.Y;
			ped_state->Vel=rob_state->Vel;

			//manually modify the observation result due to the common robot controller state
			observation=ped_state->Vel*500+ped_state->RobPos.Y*41+ped_state->PedPos.X*11+ped_state->PedPos.Y;
			if(reward<=-1000)
			{
				crush_count++;
				crushed=true;
			}
	       	outOfParticles = !mcts_list[ii]->Update(safeAction, observation, reward);
			if(terminal) break;
			if(outOfParticles) break;
		}

		
        Results.Reward.Add(reward);
        undiscountedReturn += reward;
        discountedReturn += reward * discount;
		discount *= Real.GetDiscount();

        if (terminal)
        {
            cout << "Terminated" << endl;
            break;
        }
        if (outOfParticles)
            break;

    }


    if (outOfParticles)
    {
        cout << "Out of particles, finishing episode with SelectRandom" << endl;
    }

    Results.Time.Add(timer.elapsed());
    Results.UndiscountedReturn.Add(undiscountedReturn);
    Results.DiscountedReturn.Add(discountedReturn);
	if(crushed)
	{
		cout<<"fail horizon "<<t+1<<endl;
	}
	else
	{
		cout<<"success horizon "<<t+1<<endl;
	}
    //Results.DiscountedReturn.Add(t+1);
//	cout << "Discounted return = " << discountedReturn
	cout << "Discounted return = " << discountedReturn 
        << " average = " << Results.DiscountedReturn.GetMean() << endl;
    cout << "Undiscounted return = " << undiscountedReturn
        << " average = " << Results.UndiscountedReturn.GetMean() << endl;

	
	for(int ii=0;ii<N_Ped;ii++)
	{
		if(mcts_list[ii])
		delete mcts_list[ii];
	}

	VNODE::FreeAll();
	cout<<"mcts deleted"<<endl;
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
        OBS_TYPE observation;
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
        OBS_TYPE observation;
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
        OBS_TYPE observation;
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
        OBS_TYPE observation;
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
        OBS_TYPE observation;
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

WorldSimulator world;
PEDESTRIAN_CHANGELANE* MySimulator;

void EXPERIMENT::Plan()
{
	MySimulator=new PEDESTRIAN_CHANGELANE(ModelParams::XSIZE,ModelParams::YSIZE);
	MySimulator->rob_map=world.window.rob_map;
	MySimulator->sfm=&world.sfm;
	PedestrianState startState=world.GetCurrState();
	MySimulator->SetStartState(startState);
	//MCTS::InitFastUCB(SearchParams.ExplorationConstant);

	MCTS mcts(*MySimulator, SearchParams);
	mcts.root_list.push_back(mcts.Root);
	//mcts.pedproblem_c=new PEDESTRIAN_CHANGELANE(5,10);
	mcts.pedproblem_c=MySimulator;

	bool crashed=false;



	cout<<"create solver"<<endl;

	MySimulator->DisplayState(startState,cout);



    double undiscountedReturn = 0.0;
    double discountedReturn = 0.0;
    double discount = 1.0;
    bool terminal = false;
    bool outOfParticles = false;
    int t;
	int action;
	double reward;
	for(t=0;t<ExpParams.NumSteps;t++)  
	{
		cout<<"Run "<<t<<endl;
		action=mcts.SelectAction();
		cout<<"action "<<action<<endl;
		if(world.OneStep(action)) break;	
		world.ShiftWindow();
		if(ModelParams::debug)  cout<<"after shift window"<<endl;
		if(ModelParams::debug)
		{
			MySimulator->DisplayState(world.GetCurrState(),cout);
			world.Display();
		}
		OBS_TYPE obs=world.GetCurrObs();
		MySimulator->rob_map=world.window.rob_map;
		if(ModelParams::debug)  cout<<"observation "<<obs<<endl;
		PedestrianState ped_state=world.GetCurrState();

		MySimulator->SetStartState(ped_state);
		outOfParticles = !mcts.Update(action, obs, reward, &ped_state);
		if(outOfParticles) 
		{
			cout<<"Out Of Particles!"<<endl;
			break;
		}
		
	}
	if(world.InCollision(action))
	{
		if(world.InRealCollision(action))
		{
			cout << "\nTotal  fail reward after " << t << " steps = "<<endl; 
		}
		cout << "\nTotal  danger reward after " << t << " steps = "<<endl; 
	}
	else
		cout << "\nTotal success reward after " << t << " steps = "<<endl;
}


void EXPERIMENT::MultiRun()
{
    //srand(ExpParams.Seed);
	//srand(unsigned(time(0)));
    //rockproblem->Init_QMDP2();
    //rockproblem->WriteQMDP();
    //rockproblem->LoadQMDP();
    //testproblem->display_QMDP();
	//rockproblem->DisplayQMDP();
	//
	
	//tigerproblem=new TIGER();
	/*tigerproblem->Init_QMDP();
	tigerproblem->DisplayQMDP();*/

	crush_count=0;
	world.SetSeed(rand()+2);
	world.NumPedTotal=ExpParams.NPed;
	world.Init();

    for (int n = 0; n < ExpParams.NumRuns; n++)
    {
        cout << "Starting run " << n + 1 << " with "
            << SearchParams.NumSimulations << " simulations... " << endl;
       	//Run();
		Plan();
		//Run_MultiPed();
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
	cout<<"crush count: "<<crush_count<<endl;
}

void EXPERIMENT::DiscountedReturn()
{
    cout << "Main runs" << endl;
    OutputFile << "Simulations\tRuns\tUndiscounted return\tUndiscounted error\tDiscounted return\tDiscounted error\tTime\n";

    SearchParams.MaxDepth = Simulator.GetHorizon(ExpParams.Accuracy, ExpParams.UndiscountedHorizon);
	//SearchParams.MaxDepth=20;
    ExpParams.SimSteps = Simulator.GetHorizon(ExpParams.Accuracy, ExpParams.UndiscountedHorizon);
	//ExpParams.SimSteps=20;
    ExpParams.NumSteps = Real.GetHorizon(ExpParams.Accuracy, ExpParams.UndiscountedHorizon);
	//ExpParams.NumSteps=20;

    for (int i =18; i <= 18; i++)
    {
        SearchParams.NumSimulations = 1 << i;
        //SearchParams.NumStartStates = 1 << i;
		SearchParams.NumStartStates=500; 
        if (i + ExpParams.TransformDoubles >= 0)
            SearchParams.NumTransforms = 1 << (i + ExpParams.TransformDoubles);
        else
            SearchParams.NumTransforms = 1;
        SearchParams.MaxAttempts = SearchParams.NumTransforms * ExpParams.TransformAttempts;

        Results.Clear();
	
		MultiRun();
/*
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
            << Results.Time.GetMean() << endl;*/
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
