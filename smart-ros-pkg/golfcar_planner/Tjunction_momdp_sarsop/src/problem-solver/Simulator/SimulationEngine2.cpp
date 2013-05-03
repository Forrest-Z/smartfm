#include <sstream>
#include <fstream>
#include "SimulationEngine2.h"
#include "AlphaVectorPolicy.h"
#include "CPTimer.h"
#include "solverUtils.h"
#include "PrintTuple.h"

using namespace std;
using namespace momdp;

namespace momdp
{
    SimulationEngine2::SimulationEngine2()
    {
    }

    SimulationEngine2::~SimulationEngine2(void)
    {
    }

    void SimulationEngine2::checkTerminal(string p, string s, vector<int> &bhout, vector<int> &fhout) {
        if (s.substr(0,3) == "bt2") {
            if (s.substr(9,2) == "FH") {
                int ind = atoi(p.substr(4,1).c_str());
                fhout[ind]++;
            } else if (s.substr(9,2) == "BH") {
                int ind = atoi(p.substr(4,1).c_str());
                bhout[ind]++;
            }
        }
    }

    int SimulationEngine2::getGreedyAction(vector<int> &bhout, vector<int> &fhout) {
        int greedyAction = 2; //start with BHL
        int currBest = bhout[0];

        vector<int> temp;
        for (int i=0;i<(int)bhout.size();i++){
            temp.push_back(bhout[i]);
        }
        for (int i=0;i<(int)fhout.size();i++){
            temp.push_back(fhout[i]);
        }

        for (int i=1; i<(int)temp.size();i++){
            if (temp[i]>currBest) {
                currBest = temp[i];
                greedyAction = 2+i;
            }
        }

        return greedyAction;
    }

    void SimulationEngine2::setup(SharedPointer<MOMDP> problem, SharedPointer<AlphaVectorPolicy> policy, SolverParams * solverParams)
    {
        this->policy = policy;
        this->problem = problem;
        this->solverParams = solverParams;
    }

    void SimulationEngine2::performActionObs(belief_vector& outBelObs, int action, const BeliefWithState& belSt) const
    {
        // DEBUG_SIMSPEED_270409 skip calculating outprobs for x when there is only one possible x value
        if (problem->XStates->size() == 1)
        {
            // clear out the entries
            outBelObs.resize(1);
            outBelObs.push_back(0,1.0);
        }
        else
        {
            //problem->getTransitionMatrixX(action, belSt.sval);
	    const SharedPointer<SparseMatrix>  transMatX = problem->XTrans->getMatrix(action, belSt.sval);
            mult(outBelObs, *belSt.bvec, *transMatX);
	}
    }

  void SimulationEngine2::performActionUnobs(belief_vector& outBelUnobs, int action, const BeliefWithState& belSt, int currObsState) const
  {
        const SharedPointer<SparseMatrix>  transMatY = problem->YTrans->getMatrix(action, belSt.sval, currObsState);
        mult(outBelUnobs, *belSt.bvec, *transMatY);
  }

    void SimulationEngine2::getPossibleObservations(belief_vector& possObs, int action, 	const BeliefWithState& belSt) const
    {
        //const SparseMatrix obsMat = problem->getObservationMatrix(action, belSt.sval);
		const SharedPointer<SparseMatrix>  obsMat = problem->obsProb->getMatrix(action, belSt.sval);
        mult(possObs,  *belSt.bvec, *obsMat);
    }


    double SimulationEngine2::getReward(const BeliefWithState& belst, int action)
    {
        //const SparseMatrix rewMat = problem->getRewardMatrix(belst.sval);
		const SharedPointer<SparseMatrix>  rewMat = problem->rewards->getMatrix(belst.sval);
        return inner_prod_column(*rewMat, action, *belst.bvec);
    }

    string SimulationEngine2::toString()
    {
        std::ostringstream mystrm;
        mystrm << "action selector: (replaced by Policy) ";
        return mystrm.str();
    }

    void SimulationEngine2::display(belief_vector& b, ostream& s)
    {
        for(unsigned int i = 0; i < b.filled(); i++)
        {
            s << b.data[i].index << " -> " << b.data[i].value << endl;
        }
    }

	double getDist(string XState, int& rx)
	{
		int px, py,  ry;
		double dist=-1;
		//cout << "XState " << XState << endl;

		string temp("strRob");
		char junk[20];
		if( string::npos !=  XState.find(temp))
		{
			/// XState has str .. skip this state
			dist = -10; /// reached goal
		}
		else if( string::npos !=  XState.find("strs"))
		{
			/// XState has str .. skip this state
			dist = -1; /// ped reached goal
		}
		else
		{
			sscanf(XState.c_str(),"sx%dy%dsR%d%s", &px, &py, &ry, junk);

			dist = fabs(px-rx) + fabs(py-ry);

			if(0==dist)
			{
				cout << "XState " << XState << " px:" << px << " rx:" << rx  << endl;
			}

			//if( (px==rx) && (py==ry))
				//dist=0;
			//else
				//dist = py - ry;

		}

		return dist;

	}

    int SimulationEngine2::runFor(int iters, ofstream* streamOut, double& reward, double& expReward, bool& algo, int& rx, ofstream& statfile)
    {
		/// algo ==>  0:momdp 1:ml
		map<int, string> XStateMap;
		/// Mapping states for quick reference
		for(int i = 0 ; i < problem->XStates->size() ; i ++)
		{
			//mapFile << "State : " << i <<  endl;
			//cout << "State : " << i <<  endl;
			map<string, string> obsState = problem->getFactoredObservedStatesSymbols(i);
			string state_str;
			for(map<string, string>::iterator iter = obsState.begin() ; iter != obsState.end() ; iter ++)
			{
				//cout << iter->first << " : " << iter->second << endl ;
				state_str.append(iter->second);
			}
			XStateMap[i]=state_str;
			//cout << "XStateMap[" << i <<"] = " << state_str << endl;
		}

        DEBUG_TRACE(cout << "runFor" << endl; );
        DEBUG_TRACE(cout << "iters " << iters << endl; );
        // DEBUG_TRACE(cout << "startVec sval " << startVec.sval << endl; );
        // DEBUG_TRACE(startVec.bvec->write(cout) << endl;);
        DEBUG_TRACE(cout << "startBeliefX" << endl; );
        DEBUG_TRACE(startBeliefX.write(cout) << endl;);

        //    double reward = 0, expReward = 0;
        bool enableFiling = false;
        if(streamOut == NULL)
        {
            enableFiling = false;
        }
        else
        {
            enableFiling = true;
        }

        // actual system state
        SharedPointer<BeliefWithState> actStateCompl (new BeliefWithState());
        SharedPointer<BeliefWithState> actNewStateCompl (new BeliefWithState());

        // policy follower state
        // belief with state
        SharedPointer<BeliefWithState> nextBelSt;
        SharedPointer<BeliefWithState> currBelSt (new BeliefWithState());// for policy follower based on known x value
									 // set sval to -1 if x value is not known

        // belief over x. May not be used depending on model type and commandline flags, but declared here anyways.
        DenseVector currBelX; // belief over x

        // get starting actStateCompl, the complete state (X and Y values)
        if (problem->initialBeliefStval->sval == -1) // check if the initial starting state for X is fixed
        {
          // random starting state for X
          const SharedPointer<DenseVector>& startBeliefX = problem->initialBeliefX;
          actStateCompl->sval = chooseFromDistribution(*startBeliefX);
          copy(currBelX, *startBeliefX);
        }
        else
        {
          // initial starting state for X is fixed
          actStateCompl->sval = problem->initialBeliefStval->sval;
          /// TBP : I think here the state val from real hardware must come
        }

        // now choose a starting unobserved state for the actual system
        SharedPointer<SparseVector> startBeliefVec;
        if (problem->initialBeliefStval->bvec)
          startBeliefVec = problem->initialBeliefStval->bvec;
        else
          startBeliefVec = problem->initialBeliefYByX[actStateCompl->sval];
        int currUnobsState = chooseFromDistribution(*startBeliefVec); /// TBP : initializing belief for Y
        int belSize = startBeliefVec->size();

        actStateCompl->bvec->resize(belSize);
        actStateCompl->bvec->push_back(currUnobsState, 1.0);

        DEBUG_TRACE( cout << "actStateCompl sval " << actStateCompl->sval << endl; );
        DEBUG_TRACE( actStateCompl->bvec->write(cout) << endl; );

        currBelSt->sval = actStateCompl->sval;
        copy(*currBelSt->bvec, *startBeliefVec);

        DEBUG_TRACE( cout << "currBelSt sval " << currBelSt->sval << endl; );
        DEBUG_TRACE( currBelSt->bvec->write(cout) << endl; );

        double mult=1;
        CPTimer lapTimer;

        // we now have actStateCompl (starting stateUnobs&stateObs) for "actual state" system
	// "policy follower" system has currBelSt (starting beliefUnobs&stateObs) OR currBelSt (starting beliefUnobs&-1) and currBelX (starting beliefObs) if initial x is a belief and not a state

        unsigned int firstAction;

        double gamma = problem->getDiscount();

        int xDim = 3;
        vector<int> bhout(xDim,0);
        vector<int> fhout(xDim,0);
        for(int timeIter = 0; timeIter < iters; timeIter++)
        {
            DEBUG_TRACE( cout << "timeIter " << timeIter << endl; );

            //cout << "--------- timeIter  " << timeIter << " ---------- " << endl;

            if(enableFiling && timeIter == 0)
            {
                *streamOut << ">>> begin\n";
            }

            // get action according to policy and current belief and state
            int currAction;
            //-----------------------------
            if (timeIter == 0)
            {

               if(solverParams->useLookahead) /// TBP : always using look ahead
                {
					if (currBelSt->sval == -1) // special case for first time step where X is a distribution
						currAction = policy->getBestActionLookAhead(currBelSt->bvec, currBelX);
					else
						currAction = policy->getBestActionLookAhead(*currBelSt); /// TBP : get action from policy using lookahead
                }
                else
                {
					if (currBelSt->sval == -1) // special case for first time step where X is a distribution
						currAction = policy->getBestAction(currBelSt->bvec, currBelX);
					else
						currAction = policy->getBestAction(*currBelSt); /// TBP : get action from policy and belief
                }
            }
            else
            {

                if(solverParams->useLookahead)
                {
					currAction = policy->getBestActionLookAhead(*currBelSt);

                }
                else
                {
					currAction = policy->getBestAction(*currBelSt);
                }
            }

            //if (currAction>1 && currAction<8) {
            //    currAction = getGreedyAction(bhout, fhout);
            //    //currAction = (rand()%6)+2;
            //    //cout<<"GREEDY ACTION: "<<currAction<<endl;
            //}

			bool samesame=false;
			SharedPointer<BeliefWithState> currMLBel (new BeliefWithState());
			if(algo)
			{
				//cout << " Most Likely algorithm " << endl;
				int mostProbY  = currBelSt->bvec->argmax(); 	//get the most probable Y state
				double prob = (currBelSt)->bvec->operator()(mostProbY);	//get its probability


				//SparseVector currMLBel;
				//copy(currMLBel, currBelSt);
				copy(*currMLBel->bvec, *currBelSt->bvec);

				int resize = (currBelSt)->bvec->size();
				currMLBel->bvec->resize(resize);
				currMLBel->bvec->push_back(mostProbY, 1.0);
				currMLBel->sval = currBelSt->sval;

				//int mlcurrAction = policy->getBestActionLookAhead(*currMLBel);
				int mlcurrAction = policy->getBestAction(*currMLBel);

				if(mlcurrAction==currAction)
				{
					//cout << "Ml and MOMDP have same actions here " << endl;
					samesame=true;
				}
				else
					currAction = mlcurrAction;
			}




            if(currAction < 0 )
            {
                cout << "You are using a MDP Policy, please make sure you are using a MDP policy together with one-step look ahead option turned on" << endl;
                return -1;
            }
            if (timeIter == 0)
            {
                firstAction = currAction;

            }
            //cout << "currentAction " << currAction << endl;

			double dist = getDist( XStateMap[actStateCompl->sval], rx);

			char act[10];
			if(currAction==1)
				sprintf(act, " aAcc ");
			else if(currAction==2)
				sprintf(act, " aDec ");
			else	if(currAction==0)
				sprintf(act, " aCru ");
			else
				cout << " ERROR : unknown action @ SimulationEngine2 runFor " << endl;



			if(dist>-1)
			{
				cout << endl  << " XState " << XStateMap[actStateCompl->sval] << " action: "  << act << " ";
				currBelSt->bvec->write(cout);


				if(algo)
				{
					cout << " ML Belief " ;
					currMLBel->bvec->write(cout);

					if(samesame)
						cout << " same action as momdp ";
				}
			}

			if(dist == -10)
			{
				cout << "Reached terminal state " << endl;
				statfile << XStateMap[actStateCompl->sval] << " " << timeIter << " " << dist << " " << act <<  " G " << endl << endl << endl;


				break;
			}
			else //if( (dist>-1))
			{
				statfile << XStateMap[actStateCompl->sval] <<  " " << timeIter << " " << dist << " " << act;
				if( dist == 0 )
				{
					cout << " Collision @  " << XStateMap[actStateCompl->sval] << endl;
					statfile << " X " << endl << endl << endl;		 /// collision
					break;
				}
				else if(dist>-1)
				{
					cout << " dist " << dist << " no collision " << endl;
					statfile << " O " << endl; /// safe
				}
				else if (dist ==-1)
				{
					cout << " dist " << dist << " ped reached goal"  << endl;
					statfile << " O " << endl; /// safe
				}

			}
			//else
				//statfile << " dist is screwed up " << endl;

				currBelSt->bvec->write(statfile);
				if(algo)
				{
					statfile << endl << " ML Belief " ;
					currMLBel->bvec->write(statfile);
				}
				statfile << endl;


            // this is the reward for the "actual state" system
            double currReward = getReward(*actStateCompl, currAction);

            DEBUG_TRACE( cout << "currAction " << currAction << endl; );
            DEBUG_TRACE( cout << "actStateCompl sval " << actStateCompl->sval << endl; );
            DEBUG_TRACE( actStateCompl->bvec->write(cout) << endl; );

            DEBUG_TRACE( cout << "currReward " << currReward << endl; );
            expReward += mult*currReward;
            mult *= gamma;
            reward += currReward;

            DEBUG_TRACE( cout << "expReward " << expReward << endl; );
            DEBUG_TRACE( cout << "reward " << reward << endl; );


	    // actualActionUpdObs is belief of the fully observered state
            belief_vector actualActionUpdUnobs(belSize), actualActionUpdObs(problem->XStates->size()) ;
            performActionObs(actualActionUpdObs, currAction, *actStateCompl);

            DEBUG_TRACE( cout << "actualActionUpdObs " << endl; );
            DEBUG_TRACE( actualActionUpdObs.write(cout) << endl; );

			//cout << "actualActionUpdObs " ; actualActionUpdObs.write(cout) << endl;
            // the actual next state for the observed variable
            actNewStateCompl->sval = (unsigned int) chooseFromDistribution(actualActionUpdObs, ((double)rand()/RAND_MAX)); /// TBP : This is for moving the simulator world forward, we get this from real world.

	    // now update actualActionUpdUnobs, which is the belief of unobserved states,
	    // based on prev belif and curr observed state
			performActionUnobs(actualActionUpdUnobs, currAction, *actStateCompl, actNewStateCompl->sval); /// TBP : update bel Y

			DEBUG_TRACE( cout << "actualActionUpdUnobs " << endl; );
            DEBUG_TRACE( actualActionUpdUnobs.write(cout) << endl; );

            // the actual next state for the unobserved variable
            int newUnobsState = chooseFromDistribution(actualActionUpdUnobs, ((double)rand()/RAND_MAX));
            DEBUG_TRACE( cout << "newUnobsState "<< newUnobsState << endl; );

            actNewStateCompl->bvec->resize(belSize);
            actNewStateCompl->bvec->push_back(newUnobsState, 1.0);

            DEBUG_TRACE( cout << "actNewStateCompl sval "<< actNewStateCompl->sval << endl; );
            DEBUG_TRACE( actNewStateCompl->bvec->write(cout) << endl; );

            // get observations based on actual next states for observed and unobserved variable
            belief_vector obsPoss;
            getPossibleObservations(obsPoss, currAction, *actNewStateCompl);


            DEBUG_TRACE( cout << "obsPoss"<< endl; );
            DEBUG_TRACE( obsPoss.write(cout) << endl; );

            int currObservation = chooseFromDistribution(obsPoss, ((double)rand()/RAND_MAX)); /// TBP : get current observation from the real world

            DEBUG_TRACE( cout << "currObservation "<< currObservation << endl; );
			//cout << "currObservation "<< currObservation << endl;
            //*************************************************************
            map<string, string> aa = problem->getActionsSymbols(currAction);
            //cout<<"CURRENT ACTION INDEX: "<<currAction<<" ";
            //cout<<aa["pOneAction"]<<endl;
            //cout<<"OBS ";
            map<string, string> bb = problem->getObservationsSymbols(currObservation);
            map<string, string> cc = problem->getFactoredObservedStatesSymbols(actStateCompl->sval);
            //cout<<cc["pTwo_0"]<<" "<<bb["obs_ballDp2S"]<<endl;
            //cout<<"BEFORE";
            for (int ii=0;ii<xDim;ii++){
                //cout<<" BHOUT "<<bhout[ii];
            }
            for (int ii=0;ii<xDim;ii++){
                //cout<<" FHOUT "<<fhout[ii];
            }
            //cout<<endl;
            checkTerminal(cc["pTwo_0"],bb["obs_ballDp2S"],bhout,fhout);
            //cout<<"AFTER ";
            for (int ii=0;ii<xDim;ii++){
                //cout<<" BHOUT "<<bhout[ii];
            }
            for (int ii=0;ii<xDim;ii++){
                //cout<<" FHOUT "<<fhout[ii];
            }
            //cout<<endl;
            //*************************************************************

            if(enableFiling)
            {
			//initial states and belief, before any action
                if (timeIter == 0)
                {
				//actual X state, X might be a distribution at first time step
                    map<string, string> obsState = problem->getFactoredObservedStatesSymbols(actStateCompl->sval);
					if(obsState.size()>0)
					{
						streamOut->width(4);*streamOut<<left<<"X"<<":";
						printTuple(obsState, *streamOut);
					}

    		    //actual Y state
					streamOut->width(4);*streamOut<<left<<"Y"<<":";
							map<string, string> unobsState = problem->getFactoredUnobservedStatesSymbols(currUnobsState);
					printTuple(unobsState, *streamOut);

				// if initial belief X is a distribution at first time step
					if (currBelSt->sval == -1)
					{
						SparseVector currBelXSparse;
						copy(currBelXSparse, currBelX);
						int mostProbX  = currBelXSparse.argmax(); 	//get the most probable Y state
						streamOut->width(4);*streamOut<<left<<"ML X"<<":";
						map<string, string> mostProbXState = problem->getFactoredObservedStatesSymbols(mostProbX);
						printTuple(mostProbXState, *streamOut);
					}

				//initial belief Y state
					int mostProbY  = currBelSt->bvec->argmax(); 	//get the most probable Y state
					double prob = currBelSt->bvec->operator()(mostProbY);	//get its probability
					streamOut->width(4);*streamOut<<left<<"ML Y"<<":";
							map<string, string> mostProbYState = problem->getFactoredUnobservedStatesSymbols(mostProbY);
					printTuple(mostProbYState, *streamOut);

				/// -> TBP
					streamOut->width(4);*streamOut << left << "BL Y"<<":(";
					//printTuple(currBelSt->bvec, *streamOut);
					//display(currBelSt->bvec, *streamOut);
					//currBelSt->bvec->write(*streamOut); *streamOut << endl;
					*streamOut << currBelSt->bvec->data.size() <<",";
					for(std::vector<SparseVector_Entry>::const_iterator iter = currBelSt->bvec->data.begin();iter != currBelSt->bvec->data.end(); iter++)
					{
						//*streamOut << iter->value;
						*streamOut << iter->index << "="<< iter->value;
						if(iter < currBelSt->bvec->data.end()-1){
						*streamOut <<", ";
						}
						else{
						*streamOut << ")";
						}
					}
					*streamOut << endl;
				/// <- TBP

                }

				streamOut->width(4);*streamOut<<left<<"A"<<":";
						map<string, string> actState = problem->getActionsSymbols(currAction);
				printTuple(actState, *streamOut);

				streamOut->width(4);*streamOut<<left<<"R"<<":";
				*streamOut << currReward<<endl;
            }

            // now that we have the action, state of observed variable, and observation,
            // we can update the belief of unobserved variable
			if (timeIter == 0)
			{
				// check to see if the initial X is a distribution or a known state
				if (currBelSt->sval == -1) // special case for first time step where X is a distribution
					nextBelSt = problem->beliefTransition->nextBelief(currBelSt->bvec, currBelX, currAction, currObservation, actNewStateCompl->sval);
				else
					nextBelSt = problem->beliefTransition->nextBelief(currBelSt, currAction, currObservation, actNewStateCompl->sval);
			}
			else
				nextBelSt = problem->beliefTransition->nextBelief(currBelSt, currAction, currObservation, actNewStateCompl->sval);

            //problem->getNextBeliefStval(nextBelSt, currBelSt, currAction, currObservation, actNewStateCompl->sval);


            if(enableFiling)
            {
                if(timeIter == iters - 1)
                {
                    *streamOut << "terminated\n";
                }

		//actual X state after action
		map<string, string> obsState = problem->getFactoredObservedStatesSymbols(actNewStateCompl->sval);
		if(obsState.size()>0){
		    streamOut->width(4);*streamOut<<left<<"X"<<":";
		    printTuple(obsState, *streamOut);
		}

		//actual Y state after action
		streamOut->width(4);*streamOut<<left<<"Y"<<":";
		map<string, string> unobsState = problem->getFactoredUnobservedStatesSymbols(newUnobsState);
		printTuple(unobsState, *streamOut);

		//observation after action
		streamOut->width(4);*streamOut<<left<<"O"<<":";
                map<string, string> obs = problem->getObservationsSymbols(currObservation);
		printTuple(obs, *streamOut);

		//get most probable Y state from belief after applying action A
		int mostProbY  = currBelSt->bvec->argmax(); 	//get the most probable Y state
		double prob = nextBelSt->bvec->operator()(mostProbY);	//get its probability
		streamOut->width(4);*streamOut<<left<<"ML Y"<<":";
                map<string, string> mostProbYState = problem->getFactoredUnobservedStatesSymbols(mostProbY);
		printTuple(mostProbYState, *streamOut);

                if(timeIter == iters - 1)
                {
                    //timeval timeInRunFor = getTime() - prevTime;
                    //*streamOut << "----- time: " << timevalToSeconds(timeInRunFor) <<endl;
                    double lapTime = lapTimer.elapsed();
                    *streamOut << "----- time: " << lapTime <<endl;
                }
            }
	    //actual states
            currUnobsState = newUnobsState; //Y state, hidden
			actStateCompl->sval = actNewStateCompl->sval;
            copy(*actStateCompl->bvec, *actNewStateCompl->bvec);

	    //belief states
            copy(*currBelSt->bvec, *nextBelSt->bvec);
            currBelSt->sval = nextBelSt->sval;



			/// -> TBP
				//streamOut->width(4);*streamOut << left << "BL Y"<<":(";
				////printTuple(currBelSt->bvec, *streamOut);
				////display(currBelSt->bvec, *streamOut);
				////currBelSt->bvec->write(*streamOut); *streamOut << endl;
				//*streamOut << currBelSt->bvec->data.size() <<",";
				//for(std::vector<SparseVector_Entry>::const_iterator iter = currBelSt->bvec->data.begin();iter != currBelSt->bvec->data.end(); iter++)
				//{
					////*streamOut << iter->value;
					//*streamOut << iter->index << "="<< iter->value;
					//if(iter < currBelSt->bvec->data.end()-1){
					//*streamOut <<", ";
					//}
					//else{
					//*streamOut << ")";
					//}
				//}
				//*streamOut << endl;
			/// <- TBP



            // added to stop simulation when at terminal state
            if(problem->getIsTerminalState(*actStateCompl))
            {
                // Terminal state
                // reward all 0, transfer back to it self
                // TODO Consider reward not all 0 case
                //cout <<"Terminal State!! timeIter " << timeIter << endl;
				if(enableFiling)
					*streamOut << "Reached terminal state" << endl;
                break;
            }

        }


        return firstAction;
    }
};
