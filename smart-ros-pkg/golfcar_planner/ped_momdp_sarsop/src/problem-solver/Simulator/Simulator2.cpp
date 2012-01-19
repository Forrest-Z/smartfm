/**
 * @file SimulationDriver.cc
 * @brief This is the simulation driver which will call the necessary functions and make
 * the correct calls as and when required
 * @author Amit Jain
 * @date 2007-04-10
 */

#include "SimulationEngine2.h"
#include "GlobalResource.h"
#include "SimulationRewardCollector.h"
#include <string>
#include <stdlib.h>
#include <sstream>
#include <fstream>
#include <ctime>

#include "CPTimer.h"

#ifdef _MSC_VER
#else
//for timing
#include <sys/param.h>
#include <sys/types.h>
#include <sys/times.h>
//end for timing
#endif

#include "MOMDP.h"
#include "ParserSelector.h"
#include "AlphaVectorPolicy.h"

using namespace std;
using namespace momdp;



/// Generate Stats
ofstream statfile;
int num_steps=0;
int robx;
//int rnd_seed=1;

/// read from config.txt

bool mostLikelyAlgo; 
//bool lane;
//bool cross;//=!lane;
//bool ell;

////////////////////////


void print_usage(const char* cmdName) 
{
	cout << "Usage: " << cmdName << " POMDPModelFileName --policy-file policyFileName --simLen numberSteps \n" 
<<"	--simNum numberSimulations [--fast] [--srand randomSeed] [--output-file outputFileName]\n" 
<<"    or " << cmdName << " --help (or -h)  Print this help\n" 
<<"    or " << cmdName << " --version	  Print version information\n" 
<<"\n"
<<"Simulator options:\n"
<<"  --policy-file policyFileName	Use policyFileName as the policy file name (compulsory).\n"
<<"  --simLen numberSteps		Use numberSteps as the number of steps for each\n" 
<<"				simulation run (compulsory).\n"
<<"  --simNum numberSimulations	Use numberSimulations as the number of simulation runs\n" 
<<"				(compulsory).\n"
<<"  -f or --fast			Use fast (but very picky) alternate parser for .pomdp files.\n"
<<"  --srand randomSeed		Set randomSeed as the random seed for simulation.\n" 
<<"				It is the current time by default.\n"
//<<"  --lookahead yes/no		Set 'yes' ('no') to select action with (without) one-step\n" 
//<<"				look ahead. Action selection is with one-step look ahead\n" 
//<<"				by default.\n" 
<<"\n"
<<"Output options:\n"
<<"  --output-file outputFileName	Use outputFileName as the name for the output file\n" 
<<"				that contains the simulation trace.\n"
		<< "Example:\n"
		<< "  " << cmdName << " --simLen 100 --simNum 100 --policy-file out.policy Hallway.pomdp\n";

// 	cout << "usage: binary [options] problem:\n"
// 		<< "--help, print this message\n"
// 		<< "--policy-file, policy file to be used\n"
// 		<< "--output-file, output file to be used\n"
// 		<< "--simLen, length of simulation\n"
// 		<< "--simNum, number of simulations\n"
// 		<< "--srand, random seed (default: current time)\n"
// 		<< "--lookahead, use \"one-step look ahead\" when selecting action (default: yes)\n"
// 		<< "Examples:\n"
// 		<< " ./simulate --simLen 100 --simNum 100 --policy-file out.policy Hallway.pomdp\n";

}

void generateSimLog(SolverParams& p, double& globalExpRew, double& confInterval)
{
    int length;
    char str1[102];
    string str_comb;

    int startpos = 0;
    int i;
    for (i = p.problemName.length() - 1; i >= 0; i--) {
        if (p.problemName[i] == '/') {
            startpos = i + 1;
            break;
        }
    }

    str_comb.append(p.problemName.begin() + startpos, p.problemName.end());

    str_comb.append("SimLog");
    cout << str_comb << endl;

    length = str_comb.copy(str1, 100);
    str1[length] = '\0';

    FILE *fp = fopen(str1, "a");

    //  FILE *fp = fopen("sim.log","a");
    if (fp == NULL) 
    {
        cerr << "cant open sim logfile\n";
        exit(1);
    }

    fprintf(fp, "%f ", globalExpRew);
    fprintf(fp, "%f ", globalExpRew - confInterval);
    fprintf(fp, "%f ", globalExpRew + confInterval);
    fprintf(fp, "\n");
    fclose(fp);


}

void parse_simul_config( fstream& configfile)
{
	/// simulation variables are ped pos, ped uncert

	//char input[50]; /// collect input params, white space separator

	bool lane = false;
	bool crosssmall = false;
	bool crosslarge = false;
	bool ell = false;
	
	int env, pedx, pedy, ulevel;
	
	string input;

	while(!configfile.eof())
	{
		configfile >> input;
		
		//cout << " Parsing input " << input << endl;

		if(input.find("env:") != string::npos)
		{
			sscanf(input.c_str(), " env:%d ", &env);
			
			if(env==0)
				lane=true;
			else if( env==1 )
				crosssmall=true;
			else if ( env==2)
				crosslarge=true;
			else if ( env==3)
				ell = true;
		}
		else if(input.find("algo:") != string::npos)
		{
			sscanf(input.c_str(), " algo:%d ", &mostLikelyAlgo);
		}

		else if(input.find("robx:") != string::npos)
		{
			sscanf(input.c_str(), " robx:%d ", &robx);
		}
		//else if(input.find("Y_INTER:") != string::npos)
		//{
			//sscanf(input.c_str(), " Y_INTER:%d ", &Y_INTER);
		//}
		//else if(input.find("simul:") != string::npos)
		//{
			//sscanf(input.c_str(), " simul:%d ", &simul);
		//}
		else if(input.find("pedx:") != string::npos)
		{
			sscanf(input.c_str(), " pedx:%d ", &pedx);
		}
		else if(input.find("pedy:") != string::npos)
		{
			sscanf(input.c_str(), " pedy:%d ", &pedy);
		}
		else if(input.find("pedU:") != string::npos)
		{
			sscanf(input.c_str(), " pedU:%d ", &ulevel);
		
		}
		else
		{
			cout << " unused input " << input << endl;
		}
	} /// end parsing 
	
	cout << " out of parsing " << endl;
	
	char type[10];
	char name[100];
	if(mostLikelyAlgo)
		sprintf(type,"ML");
	else
		sprintf(type,"momdp");
		
	cout << endl << " Problem " << env << " pedx:" << pedx << " pedy:" << pedy << " robx:" << robx << endl << endl;
	
	if(lane)
	{
		sprintf(name,"stats-lane-%s-x%02d-y%02d-r%02d-u%02d.dat",type,pedx,pedy,robx,ulevel);
		//rx_i = 1;
	}
	else if(crosssmall)
	{
		sprintf(name,"stats-cross-small-%s-x%02d-y%02d-r%02d-u%02d.dat",type,pedx,pedy,robx,ulevel);
		//rx_i = 5;
	}
	else if(crosslarge)
	{
		sprintf(name,"stats-cross-large-%s-x%02d-y%02d-r%02d-u%02d.dat",type,pedx,pedy,robx,ulevel);
		//rx_i = 5;
	}
	else if(ell)
	{
		sprintf(name,"stats-ell-%s-x%02d-y%02d-r%02d-u%02d.dat",type,pedx,pedy,robx,ulevel);
		//sprintf(name,"stats-ell-%s.dat",type);
	}
	else
	{
		cout << " unknown environment " << endl;	
		exit(1);
	}
	statfile.open (name);
	
}


int main(int argc, char **argv) 
{

	if(argc<2)
	{
		cout << " check usage " << endl;
		cout << " pomdpsim-rss --simLen 100 --simNum 100 --policy-file rss-cross-700.policy rss-cross-simul.pomdpx " << endl;
		exit(1);
	}
	
	//fstream varfileLoc;
	//variablefile.open("varFileLoc.txt");
	
	//char varfilename[50];
	fstream  variablefile;
	
	variablefile.open("config-simul-rss.txt");
	assert (!variablefile.fail( ));     
	
	cout << " done opening " << endl; 
	
	parse_simul_config(variablefile);
	
	cout << " done parsing " << endl; 

	


	//char name[50];

		
	
	
	//configfile >> input;
	//sscanf(input, " pedx:%d ", &px_i);
	//configfile >> input;
	//sscanf(input, " pedy:%d ", &py_i);
	//configfile >> input;
	//sscanf(input, " pedU:%d ", &uncert_i);

	//configfile >> input;
	//sscanf(input, " robx:%d ", &rx_i);

	//configfile >> px_i;
	//configfile >> py_i;
	//configfile >> rx_i;
	
	//configfile >> uncert_i;


	
	
    try
    {
        SolverParams* p = &GlobalResource::getInstance()->solverParams;

        bool parseCorrect = SolverParams::parseCommandLineOption(argc, argv, *p);
        if(!parseCorrect)
        {
            print_usage(p->cmdName); 
            exit(EXIT_FAILURE);
        }

        //check validity of options
        if (p->policyFile == "" || p->simLen == -1 || p->simNum == -1) 
        {
            print_usage(p->cmdName); 
            return 0;
        }


        bool enableFiling = false;
        if (p->outputFile.length() == 0) 
        {
            enableFiling = false;
        } 
        else 
        {
            enableFiling = true;
        }

	cout << "\nLoading the model ..." << endl << "  ";
        SharedPointer<MOMDP> problem = ParserSelector::loadProblem(p->problemName, *p);

        if(p->stateMapFile.length() > 0 )
        {
            // generate unobserved state to variable value map
            ofstream mapFile(p->stateMapFile.c_str());
            for(int i = 0 ; i < problem->YStates->size() ; i ++)
            {
                mapFile << "State : " << i <<  endl;
                map<string, string> obsState = problem->getFactoredUnobservedStatesSymbols(i);
                for(map<string, string>::iterator iter = obsState.begin() ; iter != obsState.end() ; iter ++)
                {
                    mapFile << iter->first << " : " << iter->second << endl ;
                }
            }
            mapFile.close();
        }

        SharedPointer<AlphaVectorPolicy> policy = new AlphaVectorPolicy(problem);

	cout << "\nLoading the policy ..." << endl;
	cout << "  input file   : " << p->policyFile << endl;
        bool policyRead = policy->readFromFile(p->policyFile);
        if(!policyRead)
        {
            return 0;
        }


	cout << "\nSimulating ..." << endl;
	
        if(p->useLookahead) /// TBP : always using one step look ahead
        {
            cout << "  action selection :  one-step look ahead" << endl;
        }
        else
        {
        }

        SimulationRewardCollector rewardCollector;
        rewardCollector.setup(*p);

        ofstream * foutStream = NULL;        
        srand(p->seed);//Seed for random number.  Xan
        cout << p->seed << endl;



        if (enableFiling) 
        {
            foutStream = new ofstream(p->outputFile.c_str());
        }

        for (int currSim = 0; currSim < p->simNum; currSim++) 
        {
            SimulationEngine2 engine;
            
            engine.setup(problem, policy, p); /// TBP : p is params

            double reward = 0, expReward = 0;

			cout << "running algo " << mostLikelyAlgo << endl;
            int firstAction = engine.runFor(p->simLen, foutStream, reward, expReward,mostLikelyAlgo,robx,statfile);
            if(firstAction < 0)
            {
                // something wrong happend, exit
                return 0;
            }

            rewardCollector.addEntry(currSim, reward, expReward);
            rewardCollector.printReward(currSim);

        }

        if (enableFiling) 
        {
            foutStream->close();
        }

        rewardCollector.printFinalReward();
        DEBUG_LOG( generateSimLog(*p, rewardCollector.globalExpRew, rewardCollector.confInterval); );

    }
    catch(bad_alloc &e)
    {
        if(GlobalResource::getInstance()->solverParams.memoryLimit == 0)
        {
            cout << "Memory allocation failed. Exit." << endl;
        }
        else
        {
            cout << "Memory limit reached. Please try increase memory limit" << endl;
        }

    }
    catch(exception &e)
    {
        cout << "Exception: " << e.what() << endl ;
    }
    return 0;
}

