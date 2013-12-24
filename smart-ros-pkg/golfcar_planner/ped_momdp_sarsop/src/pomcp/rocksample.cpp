#include "rocksample.h"
#include "utils.h"
#include "math.h"
#include <algorithm>
#include <fstream>
#include <sys/time.h>

using namespace std;
using namespace UTILS;

ROCKSAMPLE::ROCKSAMPLE(int size, int rocks)
:   Grid(size, size),
    Size(size),

    NumRocks(rocks),
    SmartMoveProb(0.95),
    UncertaintyCount(0)
{
    NumActions = NumRocks + 5;
    NumObservations = 3;
    RewardRange = 20;
    Discount = 0.95;
	map_time=0;
	map_counter=0;
	if (size == 4 && rocks == 4)
		Init_4_4();
	else if (size == 7 && rocks == 8)
        Init_7_8();
    else if (size == 11 && rocks == 11)
        Init_11_11();
	else if (size == 10 && rocks == 4)
		Init_10_4();
    else
        InitGeneral();
}

void ROCKSAMPLE::InitGeneral()
{
    HalfEfficiencyDistance = 20;
    StartPos = COORD(0, Size / 2);
    RandomSeed(0);
    Grid.SetAllValues(-1);
    for (int i = 0; i < NumRocks; ++i)
    {
        COORD pos;
        do
        {
            pos = COORD(Random(Size), Random(Size));
        }
        while (Grid(pos) >= 0);
        Grid(pos) = i;
        RockPos.push_back(pos);
    }
}

void ROCKSAMPLE::Init_10_4()
{
    cout << "Using special layout for rocksample(4, 4)"<<endl;
    COORD rocks[] =
    {
        COORD(0, 3),
        COORD(7, 0),
        COORD(9, 7),
        COORD(3, 9),
    };

    HalfEfficiencyDistance = 20;
    StartPos = COORD(0, 0);
    Grid.SetAllValues(-1);
    for (int i = 0; i < NumRocks; ++i)
    {
        Grid(rocks[i]) = i;
        RockPos.push_back(rocks[i]);
    }

}
void ROCKSAMPLE::Init_4_4()
{
    cout << "Using special layout for rocksample(4, 4)"<<endl;
    COORD rocks[] =
    {
        COORD(0, 1),
        COORD(2, 0),
        COORD(3, 2),
        COORD(1, 3),
    };

    HalfEfficiencyDistance = 20;
    StartPos = COORD(0, 0);
    Grid.SetAllValues(-1);
    for (int i = 0; i < NumRocks; ++i)
    {
        Grid(rocks[i]) = i;
        RockPos.push_back(rocks[i]);
    }

}

void ROCKSAMPLE::Init_7_8()
{
    // Equivalent to RockSample_7_8.pomdpx
    cout << "Using special layout for rocksample(7, 8)" << endl;

    COORD rocks[] =
    {
        COORD(2, 0),
        COORD(0, 1),
        COORD(3, 1),
        COORD(6, 3),
        COORD(2, 4),
        COORD(3, 4),
        COORD(5, 5),
        COORD(1, 6)
    };

    HalfEfficiencyDistance = 20;
    StartPos = COORD(0, 3);
    Grid.SetAllValues(-1);
    for (int i = 0; i < NumRocks; ++i)
    {
        Grid(rocks[i]) = i;
        RockPos.push_back(rocks[i]);
    }
}

void ROCKSAMPLE::Init_11_11()
{
    // Equivalent to RockSample_11_11.pomdp(x)
    cout << "Using special layout for rocksample(11, 11)" << endl;

    COORD rocks[] =
    {
        COORD(0, 3),
        COORD(0, 7),
        COORD(1, 8),
        COORD(2, 4),
        COORD(3, 3),
        COORD(3, 8),
        COORD(4, 3),
        COORD(5, 8),
        COORD(6, 1),
        COORD(9, 3),
        COORD(9, 9)
    };

    HalfEfficiencyDistance = 20;
    StartPos = COORD(0, 5);
    Grid.SetAllValues(-1);
    for (int i = 0; i < NumRocks; ++i)
    {
        Grid(rocks[i]) = i;
        RockPos.push_back(rocks[i]);
    }
}


STATE* ROCKSAMPLE::Copy(const STATE& state) const
{
    const ROCKSAMPLE_STATE& rockstate = safe_cast<const ROCKSAMPLE_STATE&>(state);
    ROCKSAMPLE_STATE* newstate = MemoryPool.Allocate();
    *newstate = rockstate;
    return newstate;
}

void ROCKSAMPLE::Validate(const STATE& state) const
{
    const ROCKSAMPLE_STATE& rockstate = safe_cast<const ROCKSAMPLE_STATE&>(state);
    assert(Grid.Inside(rockstate.AgentPos));
}

STATE* ROCKSAMPLE::CreateStartState() const
{
    ROCKSAMPLE_STATE* rockstate = MemoryPool.Allocate();
    rockstate->AgentPos = StartPos;
    rockstate->Rocks.clear();
    for (int i = 0; i < NumRocks; i++)
    {
        ROCKSAMPLE_STATE::ENTRY entry;
        entry.Collected = false;
        entry.Valuable = Bernoulli(0.5);
        entry.Count = 0;
        entry.Measured = 0;
        entry.ProbValuable = 0.5;
        entry.LikelihoodValuable = 1.0;
        entry.LikelihoodWorthless = 1.0;
        rockstate->Rocks.push_back(entry);
    }
    rockstate->Target = SelectTarget(*rockstate);
    return rockstate;
}

void ROCKSAMPLE::FreeState(STATE* state) const
{
    ROCKSAMPLE_STATE* rockstate = safe_cast<ROCKSAMPLE_STATE*>(state);
    MemoryPool.Free(rockstate);
}

bool ROCKSAMPLE::Step(STATE& state, int action,
    int& observation, double& reward) const
{
    ROCKSAMPLE_STATE& rockstate = safe_cast<ROCKSAMPLE_STATE&>(state);
    reward = 0;
    observation = E_NONE;

    if (action < E_SAMPLE) // move
    {
        switch (action)
        {
            case COORD::E_EAST:
                if (rockstate.AgentPos.X + 1 < Size)
                {
                    rockstate.AgentPos.X++;
                    break;
                }
                else
                {
                    reward = +10;
                    return true;
                }

            case COORD::E_NORTH:
                if (rockstate.AgentPos.Y + 1 < Size)
                    rockstate.AgentPos.Y++;
                else
                    reward = -100;
                break;

            case COORD::E_SOUTH:
                if (rockstate.AgentPos.Y - 1 >= 0)
                    rockstate.AgentPos.Y--;
                else
                    reward = -100;
                break;

            case COORD::E_WEST:
                if (rockstate.AgentPos.X - 1 >= 0)
                    rockstate.AgentPos.X--;
                else
                    reward = -100;
                break;
        }
    }

    if (action == E_SAMPLE) // sample
    {
        int rock = Grid(rockstate.AgentPos);
        if (rock >= 0 && !rockstate.Rocks[rock].Collected)
        {
            rockstate.Rocks[rock].Collected = true;
            if (rockstate.Rocks[rock].Valuable)
                reward = +10;
            else
                reward = -10;
        }
        else
        {
            reward = -100;
        }
    }

    if (action > E_SAMPLE) // check
    {
        int rock = action - E_SAMPLE - 1;
        assert(rock < NumRocks);
        observation = GetObservation(rockstate, rock);
        rockstate.Rocks[rock].Measured++;

        double distance = COORD::EuclideanDistance(rockstate.AgentPos, RockPos[rock]);
    	double efficiency = (1 + pow(2, -distance / HalfEfficiencyDistance)) * 0.5;

        if (observation == E_GOOD)
        {
            rockstate.Rocks[rock].Count++;
            rockstate.Rocks[rock].LikelihoodValuable *= efficiency;
            rockstate.Rocks[rock].LikelihoodWorthless *= 1.0 - efficiency;

        }
        else
        {
            rockstate.Rocks[rock].Count--;
            rockstate.Rocks[rock].LikelihoodWorthless *= efficiency;
            rockstate.Rocks[rock].LikelihoodValuable *= 1.0 - efficiency;
		}
		double denom = (0.5 * rockstate.Rocks[rock].LikelihoodValuable) +
			(0.5 * rockstate.Rocks[rock].LikelihoodWorthless);
		rockstate.Rocks[rock].ProbValuable = (0.5 * rockstate.Rocks[rock].LikelihoodValuable) / denom;
    }

    if (rockstate.Target < 0 || rockstate.AgentPos == RockPos[rockstate.Target])
        rockstate.Target = SelectTarget(rockstate);

    assert(reward != -100);
    return false;
}

bool ROCKSAMPLE::LocalMove(STATE& state, const HISTORY& history,
    int stepObs, const STATUS& status) const
{
    ROCKSAMPLE_STATE& rockstate = safe_cast<ROCKSAMPLE_STATE&>(state);
    int rock = Random(NumRocks);
    rockstate.Rocks[rock].Valuable = !rockstate.Rocks[rock].Valuable;

    if (history.Back()._Action > E_SAMPLE) // check rock
    {
        rock = history.Back()._Action - E_SAMPLE - 1;
        int realObs = history.Back()._Observation;
        // Condition new state on real observation
        int newObs = GetObservation(rockstate, rock);
        if (newObs != realObs)
            return false;

        // Update counts to be consistent with real observation
        if (realObs == E_GOOD && stepObs == E_BAD)
            rockstate.Rocks[rock].Count += 2;
        if (realObs == E_BAD && stepObs == E_GOOD)
            rockstate.Rocks[rock].Count -= 2;
    }
    return true;
}

void ROCKSAMPLE::GenerateLegal(const STATE& state, const HISTORY& history,
    vector<int>& legal, const STATUS& status) const
{

    const ROCKSAMPLE_STATE& rockstate =
        safe_cast<const ROCKSAMPLE_STATE&>(state);

    if (rockstate.AgentPos.Y + 1 < Size)
        legal.push_back(COORD::E_NORTH);

    legal.push_back(COORD::E_EAST);

    if (rockstate.AgentPos.Y - 1 >= 0)
        legal.push_back(COORD::E_SOUTH);

    if (rockstate.AgentPos.X - 1 >= 0)
        legal.push_back(COORD::E_WEST);

    int rock = Grid(rockstate.AgentPos);
    if (rock >= 0 && !rockstate.Rocks[rock].Collected)
        legal.push_back(E_SAMPLE);

    for (rock = 0; rock < NumRocks; ++rock)
        if (!rockstate.Rocks[rock].Collected)
            legal.push_back(rock + 1 + E_SAMPLE);
}


void ROCKSAMPLE::GeneratePreferred(const STATE& state, const HISTORY& history,
    vector<int>& actions, const STATUS& status) const
{

	static const bool UseBlindPolicy = false;

	if (UseBlindPolicy)
	{
		actions.push_back(COORD::E_EAST);
		return;
	}

	const ROCKSAMPLE_STATE& rockstate =
	        safe_cast<const ROCKSAMPLE_STATE&>(state);

	// Sample rocks with more +ve than -ve observations
	int rock = Grid(rockstate.AgentPos);
	if (rock >= 0 && !rockstate.Rocks[rock].Collected)
	{
		int total = 0;
		for (int t = 0; t < history.Size(); ++t)
		{
			if (history[t]._Action == rock + 1 + E_SAMPLE)
			{
				if (history[t]._Observation == E_GOOD)
					total++;
				if (history[t]._Observation == E_BAD)
					total--;
			}
		}
		if (total > 0)
		{
			actions.push_back(E_SAMPLE);
			return;
		}

	}

	// processes the rocks
	bool all_bad = true;
	bool north_interesting = false;
	bool south_interesting = false;
	bool west_interesting  = false;
	bool east_interesting  = false;

	for (int rock = 0; rock < NumRocks; ++rock)
	{
		const ROCKSAMPLE_STATE::ENTRY& entry = rockstate.Rocks[rock];
		if (!entry.Collected)
		{
			int total = 0;
			for (int t = 0; t < history.Size(); ++t)
			{
				if (history[t]._Action == rock + 1 + E_SAMPLE)
				{
					if (history[t]._Observation == E_GOOD)
						total++;
					if (history[t]._Observation == E_BAD)
						total--;
				}

			}

			if (total >= 0)
			{
				all_bad = false;

				if (RockPos[rock].Y > rockstate.AgentPos.Y)
					north_interesting = true;
				if (RockPos[rock].Y < rockstate.AgentPos.Y)
					south_interesting = true;
				if (RockPos[rock].X < rockstate.AgentPos.X)
					west_interesting = true;
				if (RockPos[rock].X > rockstate.AgentPos.X)
					east_interesting = true;
			}
		}
	}

	// if all remaining rocks seem bad, then head east
	if (all_bad)
	{
		actions.push_back(COORD::E_EAST);
		return;
	}

	// generate a random legal move, with the exceptions that:
	//   a) there is no point measuring a rock that is already collected
	//   b) there is no point measuring a rock too often
	//   c) there is no point measuring a rock which is clearly bad or good
	//   d) we never sample a rock (since we need to be sure)
	//   e) we never move in a direction that doesn't take us closer to
	//      either the edge of the map or an interesting rock
	if (rockstate.AgentPos.Y + 1 < Size && north_interesting)
			actions.push_back(COORD::E_NORTH);

	if (east_interesting)
		actions.push_back(COORD::E_EAST);

	if (rockstate.AgentPos.Y - 1 >= 0 && south_interesting)
		actions.push_back(COORD::E_SOUTH);

	if (rockstate.AgentPos.X - 1 >= 0 && west_interesting)
		actions.push_back(COORD::E_WEST);


	for (rock = 0; rock < NumRocks; ++rock)
	{
		if (!rockstate.Rocks[rock].Collected    &&
			rockstate.Rocks[rock].ProbValuable != 0.0 &&
			rockstate.Rocks[rock].ProbValuable != 1.0 &&
			rockstate.Rocks[rock].Measured < 5  &&
			std::abs(rockstate.Rocks[rock].Count) < 2)
		{
			actions.push_back(rock + 1 + E_SAMPLE);
		}
	}
}

int ROCKSAMPLE::GetObservation(const ROCKSAMPLE_STATE& rockstate, int rock) const
{
    double distance = COORD::EuclideanDistance(rockstate.AgentPos, RockPos[rock]);
    double efficiency = (1 + pow(2, -distance / HalfEfficiencyDistance)) * 0.5;

    if (Bernoulli(efficiency))
        return rockstate.Rocks[rock].Valuable ? E_GOOD : E_BAD;
    else
        return rockstate.Rocks[rock].Valuable ? E_BAD : E_GOOD;
}

int ROCKSAMPLE::SelectTarget(const ROCKSAMPLE_STATE& rockstate) const
{
    int bestDist = Size * 2;
    int bestRock = -1;
    for (int rock = 0; rock < NumRocks; ++rock)
    {
        if (!rockstate.Rocks[rock].Collected
            && rockstate.Rocks[rock].Count >= UncertaintyCount)
        {
            int dist = COORD::ManhattanDistance(rockstate.AgentPos, RockPos[rock]);
            if (dist < bestDist)
                bestDist = dist;
        }
    }
    return bestRock;
}

bool ROCKSAMPLE::Same(const BELIEF_STATE& beliefState1,const BELIEF_STATE& beliefState2,double & distance)
{
	//return false;
	const ROCKSAMPLE_STATE* rockstate1;
	const ROCKSAMPLE_STATE* rockstate2;
	if(beliefState1.GetNumSamples()==0||beliefState2.GetNumSamples()==0) 	return false;
	rockstate1=safe_cast<const ROCKSAMPLE_STATE*>(beliefState1.GetSample(0));
	rockstate2=safe_cast<const ROCKSAMPLE_STATE*>(beliefState2.GetSample(0));
	if((rockstate1->AgentPos.X!=rockstate2->AgentPos.X)||(rockstate1->AgentPos.Y!=rockstate2->AgentPos.Y)) return false;

	int rock_count1[20][2]={0};
	int rock_count2[20][2]={0};

	//cout<<"num of beliefs "<<beliefState1.GetNumSamples()<<endl;
	int size1=100;
	if(beliefState1.GetNumSamples()<100) size1=beliefState1.GetNumSamples();
	for(int i=0;i<size1;i++)
	{
    	rockstate1=safe_cast<const ROCKSAMPLE_STATE*>(beliefState1.GetSample(i));
		for(int j=0;j<NumRocks;j++)
		{
			if(rockstate1->Rocks[j].Valuable==false ||rockstate1->Rocks[j].Collected==true)
			{
				rock_count1[j][0]++;
			}
			else
			{
				rock_count1[j][1]++;
			}
	
		}
	}
	//cout<<"num of beliefs "<<beliefState2.GetNumSamples()<<endl;
	int size2=100;
	if(beliefState2.GetNumSamples()<100) 	size2=beliefState2.GetNumSamples();
	for(int i=0;i<size2;i++)
	{
    	rockstate2=safe_cast<const ROCKSAMPLE_STATE*>(beliefState2.GetSample(i));
		for(int j=0;j<NumRocks;j++)
		{
			if(rockstate2->Rocks[j].Valuable==false ||rockstate2->Rocks[j].Collected==true)
			{
				rock_count2[j][0]++;
			}
			else
			{
				rock_count2[j][1]++;
			}
	
		}
	}
	
    distance=0;
	//L1 distance
	
	for(int i=0;i<NumRocks;i++)
	{
		double p1,p2;
		p1=(rock_count1[i][0]+0.0)/(rock_count1[i][0]+rock_count1[i][1]);
		p2=(rock_count2[i][0]+0.0)/(rock_count2[i][0]+rock_count2[i][1]);
		if(fabs(p1-p2)>0.2)   return false;
		if(fabs(p1-p2)>distance) 
		{
			distance=fabs(p1-p2);
		}
	}
	//LMax Distance
	/*
	double my_dist=0;
	for(int i=0;i<NumRocks;i++)
	{
		double p1,p2;
		p1=(rock_count1[i][0]+0.0)/(rock_count1[i][0]+rock_count1[i][1]);
		p2=(rock_count2[i][0]+0.0)/(rock_count2[i][0]+rock_count2[i][1]);
		my_dist+=fabs(p1-p2);
		if(my_dist>0.65)   return false;
	}
	distance=my_dist;*/
	return true;
}

VNODE* ROCKSAMPLE::Find(VNODE*v1,const HISTORY &h)
{
	bool found=false;
	int i;
	double distance;
	double min_dist=100;
	int min_index=-1;
	for(i=0;i<vnode_list.size();i++)
	{
		//if(vnode_list[i]==v1) 	return 0;
		if(Same(v1->Beliefs(),vnode_list[i]->Beliefs(),distance))
		{
			found=true;
			//cout<<distance<<endl;
			if(distance<min_dist)
			{
				min_index=i;
				min_dist=distance;
			}
		}
	}
	if(found)
	{
			return vnode_list[min_index];
	}
	else if(v1->inserted==false)
	{
			vnode_list.push_back(v1);
			history_list.push_back(h);
			v1->inserted=true;
			return v1;
	}
}

VNODE* ROCKSAMPLE::MapBelief(VNODE*vn,VNODE*r,const HISTORY &h,int historyDepth)
{
	//return vn;
	if(vn==0) 	return 0;
	if(vn->inserted) return vn;
	//return vn;
	BELIEF_STATE&beliefState=vn->Beliefs();
	/*	
	if(h.Size()-historyDepth==2)
	{
		int begin=historyDepth;
		int a1=h[begin]._Action;
		int o1=h[begin]._Observation;
		int a2=h[begin+1]._Action;
		int o2=h[begin+1]._Observation;
		if(a1>a2&&(a1!=4)&&(a2!=4))
		{
			//cout<<a1<<" "<<o1<<" "<<a2<<" "<<o2<<endl;
			if(r->Child(a2).Child(o2)==0) 	
			{
				//cout<<"end map"<<endl;
				return 0;
			}
			else  
			{
				//cout<<"end map"<<endl;
				return r->Child(a2).Child(o2)->Child(a1).Child(o1);
			}
		}
	}
	else
	{
		return vn;
	}*/
	
	if(beliefState.GetNumSamples()<20)  
	{
		//VNODE*parent;
		//if(
		return vn;
	}


	else if(beliefState.GetNumSamples()<30)
	{

		for(int i=0;i<20;i++)
		{
			VNODE*prt=r;
			for(int i=historyDepth;i<h.Size()-1;i++)
			{
				prt=prt->Child(h[i]._Action).Child(h[i]._Observation);
			}
			//VNODE*prt=vn.GetParent();
			STATE*prt_state=prt->Beliefs().CreateSample(*this);
			double rwd;
			int obs;
			Step(*prt_state,h.Back()._Action,obs,rwd);
			if(prt->Child(h.Back()._Action).Child(obs))
				prt->Child(h.Back()._Action).Child(obs)->Beliefs().AddSample(prt_state);
			//vn->Beliefs().AddSample(prt_state);
		}

		return vn;

	}
	//if(beliefState.GetNumSamples()<20) return vn;
	//ROCKSAMPLE_BELIEF rb(beliefState,NumRocks);
	//VNODE * nn=belief_map[rb];
	//
	/*	
	if(h.Size()==2&&h[h.Size()-2]._Action==0&&h.Back()._Action==2)
	{
		int watch;
		watch=0;
	}*/
	/*
	long long L1,L2;
	timeval tv;
	gettimeofday(&tv,NULL);
	L1=tv.tv_sec*1000*1000+tv.tv_usec;	*/
	VNODE*nn=Find(vn,h);
	/*
	gettimeofday(&tv,NULL);
	L2=tv.tv_sec*1000*1000+tv.tv_usec;
	map_time+=(L2-L1);*/
	return nn;
	//BeliefPool.Find(beliefState);
}

void ROCKSAMPLE::DisplayBeliefs(const BELIEF_STATE& beliefState,
    std::ostream& ostr) const
{

	ROCKSAMPLE_STATE rockstate; 
	int rock_count[20][2]={0};
	for (int i=0;i<beliefState.GetNumSamples();i++)
    {
    	const STATE* p_rockstate=beliefState.GetSample(i);
    	rockstate=safe_cast<const ROCKSAMPLE_STATE&>(*p_rockstate);
		if(i==0)
		{
			ostr<<"Agent Pos "<<rockstate.AgentPos.X<<" "<<rockstate.AgentPos.Y<<endl;
		}
		for(int j=0;j<NumRocks;j++)
		{
			if(rockstate.Rocks[j].Valuable==false ||rockstate.Rocks[j].Collected==true)
			{
				rock_count[j][0]++;
			}
			else
			{
				rock_count[j][1]++;
			}
	
		}
    }
	ostr<<"beliefs are:"<<endl;
	for(int i=0;i<NumRocks;i++)
	{
		ostr<<(rock_count[i][0]+0.0)/(rock_count[i][0]+rock_count[i][1])<<" ";
	}
	ostr<<endl;
}

void ROCKSAMPLE::DisplayState(const STATE& state, std::ostream& ostr) const
{
    const ROCKSAMPLE_STATE& rockstate = safe_cast<const ROCKSAMPLE_STATE&>(state);
    ostr << endl;
    for (int x = 0; x < Size + 2; x++)
        ostr << "# ";
    ostr << endl;
    for (int y = Size - 1; y >= 0; y--)
    {
        ostr << "# ";
        for (int x = 0; x < Size; x++)
        {
            COORD pos(x, y);
            int rock = Grid(pos);
            const ROCKSAMPLE_STATE::ENTRY& entry = rockstate.Rocks[rock];
            if (rockstate.AgentPos == COORD(x, y))
                ostr << "* ";
            else if (rock >= 0 && !entry.Collected)
                ostr << rock << (entry.Valuable ? "$" : "X");
            else
                ostr << ". ";
        }
        ostr << "#" << endl;
    }
    for (int x = 0; x < Size + 2; x++)
        ostr << "# ";
    ostr << endl;
}

void ROCKSAMPLE::DisplayObservation(const STATE& state, int observation, std::ostream& ostr) const
{
    switch (observation)
    {
    case E_NONE:
        break;
    case E_GOOD:
        ostr << "Observed good" << endl;
        break;
    case E_BAD:
        ostr << "Observed bad" << endl;
        break;
    }
}

void ROCKSAMPLE::DisplayAction(int action, std::ostream& ostr) const
{
    if (action < E_SAMPLE)
        ostr << COORD::CompassString[action] << endl;
    if (action == E_SAMPLE)
        ostr << "Sample" << endl;
    if (action > E_SAMPLE)
        ostr << "Check " << action - E_SAMPLE-1 << endl;
}

int ROCKSAMPLE::GetNumOfStates()
{
	return Size*Size*pow(2,NumRocks);	
}

double ROCKSAMPLE::QMDP_Heuristic(STATE*state)
{
	int n=StateToN(state);
	return QValues[n];
}

int ROCKSAMPLE::QMDP_SelectAction(STATE*state)
{
	int observation;
	double action_reward;
	double delayed_reward;
	double max_reward=-10000;
	int best_action=-1;
	for(int i=0;i<5;i++)
	{
		STATE * newstate=Copy(*state);
		bool terminal=Step(*newstate,i,observation,action_reward);
		if(!terminal)
		{
			int N=StateToN(newstate);
			delayed_reward=QValues[N];
		}
		else
		{
			delayed_reward=0;
		}
		if(action_reward+Discount*delayed_reward>max_reward) 
		{
				max_reward=(action_reward+Discount*delayed_reward);
				best_action=i;
		}
		FreeState(newstate);
	}
	return best_action;
}

void ROCKSAMPLE::Init_QMDP()
{
	ROCKSAMPLE_STATE*rockstate=MemoryPool.Allocate();
	for(int i=0;i<GetNumOfStates();i++)
	{
		NToState(i,rockstate);
		QValues[i]=QMDP_Value(rockstate);
	}
}

void ROCKSAMPLE::Init_QMDP2()
{
	ROCKSAMPLE_STATE*rockstate= MemoryPool.Allocate();
	int observation;
	double reward;
	for(int i=0;i<GetNumOfStates();i++)  	QValues[i]=0;
	for(int lv=0;lv<100;lv++)   //Number Of Iterations
	{
		for(int i=0;i<GetNumOfStates();i++)
		{
			for(int a=0;a<5;a++)
			{
				if(lv==90)
				{
					int z=0;
					z++;
				}
				NToState(i,rockstate);
				bool terminal=Step(*rockstate,a,observation,reward);
				if(terminal)
				{
					if(reward>QValues[i])  QValues[i]=reward;
				}
				else
				{
						int target=StateToN(rockstate);
						if(QValues[target]*Discount+reward>QValues[i])
						{
								QValues[i]=QValues[target]*Discount+reward;
						}
				}
			}
		}
	}
}

double ROCKSAMPLE::QMDP_Value(ROCKSAMPLE_STATE*rockstate)
{
	int nr=0;
	int *perm=new int[NumRocks];
	for(int i=0;i<NumRocks;i++)
	{
		if(rockstate->Rocks[i].Collected==false&&rockstate->Rocks[i].Valuable==true)
		{
				perm[nr]=i;
				nr++;
		}
	}

	double max_value=0;
	max_value=pow(Discount,(Size-rockstate->AgentPos.X))*10;
			//generate all the permutations of different length;
	std::sort(perm,perm+nr);
	do{
			for(int i=0;i<nr;i++)    //how many rocks to sample before leave
			{

					int j;
					double value=0;
					int steps;
					double dis=1;
					steps=(abs(RockPos[perm[0]].X-rockstate->AgentPos.X)+abs(RockPos[perm[0]].Y-rockstate->AgentPos.Y));
					dis=dis*pow(Discount,steps);
					value+=dis*10;
					for(j=0;j<i;j++)
					{
							steps=abs(RockPos[perm[j+1]].X-RockPos[perm[j]].X)+abs(RockPos[perm[j+1]].Y-RockPos[perm[j]].Y);
							dis=dis*pow(Discount,steps);
							value+=dis*10;
					}
					steps=Size-RockPos[perm[i]].X;
					dis=dis*pow(Discount,steps);
					value+=dis*10;
					if(value>max_value)
					{
							max_value=value;
					}
			}
			/*
			for(int i=0;i<nr;i++)
			{
				cout<<perm[i]<<" ";
			}
			cout<<endl;*/
	}while(std::next_permutation(perm,perm+nr));

	
	return max_value;

}


int ROCKSAMPLE::StateToN(STATE* state)
{
	ROCKSAMPLE_STATE* rockstate=safe_cast<ROCKSAMPLE_STATE*>(state);
	int add_agentpos=(rockstate->AgentPos.X*Size+rockstate->AgentPos.Y)*pow(2,NumRocks); 
	int add_rockvalue=0;
	for(int i=NumRocks-1;i>=0;i--)
	{
		add_rockvalue<<=1;
		if(rockstate->Rocks[i].Collected==false&&rockstate->Rocks[i].Valuable==true)
		{
				add_rockvalue+=1;
		}
	}
	return add_agentpos+add_rockvalue;
	//teststate.RobPos.Y
}

void ROCKSAMPLE::NToState(int n,ROCKSAMPLE_STATE*rockstate)
{
	
	//5 lanes
	int pos=n/pow(2,NumRocks);
	rockstate->AgentPos.X=pos/Size;
	rockstate->AgentPos.Y=pos-Size*rockstate->AgentPos.X;
	int rockseq=n%(int)pow(2,NumRocks);
	rockstate->Rocks.clear();
	for(int i=0;i<NumRocks;i++)
	{
		//r=rockseq%2; 
		if(rockseq%2==1)
		{
				ROCKSAMPLE_STATE::ENTRY entry;
				entry.Collected = false;
				entry.Valuable = true;     
				rockstate->Rocks.push_back(entry);
		}
		else
		{
				ROCKSAMPLE_STATE::ENTRY entry;
				entry.Collected = true;
				entry.Valuable = false;     
				rockstate->Rocks.push_back(entry);

		}
		rockseq>>=1;
	}
}

void ROCKSAMPLE::DisplayQMDP()
{
	ROCKSAMPLE_STATE*rockstate=MemoryPool.Allocate();
	for(int i=0;i<10;i++)
	{
		NToState(i,rockstate);
		DisplayState(*rockstate,std::cout);	
		cout<<"QValues: "<<QValues[i];
	}
}


void ROCKSAMPLE::WriteQMDP()
{
	ofstream out("rocksample_qvalues.txt");
	int i,j;
	for(i=0;i<GetNumOfStates();i++)
	{
		out<<QValues[i]<<" ";
	}
}

void ROCKSAMPLE::LoadQMDP()
{
	int i,j;
	ifstream in("rocksample_qvalues.txt");
	for(i=0;i<GetNumOfStates();i++)
	{
		in>>QValues[i];
	}
}
/*
void ROCKSAMPLE::Init_QMDP()
{
			int i,j,k,ns1,ns2;
	//ROS_INFO("num of states %d",GetNumOfStates());
	
	for (i=1;i<10;i++)		//num of bellman-iterations
	{
		for(ns1=0;ns1<GetNumOfStates();ns1++)	//all numerations
		{

			for(j=0;j<NumActions;j++)
			{
				qValue[ns1][j]=0;
				double prob=0;	{
		int r;
		r=rockseq%2;
		if(r)
		{
			rockstate->Rocks[i].Collected=true;
			rockstate->Rocks[i].Valuable=true;
		}
		r>>=1;
	}
}

/*
void ROCKSAMPLE::Init_QMDP()
{
			int i,j,k,ns1,ns2;
	//ROS_INFO("num of states %d",GetNumOfStates());
	
	for (i=1;i<10;i++)		//num of bellman-iterations
	{
		for(ns1=0;ns1<GetNumOfStates();ns1++)	//all numerations
		{
			for(j=0;j<NumActions;j++)
			{
				qValue[ns1][j]=0;
				double prob=0;
				for(ns2=0;ns2<GetNumOfStates();ns2++)
				{
					prob+=TransFn(ns1,j,ns2);
					qValue[ns1][j]+=TransFn(ns1,j,ns2)*Value[ns2];		
				}
					//ROS_INFO("%f",prob);
				if(prob!=0)
				{

				for(ns2=0;ns2<GetNumOfStates();ns2++)
				{
					prob+=TransFn(ns1,j,ns2);
					qValue[ns1][j]+=TransFn(ns1,j,ns2)*Value[ns2];		
				}
					//ROS_INFO("%f",prob);
				if(prob!=0)
				{
					qValue[ns1][j]/=prob;
				}
				qValue[ns1][j]*=Discount;
			}
			Value[ns1]=MaxQ(ns1)+GetReward(ns1);
		}
		//ROS_INFO("initialization finished");
	}
	//ROS_INFO("initialization finished");

}

double ROCKSAMPLE::Init_TransFn()
{
	for(int ns1=0;ns1<GetNumOfStates();ns1++)
	{
		for(int a=0;a<NumActions;a++)
		{
			
		}
	}
	NToState(s1,qstate1);
	NToState(s2,qstate2);
	if(qstate1
}*/
